// Copyright 2008 Isis Innovation Limited
#include "ptam/System.h"
#include "ptam/OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/Tracker.h"
#include "ptam/TrackerData.h"
//#include "ptam/ARDriver.h"
#include "ptam/MapViewer.h"
#include "ptam/LevelHelpers.h"
#include "ptam/MapPoint.h"
#include "ptam/Params.h"
#include "ptam/KeyFrame.h"
#include <ptam_com/ptam_info.h>
#include <opencv/cv.h>
#include <cvd/vision.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <cmath>
using namespace CVD;
using namespace std;
using namespace GVars3;

pthread_mutex_t System::odom_mutex = PTHREAD_MUTEX_INITIALIZER;

System::System() :
      nh_("vslam"), image_nh_(""), first_frame_(true), mpMap(NULL)
{

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pub_pose_world_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_world", 1);
  pub_info_ = nh_.advertise<ptam_com::ptam_info> ("info", 1);
  pub_visibleCloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("frame_points", 1);
  
  pub_odom_ = nh_.advertise<nav_msgs::Odometry> ("odom", 1);
  srvPC_ = nh_.advertiseService("pointcloud", &System::pointcloudservice,this);
  srvKF_ = nh_.advertiseService("keyframes", &System::keyframesservice,this);
  srvPosePC_ = nh_.advertiseService("posepointcloud", &System::posepointcloudservice,this);

  sub_imu_ = nh_.subscribe("/mobile_base/sensors/imu_data", 100, &System::imuCallback, this);
  sub_kb_input_ = nh_.subscribe("key_pressed", 100, &System::keyboardCallback, this);
  
  image_nh_.setCallbackQueue(&image_queue_);

  // get topic to subscribe to:
  std::string topic = image_nh_.resolveName("image");
  if (topic == "/image")
  {
    ROS_WARN("video source: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./ptam image:=<image topic>");
  }
  image_transport::ImageTransport it(image_nh_);
  sub_image_ = it.subscribe(topic, 1, &System::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));
  pub_preview_image_ = it.advertise("vslam/preview", 1);
}


void System::init(const CVD::ImageRef & size)
{
  img_bw_.resize(size);
  img_rgb_.resize(size);

  mpCamera = new ATANCamera("Camera");
  
  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera, nh_);
  mpTracker = new Tracker(size, *mpCamera, *mpMap, *mpMapMaker);
  sub_odom_ = nh_.subscribe("/odom", 100, &System::odomCallback, this);
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);

  if(PtamParameters::fixparams().gui){
    mGLWindow = new GLWindow2(size, "PTAM");
    mpMapViewer = new MapViewer(*mpMap, *mGLWindow);

    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  }
  
}


void System::Run()
{
  while(ros::ok()){
    //    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
    //    image_queue_.callAvailable();

    ros::getGlobalCallbackQueue()->callAvailable();
    image_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

void System::imageCallback(const sensor_msgs::ImageConstPtr & img)
{
  //	static ros::Time t = img->header.stamp;


  //ROS_ASSERT(img->encoding == sensor_msgs::image_encodings::MONO8 && img->step == img->width);

  const VarParams& varParams = PtamParameters::varparams();

  if(first_frame_){
    init(CVD::ImageRef(img->width, img->height));
    first_frame_ = false;
  }

  TooN::SO3<double> imu_orientation;
  if (varParams.MotionModelSource == ptam::PtamParams_MM_IMU)
  {
    sensor_msgs::Imu imu;


    if (!findClosest(img->header.stamp, imu_msgs_, &imu, 0.01))
    {
      ROS_WARN("no imu match, skipping frame");
      return;
    }
    if (!transformQuaternion(img->header.frame_id, imu.header, imu.orientation, imu_orientation))
    {
      return;
    }
  }


//  -------------------
  // TODO: avoid copy, by calling TrackFrame, with the ros image, because there is another copy inside TrackFrame
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  memcpy(img_bw_.data(),cv_ptr->image.data,img->width * img->height);
  /*CVD::BasicImage<CVD::byte> img_tmp((CVD::byte *)&img->data[0], CVD::ImageRef(img->width, img->height));
  CVD::copy(img_tmp, img_bw_);
*/
  bool tracker_draw = false;

  static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
  bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;

  if(PtamParameters::fixparams().gui){
    //CVD::copy(img_tmp, img_rgb_);
    memcpy(img_rgb_.data(),cv_ptr->image.data,img->width * img->height);

    mGLWindow->SetupViewport();
    mGLWindow->SetupVideoOrtho();
    mGLWindow->SetupVideoRasterPosAndZoom();
    tracker_draw = !bDrawMap;
  }

  mpTracker->TrackFrame(img_bw_, tracker_draw, imu_orientation);

  publishPoseAndInfo(img->header);

  publishPreviewImage(img_bw_, img->header);
  std::cout << mpMapMaker->getMessageForUser();

  if(PtamParameters::fixparams().gui){
    string sCaption;

    if (bDrawMap)
      mpMapViewer->DrawMap(mpTracker->GetCurrentPose());

    if (bDrawMap)
      sCaption = mpMapViewer->GetMessageForUser();
    else
      sCaption = mpTracker->GetMessageForUser();
    mGLWindow->DrawCaption(sCaption);
    mGLWindow->DrawMenus();
    mGLWindow->swap_buffers();
    mGLWindow->HandlePendingEvents();
  }
  //	usleep(50000);
  //
  //  ros::Time t1 = img->header.stamp;
  //
  //  static unsigned long c=0;
  //  if((c+1)<(img->header.seq))
  //  {
  //	  ROS_WARN_STREAM("missed " << img->header.seq-c+1<< " frame(s)");
  //	  ROS_WARN_STREAM("time: " <<  (t1.toSec()-t.toSec()));
  //  }
  //  ROS_WARN_STREAM("time: " <<  1/(t1.toSec()-t.toSec()));
  //  c=img->header.seq;
  //  t = img->header.stamp;

}


void System::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  imu_msgs_.push(*msg);
  if (imu_msgs_.size() > 100)
    imu_msgs_.pop();
}

void System::odomCallback(const nav_msgs::OdometryPtr &odomPtr)
{
  pthread_mutex_lock(&odom_mutex);
  if(mpTracker!=NULL)
    mpTracker->odometry = *odomPtr;

  pthread_mutex_unlock(&odom_mutex);
}

template<class T>
bool System::findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay)
{
  double best_dt(1e9);
  double tmp_dt;
  //  size_t qs_before = queue.size();
  //  int i = 0;
  while (!queue.empty())
  {
    const T & curr_obj = queue.front();
    tmp_dt = (timestamp - curr_obj.header.stamp).toSec();

    if (tmp_dt < -max_delay)
      break;
    if (std::abs(tmp_dt) < best_dt)
    {
      best_dt = std::abs(tmp_dt);
      *obj = curr_obj;
      //      i++;
    }
    queue.pop();
  }
  if (best_dt > max_delay)
  {
    //    ROS_WARN("dt(%f) > 0.01 qs:%d, %d/%d", best_dt, queue.size(), qs_before, i);
    return false;
  }
  else
  {
    return true;
  };
}

void System::keyboardCallback(const std_msgs::StringConstPtr & kb_input){
  mpTracker->command(kb_input->data);
}

bool System::transformQuaternion(const std::string & target_frame, const std_msgs::Header & header,
                                 const geometry_msgs::Quaternion & _q_in, TooN::SO3<double> & r_out)
{
  geometry_msgs::QuaternionStamped q_in, q_out;
  q_in.header = header;
  q_in.quaternion = _q_in;
  try
  {
    tf_sub_.transformQuaternion(target_frame, q_in, q_out);
    quaternionToRotationMatrix(q_out.quaternion, r_out);
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform quaternion: "<<e.what());
    return false;
  }
  return true;
}

bool System::transformPoint(const std::string & target_frame, const std_msgs::Header & header,
                            const geometry_msgs::Point & _p_in, TooN::Vector<3> & _p_out)
{
  geometry_msgs::PointStamped p_in, p_out;
  p_in.header = header;
  p_in.point = _p_in;
  try
  {
    tf_sub_.transformPoint(target_frame, p_in, p_out);
    _p_out[0] = p_out.point.x;
    _p_out[1] = p_out.point.y;
    _p_out[2] = p_out.point.z;
    return true;
  }
  catch (tf::TransformException & e)
  {
    ROS_WARN_STREAM("could not transform point: "<<e.what());
    return false;
  }
  return true;
}

vector<float> Q2RPY(int w, int x, int y, int z)
{
  vector<float> res;
  res.push_back(atan2(
        2*(w*x + z*y),
        1-2*(x*x + y*y)));
  res.push_back(asin(2*(w*y-z*x)));
  res.push_back(atan2(
        2*(w*z + x*y),
        1-2*(z*z + y*y)));
  return res;

}

void System::publishPoseAndInfo(const std_msgs::Header & header)
{
  
  double scale = PtamParameters::varparams().Scale;

  static float fps = 0;
  static double last_time = 0;

  std::string frame_id(header.frame_id);

  if (frame_id == "")
  {
    ROS_WARN_ONCE("camera frame id not set, will set it to \"ptam\"");
    frame_id = "ptam";
  }

  if (scale <= 0)
  {
    ROS_WARN_STREAM("scale ("<<scale<<") <= 0, set to 1");
    scale = 1;
  }
  

  if (mpTracker->getTrackingQuality() && mpMap->IsGood())
  {
    TooN::SE3<double> pose = mpTracker->GetCurrentPose();
    //world in the camera frame
    TooN::Matrix<3, 3, double> r_ptam = pose.get_rotation().get_matrix();
    TooN::Vector<3, double> t_ptam =  pose.get_translation();
    tf::Matrix3x3 m(r_ptam(0, 0), r_ptam(0, 1), r_ptam(0, 2), r_ptam(1, 0), r_ptam(1, 1), r_ptam(1, 2), r_ptam(2, 0), r_ptam(2, 1), r_ptam(2, 2));
     
    tf::StampedTransform transform_ptam(tf::Transform(tf::Matrix3x3(r_ptam(0, 0), r_ptam(0, 1), r_ptam(0, 2), r_ptam(1, 0), r_ptam(1, 1), r_ptam(1, 2), r_ptam(2, 0), r_ptam(2, 1), r_ptam(2, 2))
    , tf::Vector3(t_ptam[0] / scale, t_ptam[1] / scale, t_ptam[2] / scale)), header.stamp, frame_id, PtamParameters::fixparams().parent_frame);

    //camera in the world frame
    TooN::Matrix<3, 3, double> r_world = pose.get_rotation().get_matrix().T();
    TooN::Vector<3, double> t_world =  - r_world * pose.get_translation();

    tf::StampedTransform transform_world(tf::Transform(tf::Matrix3x3(r_world(0, 0), r_world(0, 1), r_world(0, 2), r_world(1, 0), r_world(1, 1), r_world(1, 2), r_world(2, 0), r_world(2, 1), r_world(2, 2))
        , tf::Vector3(t_world[0] / scale, t_world[1] / scale, t_world[2] / scale)), header.stamp, PtamParameters::fixparams().parent_frame, frame_id);
    
    tf_pub_.sendTransform(transform_world);
    if (pub_pose_.getNumSubscribers() > 0 || pub_pose_world_.getNumSubscribers() > 0)
    {
      //world in the camera frame
      geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
      nav_msgs::OdometryPtr msg_odom(new nav_msgs::Odometry);
      const tf::Quaternion & q_tf_ptam = transform_ptam.getRotation();
      const tf::Vector3 & t_tf_ptam = transform_ptam.getOrigin();
      msg_pose->pose.pose.orientation.w = q_tf_ptam.w();
      msg_pose->pose.pose.orientation.x = q_tf_ptam.x();
      msg_pose->pose.pose.orientation.y = q_tf_ptam.y();
      msg_pose->pose.pose.orientation.z = q_tf_ptam.z();
      msg_pose->pose.pose.position.x = t_tf_ptam.x();
      msg_pose->pose.pose.position.y = t_tf_ptam.y();
      msg_pose->pose.pose.position.z = t_tf_ptam.z();

      TooN::Matrix<6> covar = mpTracker->GetCurrentCov();
      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
        msg_pose->pose.covariance[i] = sqrt(fabs(covar[i % 6][i / 6]));

      msg_pose->header = header;
      pub_pose_.publish(msg_pose);
              


      //camera in the world frame
      const tf::Quaternion & q_tf_world = transform_world.getRotation();
      const tf::Vector3 & t_tf_world = transform_world.getOrigin();
      msg_pose->pose.pose.orientation.w = q_tf_world.w();
      msg_pose->pose.pose.orientation.x = q_tf_world.x();
      msg_pose->pose.pose.orientation.y = q_tf_world.y();
      msg_pose->pose.pose.orientation.z = q_tf_world.z();
      msg_pose->pose.pose.position.x = t_tf_world.x();
      msg_pose->pose.pose.position.y = t_tf_world.y();
      msg_pose->pose.pose.position.z = t_tf_world.z();

      TooN::Matrix<6> CovRot;
      CovRot.slice(0,0,3,3) = r_ptam;
      CovRot.slice(3,3,3,3) = CovRot.slice(0,0,3,3);

      TooN::Matrix<6> covar_world = CovRot.T() * covar * CovRot;

      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
        msg_pose->pose.covariance[i] = sqrt(fabs(covar_world[i % 6][i / 6]));

      pub_pose_world_.publish(msg_pose);

      if(prev_nav_msg)
      {
        msg_odom->header = header;
        msg_odom->pose = msg_pose->pose;
        msg_odom->child_frame_id = "camera_depth_optical_frame";
        prev_nav_msg->header.stamp.sec;
        float time_diff = msg_pose->header.stamp.sec - prev_nav_msg->header.stamp.sec + (msg_pose->header.stamp.nsec - prev_nav_msg->header.stamp.nsec)*1e-9;
        msg_odom->twist.twist.linear.x = (msg_pose->pose.pose.position.x - prev_nav_msg->pose.pose.position.x)/time_diff;
        msg_odom->twist.twist.linear.y = (msg_pose->pose.pose.position.y - prev_nav_msg->pose.pose.position.y)/time_diff;
        msg_odom->twist.twist.linear.z = (msg_pose->pose.pose.position.z - prev_nav_msg->pose.pose.position.z)/time_diff;

        vector<float> rpy_new = Q2RPY(msg_pose->pose.pose.orientation.w, msg_pose->pose.pose.orientation.x, msg_pose->pose.pose.orientation.y, msg_pose->pose.pose.orientation.z);
        vector<float> rpy_old = Q2RPY(prev_nav_msg->pose.pose.orientation.w, prev_nav_msg->pose.pose.orientation.x, prev_nav_msg->pose.pose.orientation.y, prev_nav_msg->pose.pose.orientation.z);


        msg_odom->twist.twist.angular.x = (rpy_new[0] - rpy_old[0])/time_diff;
        msg_odom->twist.twist.angular.y = (rpy_new[1] - rpy_old[1])/time_diff;
        msg_odom->twist.twist.angular.z = (rpy_new[2] - rpy_old[2])/time_diff;
        for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
          msg_odom->twist.covariance[i] = msg_pose->pose.covariance[i]/time_diff;

        pub_odom_.publish(msg_odom);
      }
      prev_nav_msg = msg_pose;

    }

    
  };
  if (pub_info_.getNumSubscribers() > 0)
    {
      ptam_com::ptam_infoPtr msg_info(new ptam_com::ptam_info);
      double diff = header.stamp.toSec() - last_time;
      if (diff < 1.0 && diff > 0.005)
        fps = fps * 0.8 + 0.2 / diff;
      if (fabs(fps) > 200)
        fps = 200;
      last_time = header.stamp.toSec();

      msg_info->header = header;
      msg_info->fps = fps;
      msg_info->wiggle = mpMapMaker->mdWiggleScale;
      msg_info->mapQuality = mpMap->bGood;
      msg_info->trackingQuality = mpTracker->getTrackingQuality();
      msg_info->trackerMessage = mpTracker->GetMessageForUser();
      msg_info->mapViewerMessage = mpMapViewer->GetMessageForUser();
      msg_info->keyframes = mpMap->vpKeyFrames.size();
      pub_info_.publish(msg_info);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr System::getVisiblePointsFromPose(TooN::SE3<double> pose)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr visiblePoints(new pcl::PointCloud<pcl::PointXYZ>);
    visiblePoints->header.frame_id = "world";
    visiblePoints->height = 1;
  #ifdef KF_REPROJ
  //possibly visible Keyframes
  vector<KeyFrame::Ptr> vpPVKeyFrames;
  //for all keyframes
  if(mpTracker->mTrackingQuality == mpTracker->GOOD){ //if tracking quality is not good, we use all KF
    for(unsigned int k=0; k<mpMap->vpKeyFrames.size(); k++)
    { //project best points
      int foundKFPoints = 0;
      if (!mpMap->vpKeyFrames.at(k)->vpPoints.size()) continue; //if this keyframe doesn't yet contain any points

      for(int j=0; j<mpMap->vpKeyFrames.at(k)->iBestPointsCount; j++)
      {
        if(mpMap->vpKeyFrames.at(k)->apCurrentBestPoints[j]==NULL) continue;
        MapPoint::Ptr p = (mpMap->vpKeyFrames.at(k)->apCurrentBestPoints[j]);
        // Ensure that this map point has an associated TrackerData struct.
        if(!p->pTData) p->pTData = new TrackerData(p);
        TrackerData &TData = *p->pTData;

        // Project according to current view, and if it's not in the image, skip.
        TData.Project(pose, *mpCamera);
        if(TData.bInImage)
          foundKFPoints++;
      };
      //have at least some points of this keyframe been found?
      if(foundKFPoints>1) vpPVKeyFrames.push_back(mpMap->vpKeyFrames.at(k));
    };
  };
  //if we didn't find any Keyframes or tracking quality is bad
  //we fall back to reprojecting all keyframes, thus all points
  if (vpPVKeyFrames.size() < 1) {
    for(unsigned int k=0; k<mpMap->vpKeyFrames.size(); k++){
      vpPVKeyFrames.push_back(mpMap->vpKeyFrames.at(k));
    }
  }
  for(unsigned int k=0; k<vpPVKeyFrames.size(); k++){//for all possibly visible keyframes
    for(unsigned int i=0; i<vpPVKeyFrames.at(k)->vpPoints.size(); i++)// For all points in the visible keyframes..
    {
      MapPoint::Ptr p= (vpPVKeyFrames.at(k)->vpPoints.at(i));
      if(p->bAlreadyProjected) continue;//check whether we already projected that point from another KF
      p->bAlreadyProjected=true;
      if(!p->pTData)
        p->pTData = new TrackerData(p);// Ensure that this map point has an associated TrackerData struct.
      TrackerData &TData = *p->pTData;
      TData.Project(pose, *mpCamera);// Project according to current view, and if it's not in the image, skip.
      if(!TData.bInImage)
        continue;

      TData.GetDerivsUnsafe(*mpCamera);// Calculate camera projection derivatives of this point.

      // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..

      TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, pose, TData.m2CamDerivs);

      if(TData.nSearchLevel == -1)
        continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

      // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
      TData.bSearched = false;
      TData.bFound = false;
      visiblePoints->points.push_back(pcl::PointXYZ(TData.Point->v3WorldPos[0], TData.Point->v3WorldPos[1], TData.Point->v3WorldPos[2]));
    };
  };

  //reset alreadyprojected marker
  for(unsigned int k=0; k<vpPVKeyFrames.size(); k++)
    for(unsigned int i=0; i<vpPVKeyFrames.at(k)->vpPoints.size(); i++)
      vpPVKeyFrames.at(k)->vpPoints.at(i)->bAlreadyProjected = false;
#else

  // For all points in the map..
  for(unsigned int i=0; i<mpMap->vpPoints.size(); i++)
  {
    MapPoint &p= *(mpMap->vpPoints[i]);
    // Ensure that this map point has an associated TrackerData struct.
    if(!p.pTData) p.pTData = new TrackerData(&p);
    TrackerData &TData = *p.pTData;

    // Project according to current view, and if it's not in the image, skip.
    TData.Project(pose, *mpCamera);
    if(!TData.bInImage)
      continue;

    // Calculate camera projection derivatives of this point.
    TData.GetDerivsUnsafe(*mpCamera);

    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, pose, TData.m2CamDerivs);
    if(TData.nSearchLevel == -1)
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    TData.bSearched = false;
    TData.bFound = false;
    visiblePoints->points.push_back(pcl::PointXYZ(TData.Point->v3WorldPos[0], TData.Point->v3WorldPos[1], TData.Point->v3WorldPos[2]));
  };
  //slynen{ reprojection
#endif
    visiblePoints->width = visiblePoints->points.size();
    return visiblePoints;
}

void System::publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header)
{
  CVD::Image<TooN::Vector<2> > & grid = mpTracker->ComputeGrid();
  std::list<Trail> & trails = mpTracker->getTrails();
  bool drawGrid = mpTracker->getTrailTrackingComplete();
  bool drawTrails = mpTracker->getTrailTrackingStarted();

  if (pub_preview_image_.getNumSubscribers() > 0)
  {
    CVD::ImageRef sub_size(img.size()/2);
    sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
    img_msg->header = header;
    img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    img_msg->width = sub_size.x;
    img_msg->height = sub_size.y;
    img_msg->step = sub_size.x;
    img_msg->is_bigendian = 0;
    img_msg->data.resize(sub_size.x * sub_size.y);

    // subsample image
    CVD::BasicImage<CVD::byte> img_sub((CVD::byte *)&img_msg->data[0], sub_size);
    CVD::halfSample(img, img_sub);

    // set opencv pointer to image
    IplImage * ocv_img = cvCreateImageHeader(cvSize(img_sub.size().x, img_sub.size().y), IPL_DEPTH_8U, 1);
    ocv_img->imageData = (char*)&img_msg->data[0];

    int dim0 = grid.size().x;
    int dim1 = grid.size().y;

    if (drawGrid)
    {
      for (int i = 0; i < dim0; i++)
      {
        for (int j = 0; j < dim1 - 1; j++)
          cvLine( ocv_img, cvPoint(grid[i][j][0]/2, grid[i][j][1]/2), cvPoint(grid[i][j + 1][0]/2, grid[i][j + 1][1]/2),
                  CV_RGB(50, 50, 50)
          );

        for (int j = 0; j < dim1 - 1; j++)
          cvLine(ocv_img, cvPoint(grid[j][i][0]/2, grid[j][i][1]/2), cvPoint(grid[j + 1][i][0]/2, grid[j + 1][i][1]/2),
                 CV_RGB(50, 50, 50)
          );
      }
    }

    if (drawTrails)
    {

      
      int level = PtamParameters::fixparams().InitLevel;

      for (std::list<Trail>::iterator i = trails.begin(); i != trails.end(); i++)
      {
        cvLine(ocv_img, cvPoint(LevelZeroPos(i->irCurrentPos.x, level)/2, LevelZeroPos(i->irCurrentPos.y, level)/2),
               cvPoint(LevelZeroPos(i->irInitialPos.x, level)/2, LevelZeroPos(i->irInitialPos.y, level)/2),
               CV_RGB(0, 0, 0), 2);
      }
    }

    pub_preview_image_.publish(img_msg);
    cvReleaseImageHeader(&ocv_img);
  }
    
    pub_visibleCloud_.publish(getVisiblePointsFromPose(mpTracker->GetCurrentPose()));

}

bool System::posepointcloudservice(ptam_com::PosePointCloudRequest & req, ptam_com::PosePointCloudResponse & resp)
{
  TooN::Vector<3, double> T;
  T[0] = req.pose.pose.position.x;
  T[1] = req.pose.pose.position.y;
  T[2] = req.pose.pose.position.z;

  tf::Quaternion currentQuat;
  tf::quaternionMsgToTF(req.pose.pose.orientation, currentQuat);
  tf::Matrix3x3 m(currentQuat);

  TooN::Matrix<3,3, double> R(TooN::Data(m[0][0],  m[0][1],  m[0][2], 
                                          m[1][0],  m[1][1],  m[1][2], 
                                          m[2][0],  m[2][1],  m[2][2] ));  

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = getVisiblePointsFromPose(TooN::SE3<double>(TooN::SO3<double>(R),-R*T));
  pcl::toROSMsg(*pointCloud, resp.pointCloud);

  return true;
}

//Weiss{
bool System::pointcloudservice(ptam_com::PointCloudRequest & req, ptam_com::PointCloudResponse & resp)
{
  static unsigned int seq=0;
  int dimension   = 6;

  resp.pointcloud.header.seq=seq;
  seq++;
  resp.pointcloud.header.stamp = ros::Time::now();
  resp.pointcloud.height = 1;
  resp.pointcloud.header.frame_id = "/world";
  if(mpMap!=NULL)
  if(mpMap->bGood)
  {
    resp.pointcloud.width = mpMap->vpPoints.size();
    resp.pointcloud.fields.resize(dimension);
    resp.pointcloud.fields[0].name = "x";
    resp.pointcloud.fields[0].offset = 0*sizeof(uint32_t);
    resp.pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    resp.pointcloud.fields[0].count = 1;
    resp.pointcloud.fields[1].name = "y";
    resp.pointcloud.fields[1].offset = 1*sizeof(uint32_t);
    resp.pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    resp.pointcloud.fields[1].count = 1;
    resp.pointcloud.fields[2].name = "z";
    resp.pointcloud.fields[2].offset = 2*sizeof(uint32_t);
    resp.pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    resp.pointcloud.fields[2].count = 1;
    resp.pointcloud.fields[3].name = "rgb";
    resp.pointcloud.fields[3].offset = 3*sizeof(uint32_t);
    resp.pointcloud.fields[3].datatype = sensor_msgs::PointField::INT32;
    resp.pointcloud.fields[3].count = 1;
    resp.pointcloud.fields[4].name = "KF";
    resp.pointcloud.fields[4].offset = 4*sizeof(uint32_t);
    resp.pointcloud.fields[4].datatype = sensor_msgs::PointField::INT32;
    resp.pointcloud.fields[4].count = 1;
    resp.pointcloud.fields[5].name = "lvl";
    resp.pointcloud.fields[5].offset = 5*sizeof(uint32_t);
    resp.pointcloud.fields[5].datatype = sensor_msgs::PointField::INT32;
    resp.pointcloud.fields[5].count = 1;
    
    resp.pointcloud.point_step = dimension*sizeof(uint32_t);
    resp.pointcloud.row_step = resp.pointcloud.point_step * resp.pointcloud.width;
    resp.pointcloud.data.resize(resp.pointcloud.row_step * resp.pointcloud.height);
    resp.pointcloud.is_dense = false;


    unsigned char* dat = &(resp.pointcloud.data[0]);
    unsigned int n=0;
    
    for(std::vector<MapPoint::Ptr>::iterator it=mpMap->vpPoints.begin(); it!=mpMap->vpPoints.end(); ++it,++n)
    {
      if(n>resp.pointcloud.width-1) break;
      MapPoint& p = *(*it);

      Vector<3,float> fvec = p.v3WorldPos;
      uint32_t colorlvl = 0xff<<((3-p.nSourceLevel)*8);
      uint32_t lvl = p.nSourceLevel;
      uint32_t KF = p.pPatchSourceKF->ID;

      memcpy(dat, &(fvec),3*sizeof(float));
      memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
      memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
      memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
      dat+=resp.pointcloud.point_step;

    }
  }

  return true;
}


bool System::keyframesservice(ptam_com::KeyFrame_srvRequest & req, ptam_com::KeyFrame_srvResponse & resp)
{
  // flags: 	negative number = send newest N KeyFrames
  //			zero = send all available KeyFrames
  //			positive number = send all KeyFrames with ID>N

  
  double scale = PtamParameters::varparams().Scale;

  TooN::SE3<double> pose;
  TooN::Matrix<3, 3, double> rot;
  TooN::Vector<3, double> trans;
  tf::Quaternion q;
  tf::Vector3 t;
  geometry_msgs::PoseWithCovarianceStamped buffpose;
  bool takeKF=false;
  int k=0;
  static unsigned int seq=0;

  if(!mpMap || mpMap->vpKeyFrames.empty() || !mpMap->bGood)
    return false;

  resp.KFids.reserve(mpMap->vpKeyFrames.size());
  resp.KFs.reserve(mpMap->vpKeyFrames.size());

  for(std::vector<KeyFrame::Ptr>::reverse_iterator rit=mpMap->vpKeyFrames.rbegin(); rit!=mpMap->vpKeyFrames.rend();++rit)
  {
    if((req.flags>0) & ((*rit)->ID>=req.flags))
      takeKF=true;
    else if((req.flags<0) & (k<=abs(req.flags)))
      takeKF=true;
    else if(req.flags==0)
      takeKF=true;
    else if((req.flags>0) & ((*rit)->ID<req.flags))
      break;
    else if((req.flags<0) & (k>abs(req.flags)))
      break;

    if(takeKF)
    {
      takeKF=false;
      resp.KFids.push_back((*rit)->ID);
      pose = (*rit)->se3CfromW;
      rot =pose.get_rotation().get_matrix();
      trans = pose.get_translation();
      tf::Transform transform(tf::Matrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
                                          rot(1, 0), rot(1, 1), rot(1, 2),
                                          rot(2, 0), rot(2, 1), rot(2, 2)),
                              tf::Vector3(trans[0] / scale, trans[1]/ scale, trans[2] / scale));
      q = transform.getRotation();
      t = transform.getOrigin();
      buffpose.header.seq=seq;
      buffpose.header.stamp=ros::Time::now();
      buffpose.pose.pose.position.x=t[0];
      buffpose.pose.pose.position.y=t[1];
      buffpose.pose.pose.position.z=t[2];
      buffpose.pose.pose.orientation.w=q.w();
      buffpose.pose.pose.orientation.x=q.x();
      buffpose.pose.pose.orientation.y=q.y();
      buffpose.pose.pose.orientation.z=q.z();
      memset(&(buffpose.pose.covariance[0]),0,sizeof(double)*6*6);
      resp.KFs.push_back(buffpose);
      seq++;
    }
    k++;
  }
  return true;
}


//}

void System::quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R)
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q.x;
  const double ty = 2 * q.y;
  const double tz = 2 * q.z;
  const double twx = tx * q.w;
  const double twy = ty * q.w;
  const double twz = tz * q.w;
  const double txx = tx * q.x;
  const double txy = ty * q.x;
  const double txz = tz * q.x;
  const double tyy = ty * q.y;
  const double tyz = tz * q.y;
  const double tzz = tz * q.z;

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  R = res;

  //  R = TooN::SO3<double>::exp(TooN::makeVector<double>(q.x, q.y, q.z) * acos(q.w) * 2.0 / sqrt(q.x * q.x + q.y * q.y + q.z * q.z));
}



void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if (sCommand == "quit" || sCommand == "exit")
    ros::shutdown();
}









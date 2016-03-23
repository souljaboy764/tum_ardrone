<<<<<<< HEAD
all:
	cd ptam_com && make -j8
	cd ..
	cd ptam && make -j8
	cd ..

clean: 
	cd ptam && make clean 
	cd ..
	cd ptam_com && make clean
	cd ..
	
	
distclean: 
	cd ptam && make distclean 
	cd ..
	cd ptam_com && make distclean
	cd ..
=======
all:
	cd thirdparty && make
	make -f Makefile_package

.DEFAULT:
	cd thirdparty && make $@
	make -f Makefile_package $@


>>>>>>> b173b58858fa9c0b3afe49511d3614e1a3108e42

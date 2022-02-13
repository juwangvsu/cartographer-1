c++ test.cc -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.8 -lboost_system -lpcl_io -lOpenNI -lOpenNI2 -lpcl_octree -lpcl_common
#pkg-config --libs pcl_io-1.8 
#	       this show the library path for linking
#pkg-config --list-all|grep pcl


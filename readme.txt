-------------sticky-------------------------
"readme pointcloud ndt icp slam cartographer code "

------12/20/21 hacking ceres_scan_matcher_3d_test.cc ---------
 test ceres scan match using turtlebot data:
   reproduce result in ceres_scan_matcher_3d_test.cc
 
------12/11/21 hacking ceres_scan_matcher_3d_test.cc ---------
to test pcd files 
	- add pcl library to the CMakeLists.txt
	- it seems work. pcl code works fine inside the test code
to rebuild:
	cd build; cmake .. -G Ninja; ninja
to run test:
	cd build
	./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test

------11/27/21 homepc build  cartographer_ros from src----------------------
/media/student/data6/catkin_ws
using ninja tool
        catkin_make_isolated --install --use-ninja
        this will download both cartographer and cartographer_ros
        and build cartographer_ros

# To Build  Cartographer seperately (to access test)
cd /media/student/data6/catkin_ws/src/cartographer
mkdir build
cd build
cmake .. -G Ninja
ninja
        all the tests will be compiled under build/
        ./cartographer.transform.transform_test

CTEST_OUTPUT_ON_FAILURE=1 ninja test
        this run all tests

#sudo ninja install


-------------sticky-------------------------
"readme pointcloud ndt icp slam cartographer code "
~/Documents/cartographer$ vi readme_carto_homepc.txt

--- pkg-config find library path link -------
to find a package include and link path, use pkg-config tool:
#pkg-config --libs pcl_io-1.8
	this show the library path for linking
#pkg-config --list-all|grep pcl
# not all library can be find this way. some such as boost not find this way

------12/20/21 hacking ceres_scan_matcher_3d_test.cc ---------
 test ceres scan match using turtlebot data:
   reproduce result in ceres_scan_matcher_3d_test.cc
 
------12/11/21, 1/25/2022 hacking ceres_scan_matcher_3d_test.cc ---------
to test pcd files 
	- add pcl library to the CMakeLists.txt
	- it seems work. pcl code works fine inside the test code
to rebuild:
	cd build; cmake .. -G Ninja; ninja
to run test:
	cd build
	./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test

test options:
	testopt2.txt
	testopt.lua
test mode:
	(1) ptselection = 0
	(2) ptselection = 1
		use a single pcd file for demo test
	(3) ptselection = 2
		use 2 or 4 pcd files. filename and other options
			 from testopt2.txt, 
		usehighres=1|0 uselowres=1|0
			if both 0, do sweep test

dbgfilename: dbg_pc3.txt
ptselection: 1
#pcdfilename: /home/student/Documents/cartographer/test/mavros_realsense_pcd/mapdata_5.pcd
pcdfilename1: /home/student/Documents/cartographer/test_ceres_pcd/submap_test_h.pcd
pcdfilename2: /home/student/Documents/cartographer/test_ceres_pcd/scan_test_h.pcd

 * ptselection = 0|1|2
 * 0 for pcd file
 * 1 for dbg file
 * 2 for two pcd files: one for submap and one for curr scan
 * point_cloud_3_ from debug file
 * point_cloud_2_ from pcdfilename
 *




------11/27/21 homepc build  cartographer_ros from src----------------------
/media/student/data6/catkin_ws
using ninja tool
        catkin_make_isolated --install --use-ninja
        this will download both cartographer and cartographer_ros
        and build cartographer_ros
	#6/2/22 "catkin build" fail to build cartographer_ros

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

add executable:
	CMakeLists.txt: google_binary(cartographer_wangtest...
	
#sudo ninja install

to use carto build here (carto 2.0.0):
        export ROS_PACKAGE_PATH=
        source /media/student/data6/catkin_ws/devel_isolated/setup.bash

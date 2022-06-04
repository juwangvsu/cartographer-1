/*
 *
 * Ju Wang, 
 *
 * 2/9/22 adopted from juwangvsu/hdl_graph_slam.
 * 3/19/21
 * load two pcd cloud and estimate the transform matrix
 * -if input cloud fn
 *  -tf target cloud fn
 *  -ms select cloud file name from global array, -1 use -if -tf override
 *  -xg, -yg, -yawg initial guess for xyz yaw
 *  -md methods: icp, gicp, ndt, gicp0, default gicp
 *  -nl noiselevel : 0-1, default 0
 * ex: ~/Documents/AirSim/ros/src/hdl_graph_slam/ndt$ ./build/icp_example -pst=1  -rsl=0.02 -md=gicp -ms=6
 * ex: ~/Documents/AirSim/ros/src/hdl_graph_slam/ndt$ ./build/icp_example -pst=1 -if=../mapdata_14.pcd -tf=../mapdata_15.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1
 */
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "pclomp/voxel_grid_covariance_omp.h"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"
#include "pclomp/ndt_omp.h"
#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/gicp_omp.h"
#include "pclomp/gicp_omp_impl.hpp"

#include <pcl/visualization/pcl_visualizer.h>

#include <yaml-cpp/yaml.h>
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/mapping/internal/3d/icp_example.h"
//#include "clpp/parser.hpp"
//#include "myparam.h"
namespace cartographer {
namespace mapping {

  void wait_pcl();
//Eigen::Matrix4f load2yaml();
  void init_pcl_viewer(std::string title);
 void show_pcl_2cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string title);
  void init_pcl_viewer(std::string title);

pcl::visualization::PCLVisualizer::Ptr viewer_final;

//extern Myparam myparam;
using namespace std::chrono_literals;
Eigen::Matrix4f loadyaml(std::string filename);
std::string pcd_fns[]={"mapdata_13.pcd","mapdata_14.pcd", 
"mapdata_15.pcd",  "mapdata_16.pcd", "mapdata_17.pcd", "mapdata_18.pcd",
"mapdata_24.pcd", "mapdata_25.pcd", "mapdata_26.pcd", "mapdata_27.pcd"
};
std::string pose_fns[]={"mappose_13.yaml","mappose_14.yaml",
	"mappose_15.yaml", "mappose_16.yaml",
	"mappose_17.yaml", "mappose_18.yaml",
	"mappose_24.yaml", "mappose_25.yaml",
	"mappose_26.yaml", "mappose_27.yaml"};

Eigen::Matrix4f load2yaml(){
	//default matchseq=0
	const std::string filename_pose1 = "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/";
	const std::string filename_pose2 = "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/";
    	Eigen::Matrix4f m1= loadyaml(filename_pose1);
    	Eigen::Matrix4f m2= loadyaml(filename_pose2);
	Eigen::Matrix4f m1_2 = m1.inverse()*m2;
	Eigen::Matrix4f m2_1 = m2.inverse()*m1;
	double da = std::acos(Eigen::Quaternionf(m1_2.block<3, 3>(0, 0)).w());
	double da2 = std::acos(Eigen::Quaternionf(m2_1.block<3, 3>(0, 0)).w());
	std::cout<<"m12\n"<<m1_2 <<"deg\n " << da*360/3.14<<std::endl;
//	std::cout<<"m21\n"<<m2_1<<"deg\n " << da2*360/3.14<<std::endl;
	return m1_2;

}
 Eigen::Matrix4f loadyaml(std::string filename)
{
	//default matchseq=14
  	Eigen::Matrix4f tr0;
	float noiselevel=0.1;
	YAML::Node posenode1 = YAML::LoadFile(filename);
	std::cout<<"yaml file: "<< filename<<" "<<posenode1["pose"]["position"]["x"].as<double>()<<std::endl;
	double x,y,z, qx,qy,qz,qw;
	double rn = rand() / ((double)RAND_MAX + 1); //add random noise
	rn = rn*noiselevel;
	x=posenode1["pose"]["position"]["x"].as<double>() + rn;
	y=posenode1["pose"]["position"]["y"].as<double>();
	z=posenode1["pose"]["position"]["z"].as<double>();
	qx=posenode1["pose"]["orientation"]["x"].as<double>();
	qy=posenode1["pose"]["orientation"]["y"].as<double>();
	qz=posenode1["pose"]["orientation"]["z"].as<double>();
	qw=posenode1["pose"]["orientation"]["w"].as<double>()+rn/4;
  	Eigen::Quaternionf quat(qw, qx,qy,qz);
	quat.normalize();
	std::cout << "quat= " <<  quat.x() <<" " << quat.y()<<" " << quat.z()<< " " << quat.w() << " noise: "<<rn<<std::endl;
  	double angle = 2*std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2))
  	Eigen::Translation3f translation (x,y,z);
  	Eigen::Matrix3f R = quat.normalized().toRotationMatrix();
	//std::cout << "R=" << std::endl << R << std::endl;
	std::cout << "rotation angle=" << angle *180/3.14 <<" deg"<< std::endl;

  	Eigen::Matrix4f TR = (translation * quat).matrix ();
  	std::cout << "TR=" << std::endl << TR << std::endl;
	return TR;
}
 void show_pcl_2cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string title)
  {
          init_pcl_viewer(title); //need to init pcl every time it was closed.

          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                color1 (cloud1, 255, 0, 0);

          viewer_final->addPointCloud<pcl::PointXYZ> (cloud1, color1, "target cloud");
          viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                color2 (cloud2, 0, 255, 0);
          viewer_final->addPointCloud<pcl::PointXYZ> (cloud2, color2, "output cloud");
          viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");
          wait_pcl();
  }
  void wait_pcl()
  {
  // Wait until visualizer window is closed.
        std::chrono::milliseconds dura( 100 );
        while (!viewer_final->wasStopped ())
        {
                viewer_final->spinOnce (100);
                std::this_thread::sleep_for(dura);
        }
  }
  void init_pcl_viewer(std::string title)
  {
          // Initializing point cloud visualizer
 // pcl::visualization::PCLVisualizer::Ptr
  viewer_final.reset( new pcl::visualization::PCLVisualizer (title));
  viewer_final->setBackgroundColor (0, 0, 0);
  // Coloring and visualizing target cloud (red).
//  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();
  viewer_final->setCameraPosition(0, 0, -4, 10, 5, 10, 10, 0, 10);

  }

int
icp_main (int argc, char** argv)
{
	srand(4577231);
//	proc_param(argc, argv);
	int matchseq=0;

	Eigen::Matrix4f guess0;
	//if matchseq >=0 then both pcd and yaml files from the global array
	//else pcd filename provided by if and tf, initial guess from xg, yawg 
	if (matchseq>=0)
		guess0=load2yaml();
  
	// Loading first scan, as reference or target cloud
  	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	std::string pcdfn1;
  	if (matchseq<0) ;//pcdfn1=myparam.inputfilename;
	else 
		pcdfn1= "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/"+pcd_fns[matchseq];
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud) == -1)
  	{
    		PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    		return (-1);
  	}
  	std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  	// Loading second scan of room from new perspective.
  	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	std::string pcdfn2;
  	if (matchseq<0);// pcdfn2=targetfilename;
	else 
		pcdfn2= "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/"+pcd_fns[matchseq+1];
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn2, *input_cloud) == -1)
  	{
    		PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    		return (-1);
  	}
  	std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_t (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;

  float resolution=0.5f;
  approximate_voxel_filter.setLeafSize (resolution, resolution, resolution);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
  approximate_voxel_filter.setInputCloud (target_cloud);
  approximate_voxel_filter.filter (*filtered_cloud_t);

  // select icp or ndt or gicp
 boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> icp;
 boost::shared_ptr< pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);
 boost::shared_ptr< pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> icp0 (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
 boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> icp2(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
 boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp0(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);

 std::string method="icp";
  if (method=="icp")
    icp = icp0;
  else if(method=="gicp")
    icp = icp2;
  else if(method=="gicp0")
    icp = gicp0;
  else if(method=="ndt")
    icp = ndt;

  //setting=1 use param setting from hdl, else use the one from this example
  std::cout<<"using stock icp verison\n";
  icp->setTransformationEpsilon(0.01);
      icp->setMaximumIterations(64);
      //icp.setResolution(0.5);
//ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
 //std::cout<<"# threads: "<<ndt.getNumThreads()<<" resolution "<< ndt.getResolution() <<" stepsize "<< ndt.getStepSize()<<std::endl;


  // Setting point cloud to be aligned.
  icp->setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  icp->setInputTarget (filtered_cloud_t);
  //icp.setInputTarget (target_cloud);

  	// Set initial alignment estimate found using robot odometry.
	//if matchseq >=0 then both pcd and yaml files from the global array
	//else pcd filename provided by if and tf, initial guess from xg, yawg 
	Eigen::Matrix4f init_guess;
	float yawguess=0, xguess=0;

	if (matchseq>=0)
		init_guess=load2yaml();
	else{
  		Eigen::AngleAxisf init_rotation (yawguess, Eigen::Vector3f::UnitZ ());
  		Eigen::Translation3f init_translation (xguess, 0.0, 0);
   		init_guess = (init_translation * init_rotation).matrix ();
	}
  	std::cout<<"init matrix:\n"<<init_guess;

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f guess = Eigen::Isometry3d::Identity().matrix().cast<float>();

  auto start = std::chrono::system_clock::now();

  //ndt.align (*output_cloud, guess);
  icp->align (*output_cloud, init_guess);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout<< "\nelapsed time: " << elapsed_seconds.count() << "s";
  std::cout << "\nicp has converged:" << icp->hasConverged ()
            << " score: " << icp->getFitnessScore () << std::endl;
  std::cout<<"trans matrix:\n"<<icp->getFinalTransformation();

  //Eigen::Matrix4f trans = init_guess; 
  Eigen::Matrix4f trans = icp->getFinalTransformation();
  double xyz_d = trans.block<3, 1>(0, 3).norm();
  Eigen::Quaternionf quat(trans.block<3, 3>(0, 0));
  double angle = std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2)), so angle is theta/2

  std::cout<<"\ntrans xyz:\n"<<xyz_d << " quat " << quat.x() <<" "<< quat.y() <<" "<< quat.z() <<" "<< quat.w() <<" " <<" angle in rad/deg): "<< 2*angle << "," << angle * 360 / 3.14 << "deg"<<std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, icp->getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}

/********************
 * icp match main func, called from local_trajectory_builder_3d.cc
 * init pose: initial_ceres_pose,
 * output: pose_observation_in_submap,
 * 		pose w.r.t the reference frame
 * 		point cloud after transform with the estimated pose
 */
void icp_main2 (transform::Rigid3d * pose_observation_in_submap, transform::Rigid3d initial_ceres_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr scan,pcl::PointCloud<pcl::PointXYZ>::Ptr map, std::string icp_method, int pcl_viewerflag)
{
 boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> icp;
 boost::shared_ptr< pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> icp0 (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
 boost::shared_ptr< pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);

// std::string icp_method="icp";
  if (icp_method=="icp")
  {
  std::cout<<"using stock icp verison\n";
    icp = icp0;
  }
  else if(icp_method=="ndt")
  {
  std::cout<<"using ndt verison\n";
    icp = ndt;
  }
  //setting=1 use param setting from hdl, else use the one from this example
  icp->setTransformationEpsilon(0.01);
      icp->setMaximumIterations(64);
      //icp.setResolution(0.5);
//ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);

  // Setting point cloud to be aligned.
  icp->setInputSource (scan);
  // Setting point cloud to be aligned to.
  icp->setInputTarget (map);
  //icp.setInputTarget (target_cloud);
  
//  	std::cout<<"initial_ceres_pose.translation: "<<initial_ceres_pose.translation()<<"\n";
	Eigen::Matrix4f init_guess;
	float yawguess=0, xguess=0;
	int icpinitsrc=1;
	if (icpinitsrc=0)
		init_guess=load2yaml();
	else{
  		Eigen::AngleAxisf init_rotation (initial_ceres_pose.rotation().cast<float>());
  		Eigen::Translation3f init_translation (initial_ceres_pose.translation()[0],initial_ceres_pose.translation()[1], initial_ceres_pose.translation()[2]);
   		init_guess = (init_translation * init_rotation).matrix ();
	}
  	std::cout<<"init matrix:\n"<<init_guess;

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  auto start = std::chrono::system_clock::now();

  //ndt.align (*output_cloud, init_guess);
  icp->align (*output_cloud, init_guess);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout<< "\nelapsed time: " << elapsed_seconds.count() << "s";
  std::cout << "\nicp has converged:" << icp->hasConverged ()
            << " score: " << icp->getFitnessScore () << std::endl;
  std::cout<<"trans matrix:\n"<<icp->getFinalTransformation();

  Eigen::Matrix4f trans = icp->getFinalTransformation();
  Eigen::Matrix<float, 3, 1> translation0 = trans.block<3, 1>(0, 3);
  double xyz_d = trans.block<3, 1>(0, 3).norm();
  Eigen::Quaternionf quat(trans.block<3, 3>(0, 0));
  double angle = std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2)), so angle is theta/2

  std::cout<<"\ntrans xyz:\n"<<xyz_d << " quat " << quat.x() <<" "<< quat.y() <<" "<< quat.z() <<" "<< quat.w() <<" " <<" angle in rad/deg): "<< 2*angle << "," << angle * 360 / 3.14 << "deg"<<std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*scan, *output_cloud, icp->getFinalTransformation ());

  Eigen::Matrix<double, 3, 1> translation_est(translation0.cast<double>());
  Eigen::Quaterniond quat_est(quat);
  //translation = trans.block<3, 1>(0, 3);
  *pose_observation_in_submap = transform::Rigid3d(translation_est, quat_est);
//  std::cout<<"icp_main2 pose_observation_in_submap->trans "<<pose_observation_in_submap->translation() << " quat " << pose_observation_in_submap<<std::endl;
  std::cout<<"icp_main2 pose_observation_in_submap->trans "<<pose_observation_in_submap->translation()[0] << " quat " << pose_observation_in_submap->rotation().x()<<std::endl;
  if (pcl_viewerflag==1) show_pcl_2cloud(scan, map, "icp_main scan  vs map no xform 1");
  //show_pcl_2cloud(scan, map, "icp_main scan  vs map no xform 2");
}

}//namespace
}//namespace

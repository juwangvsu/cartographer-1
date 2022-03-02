/*
 * Ju Wang, 3/1/22
 * additional function from hdl_graph_ icp_example.cc 
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
//#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/gicp_omp.h"
//#include "pclomp/gicp_omp_impl.hpp"

#include <pcl/visualization/pcl_visualizer.h>

#include <yaml-cpp/yaml.h>
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/mapping/internal/3d/icp_example.h"
namespace cartographer {
namespace mapping {
/**** load gtposition info from testcfg.yaml, for -ms=-1 
 * input test.yaml: 
 *  return init tranform matrix, two pcd file names
 */
 Eigen::Matrix4f loadyaml_gt(std::string filename, std::string & pcdfn1, std::string & pcdfn2)
{
        double x,y,z, qx,qy,qz,qw;
        YAML::Node testcfg = YAML::LoadFile(filename);
        x=testcfg["pose"]["gtposition"]["x"].as<double>();
        y=testcfg["pose"]["gtposition"]["y"].as<double>();
        z=testcfg["pose"]["gtposition"]["z"].as<double>();
        qx=testcfg["pose"]["gtorientation"]["x"].as<double>();
        qy=testcfg["pose"]["gtorientation"]["y"].as<double>();
        qz=testcfg["pose"]["gtorientation"]["z"].as<double>();
        qw=testcfg["pose"]["gtorientation"]["w"].as<double>();
        Eigen::Quaternionf quat(qw, qx,qy,qz);
        quat.normalize();
        std::cout << "gt quat= " <<  quat.x() <<" " << quat.y()<<" " << quat.z()<< " " << quat.w() <<std::endl;
        double angle = 2*std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2))
        Eigen::Translation3f translation (x,y,z);
        Eigen::Matrix3f R = quat.normalized().toRotationMatrix();
        //std::cout << "R=" << std::endl << R << std::endl;
        std::cout << "rotation angle=" << angle *180/3.14 <<" deg"<< std::endl;
        Eigen::Matrix4f TR = (translation * quat).matrix ();
        std::cout << "TR=" << std::endl << TR << std::endl;
        return TR;
}
/**** load most info from testcfg.yaml,  
 * input test.yaml: 
 *  return init tranform matrix, two pcd file names, method, resolution
 */
 Eigen::Matrix4f loadyaml3(std::string filename, std::string & pcdfn1, std::string & pcdfn2, std::string & method, double & resolution, Eigen::Quaternion<double>& quat0)
{
        double x,y,z, qx,qy,qz,qw;
        YAML::Node testcfg = YAML::LoadFile(filename);
        pcdfn1=testcfg["pose"]["pcdfn1"].as<std::string>();
        pcdfn2=testcfg["pose"]["pcdfn2"].as<std::string>();
        method=testcfg["pose"]["method"].as<std::string>();
        resolution=testcfg["pose"]["resolution"].as<double>();
        x=testcfg["pose"]["position"]["x"].as<double>();
        y=testcfg["pose"]["position"]["y"].as<double>();
        z=testcfg["pose"]["position"]["z"].as<double>();
        qx=testcfg["pose"]["orientation"]["x"].as<double>();
        qy=testcfg["pose"]["orientation"]["y"].as<double>();
        qz=testcfg["pose"]["orientation"]["z"].as<double>();
        qw=testcfg["pose"]["orientation"]["w"].as<double>();
        Eigen::Quaternionf quat(qw, qx,qy,qz);
        quat.normalize();
	quat0=quat.cast<double>();
        std::cout << "quat= " <<  quat.x() <<" " << quat.y()<<" " << quat.z()<< " " << quat.w() <<std::endl;
        double angle = 2*std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2))
        Eigen::Translation3f translation (x,y,z);
        Eigen::Matrix3f R = quat.normalized().toRotationMatrix();
        //std::cout << "R=" << std::endl << R << std::endl;
        std::cout << "rotation angle=" << angle *180/3.14 <<" deg"<< std::endl;
        Eigen::Matrix4f TR = (translation * quat).matrix ();
        std::cout << "TR=" << std::endl << TR << std::endl;
        return TR;
}
// ride of myparam stuff, moved most cmd option to either default
// or testcfg.yaml
int icp_main3(int argc, char ** argv)
{

        srand(4577231);

        Eigen::Matrix4f guess0;
        Eigen::Matrix4f gt0; //ground truth position 

        // Loading first scan, as reference or target cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::string pcdfn1;
        std::string pcdfn2;
        std::string method;
	double resolution;

	std::cout<<"icp_main3\n";
        // Set initial alignment estimate found using robot odometry.
        //if matchseq >=0 then both pcd and yaml files from the global array
        //else pcd filename provided by if and tf, initial guess from xg, yawg 

	Eigen::Quaternion<double> quat0;
        guess0 = loadyaml3("testcfg.yaml", pcdfn1, pcdfn2, method, resolution, quat0);
        gt0 = loadyaml_gt("testcfg.yaml", pcdfn1, pcdfn2);
                std::cout<<"\n\n********************\nms=-2: pcdfn1, pcdfn2: "<<pcdfn1 <<" "<<pcdfn2<<"\n";
        std::cout<<"init matrix:\n"<<guess0;
       if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud) == -1)
        {
                PCL_ERROR ("Couldn't read file target_cloud (reference) \n");
                return (-1);
        }
        std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

        // Loading second scan of room from new perspective.
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn2, *input_cloud) == -1)
        {
                PCL_ERROR ("Couldn't read file input_cloud (to be transformed) \n");
                return (-1);
        }
        std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_t (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;

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

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_gt (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f guess = Eigen::Isometry3d::Identity().matrix().cast<float>();

  auto start = std::chrono::system_clock::now();

  //ndt.align (*output_cloud, guess);
  icp->align (*output_cloud, guess0);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout<< "\nelapsed time: " << elapsed_seconds.count() << "s";
  std::cout << "\nicp has converged:" << icp->hasConverged ()
            << " score: " << icp->getFitnessScore () << std::endl;
  std::cout<<"trans matrix:\n"<<icp->getFinalTransformation();

  Eigen::Matrix4f trans = icp->getFinalTransformation();
  double xyz_d = trans.block<3, 1>(0, 3).norm();
  Eigen::Quaternionf quat(trans.block<3, 3>(0, 0));
  double angle = std::acos(quat.w()); //w of quaternion is cos(theta/2), assuming the quaternion is normalized (x.sin() y.sin() z.sin() cos(theta/2)), so angle is theta/2

  std::cout<<"\ntrans xyz:\n"<<xyz_d << " quat " << quat.x() <<" "<< quat.y() <<" "<< quat.z() <<" "<< quat.w() <<" " <<" angle in rad/deg): "<< 2*angle << "," << angle * 360 / 3.14 << "deg"<<std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, icp->getFinalTransformation ());
  pcl::transformPointCloud (*input_cloud, *output_cloud_gt, gt0);

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  show_pcl_2cloud(filtered_cloud_t,filtered_cloud, "original clouds");
  show_pcl_2cloud(target_cloud,output_cloud, "after alignment");
  show_pcl_2cloud(target_cloud,output_cloud_gt, "after gt alignment");
  return 0;
}


}//end namespace mapping
}//end namespace cartographer


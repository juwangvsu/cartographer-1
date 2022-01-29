/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"

#include <memory>

#include "Eigen/Core"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include <chrono>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
//using namespace std::chrono_literals;
#include <map>
#include <boost/algorithm/string/trim.hpp>
#include <string.h>
#include <regex>
/* jwang 1/6/22
 *
 * TestFromInitialPose2()
 *
 * ptselection = 0|1|2
 * 0 for pcd file
 * 1 for dbg file
 * 2 for two pcd files: one for submap and one for curr scan
 * point_cloud_3_ from debug file
 * point_cloud_2_ from pcdfilename
 *
 *
 */
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {
class CeresScanMatcher3DTest : public ::testing::Test {
 protected:
  CeresScanMatcher3DTest()
      : hybrid_grid_(0.1f),
        intensity_hybrid_grid_(0.1f),
        expected_pose_(
            transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.))) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    for (const Eigen::Vector3f& point :
         {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
          Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
          Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
          Eigen::Vector3f(-7.f, 3.f, 1.f)}) 
    {
      points.push_back({point});
      intensities.push_back(50);
      hybrid_grid_.SetProbability(
          hybrid_grid_.GetCellIndex(expected_pose_.cast<float>() * point), 1.);
      intensity_hybrid_grid_.AddIntensity(
          intensity_hybrid_grid_.GetCellIndex(expected_pose_.cast<float>() *
                                              point),
          50);
    }
    point_cloud_ = sensor::PointCloud(points, intensities);
    std::string filename2="testopt2.txt";
    std::ifstream stream2(filename2.c_str());
    std::string myoptstr2= std::string((std::istreambuf_iterator<char>(stream2)),
                     std::istreambuf_iterator<char>());
    char *token;
    token = strtok(const_cast<char*>(myoptstr2.c_str()), "\n");
    while (token != NULL) {
	    std::string s(token);
	    size_t pos = s.find(":");
	    std::string svalue = s.substr(pos + 1, std::string::npos);
	    boost::algorithm::trim(svalue);
	    mymap[s.substr(0, pos)] = svalue;
	    token = strtok(NULL, "\n");
    }

    //for (auto keyval : mymap)
    //	    std::cout << keyval.first << ":" << keyval.second << std::endl;
    //std::cout<< mymap["pcdfilename"] <<std::endl;

    std::string filename="testopt.lua";
    std::ifstream stream(filename.c_str());
    myoptstr= std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
    //LOG(INFO)<< myoptstr;

    std::string testoptstr=R"text(
        return {
          occupied_space_weight_0 = 1.,
          intensity_cost_function_options_0 = {
            weight = 0.5,
            huber_scale = 55,
            intensity_threshold = 100,
          },
          translation_weight = 0.01,
          rotation_weight = 0.1,
          only_optimize_yaw = false,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 100,
            num_threads = 1,
          },
        })text";

    auto parameter_dictionary = common::MakeDictionary(myoptstr);
    options_ = CreateCeresScanMatcherOptions3D(parameter_dictionary.get());
    //LOG(INFO) << parameter_dictionary->GetKeys();
    //LOG(INFO) << parameter_dictionary->ToString();
    ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  }
/********************************************************************/
  void TestFromInitialPose(const transform::Rigid3d& initial_pose) {
    transform::Rigid3d pose;

    ceres::Solver::Summary summary;

    IntensityHybridGrid* intensity_hybrid_grid_ptr =
        point_cloud_.intensities().empty() ? nullptr : &intensity_hybrid_grid_;

    ceres_scan_matcher_->Match(
        initial_pose.translation(), initial_pose,
        {{&point_cloud_, &hybrid_grid_, intensity_hybrid_grid_ptr}}, &pose,
        &summary);
    LOG(INFO) << "estimated pose: "<< pose;
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    EXPECT_THAT(pose, transform::IsNearly(expected_pose_, 3e-2));
  }


/* ******************************************************
 * convert initpose string to Rigid3d
 */
  transform::Rigid3d makepose(std::string posestring)
  {
	  std::cout<<"\nmake pose: " <<"\n";
	  std::istringstream ss(posestring);
	  float x,y,z,qw,qx,qy,qz;
	  ss>> x;
	  ss>> y;
	  ss>> z;
	  ss>> qw;
	  ss>> qx;
	  ss>> qy;
	  ss>> qz;
	  std::cout << "xyz: " <<x <<" " <<y<<" " <<z<<" quat " <<qw<<" " <<qx<<" " <<qy<<" " <<qz <<"\n";
	  Eigen::Matrix<double, 3, 1> trans;
	  Eigen::Vector3d trans2;
	  trans2 = Eigen::Vector3d(x, y,z);
	  //trans2 = Eigen::Vector3d(-0.95, -0.05, 0.05);
	  std::cout<<"\nmake pose: input pose: wxyz " <<qw<<" " << qx<<" "<< qy<<" " <<qz<<"\n";
	  Eigen::Quaternion<double> rotation(qw,qx,qy,qz);
	  Eigen::Quaternion<double> rotation3=rotation.normalized();
	  Eigen::Matrix3d mat3 = rotation3.toRotationMatrix();
	  Eigen::Quaternion<double> rotation2(mat3);
	  std::cout<<"\nmake pose: eigen pose normalized: wxyz " <<rotation3.w()<<" " << rotation3.x()<<" "<< rotation3.y()<<" " <<rotation3.z()<<"\n";
	  std::cout<<"\nmake pose: eigen pose recreated: wxyz " <<rotation2.w()<<" " << rotation2.x()<<" "<< rotation2.y()<<" " <<rotation2.z()<<"\n";
	  //Eigen::Quaternion<double> rotation(1,0,0,0);
	  return transform::Rigid3d(trans2, rotation2);
  }
/******************************************************
 * convert PCD cloud to carto's hybrid_grid
 */
  void PCDCloud_2_HybridGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud, HybridGrid & hybrid_grid, IntensityHybridGrid & intensity_hybrid_grid)
  {
    int ptcount=0;

    Eigen::Vector3f point;
    // pc2 points number: 33024
    for (const pcl::PointXYZ& pointxyz : pclcloud->points)
    {
	ptcount++;
	point(0)=pointxyz.x;
	point(1)=pointxyz.y;
	point(2)=pointxyz.z;
	hybrid_grid.SetProbability(
          hybrid_grid.GetCellIndex(point), 1.);
	intensity_hybrid_grid.AddIntensity(
          intensity_hybrid_grid.GetCellIndex(point),
          50);
    }
    std::cout<< "pcd RangefinderPoint: ";
  }
/******************************************************
 * convert PCD cloud to carto's sensor::PointCloud
 */
  void PCDCloud_2_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud,sensor::PointCloud & point_cloud)
  {
    int ptcount=0;
    Eigen::Vector3f point;
    std::vector<float> intensities;
    std::vector<sensor::RangefinderPoint> points;
    // pc2 points number: 33024
    for (const pcl::PointXYZ& pointxyz : pclcloud->points)
    {
	ptcount++;
        //LOG(INFO) << "point: "<< pointxyz;
	point(0)=pointxyz.x;
	point(1)=pointxyz.y;
	point(2)=pointxyz.z;
      	points.push_back({point});
        intensities.push_back(50);
	/*
	hybrid_grid_2_.SetProbability(
          hybrid_grid_2_.GetCellIndex(expected_pose_.cast<float>() * point), 1.);
	intensity_hybrid_grid_2_.AddIntensity(
          intensity_hybrid_grid_2_.GetCellIndex(expected_pose_.cast<float>() *
                                              point),
          50);
	  */
    }
    std::cout<< "pcd RangefinderPoint: ";
    point_cloud = sensor::PointCloud(points, intensities);
  }

/***********************************************
 * read a pcd file, return cloud
 */
  pcl::PointCloud<pcl::PointXYZ>::Ptr readPCD_file(std::string pcdfn1)
  {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	pcdfn1= "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/mapdata_13.pcd";
	pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud);
	LOG(INFO) << *target_cloud;
	return target_cloud;
  }

  /*********************************************************
   * reset scanmatch to use one pair of scan/submap
   */
  void scanmatch_reset_single()
  {
    myoptstr = std::regex_replace(myoptstr, std::regex("occupied_space_weight_1"), "occupied_space_weight_0");
  //  std::cout<<myoptstr;
    auto parameter_dictionary = common::MakeDictionary(myoptstr);
    options_ = CreateCeresScanMatcherOptions3D(parameter_dictionary.get());
    ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  }

  /*********************************************************
   * sweep init pose to test costfun 
   */
void sweeptest(const transform::Rigid3d& initial_pose, sensor::PointCloud &point_cloud, HybridGrid * hybrid_grid)
{
    std::ofstream outFile("/home/student//Documents/cartographer/test_ceres_pcd/sweep4.txt");
    std::ofstream outFile2("/home/student//Documents/cartographer/test_ceres_pcd/sweep4.csv"); //gnuplot csv
    outFile2 << "# pose_x, y, z, costfun, ceres_initcost, ceres_finalcost\n";
    std::ostringstream ssout;
    transform::Rigid3d pose;
    ceres::Solver::Summary summary;
    double *residual = new double [point_cloud_5_.size()];
    double trans[3]= {initial_pose.translation().x(), initial_pose.translation().y(),initial_pose.translation().z()};
		    
    double quat[4] ={initial_pose.rotation().w(),initial_pose.rotation().x(),initial_pose.rotation().y(),initial_pose.rotation().z()};
    //OccupiedSpaceCostFunction3D::scan_matching_verbose=1;
    OccupiedSpaceCostFunction3D  costfun (1.0, point_cloud, *hybrid_grid);
    double sweepstep,sweeprange;
    std::istringstream(mymap["sweepstep"])>>sweepstep;	    
    std::istringstream(mymap["sweeprange"])>>sweeprange;	
    int sweepcnt = sweeprange/sweepstep;	
    for (int j = -sweepcnt; j<sweepcnt; j++){
	for(int k=-sweepcnt; k<sweepcnt; k++){
	    trans[0]= sweepstep*j;
	    trans[1]= sweepstep*k;
	    transform::Rigid3d pose2(Eigen::Matrix<double, 3, 1>(trans[0],trans[1],trans[2]),initial_pose.rotation());
	    costfun(trans,quat, residual);
	    double sqsum_residual=0;
	    for (int i =0; i< point_cloud.size();i++){
		    sqsum_residual +=residual[i]*residual[i];
	    }
	    summary = run_lowreso_match(pose2,point_cloud,hybrid_grid, pose);
	    ssout.str("");
	    ssout<<"\n\t init_xyz: "<<trans[0]<<" "<<trans[1]<<" "<<trans[2]<<" residual " 
			    << sqsum_residual <<" ceres init/final cost " << summary.initial_cost<<" " << summary.final_cost <<" final pose "
			    <<pose.translation().x() <<" " << pose.translation().y() <<" " <<pose.translation().z()<<"";
	    outFile << ssout.str();
	    outFile2 << trans[0]<<", "<<trans[1]<<", "<<trans[2]<<", " <<sqsum_residual<<", " << summary.initial_cost<<", " << summary.final_cost<<"\n";
	    std::cout<< ssout.str();
	}
    }
}
  /*********************************************************
   * run low resolution scanmatch 
   */
  ceres::Solver::Summary run_lowreso_match(const transform::Rigid3d& initial_pose, sensor::PointCloud &point_cloud, HybridGrid * hybrid_grid, transform::Rigid3d & pose_o)
  {
    transform::Rigid3d pose_1;
    ceres::Solver::Summary summary_1;
    std::vector<PointCloudAndHybridGridsPointers> point_clouds_and_hybrid_grids_1;
    scanmatch_reset_single();
    point_clouds_and_hybrid_grids_1={{&point_cloud, hybrid_grid, nullptr}};
    ceres_scan_matcher_->Match(initial_pose.translation(), initial_pose, point_clouds_and_hybrid_grids_1, &pose_1,&summary_1);
	    
    pose_o=pose_1;
    return summary_1;
  }
  /*********************************************************
 * jwang: new scan match test code from here
 * ptselection: 0|1|2
 * 2 for testing two pcd files scan match
 * *********************************************************/
  void TestFromInitialPose2(const transform::Rigid3d& initial_pose) {
    transform::Rigid3d pose;

     Eigen::Transform<double, 3, Eigen::Affine > pose_transform;
     Eigen::Matrix3f mat3 = Eigen::Quaternionf(1, 0,0,0).toRotationMatrix();
    ceres::Solver::Summary summary;
    std::vector<PointCloudAndHybridGridsPointers> point_clouds_and_hybrid_grids;

//--------------- select which one to run ceres on: pcd points or debug points
    int ptselection = std::stoi(mymap["ptselection"]);
    if (ptselection ==0){
	    //use single pcd file
	std::cout <<"use pcd file points\n";
	IntensityHybridGrid* intensity_hybrid_grid_ptr =
        point_cloud_2_.intensities().empty() ? nullptr : &intensity_hybrid_grid_2_;
	ceres_scan_matcher_->Match(
        initial_pose.translation(), initial_pose,
        {{&point_cloud_2_, &hybrid_grid_2_, intensity_hybrid_grid_ptr}}, &pose,
        &summary);
    }
    else if (ptselection ==1){
	    // use single ebug txt filea
	std::cout <<"use debug txt file points";
    
	IntensityHybridGrid* intensity_hybrid_grid_ptr =
        	point_cloud_3_.intensities().empty() ? nullptr : &intensity_hybrid_grid_3_;

	point_clouds_and_hybrid_grids={{&point_cloud_3_, &hybrid_grid_3_, intensity_hybrid_grid_ptr}};
	std::cout << "point_clouds_and_hybrid_grids.size(): "<<point_clouds_and_hybrid_grids.size() << "\npoint_cloud\n" << point_clouds_and_hybrid_grids[0].point_cloud <<std::endl;
	std::cout << "\nhybrid_grid:\n " << point_clouds_and_hybrid_grids[0].hybrid_grid <<std::endl;
    
	OccupiedSpaceCostFunction3D::scan_matching_verbose=1;
    
	ceres_scan_matcher_->Match(
        initial_pose.translation(), initial_pose,
        {{&point_cloud_3_, &hybrid_grid_3_, intensity_hybrid_grid_ptr}}, &pose,
        &summary);
    }else if (ptselection ==2){
            // use two pcd files: pcdfilename1 pcdfilename2
	std::cout <<"use two pcd file points\n";
	    std::string pcdf1, pcdf2, pcdf3, pcdf4;
	    transform::Rigid3d initial_pose2;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);
	    pcdf1 =  mymap["pcdfilename1"]; //scan pcd high res
	    pcdf2 =  mymap["pcdfilename2"]; //submap pcd high res
	    pcdf3 =  mymap["pcdfilename3"]; //scan pcd low res
	    pcdf4 =  mymap["pcdfilename4"]; //submap pcd low res
	    initial_pose2 = makepose(mymap["initpose"]);

	    cloud1 = readPCD_file(pcdf1);
	    PCDCloud_2_PointCloud(cloud1,point_cloud_4_);
	    //show_pcl_cloud(cloud1);
	    cloud2 = readPCD_file(pcdf2);
	    cloud4 = readPCD_file(pcdf3);
	    PCDCloud_2_PointCloud(cloud4,point_cloud_5_);
	    cloud5 = readPCD_file(pcdf4);

	    show_pcl_2cloud(cloud1,cloud2, "high res scan and submap");
	    transform_cloud (cloud1, cloud3, initial_pose2.translation(), initial_pose2.rotation());
	    show_pcl_2cloud(cloud3, cloud2, "high res scan and submap");
	    float hgrid_res_h;
	    float hgrid_res_l;
	    std::istringstream ss_h(mymap["hgrid_res_h"]);
	    std::istringstream ss_l(mymap["hgrid_res_l"]);
	    ss_h >> hgrid_res_h; //submap pcd
	    ss_l >> hgrid_res_l; //submap pcd
	    hybrid_grid_4_ = new HybridGrid(hgrid_res_h);
	    hybrid_grid_5_ = new HybridGrid(hgrid_res_l);

	    PCDCloud_2_HybridGrid(cloud2,*hybrid_grid_4_,intensity_hybrid_grid_4_);	    
	    PCDCloud_2_HybridGrid(cloud5,*hybrid_grid_5_,intensity_hybrid_grid_5_);	    
	
	    IntensityHybridGrid* intensity_hybrid_grid_ptr =
		    point_cloud_2_.intensities().empty() ? 
		    nullptr : &intensity_hybrid_grid_2_;
	    if (mymap["usehighres"] =="1" && mymap["uselowres"] =="1")
	    {
		    std::cout<<"******************  usehighres and uselowres***\n\n";
		    if (mymap["usehighres"] =="1" ){
			    std::cout<<"******************  twostep:  ***\n\n";
			    scanmatch_reset_single();
			    point_clouds_and_hybrid_grids={{&point_cloud_5_, hybrid_grid_5_, nullptr}};
                    
			    ceres_scan_matcher_->Match(initial_pose2.translation(), initial_pose2, point_clouds_and_hybrid_grids, &pose,&summary);
			    std::cout<<"******************  twostep: phase 1 low res ***\n\tpose: "<< pose<<"\n";
			    point_clouds_and_hybrid_grids={{&point_cloud_4_, hybrid_grid_4_, intensity_hybrid_grid_ptr}};
		    
			    ceres_scan_matcher_->Match(pose.translation(), pose, point_clouds_and_hybrid_grids, &pose,&summary);
			    std::cout<<"******************  twostep: phase 2 high res ***\n\tpose: "<< pose<<"\n";

		    }else{
			    std::cout<<"******************  twostep:0 high/low in once Match call ***\n\n";
			    point_clouds_and_hybrid_grids={{&point_cloud_4_, hybrid_grid_4_, intensity_hybrid_grid_ptr},{&point_cloud_5_,hybrid_grid_5_,nullptr}};
			    ceres_scan_matcher_->Match(initial_pose2.translation(), initial_pose2, point_clouds_and_hybrid_grids, &pose,&summary);
		    }
	    } 
	    if (mymap["usehighres"] =="1" && mymap["uselowres"] =="0")
	    {
		    std::cout<<"******************  usehighres ***\n\n";
		    scanmatch_reset_single();
		    point_clouds_and_hybrid_grids={{&point_cloud_4_, hybrid_grid_4_, intensity_hybrid_grid_ptr}};
		    ceres_scan_matcher_->Match(initial_pose2.translation(), initial_pose2, point_clouds_and_hybrid_grids, &pose,&summary);

	    }
	    if (mymap["usehighres"] =="0" && mymap["uselowres"] =="1")
	    {
		    std::cout<<"******************  uselowres ***\n\n";
		    summary = run_lowreso_match(initial_pose2,point_cloud_5_,hybrid_grid_5_, pose);
		    /*
	    LOG(INFO) << "\nestimated pose: "<< pose;
	    LOG(INFO) << summary.FullReport();
		    std::cout<<"******************  uselowres ***\n\n";
		    scanmatch_reset_single();
		    point_clouds_and_hybrid_grids={{&point_cloud_5_, hybrid_grid_5_, nullptr}};
		    ceres_scan_matcher_->Match(initial_pose2.translation(), initial_pose2, point_clouds_and_hybrid_grids, &pose,&summary);
		    */
	    }
	    if (mymap["usehighres"] =="0" && mymap["uselowres"] =="0")
	    {
		    double *residual = new double [point_cloud_5_.size()];
		    double trans[3]= {initial_pose2.translation().x(), initial_pose2.translation().y(),initial_pose2.translation().z()};
		    double quat[4] ={initial_pose2.rotation().w(),initial_pose2.rotation().x(),initial_pose2.rotation().y(),initial_pose2.rotation().z()};

		    std::cout<<"****************** both usehighres and uselowres are false, perform sweep func ***\n\ttrans" << trans[0]<<" "<<trans[1]<<" "<<trans[2] <<" quat "<< quat[0]<<" " << quat[1]<<" " <<quat[2]<<" " <<quat[3] <<"\n";

		    sweeptest(initial_pose2, point_cloud_5_, hybrid_grid_5_);
	    }

	    //ceres_scan_matcher_->Match(initial_pose2.translation(), initial_pose2, point_clouds_and_hybrid_grids, &pose,&summary);
    
	    LOG(INFO) << "\nestimated pose: "<< pose;
	    LOG(INFO) << summary.FullReport();
    
	    //transform cloud1 (scan) to cloud3 using estimated pose
	    transform_cloud (cloud1, cloud3, pose.translation(), pose.rotation());
	    transform_cloud (cloud4, cloud6, pose.translation(), pose.rotation());
	    // show high res submap and transformed scan
	    show_pcl_2cloud(cloud3,cloud2, "high res scan/submap aligned");
	    // show low res submap and transformed scan
	    show_pcl_2cloud(cloud6,cloud5, "low res scan/submap aligned");
	    return;
    }//end if ptselection


    OccupiedSpaceCostFunction3D::scan_matching_verbose=0;
    LOG(INFO) << "estimated pose: "<< pose;
    LOG(INFO) << summary.FullReport();
    //EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    //EXPECT_THAT(pose, transform::IsNearly(expected_pose_, 3e-2));
  }

  void transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Matrix<double, 3, 1> trans, Eigen::Quaternion<double> quat0)
  {
        Eigen::Quaterniond quat(1, 0,0,0);

        Eigen::Matrix3d mat3 = Eigen::Quaterniond(1, 0,0,0).toRotationMatrix();
        Eigen::Matrix3d mat4 = quat0.toRotationMatrix();
    
	Eigen::Vector3d v3d(-3., 0., 0.);
    
	Eigen::Transform<double, 3, Eigen::Affine > pose_trans;
//	pose_trans.translation()=v3d;
	pose_trans.translation()=trans;
	pose_trans.linear()=mat4;
	std::cout<<"transform_cloud quat0 wxyz: "<< quat0.w()<<" "<< quat0.x()<<" "<< quat0.y()<<" "<<quat0.z() <<"\n";
	std::cout<<"Eigen::Transform: "<< pose_trans.translation() <<"\n";
	std::cout<<"Eigen::Transform: rotation"<< pose_trans.rotation() <<"\n";
	pcl::transformPointCloud (*cloud_in, *cloud_out, pose_trans);
  }

/********************************************************************/
  /* read from txt file debugging cloud point */
  void add_dbg_pts(int dbgpt, std::vector<sensor::RangefinderPoint> &points, std::vector<float> &intensities)
  {

    //std::vector<sensor::RangefinderPoint> points;
    //std::vector<float> intensities;
    Eigen::Vector3f point;
    std::string dbgfilename = mymap["dbgfilename"];
    std::ifstream stream2(dbgfilename.c_str());
    std::string mypt_str= std::string((std::istreambuf_iterator<char>(stream2)),
                     std::istreambuf_iterator<char>());
    char *token;
    //std::cout<< "strings from dbg point file:\n" << mypt_str <<"\n";
    token = strtok(const_cast<char*>(mypt_str.c_str()), "\n");
    while (token != NULL) {
	    std::string svalue(token);
	    size_t pos = svalue.find(",");
            std::string svalue2 = svalue.substr(pos + 1, std::string::npos);
	    size_t pos2 = svalue2.find(",");
            std::string svalue3 = svalue2.substr(pos2 + 1, std::string::npos);
	    //std::cout<<"dbg " << pos<<" "<<pos2 <<" "<<svalue.substr(0, pos)<<" " <<svalue2.substr(0, pos2) << " " << svalue3 << "\n";
	    point(0) = std::stof(svalue.substr(0, pos));
	    point(1) = std::stof(svalue2.substr(0, pos2));
	    point(2) = std::stof(svalue3);

	    points.push_back({point});
	    intensities.push_back(50);
	    token = strtok(NULL, "\n");
// std::cout<<NULL will cause exception and terminate the func call, but the rest of the parent func still run.
	    if (token == NULL) std::cout<< "token is NULL" << (token ==NULL)<<"\n";
	    else std::cout<< "line " << token <<" ----\n";
    }
  }

/********************************************************************/
  // setup test points from either pcd cloud or debug file
  void initpc2(pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<sensor::RangefinderPoint> points_dbg;
    std::vector<float> intensities;
    std::vector<float> intensities_dbg;
    int maxpt = std::stoi(mymap["ptcount"]);
    maxpt = std::min(maxpt, int(pclcloud->width * pclcloud->height));
    int dbgpt = std::stoi(mymap["dbgpt"]);

    //----------------------read the dbg points from file
    add_dbg_pts(dbgpt, points_dbg, intensities_dbg);
    std::cout<< "dbg RangefinderPoint:\n";
    for(int i=0;i<dbgpt;i++){
    	std::cout<<  points_dbg[i].position(0) << ", " <<points_dbg[i].position(1) <<", "<<points_dbg[i].position(2) <<std::endl;
      hybrid_grid_3_.SetProbability(
          hybrid_grid_3_.GetCellIndex(expected_pose_.cast<float>() * points_dbg[i].position), 1.);
      intensity_hybrid_grid_3_.AddIntensity(
          intensity_hybrid_grid_3_.GetCellIndex(expected_pose_.cast<float>() *
                                              points_dbg[i].position),
          50);
    }
    point_cloud_3_ = sensor::PointCloud(points_dbg, intensities_dbg);

    //--------------------- pcd file data -----------------

    int ptcount=0;
    // pc2 points number: 33024
    for (const pcl::PointXYZ& pointxyz : pclcloud->points)
    {
	    ptcount++;
	    if (ptcount > maxpt) break;
	Eigen::Vector3f point;
        //LOG(INFO) << "point: "<< pointxyz;
	point(0)=pointxyz.x;
	point(1)=pointxyz.y;
	point(2)=pointxyz.z;
      	points.push_back({point});
        intensities.push_back(50);

      hybrid_grid_2_.SetProbability(
          hybrid_grid_2_.GetCellIndex(expected_pose_.cast<float>() * point), 1.);
      intensity_hybrid_grid_2_.AddIntensity(
          intensity_hybrid_grid_2_.GetCellIndex(expected_pose_.cast<float>() *
                                              point),
          50);
    }
    std::cout<< "pcd RangefinderPoint: ";
    for(int i=0;i<10;i++){
    	std::cout<<  points[i].position(0) << ", " <<points[i].position(1) <<", "<<points[i].position(2) <<std::endl;
    }
    point_cloud_2_ = sensor::PointCloud(points, intensities);
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

  void show_pcl_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, std::string title)
  {
	init_pcl_viewer(title); //need to init pcl every time it was closed.
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color (target_cloud, 255, 0, 0);
  
	viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");

	wait_pcl();
	/*
  // Wait until visualizer window is closed.
	std::chrono::milliseconds dura( 100 );
	while (!viewer_final->wasStopped ())
	{
	     	viewer_final->spinOnce (100);
		std::this_thread::sleep_for(dura);
	}
	*/
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
  
/********************************************************************/
  HybridGrid hybrid_grid_;
  IntensityHybridGrid intensity_hybrid_grid_;
  transform::Rigid3d expected_pose_;
  sensor::PointCloud point_cloud_;
  proto::CeresScanMatcherOptions3D options_;
  std::unique_ptr<CeresScanMatcher3D> ceres_scan_matcher_;

  pcl::visualization::PCLVisualizer::Ptr viewer_final;

  HybridGrid hybrid_grid_2_ = HybridGrid(1.f);
  IntensityHybridGrid intensity_hybrid_grid_2_ = IntensityHybridGrid(1.f);
  HybridGrid hybrid_grid_3_ = HybridGrid(1.f);
  IntensityHybridGrid intensity_hybrid_grid_3_ = IntensityHybridGrid(1.f);
  sensor::PointCloud point_cloud_2_;
  sensor::PointCloud point_cloud_3_;
  sensor::PointCloud point_cloud_4_; // for scan pcd high res
  HybridGrid * hybrid_grid_4_; // = HybridGrid(0.4f); // for submap pcd high res
  HybridGrid * hybrid_grid_5_; // = HybridGrid(0.4f); // for submap pcd low res
  IntensityHybridGrid intensity_hybrid_grid_4_ = IntensityHybridGrid(1.f);
  IntensityHybridGrid intensity_hybrid_grid_5_ = IntensityHybridGrid(1.f);
  sensor::PointCloud point_cloud_5_; // for scan pcd low res

    std::map<std::string, std::string> mymap;
    std::string myoptstr;
};

/********************************************************************/
/********************************************************************/
TEST_F(CeresScanMatcher3DTest, PerfectEstimate) {
	LOG(INFO) << "test ceres scan matching with pcd files :";
	        // Loading first scan, as reference or target cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::string pcdfn1;
	pcdfn1 = mymap["pcdfilename"];
//	pcdfn1= "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/mapdata_13.pcd";
	pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud);
	LOG(INFO) << *target_cloud;
	initpc2(target_cloud);

	//init_pcl_viewer();
	show_pcl_cloud(target_cloud, pcdfn1);
  
	LOG(INFO) << "new test case: ";
  
	TestFromInitialPose2(
		transform::Rigid3d::Translation(Eigen::Vector3d(-1.5, 0., 0.)));
  
	LOG(INFO) << "orig test case: ";
  
	TestFromInitialPose(
      		transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.)));
}

TEST_F(CeresScanMatcher3DTest, AlongX) {
  ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-2.8, 0., 0.)));
}

TEST_F(CeresScanMatcher3DTest, AlongZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., -0.2)));
}

TEST_F(CeresScanMatcher3DTest, AlongXYZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.9, -0.2, 0.2)));
}

TEST_F(CeresScanMatcher3DTest, FullPoseCorrection) {
  // We try to find the rotation around z...
  const auto additional_transform = transform::Rigid3d::Rotation(
      Eigen::AngleAxisd(0.05, Eigen::Vector3d(0., 0., 1.)));
  point_cloud_ = sensor::TransformPointCloud(
      point_cloud_, additional_transform.cast<float>());
  expected_pose_ = expected_pose_ * additional_transform.inverse();
  // ...starting initially with rotation around x.
  TestFromInitialPose(
      transform::Rigid3d(Eigen::Vector3d(-0.95, -0.05, 0.05),
                         Eigen::AngleAxisd(0.05, Eigen::Vector3d(1., 0., 0.))));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

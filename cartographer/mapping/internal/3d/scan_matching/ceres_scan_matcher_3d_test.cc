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

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class CeresScanMatcher3DTest : public ::testing::Test {
 protected:
  CeresScanMatcher3DTest()
      : hybrid_grid_(1.f),
        intensity_hybrid_grid_(1.f),
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
    for (auto keyval : mymap)
	    std::cout << keyval.first << ":" << keyval.second << std::endl;
    std::cout<< mymap["pcdfilename"] <<std::endl;

    std::string filename="testopt.lua";
    std::ifstream stream(filename.c_str());
    std::string myoptstr= std::string((std::istreambuf_iterator<char>(stream)),
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
	/*
    auto parameter_dictionary = common::MakeDictionary(R"text(
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
        })text");
	*/
    options_ = CreateCeresScanMatcherOptions3D(parameter_dictionary.get());
    //LOG(INFO) << parameter_dictionary->GetKeys();
    //LOG(INFO) << parameter_dictionary->ToString();
    ceres_scan_matcher_.reset(new CeresScanMatcher3D(options_));
  }

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
  void TestFromInitialPose2(const transform::Rigid3d& initial_pose) {
    transform::Rigid3d pose;

    ceres::Solver::Summary summary;

    IntensityHybridGrid* intensity_hybrid_grid_ptr =
        point_cloud_2_.intensities().empty() ? nullptr : &intensity_hybrid_grid_2_;

    ceres_scan_matcher_->Match(
        initial_pose.translation(), initial_pose,
        {{&point_cloud_2_, &hybrid_grid_2_, intensity_hybrid_grid_ptr}}, &pose,
        &summary);
    LOG(INFO) << "estimated pose: "<< pose;
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    EXPECT_THAT(pose, transform::IsNearly(expected_pose_, 3e-2));
  }

  void initpc2(pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    int maxpt = std::stoi(mymap["ptcount"]);
    maxpt = std::min(maxpt, int(pclcloud->width * pclcloud->height));
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
    std::cout<< "RangefinderPoint: ";
    for(int i=0;i<10;i++){
    	std::cout<<  points[i].position(0) << ", " <<points[i].position(1) <<", "<<points[i].position(2) <<std::endl;
    }

    point_cloud_2_ = sensor::PointCloud(points, intensities);
  }
  
  HybridGrid hybrid_grid_;
  IntensityHybridGrid intensity_hybrid_grid_;
  transform::Rigid3d expected_pose_;
  sensor::PointCloud point_cloud_;
  proto::CeresScanMatcherOptions3D options_;
  std::unique_ptr<CeresScanMatcher3D> ceres_scan_matcher_;

  HybridGrid hybrid_grid_2_ = HybridGrid(1.f);
  IntensityHybridGrid intensity_hybrid_grid_2_ = IntensityHybridGrid(1.f);
  sensor::PointCloud point_cloud_2_;

    std::map<std::string, std::string> mymap;
};

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

	  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);
  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  std::chrono::milliseconds dura( 100 );
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(dura);
  }

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
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.8, 0., 0.)));
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

/*
 *
 * Ju Wang
 * 3/2/2022 
 * input: testcfg.yaml
 * 	pcd files, init pose, gt pose
this is a replic of icp_example.cc but build in cartograph space
run a scan match methods on testing pcd files.
        mode: icp, ceres, ndt
build:
        cartograph build (standalone or via ros catkin)
run:
        catkin_ws/src/cartographer/build$ ./cartographer_wangtest
*
 * Copyright 2018 The Cartographer Authors
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

#include <functional>

#include "absl/container/flat_hash_set.h"
#include "cartographer/io/internal/pbstream_info.h"
#include "cartographer/io/internal/pbstream_migrate.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"
#include "cartographer/mapping/proto/submaps_options_3d.pb.h"
#include "cartographer/transform/timestamped_transform.h"
#include "glog/logging.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/mapping/internal/3d/icp_example.h"
#include <regex>
//$icp_main2 see icp_example.h
//
namespace cartographer {
namespace mapping {

/*
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
  void scanmatch_reset_single(std::unique_ptr<cartographer::mapping::scan_matching::CeresScanMatcher3D> & ceres_scan_matcher_)
  {
    std::string filename="testopt.lua";
    std::string myoptstr;
    scan_matching::proto::CeresScanMatcherOptions3D options_;

    std::ifstream stream(filename.c_str());
    myoptstr= std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());

    myoptstr = std::regex_replace(myoptstr, std::regex("occupied_space_weight_1"), "occupied_space_weight_0");
  //  std::cout<<myoptstr;
    auto parameter_dictionary = common::MakeDictionary(myoptstr);
    options_ = scan_matching::CreateCeresScanMatcherOptions3D(parameter_dictionary.get());
    ceres_scan_matcher_.reset(new scan_matching::CeresScanMatcher3D(options_));
  }

}
}
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  cartographer::transform::Rigid3d initial_ceres_pose;
  std::string icp_method;
  int pcl_viewerflag=0;
  cartographer::transform::Rigid3d  tmp_pose_observation_in_submap;
   pcl::PointCloud<pcl::PointXYZ>::Ptr scan_point_cloud;
   pcl::PointCloud<pcl::PointXYZ>::Ptr scan_point_cloud_prev;
   pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   Eigen::Matrix4f guess0;
           std::string pcdfn1;
        std::string pcdfn2;
        std::string method;
        double resolution;
	cartographer::sensor::PointCloud point_cloud_4_; // for scan pcd high res

  cartographer::mapping::HybridGrid * hybrid_grid_4_; // = HybridGrid(0.4f); // for submap pcd high res
  cartographer::mapping::HybridGrid * hybrid_grid_5_; // = HybridGrid(0.4f); // for submap pcd low res
  cartographer::mapping::IntensityHybridGrid intensity_hybrid_grid_4_ = cartographer::mapping::IntensityHybridGrid(1.f);
  std::vector<cartographer::mapping::scan_matching::PointCloudAndHybridGridsPointers> point_clouds_and_hybrid_grids;
	std::unique_ptr<cartographer::mapping::scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;
    cartographer::mapping::scanmatch_reset_single(ceres_scan_matcher_);

    cartographer::transform::Rigid3d pose;

    ceres::Solver::Summary summary;
   Eigen::Quaternion<double> quat0(1,0,0,0);

   guess0 = cartographer::mapping::loadyaml3("testcfg.yaml", pcdfn1, pcdfn2, method, resolution, quat0);
   initial_ceres_pose = cartographer::transform::Rigid3d(guess0.block<3,1>(0,3).cast<double>(), quat0);
   pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud);
   pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn2, *input_cloud);

   hybrid_grid_4_ = new cartographer::mapping::HybridGrid(0.1f);
   cartographer::mapping::PCDCloud_2_HybridGrid(target_cloud,*hybrid_grid_4_,intensity_hybrid_grid_4_);
   point_clouds_and_hybrid_grids={{&point_cloud_4_, hybrid_grid_4_, nullptr}};
   ceres_scan_matcher_->Match(initial_ceres_pose.translation(), initial_ceres_pose, point_clouds_and_hybrid_grids, &pose,&summary);
   std::cout << "estimated pose: "<< pose<<"\n";
   std::cout<<summary.FullReport();
   /*
  cartographer::mapping::icp_main2(&tmp_pose_observation_in_submap, initial_ceres_pose, scan_point_cloud,scan_point_cloud_prev, icp_method, pcl_viewerflag);

  internal/3d/icp_example_more.cc
*/
   cartographer::mapping::icp_main3(argc, argv);
  FLAGS_logtostderr = true;
  const std::string usage_message =
      "Swiss Army knife for pbstreams.\n\n"
      "Currently supported subcommands are:\n"
      "\tinfo    - Prints summary of pbstream.\n"
      "\tmigrate - Migrates pbstream to the new submap format.";
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2) {
    google::SetUsageMessage(usage_message);
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_main");
    return EXIT_FAILURE;
  } else if (std::string(argv[1]) == "info") {
    return ::cartographer::io::pbstream_info(argc, argv);
  } else if (std::string(argv[1]) == "migrate") {
    return ::cartographer::io::pbstream_migrate(argc, argv);
  } else {
    LOG(INFO) << "Unknown subtool: \"" << argv[1];
    google::SetUsageMessage(usage_message);
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_main");
    return EXIT_FAILURE;
  }
}

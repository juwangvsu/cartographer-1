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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

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

namespace cartographer {
namespace mapping {

// TODO(spielawa): Adjust metrics for multi-trajectory. So far we assume a
// single trajectory.
static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamVoxelFilterFraction = metrics::Gauge::Null();
static auto* kLocalSlamScanMatcherFraction = metrics::Gauge::Null();
static auto* kLocalSlamInsertIntoSubmapFraction = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

//jwang dbg: dump the submap as pcd files
void submap_info(std::shared_ptr<const mapping::Submap3D> submap, bool dbgflag); ////see submap_3d.cc
void submap_info2(const Submap3D * submap, bool dbgflag); //see submap_3d.cc
void hgrid_info(const HybridGrid * hgrid, bool dbgflag); //see submap_3d.cc

void hybridgrid_2_pcd(const HybridGrid * hybrid_grid, std::string pcdfn, float hitprobthresh);
void submap_2_pcd(std::shared_ptr<const mapping::Submap3D> submap, float hgrid_pcd_probthresh)
{
	std::string pcdfn1, pcdfn2, pcdfn3;

	std::cout<<"\n\nsubmap_2_pcd: " << submap->local_pose();
	const auto * hres_hybrid_grid = &submap->high_resolution_hybrid_grid();
	const HybridGrid * lres_hybrid_grid = &submap->low_resolution_hybrid_grid();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcdfn1= "/home/student/Documents/AirSim/ros/src/hdl_graph_slam/mapdata_13.pcd";
	pcdfn2= "/home/student/Documents/cartographer/submap_test_h.pcd";
	pcdfn3= "/home/student/Documents/cartographer/submap_test_l.pcd";
        pcl::io::loadPCDFile<pcl::PointXYZ> (pcdfn1, *target_cloud);
	hybridgrid_2_pcd(hres_hybrid_grid, pcdfn2, hgrid_pcd_probthresh);
	hybridgrid_2_pcd(lres_hybrid_grid, pcdfn3, hgrid_pcd_probthresh);
}

using TimedPointCloud = std::vector<cartographer::sensor::TimedRangefinderPoint>;
// write timed scan to pcd
void scan2_2_pcd(const TimedPointCloud t_cloud,std::string pcdfn)
{
	//struct RangefinderPoint {
  //Eigen::Vector3f position;
//};
//	TimedPointCloud is std::vector<TimedRangefinderPoint>;
	std::cout<<"\nscan2_2_pcd #points: "<<t_cloud.size()<<" " <<pcdfn<<"\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	scan_cloud->height   = 1;
	scan_cloud->width   = t_cloud.size();
	scan_cloud->is_dense = false;
  	scan_cloud->points.resize (scan_cloud->width * scan_cloud->height);
	int i=0;
   	for (auto& point: t_cloud)
   	{
		(*scan_cloud)[i].x = point.position[0];
		(*scan_cloud)[i].y = point.position[1];
		(*scan_cloud)[i].z = point.position[2];
		i++;
   	}
	pcl::io::savePCDFile<pcl::PointXYZ> (pcdfn, *scan_cloud, false);

}
/*
 * save a scan (in sensor::PointCloud format to pcd format
 */
void scan_2_pcd(const sensor::PointCloud h_cloud,std::string pcdfn)
{
	std::cout<<"\nscan_2_pcd #points: "<<h_cloud.size()<<" " <<pcdfn<<"\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	scan_cloud->height   = 1;
	scan_cloud->width   = h_cloud.size();
	scan_cloud->is_dense = false;
  	scan_cloud->points.resize (scan_cloud->width * scan_cloud->height);
	for (size_t i = 0; i < h_cloud.size(); ++i) {
		//const Eigen::Matrix<T, 3, 1> world=h_cloud[i];
		const sensor::RangefinderPoint&  point=h_cloud[i];
	//	std::cout<< point.position[0]<<point.position[1];

		(*scan_cloud)[i].x = point.position[0];
		(*scan_cloud)[i].y = point.position[1];
		(*scan_cloud)[i].z = point.position[2];
	}
  
	pcl::io::savePCDFile<pcl::PointXYZ> (pcdfn, *scan_cloud, false);
	//	copied_point_cloud.points()

}

/************ save HybridGrid to PCD file, hitprobthresh=0.5 to save only hit points.
 * */
void hybridgrid_2_pcd(const HybridGrid * hybrid_grid, std::string pcdfn, float hitprobthresh=0.1)
{
	std::cout<<"\n***** hybridgrid_2_pcd***************\n"<<"hitprobthresh: "<< hitprobthresh<<"\n";
	hgrid_info(hybrid_grid,false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	submap_cloud->height   = 1;
	submap_cloud->is_dense = false;
/*
  for (auto& point: *submap_cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  */
  int ii =0;
  std::vector<Eigen::Array3f> cell_vec;
  for (auto it = HybridGrid::Iterator(*hybrid_grid); !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const float iterator_probability = ValueToProbability(it.GetValue());
     Eigen::Array3f cell_center = hybrid_grid->GetCenterOfCell(cell_index);
//    std::cout << "cell center: " << cell_center[0] << "," << cell_center[1] << "," <<cell_center[2]<<"\n";
//    std::cout<<"cell index: " << cell_index[0] << ", "<<cell_index[1] <<"," <<cell_index[2] << " prob: " << iterator_probability<<"\n";
    if (iterator_probability>hitprobthresh) {// 0.5 for hit points only
	    cell_vec.push_back(cell_center);
	    ii++;
    }
  }
  submap_cloud->width    = ii;
  submap_cloud->points.resize (ii * submap_cloud->height);
  std::cout<<"\nsubmap num points: "<< ii * submap_cloud->height<<" " << pcdfn<<"\n";
  for (int iii = 0; iii<ii; iii++)
  {
    (*submap_cloud)[iii].x = cell_vec[iii][0];
    (*submap_cloud)[iii].y = cell_vec[iii][1];
    (*submap_cloud)[iii].z = cell_vec[iii][2];
  }

  pcl::io::savePCDFile<pcl::PointXYZ> (pcdfn, *submap_cloud, false);
}

LocalTrajectoryBuilder3D::LocalTrajectoryBuilder3D(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          absl::make_unique<scan_matching::RealTimeCorrelativeScanMatcher3D>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(absl::make_unique<scan_matching::CeresScanMatcher3D>(
          options_.ceres_scan_matcher_options())),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder3D::~LocalTrajectoryBuilder3D() {}

std::unique_ptr<transform::Rigid3d> LocalTrajectoryBuilder3D::ScanMatch(
    const transform::Rigid3d& pose_prediction,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking) {
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid3d>(pose_prediction);
  }
  std::shared_ptr<const mapping::Submap3D> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;

  //**** start jwang debug code
  std::cout<<"***************** LocalTrajectoryBuilder3D::ScanMatch \n";
  std::cout<<"************submap pose: \n"<<matching_submap->local_pose()<<"\n";
  std::cout<<"************initial pose (local frame): \n"<<initial_ceres_pose<<"\n";
  submap_2_pcd(matching_submap, options_.hgrid_pcd_probthresh());
  std::string scan_cloud_fn1= "/home/student/Documents/cartographer/scan_test_l.pcd";
  std::string scan_cloud_fn2= "/home/student/Documents/cartographer/scan_test_h.pcd";
  scan_2_pcd(low_resolution_point_cloud_in_tracking,scan_cloud_fn1);

  scan_2_pcd(high_resolution_point_cloud_in_tracking, scan_cloud_fn2);

  //**** end jwang debug code
  //
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    const double score = real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;
  const auto* high_resolution_intensity_hybrid_grid =
      options_.use_intensities()
          ? &matching_submap->high_resolution_intensity_hybrid_grid()
          : nullptr;
  ceres_scan_matcher_->Match(
      (matching_submap->local_pose().inverse() * pose_prediction).translation(),
      initial_ceres_pose, {{&high_resolution_point_cloud_in_tracking,
                            &matching_submap->high_resolution_hybrid_grid(),
                            high_resolution_intensity_hybrid_grid},
                           {&low_resolution_point_cloud_in_tracking,
                            &matching_submap->low_resolution_hybrid_grid(),
                            /*intensity_hybrid_grid=*/nullptr}},
      &pose_observation_in_submap, &summary);
  LOG(INFO) << summary.FullReport();
  LOG(INFO) << pose_observation_in_submap;
  kCeresScanMatcherCostMetric->Observe(summary.final_cost);
  const double residual_distance = (pose_observation_in_submap.translation() -
                                    initial_ceres_pose.translation())
                                       .norm();
  kScanMatcherResidualDistanceMetric->Observe(residual_distance);
  const double residual_angle =
      pose_observation_in_submap.rotation().angularDistance(
          initial_ceres_pose.rotation());
  kScanMatcherResidualAngleMetric->Observe(residual_angle);
  return absl::make_unique<transform::Rigid3d>(matching_submap->local_pose() *
                                               pose_observation_in_submap);
}

void LocalTrajectoryBuilder3D::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  std::vector<transform::TimestampedTransform> initial_poses;
  for (const auto& pose_proto : options_.initial_poses()) {
    initial_poses.push_back(transform::FromProto(pose_proto));
  }
  std::vector<sensor::ImuData> initial_imu_data;
  for (const auto& imu : options_.initial_imu_data()) {
    initial_imu_data.push_back(sensor::FromProto(imu));
  }
  initial_imu_data.push_back(imu_data);
  extrapolator_ = mapping::PoseExtrapolatorInterface::CreateWithImuData(
      options_.pose_extrapolator_options(), initial_imu_data, initial_poses);
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  if (options_.use_intensities()) {
    CHECK_EQ(unsynchronized_data.ranges.size(),
             unsynchronized_data.intensities.size())
        << "Passed point cloud has inconsistent number of intensities and "
           "ranges.";
  }
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  const common::Time time_first_point =
      synchronized_data.time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  if (num_accumulated_ == 0) {
    accumulated_point_cloud_origin_data_.clear();
  }
  //jwang save raw data for comparison before voxel filter, this should
  //be original resolution:

  std::string pcd_syncdata="/home/student/Documents/cartographer/test/unsync_rangedata.pcd";
  scan2_2_pcd(unsynchronized_data.ranges, pcd_syncdata);

  synchronized_data.ranges = sensor::VoxelFilter(
      synchronized_data.ranges, 0.5f * options_.voxel_filter_size());
  accumulated_point_cloud_origin_data_.emplace_back(
      std::move(synchronized_data));
  ++num_accumulated_;

  if (num_accumulated_ < options_.num_accumulated_range_data()) {
    return nullptr;
  }
  num_accumulated_ = 0;

  bool warned = false;
  std::vector<common::Time> hit_times;
  common::Time prev_time_point = extrapolator_->GetLastExtrapolatedTime();
  for (const auto& point_cloud_origin_data :
       accumulated_point_cloud_origin_data_) {
    for (const auto& hit : point_cloud_origin_data.ranges) {
      common::Time time_point = point_cloud_origin_data.time +
                                common::FromSeconds(hit.point_time.time);
      if (time_point < prev_time_point) {
        if (!warned) {
          LOG(ERROR) << "Timestamp of individual range data point jumps "
                        "backwards from "
                     << prev_time_point << " to " << time_point;
          warned = true;
        }
        time_point = prev_time_point;
      }

      hit_times.push_back(time_point);
      prev_time_point = time_point;
    }
  }
  hit_times.push_back(accumulated_point_cloud_origin_data_.back().time);

  const PoseExtrapolatorInterface::ExtrapolationResult extrapolation_result =
      extrapolator_->ExtrapolatePosesWithGravity(hit_times);
  std::vector<transform::Rigid3f> hits_poses(
      std::move(extrapolation_result.previous_poses));
  hits_poses.push_back(extrapolation_result.current_pose.cast<float>());
  CHECK_EQ(hits_poses.size(), hit_times.size());

  const size_t max_possible_number_of_accumulated_points = hit_times.size();
  std::vector<sensor::RangefinderPoint> accumulated_points;
  std::vector<float> accumulated_intensities;
  accumulated_points.reserve(max_possible_number_of_accumulated_points);
  if (options_.use_intensities()) {
    accumulated_intensities.reserve(max_possible_number_of_accumulated_points);
  }
  sensor::PointCloud misses;
  std::vector<transform::Rigid3f>::const_iterator hits_poses_it =
      hits_poses.begin();
  for (const auto& point_cloud_origin_data :
       accumulated_point_cloud_origin_data_) {
    for (const auto& hit : point_cloud_origin_data.ranges) {
      const Eigen::Vector3f hit_in_local =
          *hits_poses_it * hit.point_time.position;
      const Eigen::Vector3f origin_in_local =
          *hits_poses_it * point_cloud_origin_data.origins.at(hit.origin_index);
      const Eigen::Vector3f delta = hit_in_local - origin_in_local;
      const float range = delta.norm();
      if (range >= options_.min_range()) {
        if (range <= options_.max_range()) {
          accumulated_points.push_back(sensor::RangefinderPoint{hit_in_local});
          if (options_.use_intensities()) {
            accumulated_intensities.push_back(hit.intensity);
          }
        } else {
          // We insert a ray cropped to 'max_range' as a miss for hits beyond
          // the maximum range. This way the free space up to the maximum range
          // will be updated.
          // TODO(wohe): since `misses` are not used anywhere in 3D, consider
          // removing `misses` from `range_data` and/or everywhere in 3D.
          misses.push_back(sensor::RangefinderPoint{
              origin_in_local + options_.max_range() / range * delta});
        }
      }
      ++hits_poses_it;
    }
  }
  CHECK(std::next(hits_poses_it) == hits_poses.end());
  const sensor::PointCloud returns(std::move(accumulated_points),
                                   std::move(accumulated_intensities));

  const common::Time current_sensor_time = synchronized_data.time;
  absl::optional<common::Duration> sensor_duration;
  if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
  }
  last_sensor_time_ = current_sensor_time;

  const common::Time current_time = hit_times.back();
  const auto voxel_filter_start = std::chrono::steady_clock::now();
  const sensor::RangeData filtered_range_data = {
      extrapolation_result.current_pose.translation().cast<float>(),
      sensor::VoxelFilter(returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(misses, options_.voxel_filter_size())};
  const auto voxel_filter_stop = std::chrono::steady_clock::now();
  const auto voxel_filter_duration = voxel_filter_stop - voxel_filter_start;

  if (sensor_duration.has_value()) {
    const double voxel_filter_fraction =
        common::ToSeconds(voxel_filter_duration) /
        common::ToSeconds(sensor_duration.value());
    kLocalSlamVoxelFilterFraction->Set(voxel_filter_fraction);
  }

  return AddAccumulatedRangeData(
      current_time,
      sensor::TransformRangeData(
          filtered_range_data,
          extrapolation_result.current_pose.inverse().cast<float>()),
      sensor_duration, extrapolation_result.current_pose,
      extrapolation_result.gravity_from_tracking);
}
int scan_seq=0;
std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const absl::optional<common::Duration>& sensor_duration,
    const transform::Rigid3d& pose_prediction,
    const Eigen::Quaterniond& gravity_alignment) {
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  std::string pcd_filteredrangedata="/home/student/Documents/cartographer/test/filtered_range_data_in_tracking.pcd";
  scan_2_pcd(filtered_range_data_in_tracking.returns,pcd_filteredrangedata);

  const auto scan_matcher_start = std::chrono::steady_clock::now();

  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      sensor::AdaptiveVoxelFilter(
          filtered_range_data_in_tracking.returns,
          options_.high_resolution_adaptive_voxel_filter_options());
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      sensor::AdaptiveVoxelFilter(
          filtered_range_data_in_tracking.returns,
          options_.low_resolution_adaptive_voxel_filter_options());
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  std::unique_ptr<transform::Rigid3d> pose_estimate =
      ScanMatch(pose_prediction, low_resolution_point_cloud_in_tracking,
                high_resolution_point_cloud_in_tracking);
  if (pose_estimate == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  extrapolator_->AddPose(time, *pose_estimate);

  const auto scan_matcher_stop = std::chrono::steady_clock::now();
  const auto scan_matcher_duration = scan_matcher_stop - scan_matcher_start;
  if (sensor_duration.has_value()) {
    const double scan_matcher_fraction =
        common::ToSeconds(scan_matcher_duration) /
        common::ToSeconds(sensor_duration.value());
    kLocalSlamScanMatcherFraction->Set(scan_matcher_fraction);
  }

  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, pose_estimate->cast<float>());

  const auto insert_into_submap_start = std::chrono::steady_clock::now();

  /*jwang debug start */
  std::cout<<"\n****** insert into submap******\n";
  std::string pcd_hrt="/home/student/Documents/cartographer/test/scan_hrt_"+std::to_string(scan_seq)+".pcd";
  scan_2_pcd(high_resolution_point_cloud_in_tracking,pcd_hrt);
  scan_seq++;

//  std::cout<<"active_submap: "<< &active_submaps_ <<"\n";
  std::cout<<"active_submap.submaps.size(): "<< active_submaps_.submaps().size() <<"\n";
  std::shared_ptr<const mapping::Submap3D> matching_submap ;
	  //matching_submap=active_submaps_.submaps().front();
  //std::cout<<"matching_submap: "<< matching_submap <<"\n";
  //submap_2_pcd(matching_submap);
/*jwang debug end */
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, *pose_estimate,
      gravity_alignment);

  /*jwang debug start after scanmatch and insertmap
  std::cout<<"\n\nafter scanmatch and insertmap, active_submap: "<< &active_submaps_ <<"\n";
  std::cout<<"active_submap.submaps.size(): "<< active_submaps_.submaps().size() <<"\n";
  matching_submap =
       active_submaps_.submaps().front();
  submap_2_pcd(matching_submap);

jwang debug end */

  const auto insert_into_submap_stop = std::chrono::steady_clock::now();

  const auto insert_into_submap_duration =
      insert_into_submap_stop - insert_into_submap_start;
  if (sensor_duration.has_value()) {
    const double insert_into_submap_fraction =
        common::ToSeconds(insert_into_submap_duration) /
        common::ToSeconds(sensor_duration.value());
    kLocalSlamInsertIntoSubmapFraction->Set(insert_into_submap_fraction);
  }
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  return absl::make_unique<MatchingResult>(MatchingResult{
      time, *pose_estimate, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

void LocalTrajectoryBuilder3D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::InsertionResult>
LocalTrajectoryBuilder3D::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());


  //jwang debug start
  scan_2_pcd(filtered_range_data_in_local.returns, "range_data.pcd");
  //jwang debug end
  const Eigen::Quaterniond local_from_gravity_aligned =
      pose_estimate.rotation() * gravity_alignment.inverse();
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      active_submaps_.InsertData(filtered_range_data_in_local,
                                 local_from_gravity_aligned,
                                 rotational_scan_matcher_histogram_in_gravity);
  return absl::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram_in_gravity,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder3D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});

  auto* voxel_filter_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_voxel_filter_fraction",
      "Fraction of total sensor time taken up by voxel filter.");
  kLocalSlamVoxelFilterFraction = voxel_filter_fraction->Add({});

  auto* scan_matcher_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_scan_matcher_fraction",
      "Fraction of total sensor time taken up by scan matcher.");
  kLocalSlamScanMatcherFraction = scan_matcher_fraction->Add({});

  auto* insert_into_submap_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_insert_into_submap_fraction",
      "Fraction of total sensor time taken up by inserting into submap.");
  kLocalSlamInsertIntoSubmapFraction = insert_into_submap_fraction->Add({});

  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});

  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer

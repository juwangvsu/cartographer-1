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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
// Computes a cost for matching the 'point_cloud' to the 'hybrid_grid' with a
// 'translation' and 'rotation'. The cost increases when points fall into less
// occupied space, i.e. at voxels with lower values.
class OccupiedSpaceCostFunction3D {
 public:
  inline static int scan_matching_verbose=0;
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const sensor::PointCloud& point_cloud,
      const mapping::HybridGrid& hybrid_grid) {
    return new ceres::AutoDiffCostFunction<
        OccupiedSpaceCostFunction3D, ceres::DYNAMIC /* residuals */,
        3 /* translation variables */, 4 /* rotation variables */>(
        new OccupiedSpaceCostFunction3D(scaling_factor, point_cloud,
                                        hybrid_grid),
        point_cloud.size());
  }

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

 //private:
  OccupiedSpaceCostFunction3D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const mapping::HybridGrid& hybrid_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(hybrid_grid) {}

 private:
  OccupiedSpaceCostFunction3D(const OccupiedSpaceCostFunction3D&) = delete;
  OccupiedSpaceCostFunction3D& operator=(const OccupiedSpaceCostFunction3D&) =
      delete;

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
	  std::string vtype=typeid(transform).name();
    if (scan_matching_verbose==1) {
	  //std::cout<<typeid(transform).name()<<"\n";
	    // N12cartographer9transform6Rigid3IN5ceres3JetIdLi7EEEEE
	    // N12cartographer9transform6Rigid3IdEE
	      if (vtype=="N12cartographer9transform6Rigid3IN5ceres3JetIdLi7EEEEE")
	      {
	    	std::cout<<"translation:\n\t" <<transform.translation()[0]<< "\n\t" <<transform.translation()[1]<<  "\n\t" <<transform.translation()[2]<<"\n" ;
	    	std::cout<<"rotation quat:\n\t" <<transform.rotation().x()<<  "\n\t"<<transform.rotation().y()<<  "\n\t"<<transform.rotation().z()<<  "\n\t"<<transform.rotation().w()<<  "\n" ;
	      }else{
	    	std::cout<<"translation: " <<transform.translation()[0]<< " " <<transform.translation()[1]<<  " " <<transform.translation()[2]<<"\n" ;
	    	std::cout<<"rotation quat: " <<transform.rotation().x()<<  " "<<transform.rotation().y()<<  " "<<transform.rotation().z()<<  " "<<transform.rotation().w()<<  "\n" ;
	      }
    }
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> world =
          transform * point_cloud_[i].position.cast<T>();
      const T probability =
          interpolated_grid_.GetInterpolatedValue(world[0], world[1], world[2]);
      residual[i] = scaling_factor_ * (1. - probability);
      if (scan_matching_verbose==1) {
	      if (vtype=="N12cartographer9transform6Rigid3IN5ceres3JetIdLi7EEEEE")
	      {
	    	std::cout<<"world\n" << world[0] << "\n" << world[1] << "\n" << world[2] << "\nprob " << probability << "\n";
	      }else
	    	std::cout<<"world " << world[0] << " " << world[1] << " " << world[2] << " prob " << probability << "\n";
      }
    }
    if (scan_matching_verbose==1) {
	    std::cout<<"scan matching verbose\n" <<  "\n";
    }
    return true;
  }

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedProbabilityGrid interpolated_grid_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_3D_H_

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

#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_grid.h"
#include <map>
#include <random>
#include <tuple>

#include "gmock/gmock.h"
#include "cartographer/mapping/probability_values.h"
namespace cartographer {
namespace mapping {
namespace {
constexpr int kValueCount = 32768;
TEST(HybridGridTest, ApplyOdds) {
  HybridGrid hybrid_grid(1.f);

  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 1)));

  hybrid_grid.SetProbability(Eigen::Vector3i(1, 0, 1), 0.5f);

  //hybrid_grid.ApplyLookupTable(Eigen::Array3i(1, 0, 1),
    //                           ComputeLookupTableToApplyOdds(Odds(0.9f)));
  //hybrid_grid.FinishUpdate();
  std::cout  << "hybrid_grid.GetProbability(Eigen::Array3i(1, 0, 1)):" <<hybrid_grid.GetProbability(Eigen::Array3i(1, 0, 1))<<"\n";
  std::cout  << "hybrid_grid.GetProbability(Eigen::Array3i(3, 0, 1)):" <<hybrid_grid.GetProbability(Eigen::Array3i(3, 0, 1))<<" isKnown(): "<< hybrid_grid.IsKnown(Eigen::Array3i(3, 0, 1))<<"\n";
  EXPECT_GT(hybrid_grid.GetProbability(Eigen::Array3i(1, 0, 1)), 0.6f);

  hybrid_grid.SetProbability(Eigen::Array3i(2, 1, 1), 0.5f);
  hybrid_grid.SetProbability(Eigen::Array3i(2.1, 1, 1), 0.2f);

  std::cout<<"(2,1,1): " <<hybrid_grid.value(Eigen::Array3i(2, 1,1)    ) << "\n";
  std::cout<<"(2.1,1,1): " <<hybrid_grid.value(Eigen::Array3i(2.1, 1,1)    ) <<" coord " <<Eigen::Array3i(2.1, 1, 1)<< "\n";
  std::cout<<"(3,1,1): " <<hybrid_grid.value(Eigen::Array3i(3, 1,1)    ) << "\n";

  // test applylookuptable
  hybrid_grid.SetProbability(Eigen::Array3i(0, 1, 1), 0.5f);
  hybrid_grid.SetProbability(Eigen::Array3i(0, 1, 0), 0.5f);
  std::cout <<"ValueToProbability: "<< ValueToProbability(61439) << " " << ValueToProbability(28671)<<"\n";
  std::cout  << "ApplyLookupTable, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,0))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,0)) << "\n";
  std::cout  << "ApplyLookupTable, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 1)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,1))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,1)) << "\n";

//  hybrid_grid.ApplyLookupTable(Eigen::Array3i(0, 1, 0),
  //                             ComputeLookupTableToApplyOdds(Odds(0.3f)));
  std::vector<uint16> table1 = ComputeLookupTableToApplyOdds(Odds(0.6f));

  hybrid_grid.ApplyLookupTable(Eigen::Array3i(0, 1, 0),table1);
  hybrid_grid.FinishUpdate();
  std::cout  << "ApplyLookupTable 1, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,0))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,0)) << "\n";

  hybrid_grid.ApplyLookupTable(Eigen::Array3i(0, 1, 0),table1);

  std::cout  << "ApplyLookupTable 2nd, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,0))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,0)) << "\n";

  hybrid_grid.FinishUpdate();
  std::cout  << "FinishUpdate, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,0))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,0)) << "\n";
  std::cout  << "FinishUpdate, hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 1)):" <<hybrid_grid.GetProbability(Eigen::Array3i(0, 1,1))<<" " << hybrid_grid.value(Eigen::Array3i(0, 1,1)) << "\n";

  std::cout <<"ValueToProbability: "<< ValueToProbability(61439) << " " << ValueToProbability(28671)<<"\n";
  EXPECT_LT(hybrid_grid.GetProbability(Eigen::Array3i(0, 1, 0)), 0.5f);

  // test InterpolatedProbabilityGrid, this is used in cost function to evaluate residue and jacob

  scan_matching::InterpolatedProbabilityGrid * interpolated_grid_ = new scan_matching::InterpolatedProbabilityGrid(hybrid_grid);
  const Eigen::Matrix<double, 3, 1> world(0.0, 1.0, 0.0);
  std::cout<<"interpo_grid: " << interpolated_grid_->GetInterpolatedValue(world[0],world[1],world[2])<<"\n";
  std::cout<<"interpo_grid: " << interpolated_grid_->GetInterpolatedValue(.0, 1.0, 1.9)<<"\n";
  /*
  float odds=Odds(0.6f);
  float newodds;
  float newprob;
  float oldodd;
  float oldprob;
  for (int cell = 1; cell != kValueCount; ++cell) {
    oldodd=Odds((*kValueToProbability)[cell]);
    oldprob = ProbabilityFromOdds(oldodd);

    newodds = odds * Odds((*kValueToProbability)[cell]);
    newprob = ProbabilityFromOdds(newodds);
    std::cout<<odds <<" oldodd: " <<oldodd <<" newodd: " <<newodds <<" oldprob: " << oldprob <<" newprob: " << newprob <<" oldvalue: " << cell <<" newvalue: " << ProbabilityToValue(newprob)<<"\n";
  }
  */

  // Tests adding odds to an unknown cell.
  hybrid_grid.ApplyLookupTable(Eigen::Array3i(1, 1, 1),
                               ComputeLookupTableToApplyOdds(Odds(0.42f)));
  EXPECT_NEAR(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f, 1e-4);

  // Tests that further updates are ignored if FinishUpdate() isn't called.
  hybrid_grid.ApplyLookupTable(Eigen::Array3i(1, 1, 1),
                               ComputeLookupTableToApplyOdds(Odds(0.9f)));
  EXPECT_NEAR(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f, 1e-4);
  hybrid_grid.FinishUpdate();
  hybrid_grid.ApplyLookupTable(Eigen::Array3i(1, 1, 1),
                               ComputeLookupTableToApplyOdds(Odds(0.9f)));
  EXPECT_GT(hybrid_grid.GetProbability(Eigen::Array3i(1, 1, 1)), 0.42f);
}
TEST(HybridGridTest, wang) {
  HybridGrid hybrid_grid_(2.f);
    for (const Eigen::Vector3f& point :
         {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
          Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
          Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
          Eigen::Vector3f(-7.f, 3.f, 1.f), Eigen::Vector3f(-1.f, 3.f, 1.f)})
    {
      //points.push_back({point});
      hybrid_grid_.SetProbability(
          hybrid_grid_.GetCellIndex(point), 1.);
    }
  for (auto it = HybridGrid::Iterator(hybrid_grid_); !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const float iterator_probability = ValueToProbability(it.GetValue());
     Eigen::Array3f cellcenter = hybrid_grid_.GetCenterOfCell(cell_index);
    std::cout << "cell center: " << cellcenter[0] << "," << cellcenter[1] << "," <<cellcenter[2]<<"\n";
    std::cout<<"cell index: " << cell_index[0] << ", "<<cell_index[1] <<"," <<cell_index[2] << " prob: " << iterator_probability<<"\n";
  }

  cartographer::mapping::scan_matching::InterpolatedProbabilityGrid interpolated_grid_(hybrid_grid_);
   Eigen::Vector3f point(-7.f, 3.f, 1.f);
  for (int i=1; i<20; i++){
	   Eigen::Vector3f point2 = point + Eigen::Vector3f((float) 0.2*i, 0.f, 0.f);

	   std::cout<< "points " << point2[0] << "," << point2[1] <<"," << point2[2] << " cell index: " << hybrid_grid_.GetCellIndex(point2)[0] <<", " <<  hybrid_grid_.GetCellIndex(point2)[1] <<", " <<  hybrid_grid_.GetCellIndex(point2)[2] <<" prob: " <<hybrid_grid_.GetProbability(hybrid_grid_.GetCellIndex(point2))<<"\n";
	   std::cout<< "interpolated: " << interpolated_grid_.GetInterpolatedValue((double) point2[0], (double)point2[1], (double)point2[2])<<"\n";
  }



}
TEST(HybridGridTest, GetProbability) {
  HybridGrid hybrid_grid(1.f);

  hybrid_grid.SetProbability(
      hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 1.f, 1.f)),
      kMaxProbability);
  EXPECT_NEAR(hybrid_grid.GetProbability(
                  hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 1.f, 1.f))),
              kMaxProbability, 1e-6);
  for (const Eigen::Array3i& index :
       {hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 2.f, 1.f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(1.f, 1.f, 1.f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(1.f, 2.f, 1.f))}) {
    EXPECT_FALSE(hybrid_grid.IsKnown(index));
  }
}

TEST(HybridGridTest, GetIntensity) {
  IntensityHybridGrid hybrid_grid(1.f);
  const Eigen::Array3i cell_index =
      hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 1.f, 1.f));
  const float intensity = 58.0f;

  EXPECT_NEAR(hybrid_grid.GetIntensity(cell_index), 0.0f, 1e-9);
  hybrid_grid.AddIntensity(cell_index, intensity);
  EXPECT_NEAR(hybrid_grid.GetIntensity(cell_index), intensity, 1e-9);
  for (const Eigen::Array3i& index :
       {hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 2.f, 1.f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(1.f, 1.f, 1.f)),
        hybrid_grid.GetCellIndex(Eigen::Vector3f(1.f, 2.f, 1.f))}) {
    EXPECT_NEAR(hybrid_grid.GetIntensity(index), 0.0f, 1e-9);
  }
}

MATCHER_P(AllCwiseEqual, index, "") { return (arg == index).all(); }

TEST(HybridGridTest, GetCellIndex) {
  HybridGrid hybrid_grid(2.f);

  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 0.f, 0.f)),
              AllCwiseEqual(Eigen::Array3i(0, 0, 0)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(0.f, 26.f, 10.f)),
              AllCwiseEqual(Eigen::Array3i(0, 13, 5)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(14.f, 0.f, 10.f)),
              AllCwiseEqual(Eigen::Array3i(7, 0, 5)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(14.f, 26.f, 0.f)),
              AllCwiseEqual(Eigen::Array3i(7, 13, 0)));

  // Check around the origin.
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(8.5f, 11.5f, 0.5f)),
              AllCwiseEqual(Eigen::Array3i(4, 6, 0)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(7.5f, 12.5f, 1.5f)),
              AllCwiseEqual(Eigen::Array3i(4, 6, 1)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(6.5f, 14.5f, 2.5f)),
              AllCwiseEqual(Eigen::Array3i(3, 7, 1)));
  EXPECT_THAT(hybrid_grid.GetCellIndex(Eigen::Vector3f(5.5f, 13.5f, 3.5f)),
              AllCwiseEqual(Eigen::Array3i(3, 7, 2)));
}

TEST(HybridGridTest, GetCenterOfCell) {
  HybridGrid hybrid_grid(2.f);

  const Eigen::Array3i index(3, 2, 1);
  const Eigen::Vector3f center = hybrid_grid.GetCenterOfCell(index);
  EXPECT_NEAR(6.f, center.x(), 1e-6);
  EXPECT_NEAR(4.f, center.y(), 1e-6);
  EXPECT_NEAR(2.f, center.z(), 1e-6);
  EXPECT_THAT(hybrid_grid.GetCellIndex(center), AllCwiseEqual(index));
}

class RandomHybridGridTest : public ::testing::Test {
 public:
  RandomHybridGridTest() : hybrid_grid_(2.f), values_() {
    std::mt19937 rng(1285120005);
    std::uniform_real_distribution<float> value_distribution(kMinProbability,
                                                             kMaxProbability);
    std::uniform_int_distribution<int> xyz_distribution(-30, 29);
    for (int i = 0; i < 10; ++i) {
      const auto x = xyz_distribution(rng);
      const auto y = xyz_distribution(rng);
      const auto z = xyz_distribution(rng);
      values_.emplace(std::make_tuple(x, y, z), value_distribution(rng));
    }

    for (const auto& pair : values_) {
      const Eigen::Array3i cell_index(std::get<0>(pair.first),
                                      std::get<1>(pair.first),
                                      std::get<2>(pair.first));
      hybrid_grid_.SetProbability(cell_index, pair.second);
    }
  }

 protected:
  HybridGrid hybrid_grid_;
  using ValueMap = std::map<std::tuple<int, int, int>, float>;
  ValueMap values_;
};

TEST_F(RandomHybridGridTest, TestIteration) {
  for (auto it = HybridGrid::Iterator(hybrid_grid_); !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const float iterator_probability = ValueToProbability(it.GetValue());
    std::cout << "cell center: " << hybrid_grid_.GetCenterOfCell(cell_index)<<"\n";
    std::cout<<"cell index: " << cell_index[0] << ", "<<cell_index[1] <<"," <<cell_index[2] << " prob: " << iterator_probability<<"\n";
    EXPECT_EQ(iterator_probability, hybrid_grid_.GetProbability(cell_index));
    const std::tuple<int, int, int> key =
        std::make_tuple(cell_index[0], cell_index[1], cell_index[2]);
    EXPECT_TRUE(values_.count(key));
    EXPECT_NEAR(values_[key], iterator_probability, 1e-4);
    values_.erase(key);
  }

  // Test that range based loop is equivalent to using the iterator.
  auto it = HybridGrid::Iterator(hybrid_grid_);
  for (const auto& cell : hybrid_grid_) {
    ASSERT_FALSE(it.Done());
    EXPECT_THAT(cell.first, AllCwiseEqual(it.GetCellIndex()));
    EXPECT_EQ(cell.second, it.GetValue());
    it.Next();
  }

  // Now 'values_' must not contain values.
  for (const auto& pair : values_) {
    const Eigen::Array3i cell_index(std::get<0>(pair.first),
                                    std::get<1>(pair.first),
                                    std::get<2>(pair.first));
    ADD_FAILURE() << cell_index << " Probability: " << pair.second;
  }
}

TEST_F(RandomHybridGridTest, ToProto) {
  const auto proto = hybrid_grid_.ToProto();
  EXPECT_EQ(hybrid_grid_.resolution(), proto.resolution());
  ASSERT_EQ(proto.x_indices_size(), proto.y_indices_size());
  ASSERT_EQ(proto.x_indices_size(), proto.z_indices_size());
  ASSERT_EQ(proto.x_indices_size(), proto.values_size());

  ValueMap proto_map;
  for (int i = 0; i < proto.x_indices_size(); ++i) {
    proto_map[std::make_tuple(proto.x_indices(i), proto.y_indices(i),
                              proto.z_indices(i))] = proto.values(i);
  }

  // Get hybrid_grid_ into the same format.
  ValueMap hybrid_grid_map;
  for (const auto i : hybrid_grid_) {
    hybrid_grid_map[std::make_tuple(i.first.x(), i.first.y(), i.first.z())] =
        i.second;
  }

  EXPECT_EQ(proto_map, hybrid_grid_map);
}

struct EigenComparator {
  bool operator()(const Eigen::Vector3i& lhs,
                  const Eigen::Vector3i& rhs) const {
    return std::forward_as_tuple(lhs.x(), lhs.y(), lhs.z()) <
           std::forward_as_tuple(rhs.x(), rhs.y(), rhs.z());
  }
};

TEST_F(RandomHybridGridTest, FromProto) {
  const HybridGrid constructed_grid(hybrid_grid_.ToProto());

  std::map<Eigen::Vector3i, float, EigenComparator> member_map;
  for (const auto& cell : hybrid_grid_) {
    member_map.insert(cell);
  }

  std::map<Eigen::Vector3i, float, EigenComparator> constructed_map;
  for (const auto& cell : constructed_grid) {
    constructed_map.insert(cell);
  }

  EXPECT_EQ(member_map, constructed_map);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer

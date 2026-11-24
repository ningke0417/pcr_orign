#include <gtest/gtest.h>
#include <optimizer/poly_traj_optimizer.h>
#include <limits>
#include <numeric>

class PlanningMapTestAccess
{
public:
  static GridMap::Ptr makeMap()
  {
    GridMap::Ptr map(new GridMap);
    map->mp_.map_origin_ = Eigen::Vector3d(-5, -5, 0);
    map->mp_.map_size_ = Eigen::Vector3d(10, 10, 3);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.resolution_ = 0.1;
    map->mp_.resolution_inv_ = 10.0;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(100, 100, 30);
    map->md_.occupancy_buffer_inflate_.assign(300000, 0);
    map->md_.distance_buffer_all_.assign(300000, 10000.0);
    return map;
  }
};

namespace {
poly_traj::CoefficientMat constant(const Eigen::Vector3d &point)
{
  auto coeff = poly_traj::CoefficientMat::Zero().eval();
  coeff.col(5) = point;
  return coeff;
}
const Eigen::Vector3d lower(-5, -5, 0.0001), upper(5, 5, 2.9999);

TEST(TrajectoryBounds, AcceptsConstantAndPlanarMotion)
{
  auto coeff = constant(Eigen::Vector3d(-1, 0, 1.5));
  coeff(0, 4) = 2;
  EXPECT_TRUE(poly_traj::Trajectory({1.0}, {coeff}).checkPositionBounds(lower, upper));
}

TEST(TrajectoryBounds, RejectsExcursionBetweenTenMillisecondSamples)
{
  auto coeff = constant(Eigen::Vector3d(0, 0, 1.5));
  coeff(2, 3) = -80000;
  coeff(2, 4) = 800; // z(0)=z(.01)=1.5, z(.005)=3.5
  poly_traj::Trajectory trajectory({0.01}, {coeff});
  ASSERT_NEAR(trajectory.getPos(0.01).z(), 1.5, 1e-10);
  double time = -1;
  EXPECT_FALSE(trajectory.checkPositionBounds(lower, upper, &time));
  EXPECT_NEAR(time, 0.005, 1e-8);
}

TEST(TrajectoryBounds, RejectsNegativeHeightAndTailExcursion)
{
  auto safe = constant(Eigen::Vector3d(0, 0, 1.5));
  auto tail = safe;
  tail(2, 4) = -2;
  double time = 0;
  EXPECT_FALSE(poly_traj::Trajectory({2.0, 1.0}, {safe, tail})
      .checkPositionBounds(lower, upper, &time));
  EXPECT_GE(time, 2.0);
}

TEST(TrajectoryBounds, RejectsNonFiniteAndEmptyTrajectories)
{
  auto coeff = constant(Eigen::Vector3d(0, 0, 1.5));
  coeff(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(poly_traj::Trajectory({1.0}, {coeff}).checkPositionBounds(lower, upper));
  EXPECT_FALSE(poly_traj::Trajectory().checkPositionBounds(lower, upper));
}

TEST(TrajectoryBounds, ChecksAllAxesAndQuinticInteriorExtrema)
{
  for (int axis = 0; axis < 3; ++axis)
  {
    auto coeff = constant(Eigen::Vector3d(0, 0, 1.5));
    // 100*t^2*(1-t)^2*(1+t): stationary endpoints, quartic derivative.
    coeff(axis, 0) = 100;
    coeff(axis, 1) = -100;
    coeff(axis, 2) = -100;
    coeff(axis, 3) = 100;
    EXPECT_FALSE(poly_traj::Trajectory({1.0}, {coeff}).checkPositionBounds(lower, upper));
  }
}

TEST(MapBounds, RejectsNaNAndReturnsInwardDistanceGradient)
{
  auto map = PlanningMapTestAccess::makeMap();
  Eigen::Vector3d invalid(0, 0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(map->isInMap(invalid));
  EXPECT_EQ(map->getInflateOccupancy(invalid), -1);
  EXPECT_EQ(map->getDistance(Eigen::Vector3d(0, 0, -0.2)), -1.0);
  double distance;
  Eigen::Vector3d gradient;
  map->evaluateEDTWithGrad(Eigen::Vector3d(0, 0, -0.2), distance, gradient);
  EXPECT_LT(distance, 0);
  EXPECT_GT(gradient.z(), 0);
  map->evaluateEDTWithGrad(Eigen::Vector3d(6, 0, 1.5), distance, gradient);
  EXPECT_LT(distance, 0);
  EXPECT_LT(gradient.x(), 0);
}
} // namespace

namespace ego_planner {
class PolyTrajOptimizerTest : public testing::Test
{
protected:
  PolyTrajOptimizer optimizer;
  SwarmTrajData peers;
  std::vector<Eigen::Vector3d> desired, predicted;
  void SetUp() override
  {
    optimizer.grid_map_ = PlanningMapTestAccess::makeMap();
    optimizer.drone_id_ = 1;
    optimizer.fixed_formation_z_ = 1.5;
    optimizer.is_opt_formation_z_ = false;
    optimizer.size_swarm = optimizer.graph_size = optimizer.formation_size_all = 8;
    optimizer.formation_size_ = 8;
    optimizer.id_in_group = 1;
    optimizer.use_icp_ = true;
    optimizer.enable_fix_step_ = true;
    optimizer.time_cps_.sampling_time_step = 0.2;
    optimizer.time_cps_.enable_decouple_swarm_graph = true;
    optimizer.cps_num_prePiece_ = 4;
    optimizer.max_vel_ = optimizer.max_vel2_ = 0.5;
    optimizer.max_acc_ = 6;
    optimizer.wei_obs_ = optimizer.wei_swarm_ = optimizer.wei_feas_ = 10000;
    optimizer.wei_graph_ = 300;
    optimizer.wei_time_ = 80;
    optimizer.wei_smooth_ = 1;
    optimizer.obs_clearance_ = 0.2;
    optimizer.swarm_clearance_ = 0.5;
    optimizer.max_trajectory_duration_ = 45;
    optimizer.sparse_id.resize(8);
    std::iota(optimizer.sparse_id.begin(), optimizer.sparse_id.end(), 0);
    for (int i = 0; i < 8; ++i)
    {
      Eigen::Vector3d slot(i < 4 ? 0.5 : -0.5, i % 4 - 1.5, 0);
      optimizer.swarm_des_opt.push_back(slot);
      if (i != 1)
      {
        desired.push_back(slot);
        predicted.push_back(slot + Eigen::Vector3d(0, 0, 2));
      }
    }
  }
  bool optimize(bool formation)
  {
    Eigen::Matrix3d head, tail;
    head << Eigen::Vector3d(-1, 0, 1.5), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    tail << Eigen::Vector3d(1, 0, 1.5), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    // The production A* frontend always provides at least two pieces. The
    // single-piece MinJerkOpt shortcut does not initialize gradient buffers.
    Eigen::MatrixXd points(3, 1), output;
    points.col(0) = Eigen::Vector3d(0, 0, 1.5);
    return optimizer.optimizeTrajectory_lbfgs(head, tail, points,
        Eigen::VectorXd::Constant(2, 4.0), output, formation);
  }
  void degeneratePeers()
  {
    peers.resize(8);
    for (int i = 0; i < 8; ++i)
    {
      peers[i].drone_id = i;
      peers[i].start_time = ros::Time::now().toSec();
      peers[i].duration = 30;
      peers[i].traj = poly_traj::Trajectory({30.0}, {constant(Eigen::Vector3d(2, 2, 1.5))});
    }
    optimizer.swarm_trajs_ = &peers;
  }
  size_t cachedReferences() const { return optimizer.time_cps_.icp_pos.size(); }
  void identityCoordinates()
  {
    optimizer.is_opt_formation_z_ = true;
    desired.clear(); predicted.clear();
    for (int i = 0; i < 8; ++i)
    {
      optimizer.swarm_des_opt[i].z() = 1.5;
      if (i != 1)
      {
        desired.push_back(optimizer.swarm_des_opt[i]);
        predicted.push_back(optimizer.swarm_des_opt[i]);
      }
    }
  }
};

TEST_F(PolyTrajOptimizerTest, SuccessfulRegistrationHonorsFixedReferenceHeight)
{
  Eigen::Vector3d reference(99, 99, 99);
  ASSERT_TRUE(optimizer.pointCloudRegistration(desired, predicted, false,
      Eigen::Vector3d::Zero(), reference));
  EXPECT_NEAR(reference.x(), 0.5, 1e-4);
  EXPECT_NEAR(reference.y(), -0.5, 1e-4);
  EXPECT_DOUBLE_EQ(reference.z(), 1.5);
}

TEST_F(PolyTrajOptimizerTest, InvalidInputDoesNotReturnAnIdentityFallbackPoint)
{
  Eigen::Vector3d reference(9, 8, 7);
  predicted[0].z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(optimizer.pointCloudRegistration(desired, predicted, false,
      Eigen::Vector3d::Zero(), reference));
  EXPECT_EQ(reference, Eigen::Vector3d(9, 8, 7));
  EXPECT_FALSE(optimizer.pointCloudRegistration({}, {}, false,
      Eigen::Vector3d::Zero(), reference));
}

TEST_F(PolyTrajOptimizerTest, AcceptsLegitimateIdentityRegistration)
{
  identityCoordinates();
  Eigen::Vector3d reference;
  ASSERT_TRUE(optimizer.pointCloudRegistration(desired, predicted, false,
      Eigen::Vector3d::Zero(), reference));
  EXPECT_NEAR((reference - Eigen::Vector3d(0.5, -0.5, 1.5)).norm(), 0, 1e-4);
}

TEST_F(PolyTrajOptimizerTest, RejectsRegisteredReferenceOutsideMap)
{
  for (auto &point : predicted) point.x() += 10;
  Eigen::Vector3d reference(9, 8, 7);
  EXPECT_FALSE(optimizer.pointCloudRegistration(desired, predicted, false,
      Eigen::Vector3d::Zero(), reference));
  EXPECT_EQ(reference, Eigen::Vector3d(9, 8, 7));
}

TEST_F(PolyTrajOptimizerTest, ChecksFloat32PublishedTrajectoryAtBoundary)
{
  auto coeff = constant(Eigen::Vector3d(0, 0, 0.0001 + 1e-13));
  poly_traj::Trajectory trajectory({1.0}, {coeff});
  ASSERT_TRUE(trajectory.checkPositionBounds(lower, upper));
  EXPECT_FALSE(optimizer.trajectoryWithinMap(trajectory));
}

TEST_F(PolyTrajOptimizerTest, FailedRegistrationCanRetryWithoutFormationAndWithoutStaleReferences)
{
  degeneratePeers();
  EXPECT_FALSE(optimize(true));
  EXPECT_EQ(optimizer.getLastFailure(), PolyTrajOptimizer::FailureReason::REGISTRATION_FAILED);
  EXPECT_EQ(cachedReferences(), 0u);
  ASSERT_TRUE(optimize(false));
  EXPECT_EQ(optimizer.getLastFailure(), PolyTrajOptimizer::FailureReason::NONE);
  EXPECT_TRUE(optimizer.trajectoryWithinMap(optimizer.getMinJerkOptPtr()->getTraj()));
}
} // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "planning_validation", ros::init_options::NoSigintHandler);
  return RUN_ALL_TESTS();
}

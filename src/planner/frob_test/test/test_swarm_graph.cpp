#include <gtest/gtest.h>
#include <frob_test/swarm_graph.hpp>

namespace {

std::vector<Eigen::Vector3d> rectangle() {
  std::vector<Eigen::Vector3d> points;
  for (int row = 0; row < 2; ++row)
    for (int col = 0; col < 4; ++col)
      points.emplace_back(0.5 - row, -1.5 + col, 0.0);
  return points;
}

void configure(SwarmGraph& graph, const std::vector<Eigen::Vector3d>& points) {
  std::vector<int> in, out;
  for (int i = 0; i < static_cast<int>(points.size()); ++i)
    for (int j = 0; j < i; ++j) {
      in.push_back(i);
      out.push_back(j);
    }
  ASSERT_TRUE(graph.setDesiredForm(points, in, out));
}

TEST(SwarmGraph, DesiredRectangleAndTranslationHaveZeroError) {
  auto points = rectangle();
  SwarmGraph graph;
  configure(graph, points);
  for (int pass = 0; pass < 2; ++pass) {
    ASSERT_TRUE(graph.updateGraph(points));
    double cost = -1.0;
    ASSERT_TRUE(graph.calcFNorm2(cost));
    EXPECT_NEAR(cost, 0.0, 1e-12);
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3d gradient = graph.getGrad(i);
      EXPECT_TRUE(gradient.allFinite());
      EXPECT_NEAR(gradient.norm(), 0.0, 1e-10);
    }
    for (auto& point : points)
      point += Eigen::Vector3d(-6.0, 0.0, 1.5);
  }
}

TEST(SwarmGraph, PartialAndFullUpdatesAgreeForPerturbedRectangle) {
  auto points = rectangle();
  SwarmGraph graph;
  configure(graph, points);
  points[3] += Eigen::Vector3d(0.2, -0.1, 0.0);
  Eigen::Vector3d partial;
  ASSERT_TRUE(graph.updatePartGraphAndGetGrad(3, points, partial));
  ASSERT_TRUE(graph.updateGraph(points));
  EXPECT_TRUE(partial.allFinite());
  EXPECT_NEAR((partial - graph.getGrad(3)).norm(), 0.0, 1e-10);
  double cost = 0.0;
  ASSERT_TRUE(graph.calcFNorm2(cost));
  EXPECT_GT(cost, 0.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

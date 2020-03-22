//
// Created by jack on 2/21/20.
//

#include <stdio.h>

#include "gtest/gtest.h"
#include "eigen3/Eigen/Dense"

#include "../src/CImg.h"
#include "../src/line_extraction.h"

using Eigen::Vector2f;
using namespace VectorMaps;

TEST(LineSegmentTest, OnEndpoint) {
  Vector2f start_point (0, 0);
  Vector2f end_point(2, 3);
  LineSegment line_seg(start_point, end_point);
  ASSERT_DOUBLE_EQ(line_seg.DistanceToLineSegment(start_point), 0);
  ASSERT_DOUBLE_EQ(line_seg.DistanceToLineSegment(end_point), 0);
}

TEST(LineSegmentTest, OnLine) {
  Vector2f start_point (0, 0);
  Vector2f end_point(2, 3);
  LineSegment line_seg(start_point, end_point);
  Vector2f test_point(1, 3.0/2);
  ASSERT_DOUBLE_EQ(line_seg.DistanceToLineSegment(test_point), 0);
}

TEST(LineSegmentTest, OffLine) {
  Vector2f start_point (0, 0);
  Vector2f end_point(2, 3);
  LineSegment line_seg(start_point, end_point);
  Vector2f test_point(0, 1);
  end_point.normalize();
  double angle_between = acos(test_point.dot(end_point));
  ASSERT_FLOAT_EQ(line_seg.DistanceToLineSegment(test_point),
                                                 sin(angle_between));
}


//
// Created by jack on 2/21/20.
//

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

TEST(ExtractLinesTest, TestExtractLinesOnBasicScan) {
  vector<Vector2f> points;
  for (double i = 0; i < 2; i += 0.05) {
    points.emplace_back(i, 2);
    points.emplace_back(i, 1);
  }
  vector<LineSegment> lines = ExtractLines(points);
  cimg_library::CImgDisplay display_1;
  cimg_library::CImg<double> lines_image(200, 200, 1, 1);
  double line_color[] = {1.0};
  for (const LineSegment& line : lines) {
    lines_image.draw_line(std::floor(line.start_point.x() / 0.05),
                          std::floor(line.start_point.y() / 0.05),
                          std::floor(line.end_point.x() / 0.05),
                          std::floor(line.end_point.y() / 0.05),
                          line_color);
  }
  display_1.display(lines_image);
  while(!display_1.is_closed()) {
    display_1.wait();
  }
}
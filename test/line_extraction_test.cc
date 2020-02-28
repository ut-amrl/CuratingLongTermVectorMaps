//
// Created by jack on 2/21/20.
//

#include <stdio.h>

#include "gtest/gtest.h"
#include "eigen3/Eigen/Dense"

#include "../src/CImg.h"
#include "../src/line_extraction.h"
#include "../src/CorrelativeScanMatcher.h"

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
  display_1.display(lines_image.resize_doubleXY().resize_doubleXY());
//  while(!display_1.is_closed()) {
//    display_1.wait();
//  }
}

TEST(ExtractLinesTest, TestExtractLinesOnRealScan) {
  vector<Vector2f> points;
  CorrelativeScanMatcher scan_matcher(4, 2, 0.3, 0.03);
  // Load points in from file.
  FILE *points_file = fopen("data/scan_points.txt", "r");
  float x, y;
  while (fscanf(points_file, "%f %f\n", &x, &y) == 2) {
    points.emplace_back(x, y);
    std::cout << points.size() << std::endl;
  }
  fclose(points_file);
  std::cout << "Loaded in " << points.size() << " points" << std::endl;
  double min_x = INFINITY;
  double max_x = -INFINITY;
  double min_y = INFINITY;
  double max_y = -INFINITY;
  for (const Vector2f& point : points) {
    min_x = std::min(min_x, static_cast<double>(point.x()));
    max_x = std::max(max_x, static_cast<double>(point.x()));
    min_y = std::min(min_y, static_cast<double>(point.y()));
    max_y = std::max(max_y, static_cast<double>(point.y()));
  }
  vector<LineSegment> lines = ExtractLines(points);
  cimg_library::CImgDisplay display_1;
  double line_color[] = {1.0};
  Vector2f shift(0, 0);
  if (min_x < 0) {
    shift.x() = abs(min_x);
  }
  if (min_y < 0) {
    shift.y() = abs(min_y);
  }
  double points_width = max_x - min_x;
  double points_height = max_y - min_y;
  std::cout << "points_width: " << points_width << std::endl;
  cimg_library::CImg<double> lines_image(std::ceil(points_width) * 10, std::ceil(points_height) * 10, 1, 1, 0);
  Eigen::Matrix2f scaling_mat;
  //scaling_mat << 200 / points_width, 0, 0, 200 / points_height;
  for (const LineSegment& line : lines) {
    Vector2f line_start = (shift + line.start_point);
    Vector2f line_end = (shift + line.end_point);
    std::cout << "Line goes from (" << line_start.x() << ", " << line_start.y() << ") to (" << line_end.x() << ", " << line_end.y() << ")" << std::endl;
    lines_image.draw_line(line_start.x() * 10, line_start.y() * 10, line_end.x() * 10, line_end.y() * 10, line_color);
  }
  cimg_library::CImgDisplay display_2;
//  vector<Vector2f> transformed_points;
//  for (const Vector2f& p : points) {
//    transformed_points.push_back(scaling_mat * (shift + p));
//  }
  LookupTable trans_points = scan_matcher.GetLookupTableHighRes(points);
  display_2.display(trans_points.GetDebugImage().resize_doubleXY().resize_doubleXY());
  display_1.display(lines_image.resize_doubleXY().resize_doubleXY());
  std::cout << "Finished" << std::endl;
  while(!display_1.is_closed()) {
    display_1.wait();
  }
}
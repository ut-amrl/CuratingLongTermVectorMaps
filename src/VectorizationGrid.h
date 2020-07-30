#include <vector>
#include <map>
#include "Eigen/Dense"
#include "line_extraction.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using namespace std;

pair<pair<float, float>, pair<float, float>> GetBounds(const vector<Vector2f>& points) {
    assert(points.size() > 0);
    pair<float, float> xBounds = make_pair(points[0].x(), points[0].x());
    pair<float, float> yBounds = make_pair(points[0].y(), points[0].y());

    for(auto& p : points) {
        if (p.x() > xBounds.second) {
            xBounds.second = p.x();
        }
        if (p.x() < xBounds.first) {
            xBounds.first = p.x();
        }
        if (p.y() > yBounds.second) {
            yBounds.second = p.y();
        }
        if (p.y() < yBounds.first) {
            yBounds.first = p.y();
        }
    }

    return make_pair(xBounds, yBounds);
}

struct GridCell;

struct GridPoint {
    Vector2f point;
    GridCell* cell;

    GridPoint(Vector2f p, GridCell* c) : cell(c) {
        point = p;
    }
};

struct GridCell {
    vector<GridPoint> points;
    pair<int, int> key;

    GridCell(pair<int, int> k) {
        points = vector<GridPoint>();
        key = k;
    }

    bool RemovePoint(const GridPoint& p) {
        for(size_t i = 0;i < points.size(); i++) {
            if (points[i].point == p.point) {
                points.erase(points.begin() + i);
                break;
            }
        }

        if(points.size() == 0) {
            return false;
        }
        return true;
    }

    bool RemovePoint(const Vector2f& p) {
        for(size_t i = 0;i < points.size(); i++) {
            if (points[i].point == p) {
                points.erase(points.begin() + i);
                break;
            }
        }

        if(points.size() == 0) {
            return false;
        }
        return true;
    }

    void AddPoint(GridPoint p) {
        points.push_back(p);
    }

    GridPoint& RandomPoint() {
        return points[rand() % points.size()];
    }
};

class VectorizationGrid {
    float neighborhood_size_;
    float neighborhood_growth_size_;
    map<pair<int, int>, GridCell> grid_;
    pair<pair<float, float>, pair<float, float>> bounds_;
    size_t count_;

    public:
        VectorizationGrid(const vector<Vector2f>& points, float neighborhood_size, float neighborhood_growth_size) {
            bounds_ = GetBounds(points);
            neighborhood_size_ = neighborhood_size;
            neighborhood_growth_size_ = neighborhood_growth_size;

            for(auto& p : points) {
                auto key = CoordToGridCell(p);
                if (grid_.find(key) == grid_.end()) {
                    grid_.insert(make_pair(key, GridCell(key)));
                }
                grid_.at(key).AddPoint(GridPoint(p, &grid_.at(key)));
            }

            count_ = points.size();
        }

        void ConsumePoint(const GridPoint& point) {
            bool keepCell = point.cell->RemovePoint(point);
            if (!keepCell) {
                grid_.erase(point.cell->key);
            }
            count_--;
        }

        size_t size() {
            return count_;
        }

        GridPoint& RandomPoint() {
            auto it = grid_.begin();
            advance(it, rand() % grid_.size());
            return it->second.RandomPoint();
        }

        std::vector<Vector2f> RandomNeighborhood() {
            vector<Vector2f> neighborhood;

            while(neighborhood.size() <= 1 && count_ > 0) {
                neighborhood.clear();
                GridPoint& p = RandomPoint();
                for(int gridX = p.cell->key.first - 1; gridX <= p.cell->key.first + 1; gridX++) {
                    for(int gridY = p.cell->key.second - 1; gridY <= p.cell->key.second + 1; gridY++) {
                        auto iter = grid_.find(make_pair(gridX, gridY));
                        if (iter != grid_.end()) {
                            for (auto& gp : iter->second.points) {
                                if ((gp.point - p.point).norm() < neighborhood_size_) {
                                    neighborhood.push_back(gp.point);
                                }
                            }
                        }
                    }
                }

                if (neighborhood.size() <= 1) {
                    ConsumePoint(p);
                }
            }

            return neighborhood;
        }

        void RemovePointByCoord(const Vector2f& coord) {
            auto key = CoordToGridCell(coord);
            bool keepCell = grid_.at(key).RemovePoint(coord);
            if (!keepCell) {
                grid_.erase(key);
            }
            count_--;
        }

        vector<Vector2f> NeighborhoodAroundLine(const VectorMaps::LineSegment& line) {
            // look in all cells within line_length of either of the endpoints.
            float length = (line.end_point - line.start_point).norm();

            int num_cells = int(length / neighborhood_size_);

            auto start_key = CoordToGridCell(line.start_point);
            auto end_key = CoordToGridCell(line.end_point);

            int min_x = min(start_key.first, end_key.first) - num_cells;
            int max_x = max(start_key.first, end_key.first) + num_cells;
            int min_y = min(start_key.second, end_key.second) - num_cells;
            int max_y = max(start_key.second, end_key.second) + num_cells;

            vector<Vector2f> neighborhood;

            for(int gridX = min_x; gridX <= max_x; gridX++) {
                for(int gridY = min_y; gridY <= max_y; gridY++) {
                    auto iter = grid_.find(make_pair(gridX, gridY));
                    if (iter != grid_.end()) {
                        for (auto& gp : iter->second.points) {
                            double dist = line.DistanceToLineSegment(gp.point);
                            if (dist <= neighborhood_growth_size_) {
                                neighborhood.push_back(gp.point);
                            }
                        }
                    }
                }
            }

            return neighborhood;
        }

    
    protected:
        pair<int, int> CoordToGridCell(const Vector2f& coord) {
            if (coord.x() < bounds_.first.first || coord.x() > bounds_.first.second || coord.y() < bounds_.second.first || coord.y() > bounds_.second.second) {
                printf("Coordinate out of bounds for grid! %f %f\n", coord.x(), coord.y());
                return make_pair(-1, -1);
            }
            int x = int(floor((coord.x() - bounds_.first.first) / neighborhood_size_));
            int y = int(floor((coord.y() - bounds_.second.first) / neighborhood_size_));
            return make_pair(x, y);
        }
};
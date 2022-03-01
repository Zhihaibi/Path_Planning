#ifndef A_STAR_H
#define A_STAR_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <boost/range/combine.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <map>

#include "node.h"
#include "matplotlibcpp.h"

class AstarPathFinder{

    private:

    protected:
        int max_x_, max_y_, min_x_, min_y_;
        int x_width_, y_width_;
        int goal_idx_;

        std::vector<double> start_end_x; // for plot
        std::vector<double> start_end_y;

        std::vector<double> ox_, oy_;
        std::map<std::pair<int,int>, bool> obs_map_;

        double grid_resolution_ = 1;
        double robot_radius_ = 2;
        std::vector<std::vector<double>> motion_;

        std::map<double, GridNodePtr> open_set_;
        std::map<double, GridNodePtr> close_set_;

        GridNodePtr terminate_ptr_;

    public:
        AstarPathFinder() = default;
        ~AstarPathFinder() = default;

        void InitMap();
        void AstarPlanning(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
        int Coor2GridIndex(Eigen::Vector2d pt);
        double GetHue(GridNodePtr node1, GridNodePtr node2);
        void GetMotion();

        bool IsObstacle(Eigen::Vector2d & pt);
        void PlotPath(std::vector<double> pathx, std::vector<double> pathy);

        std::tuple<std::vector<double>, std::vector<double>> GetPath();
        std::vector<Eigen::Vector2d> GetVisitedNodes();
};
#endif //A_STAR_H

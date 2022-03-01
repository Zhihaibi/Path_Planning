//###################################################
//                   A* ALGORITHM
//  AUTHOR:   Zhihai Bi
//  WRITTEN:  2022-02-28
//###################################################

#include "a_star.h"
using namespace std;
namespace plt = matplotlibcpp;

void AstarPathFinder::InitMap(){
    // Initial the boundary and obstacle
    for(int i = 0; i < 60; i++){
        ox_.push_back(i);
        oy_.push_back(0);
    }
    for(int i = 0; i < 60; i++){
        ox_.push_back(60);
        oy_.push_back(i);
    }
    for(int i = 0; i < 61; i++){
        ox_.push_back(i);
        oy_.push_back(60);
    }
    for(int i = 0; i < 61; i++){
        ox_.push_back(0);
        oy_.push_back(i);
    }
    for(int i = 0; i < 40; i++){
        ox_.push_back(20);
        oy_.push_back(i);
    }
    for(int i = 0; i < 40; i++){
        ox_.push_back(40);
        oy_.push_back(60-i);
    }

    // Map parameters
    max_x_ = round(*std::max_element(ox_.begin(),ox_.end()));
    max_y_ = round(*std::max_element(oy_.begin(),oy_.end()));
    min_x_ = round(*std::min_element(ox_.begin(),ox_.end()));
    min_y_ = round(*std::min_element(oy_.begin(),oy_.end()));
    x_width_ = max_x_ -  min_x_;
    y_width_ = max_y_ -  min_y_;

    // Calculate obstacle map
    for(int i = min_x_; i < x_width_; i++)
        for(int j = min_y_; j < y_width_; j++)
            obs_map_[std::make_pair(i,j)] = false;

    for(int i = min_x_; i < x_width_; i++)
        for(int j = min_y_; j < y_width_; j++)
            for(auto tup : boost::combine(ox_, oy_)){
                double x, y;
                boost::tie(x, y) = tup;
                if(sqrt(pow(x-i, 2) + pow(y-j, 2)) <= robot_radius_ / grid_resolution_){
                    obs_map_[std::make_pair(i,j)] = true;
                    break;
                }
            }
}


int AstarPathFinder::Coor2GridIndex(Eigen::Vector2d pt){
    return (pt(1) - min_y_) * x_width_ + pt(0) - min_x_;
}


double AstarPathFinder::GetHue(GridNodePtr node1, GridNodePtr node2){
    return (node1->coord - node2->coord).norm();  // Euclidean
}


void AstarPathFinder::GetMotion(){
    std::vector<std::vector<double>> motion = {{-1,0}, {-1,1}, {0,1}, {1,1},
                                               {1,0},  {1,-1}, {0,-1}, {-1,-1}};
    motion_ = motion;
}


bool AstarPathFinder::IsObstacle(Eigen::Vector2d & pt) {
    if(pt(0) <= min_x_ || pt(0) >= max_x_ || pt(1) <= min_y_ || pt(1) >= max_y_)
        return true;
    return obs_map_[std::make_pair(pt(0),pt(1))];
}


void AstarPathFinder::AstarPlanning(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt){
    // for plotting only
    start_end_x.push_back(start_pt(0));
    start_end_x.push_back(end_pt(0));
    start_end_y.push_back(start_pt(1));
    start_end_y.push_back(end_pt(1));

    // a star start
    std::cout << "A star search begin!" << std::endl;
    int start_idx = Coor2GridIndex(start_pt);
    int end_idx = Coor2GridIndex(end_pt);
    goal_idx_ = end_idx;

    GridNodePtr node_start_ptr = new GridNode(start_idx, start_pt);
    GridNodePtr node_end_ptr = new GridNode(end_idx, end_pt);

    node_start_ptr->g_score = 0;
    node_start_ptr->f_score = GetHue(node_start_ptr, node_end_ptr);

    open_set_.clear();
    open_set_[start_idx] = node_start_ptr;

    std::multimap<double,int> q_priority;
    q_priority.insert(std::make_pair(node_start_ptr->f_score, node_start_ptr->index));
    GetMotion();

    while(!open_set_.empty()){
        int best_index = q_priority.begin()->second;
        q_priority.erase(q_priority.begin());
        GridNodePtr node_current_ptr = open_set_[best_index];
        open_set_.erase(best_index);
        close_set_[best_index] = node_current_ptr;

        // check if the current node is the goal
        if(node_current_ptr->index == goal_idx_){
            terminate_ptr_ = node_current_ptr;
            std::cout << "A star search finish!" << std::endl;
            return;
        }

        // get the successive(neighbor) nodes
        int new_index;
        Eigen::Vector2d new_coord;
        for(int i = 0; i < motion_.size(); i++){
            new_coord(0) = node_current_ptr->coord(0) + motion_[i][0];
            new_coord(1) = node_current_ptr->coord(1) + motion_[i][1];
            if(IsObstacle(new_coord))
                continue;

            new_index = Coor2GridIndex(new_coord);
            GridNodePtr node_new_ptr = new GridNode(new_index, new_coord);
            node_new_ptr->g_score = node_current_ptr->g_score + hypot(motion_[i][0], motion_[i][1]);
            node_new_ptr->f_score = GetHue(node_new_ptr, node_end_ptr) + node_new_ptr->g_score;

            if(close_set_.find(new_index) == close_set_.end()){  // check if not in the close set
                if(open_set_.find(new_index) != open_set_.end()){  // check if in the open set
                    if(open_set_[new_index]->g_score > node_new_ptr->g_score){
                        open_set_[new_index]->g_score = node_new_ptr->g_score;
                        open_set_[new_index]->parent = node_current_ptr;
                        open_set_[new_index]->f_score = node_new_ptr->f_score;
                        delete node_new_ptr;
                    }
                }
                else{ // not in the close set and not in the open set
                    node_new_ptr->parent = node_current_ptr;
                    open_set_[new_index] = node_new_ptr;
                    q_priority.insert(std::make_pair(node_new_ptr->f_score, node_new_ptr->index));
                }
            }
            else
                delete node_new_ptr;
        }
    }
}


std::tuple<vector<double>, vector<double>> AstarPathFinder::GetPath(){
    GridNodePtr temp_ptr = terminate_ptr_;
    std::vector<double> pathx;
    std::vector<double> pathy;

    while(temp_ptr){
        pathx.push_back(temp_ptr->coord(0));
        pathy.push_back(temp_ptr->coord(1));
        temp_ptr = temp_ptr->parent;
    }

    std::reverse(pathx.begin(), pathx.end());
    std::reverse(pathy.begin(), pathy.end());

    return make_tuple(pathx,pathy);
}


void AstarPathFinder::PlotPath(vector<double> pathx, vector<double> pathy){
    plt::plot(ox_, oy_, "sk");
    plt::plot(pathx, pathy, "-r");
    plt::plot(start_end_x, start_end_y, "sg");
    plt::axis("equal");
    plt::show();
}
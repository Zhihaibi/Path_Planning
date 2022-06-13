#include "dfs.h"
using namespace std;
namespace plt = matplotlibcpp;

void DfsPathFinder::InitMap(){
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


int DfsPathFinder::Coor2GridIndex(Eigen::Vector2d pt){
    return (pt(1) - min_y_) * x_width_ + pt(0) - min_x_;
}


Eigen::Vector2d DfsPathFinder::GridIndex2Coor(int index){
    Eigen::Vector2d pt;
    pt(1) =  int((index-min_x_)/x_width_) + min_y_;
    pt(0) = index - (pt(1)-min_y_)*x_width_ + min_x_;
    return pt;
}

void DfsPathFinder::GetMotion(){
    std::vector<std::vector<double>> motion = {{-1,0}, {-1,1}, {0,1}, {1,1},
                                               {1,0},  {1,-1}, {0,-1}, {-1,-1}};
    motion_ = motion;
}


bool DfsPathFinder::IsObstacle(Eigen::Vector2d & pt) {
    if(pt(0) <= min_x_ || pt(0) >= max_x_ || pt(1) <= min_y_ || pt(1) >= max_y_)
        return true;
    return obs_map_[std::make_pair(pt(0),pt(1))];
}


void DfsPathFinder::Planning(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt){
    // for plotting only
    start_end_x.push_back(start_pt(0));
    start_end_x.push_back(end_pt(0));
    start_end_y.push_back(start_pt(1));
    start_end_y.push_back(end_pt(1));

    // bfs start
    std::cout << "Depth-First Searching begin!" << std::endl;
    int start_idx = Coor2GridIndex(start_pt);
    int end_idx = Coor2GridIndex(end_pt);
    goal_idx_ = end_idx;

    GridNodePtr node_start_ptr = new GridNode(start_idx, start_pt);
    GridNodePtr node_end_ptr = new GridNode(end_idx, end_pt);


    open_set_.clear();
    open_set_[start_idx] = node_start_ptr;  // put the start node in the open set

    std::stack<int> q_normal;
    q_normal.push(node_start_ptr->index);
    GetMotion();

    while(!open_set_.empty()){
        int next_index = q_normal.top();
        q_normal.pop();
        GridNodePtr node_current_ptr = open_set_[next_index];
        open_set_.erase(next_index);
        close_set_.push_back(next_index);

        // check if the current node is the goal
        if(node_current_ptr->index == goal_idx_){
            terminate_ptr_ = node_current_ptr;
            std::cout << "Depth-First Searching finish!" << std::endl;
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

            int iFind = new_index;
            vector<int>::iterator iter = find(close_set_.begin(), close_set_.end(), iFind);

            if(iter == close_set_.end()){  // check if not in the close set
                if(open_set_.find(new_index) == open_set_.end()){  // check if not in the open set
                    GridNodePtr node_new_ptr = new GridNode(new_index, new_coord);
                    node_new_ptr->parent = node_current_ptr;
                    open_set_[new_index] = node_new_ptr;
                    q_normal.push( node_new_ptr->index);
                }
            }
        }
    }
}


std::tuple<vector<double>, vector<double>> DfsPathFinder::GetPath(){
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


void DfsPathFinder::PlotPath(vector<double> pathx, vector<double> pathy){

    std::vector<double> visited_x;
    std::vector<double> visited_y;
    Eigen::Vector2d pt;
    for(int i = 0; i<close_set_.size(); ++i)
    {
        pt = GridIndex2Coor(close_set_[i]);
        visited_x.push_back(pt(0));
        visited_y.push_back(pt(1));

        if(i % 5 == 0){
            plt::clf();
            plt::plot(start_end_x, start_end_y, "sb");
            plt::plot(ox_, oy_, "sk");
            plt::plot(visited_x, visited_y, "sy");  // plot visited nodes
            plt::axis("equal");
            plt::title("Depth-First Searching");
            plt::pause(0.01);
        }
    }

    plt::plot(pathx, pathy, "-r");
    plt::plot(start_end_x, start_end_y, "sb");
    plt::show();
}

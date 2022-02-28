#include "a_star.h"
using namespace std;

int main() {
    // define the start and goal point
    Eigen::Vector2d start_pt = {5,5};
    Eigen::Vector2d end_pt = {55,55};

    clock_t start, end;
    AstarPathFinder astar;
    astar.InitMap();

    start = clock();
    astar.AstarPlanning(start_pt, end_pt); // A star
    end = clock();

    cout << "A star running time:" << double(end-start)/CLOCKS_PER_SEC << "s"<< endl;

    std::vector<double> pathx;
    std::vector<double> pathy;
    tie(pathx,pathy) = astar.GetPath();
    astar.PlotPath(pathx, pathy);

    return 0;
}

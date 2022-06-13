#include "a_star.h"
#include "bfs.h"
#include "dfs.h"

using namespace std;

int main() {
    // define the start and goal point
    Eigen::Vector2d start_pt = {5,5};
    Eigen::Vector2d end_pt = {55,55};

    clock_t start, end;
    BfsPathFinder hn;    // TODO: 1) modified here if you want to change the algorithm
    hn.InitMap();

    start = clock();
    hn.Planning(start_pt, end_pt);
    end = clock();

    cout << "Running time:" << double(end-start)/CLOCKS_PER_SEC << "s"<< endl;

    std::vector<double> pathx;
    std::vector<double> pathy;
    tie(pathx,pathy) = hn.GetPath();
    hn.PlotPath(pathx, pathy);

    return 0;
}

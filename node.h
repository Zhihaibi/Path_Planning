#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <map>

#define inf 1<<20;

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode{
    int id;     // 1 -- > Open set, 0 --> new,  -1 --> Closed set
    Eigen::Vector2d coord;
    int index;

    double g_score, f_score;
    GridNodePtr parent;

    GridNode(int _index, Eigen::Vector2d _coord){
        id = 0;
        index = _index;
        coord = _coord;

        g_score = inf;
        f_score = inf;
        parent = nullptr;
    }
    GridNode() = default;
    ~GridNode() = default;
};

#endif //NODE_H

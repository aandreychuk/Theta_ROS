#ifndef STRUCTS_H
#define STRUCTS_H
#include <vector>

struct Node
{
    int     i, j;
    double  F, g, H;
    Node    *parent;

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
};

struct SearchResult
{
        bool pathfound;
        float pathlength;
        std::vector<Node> hppath;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
        }
};


struct position
{
    float x;
    float y;
    float z;
};

struct orientation
{
    float x;
    float y;
    float z;
    float w;
};

struct origin
{
    position position;
    orientation orientation;
};

struct info
{
    int width;
    int height;
    origin origin;
    float resolution;
};

struct OccupancyGrid
{
    info info;
    int data[];
};

struct pose
{
    position position;
    orientation orientation;
};

struct goalPose
{
    pose pose;
};

#endif // STRUCTS_H

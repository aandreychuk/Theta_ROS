#include "map.h"

Map::Map()
{
    height = -1;
    width = -1;
    start_i = -1;
    start_j = -1;
    goal_i = -1;
    goal_j = -1;
    cellSize = 1;
}

void Map::initialize(const OccupancyGrid &grid)
{
    height = grid.info.height;
    width = grid.info.width;
    cellSize = grid.info.resolution;
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width);
    int k(0);
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            Grid[i][j] = bool(grid.data[k] == -1 || grid.data[k]>=50);
            k++;
        }
    return;
}

void Map::setStartPos(const position &start)
{
    start_i = start.y;
    start_j = start.x;
}

void Map::setGoalPos(const position &goal)
{
    goal_i = goal.y;
    goal_j = goal.x;
}

bool Map::isObstacle(int i, int j) const
{
    if (i < 0 || i >= height)
        return true;

    if (j < 0 || j >= width)
        return true;

    return Grid[i][j];
}

std::list<goalPose> Map::getGoalPoses(SearchResult result)
{
    std::list<goalPose> poses;
    goalPose gPose;
    for(int i = 0; i < result.hppath.size(); i++)
    {
        gPose.pose.position.x = result.hppath[i].j;
        gPose.pose.position.y = result.hppath[i].i;
        gPose.pose.position.z = 0.0;
        if(i + 1 < result.hppath.size())
        {
            gPose.pose.orientation.x = result.hppath[i+1].j;
            gPose.pose.orientation.y = result.hppath[i+1].i;
        }
        else
        {
            gPose.pose.orientation.x = 0.0;
            gPose.pose.orientation.y = 0.0;
        }
        gPose.pose.orientation.z = 0.0;
        gPose.pose.orientation.w = 1.0;
        poses.push_front(gPose);
    }
    return poses;
}

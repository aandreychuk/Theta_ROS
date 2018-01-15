#include "structs.h"
#include "map.h"
#include "theta.h"
#include <iostream>
int main(int argc, char* argv[])
{
    OccupancyGrid grid;
    grid.info.height = 960;
    grid.info.width = 960;
    grid.info.resolution = 0.05;
    grid.info.origin.position.x = 480;
    grid.info.origin.position.y = 480;
    goalPose gPose;
    gPose.pose.position.x = 470;
    gPose.pose.position.y = 480;

    Map map;
    map.initialize(grid);
    map.setStartPos(grid.info.origin.position);
    map.setGoalPos(gPose.pose.position);
    map.setAgentSize(1);

    Theta theta;
    SearchResult result;
    result = theta.startSearch(map);
    std::list<goalPose> poses;

    if(result.pathfound)
         poses = map.getGoalPoses(result);
    return 0;
}


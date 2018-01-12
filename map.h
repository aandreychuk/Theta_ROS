#ifndef MAP_H
#define	MAP_H
#include <vector>
#include <list>
#include "structs.h"
class Map
{
    public:
        Map();
        ~Map(){}
        void initialize(const OccupancyGrid& grid);
        void setStartPos(const position& start);
        void setGoalPos(const position& goal);
        bool isObstacle(int i, int j) const;
        std::list<goalPose> getGoalPoses(SearchResult result);


        int     height, width;
        int     start_i, start_j;
        int     goal_i, goal_j;
        double  cellSize;
        std::vector<std::vector<bool>> Grid;
};

#endif


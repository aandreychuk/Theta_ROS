#ifndef MAP_H
#define	MAP_H
#include <vector>
#include <list>
#include <math.h>
#include "structs.h"
#include <algorithm>

#define CN_EPSILON 1e-7

class Map
{
    public:
        Map();
        ~Map(){}
        void initialize(const OccupancyGrid& grid);
        void setStartPos(const position& start);
        void setGoalPos(const position& goal);
        bool isObstacle(int i, int j) const;
        void setAgentSize(double agentSize);
        std::list<goalPose> getGoalPoses(SearchResult result);
        bool checkLine(int x1, int y1, int x2, int y2) const;
        bool checkTraversability(int x, int y) const;

        int getStart_i()const{return start_i;}
        int getStart_j()const{return start_j;}
        int getGoal_i()const{return goal_i;}
        int getGoal_j()const{return goal_j;}
        int getWidth()const{return width;}
        int getHeight()const{return height;}

    private:
        int     height, width;
        int     start_i, start_j;
        int     goal_i, goal_j;
        double  cellSize;
        double  agentSize;
        std::vector<std::pair<int, int>> cells;
        std::vector<std::vector<bool>> Grid;
};

#endif


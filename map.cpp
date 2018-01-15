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
    agentSize = 1;
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

void Map::setAgentSize(double agentSize)
{
    this->agentSize = agentSize;
    int add_x, add_y, num = agentSize + 0.5 - CN_EPSILON;
    cells.clear();
    for(int x = -num; x <= +num; x++)
        for(int y = -num; y <= +num; y++)
        {
            add_x = x != 0 ? 1 : 0;
            add_y = y != 0 ? 1 : 0;
            if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                cells.push_back({x, y});
        }
    if(cells.empty())
        cells.push_back({0,0});
}

bool Map::checkTraversability(int x, int y)const//checks traversability of all cells affected by agent's body
{
    for(int k = 0; k < cells.size(); k++)
        if(this->isObstacle(x + cells[k].first, y + cells[k].second))
            return false;
    return true;
}

bool Map::checkLine(int x1, int y1, int x2, int y2)const
{
    //if(!checkTraversability(x1, y1) || !checkTraversability(x2, y2)) //additional check of start and goal traversability,
    //    return false;                                                //it can be removed if they are already checked

    int delta_x = std::abs(x1 - x2);
    int delta_y = std::abs(y1 - y2);
    if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    int step_x = (x1 < x2 ? 1 : -1);
    int step_y = (y1 < y2 ? 1 : -1);
    int error = 0, x = x1, y = y1;
    int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;
    int k, num;

    if(delta_x > delta_y)
    {
        int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_y;
            num = (gap - error)/delta_x;
            for(k = 1; k <= num; k++)
                if(this->isObstacle(x1 - n*step_x, y1 + k*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(this->isObstacle(x2 + n*step_x, y2 - k*step_y))
                    return false;
        }
        error = 0;
        for(x = x1; x != x2 + step_x; x++)
        {
            if(this->isObstacle(x, y))
                return false;
            if(x < x2 - extraCheck)
            {
                num = (gap + error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(this->isObstacle(x, y + k*step_y))
                        return false;
            }
            if(x > x1 + extraCheck)
            {
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(this->isObstacle(x, y - k*step_y))
                        return false;
            }
            error += delta_y;
            if((error<<1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else
    {
        int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_x;
            num = (gap - error)/delta_y;
            for(k = 1; k <= num; k++)
                if(this->isObstacle(x1 + k*step_x, y1 - n*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(this->isObstacle(x2 - k*step_x, y2 + n*step_y))
                    return false;
        }
        error = 0;
        for(y = y1; y != y2 + step_y; y += step_y)
        {
            if(this->isObstacle(x, y))
                return false;
            if(y < y2 - extraCheck)
            {
                num = (gap + error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(this->isObstacle(x + k*step_x, y))
                        return false;
            }
            if(y > y1 + extraCheck)
            {
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(this->isObstacle(x - k*step_x, y))
                        return false;
            }
            error += delta_x;
            if((error<<1) > delta_y)
            {
                x += step_x;
                error -= delta_y;
            }
        }
    }
    return true;
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

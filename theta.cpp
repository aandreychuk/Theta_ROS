#include "theta.h"

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map)
{
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if(delta_i == 0) {
        for(; j != j2; j += step_j)
            if(map.isObstacle(i, j))
                return false;
        return true;
    }
    else if(delta_j == 0) {
        for(; i != i2; i += step_i)
            if(map.isObstacle(i, j))
                return false;
        return true;
    }
        int sep_value = delta_i*delta_i + delta_j*delta_j;
        if(delta_i > delta_j) {
            for(; i != i2; i += step_i) {
                if(map.isObstacle(i, j))
                    return false;
                if(map.isObstacle(i, j + step_j))
                    return false;
                error += delta_j;
                if(error >= delta_i) {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.isObstacle(i + step_i,j))
                            return false;
                    if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                        if(map.isObstacle(i, j + 2*step_j))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
            if(map.isObstacle(i, j))
                return false;
        }
        else {
            for(; j != j2; j += step_j) {
                if(map.isObstacle(i, j))
                    return false;
                if(map.isObstacle(i + step_i, j))
                    return false;
                error += delta_i;
                if(error >= delta_j) {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.isObstacle(i, j + step_j))
                            return false;
                    if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.isObstacle(i + 2*step_i, j))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
            if(map.isObstacle(i, j))
                return false;
    }
    return true;
}

Node Theta::resetParent(Node current, Node parent, const Map &map)
{
    if (parent.parent == nullptr)
        return current;
    if(current == *parent.parent)
        return current;
    if (lineOfSight(parent.parent->i, parent.parent->j, current.i, current.j, map)) {
        current.g = parent.parent->g + distance(parent.parent->i, parent.parent->j, current.i, current.j);
        current.parent = parent.parent;
        return current;
    }
    return current;
}

double Theta::distance(int i1, int j1, int i2, int j2)
{
    return sqrt(pow(i1 - i2, 2) + pow(j1 - j2, 2));
}

SearchResult Theta::startSearch(const Map &map)
{
    open.resize(map.height);
    Node curNode;
    curNode.i = map.start_i;
    curNode.j = map.start_j;
    curNode.g = 0;
    curNode.H = distance(curNode.i, curNode.j, map.goal_i, map.goal_j);
    curNode.F = curNode.H;
    curNode.parent = nullptr;
    addOpen(curNode);
    int closeSize = 0;
    bool pathfound = false;
    while (openSize > 0) {
        curNode = findMin();
        close.insert({curNode.i * map.width + curNode.j, curNode});
        closeSize++;
        open[curNode.i].pop_front();
        openSize--;
        if (curNode.i == map.goal_i && curNode.j == map.goal_j) {
            pathfound = true;
            break;
        }
        std::list<Node> successors = findSuccessors(curNode, map);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while (it != successors.end()) {
            it->parent = parent;
            it->H = distance(it->i, it->j, map.goal_i, map.goal_j);
            *it = resetParent(*it, *it->parent, map);
            it->F = it->g + it->H;
            addOpen(*it);
            it++;
        }
    }
    if (pathfound) {
        sresult.pathfound = true;
        makePrimaryPath(curNode);
        sresult.pathlength = curNode.g;
    }
    else
        sresult.pathfound = false;
    sresult.hppath = hppath;
    return sresult;
}

Node Theta::findMin()
{
    Node min;
    min.F = std::numeric_limits<double>::infinity();
    for (int i = 0; i < open.size(); i++)
        if (!open[i].empty() && open[i].begin()->F <= min.F)
            if (open[i].begin()->F == min.F){
                if(open[i].begin()->g >= min.g)
                    min = *open[i].begin();
            }
            else
                min = *open[i].begin();
    return min;
}

std::list<Node> Theta::findSuccessors(Node curNode, const Map &map)
{
    Node newNode;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++)
        for (int j = -1; j <= +1; j++)
            if ((i != 0 || j != 0) && !map.isObstacle(curNode.i + i, curNode.j + j)) {
                if (close.find((curNode.i + i) * map.width + curNode.j + j) == close.end()) {
                    newNode.i = curNode.i + i;
                    newNode.j = curNode.j + j;
                    if(i == 0 || j == 0)
                        newNode.g = curNode.g + 1;
                    else
                        newNode.g = curNode.g + sqrt(2);
                    successors.push_front(newNode);
                }
            }
    return successors;
}

void Theta::addOpen(Node newNode)
{
    std::list<Node>::iterator iter, pos;

    if (open[newNode.i].empty()) {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }

    pos = open[newNode.i].end();
    bool posFound = false;
    for (iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter) {
        if (!posFound && iter->F >= newNode.F)
            if (iter->F == newNode.F) {
                if(newNode.g >= iter->g){
                    pos=iter;
                    posFound=true;
                }
            }
            else {
                pos = iter;
                posFound = true;
            }

        if (iter->j == newNode.j) {
            if (newNode.F >= iter->F)
                return;
            else {
                if (pos == iter) {
                    iter->F = newNode.F;
                    iter->g = newNode.g;
                    iter->parent = newNode.parent;
                    return;
                }
                open[newNode.i].erase(iter);
                openSize--;
                break;
            }
        }
    }
    openSize++;
    open[newNode.i].insert(pos, newNode);
}

void Theta::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    hppath.clear();
    while(current.parent) {
        hppath.push_back(current);
        current = *current.parent;
    }
    //hppath.push_back(current);
}

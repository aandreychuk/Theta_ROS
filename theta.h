#ifndef THETA_H
#define THETA_H
#include "map.h"
#include <unordered_map>
#include <limits>

class Theta
{
    public:
        Theta(){}
        ~Theta(void){}
        SearchResult startSearch(const Map& map);

    private:
        double distance(int i1, int j1, int i2, int j2);
        bool stopCriterion();
        Node findMin();
        std::list<Node> findSuccessors(Node curNode, const Map &map);
        void addOpen(Node newNode);
        Node resetParent(Node current, Node parent, const Map &map);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath();
        std::vector<Node>               hppath;
        SearchResult                    sresult;
        std::unordered_map<int, Node>   close;
        std::vector<std::list<Node>>    open;
        int                             openSize;
};


#endif // THETA_H

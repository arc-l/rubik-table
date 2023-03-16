/**
 * @file search.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include"common.h"
#include <queue>




/**
 * @brief A star
 * 
 */

class AStarSolver{
public:
    struct AStarNode
    {
        using AStarNode_p=std::shared_ptr<AStarNode>;
        AStarNode(Location& v,int t,int f,int hc,AStarNode_p parent=nullptr):v(v),t(t),f(f),hc(hc),parent(parent){}
        int t;
        Location v;
        int f;
        int hc; //number of conflicts; used for tie-breaking
        AStarNode_p parent;
    };

    using AStarNodes=std::vector<AStarNode>;
    using AStarNode_p=std::shared_ptr<AStarNode>;
    AStarSolver(Location &start,Location &goal);
    Path search();
    std::function<bool(AStarNode_p)> checkValid;
    std::function<AStarNodes(AStarNode_p)> getNeighbors;
    std::function<bool(AStarNode_p)> isSolution;
    std::function<int(AStarNode_p)> computeConflicts; 
    std::function<int(AStarNode_p)> computeHeuristic;
    std::function<bool(AStarNode_p,AStarNode_p)> compareOpen;
    using openList=std::priority_queue<AStarNode_p,AStarNodes,decltype(compareOpen)>;
private:
    Location start,goal;
};


/**
 * @brief  BFS
 * 
 */
class BFS_solver{
public:
    BFS_solver(Location &start):start(start){}
    std::function<bool(Location &v)> isGoal;
    std::function<Configs(Location &)> getNeighbors;
    using openList=std::queue<Location>;
    Path solve();   
private:
    int depth;
    Location start;
};

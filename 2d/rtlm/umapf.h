
/**
 * @file umapf.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include<iostream>
#include<cmath>
#include"common.h"
#include <queue>
/**
 * @brief  max-flow based UMAPF
 * 
 * @param starts 
 * @param goals 
 * @param xmax 
 * @param ymax 
 * @param obstacles 
 * @return Paths 
 */
Paths umapf_solve(Configs &starts, 
                  Configs &goals,
                  int xmax,int ymax,
                  Obstacles &obstacles);


/**
 * @brief 
 * distance -optimal 
 */
class FormationControl{
public:
    using WNode=std::pair<Location,int>;
    using WNodes=std::vector<WNode>;
    using DAG=std::unordered_map<Location,WNodes>;
    FormationControl(Configs &starts,Configs &goals,point2d dims,Obstacles obstacles):
        starts(starts),goals(goals),xmax(dims.first),ymax(dims.second),obstacles(obstacles){}
    void solve();
    Paths get_result();

protected:
    Configs starts;
    Configs goals;
    Obstacles obstacles;
    void find_initial_paths(Paths &);
    void update_paths(Paths &);
    void schedule(Paths &);
    int xmax;
    int ymax;
    Paths result;
    std::function<Configs(Location& v)> getNeighbors;
    void formDAG(const Paths &,DAG& dag_graph);
    void BFS(Location &s,Path& path);   //run BFS on DAG 
    void astar_search(Location &start,Location &goal,Path &path);

};





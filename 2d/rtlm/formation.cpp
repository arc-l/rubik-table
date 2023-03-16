/**
 * @file formation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "umapf.h"
#include <memory>
#include "labp.h"
#include "search.h"

const int BIG=100000;
/**
 * @brief Construct a new Formation Control:: Formation Control object
 * 
 * @param starts 
 * @param goals 
 * @param dims 
 * @param obstacles 
 */
// FormationControl::FormationControl(Configs &starts, 
//     Configs &goals,
//     point2d dims,
//     Obstacles &obstacles):starts(starts),goals(goals),xmax(dims.first),ymax(dims.second),obstacles(obstacles){

// }

/**
 * @brief build the DAG
 * 
 * @param paths 
 * @param dag_graph 
 */
void FormationControl::formDAG(const Paths &paths,DAG&dag_graph){
    for(auto &p:paths){
        for(int t=p.size()-1;t>=1;t--){
            auto it1=dag_graph[p[t]].begin();
            auto it2=dag_graph[p[t]].end();
            auto cond=[&](WNode & v){
                return v.first==p[t-1];
            };
            auto it=std::find_if(it1,it2,cond);
            // auto it=std::find(it1,it2,p[t-1]);
            if(it==it2) dag_graph[p[t]].push_back({p[t-1],1});
            else it->second=it->second+1;
            
        }
    }
}

/**
 * @brief  find initial paths
 * 
 * @param paths 
 */
void FormationControl::find_initial_paths(Paths &paths){
    using costEdge=std::tuple<int,int,double>;
    using costEdges=std::vector<costEdge>;
    using costMatrix=std::vector<std::vector<double>>;
    int threshold=4;
    int m=starts.size();
    bool feasible=false;
    std::vector<int> rowsol(m);
    std::vector<int> colsol(m);
    std::vector<double> u(m);
    std::vector<double> v(m);

    // for(int i=0;i<starts.size();i++){
    //     std::cout<<i<<": "<<starts[i]<<"  "<<goals[i]<<std::endl;
    // }
    // std::cout<<"starts and goals\n";
    while(!feasible){
       
        costMatrix costs(m,std::vector<double>(m));
        for(int i=0;i<starts.size();i++){
            for(int j=0;j<goals.size();j++){
                double dist=distance(starts[i],goals[j]);
                if(dist<threshold) costs[i][j]=dist;
                else costs[i][j]=BIG;
            }
            
        } 
        lap(m,costs,rowsol,colsol,u,v);
        feasible=true;
        // std::cout<<"found"<<std::endl;

    }
  
    Configs new_goals;
    for(int i=0;i<starts.size();i++){
        new_goals.emplace_back(goals[rowsol[i]]);
    }
    goals.swap(new_goals);
    // std::vector<int> assignment(m);
    // while(!feasible){
    //     costEdges costs;
    //     for(int i=0;i<starts.size();i++){
    //         for(int j=0;j<goals.size();j++){
    //             int dist=distance(starts[i],goals[j]);
    //             if(dist<threshold) costs.push_back({i,j,dist});
    //         }
    //     }   
    //     feasible=check_feasible_sparse(costs,assignment,m,threshold);
    //     // if(feasible){
    //     //     assignment=std::vector<int>(m,0);
    //     //     hungarian_s(costs,assignment);
    //     // }
 
    //     // feasible=check_feasible_sparse(costs,assignment,m,threshold);
    //     std::cout<<feasible<< "  "<<threshold<<std::endl;
    //     if(!feasible) threshold++;
    // }
    // Configs new_goals;
    // for(int i=0;i<starts.size();i++){
    //     new_goals.emplace_back(goals[assignment[i]]);
    // }
    // goals.swap(new_goals);
    // for(int i=0;i<starts.size();i++){
    //     std::cout<<starts[i]<<"    "<<goals[i]<<std::endl;
    // }
    //using LBA to do the matching  first and then apply BFS
    for(int i=0;i<starts.size();i++){
        BFS_solver solver(starts[i]);
        solver.isGoal=[&](Location &v){
            return v==goals[i];
        };
        solver.getNeighbors=getNeighbors;
        auto pi=solver.solve();
        paths.emplace_back(pi);
    }
    // std::cout<<"debug "<<paths.size()<<std::endl;
    // assert(check_cycle(paths)==false);
    // for(auto &p:paths){
    //     for(auto &v:p) std::cout<<v<<" ";
    //     std::cout<<std::endl;
    // }
    // std::cout<<"BFS done"<<std::endl;
    // save_paths_as_txt("test.txt",paths,0,true);
    // exit(0);
}

/**
 * @brief schedule the paths to time parametrized
 * 
 * @param old_paths 
 */

void FormationControl::schedule(Paths &old_paths){
    int num_agents=old_paths.size();
    using timeObstacle=std::tuple<int,int,int>;
    Paths timed_paths;
    std::unordered_set<timeObstacle,boost::hash<timeObstacle>> reserveTable;

    for(auto &p:old_paths){
        Path pi;
        int t=0,k=0;
        while(k<p.size()){
            timeObstacle obsk={p[k].x,p[k].y,t};
       
            if(reserveTable.find(obsk)==reserveTable.end()){
                pi.push_back(p[k]);
                reserveTable.insert(obsk);
                t++;
                k++;
            }else{
                //wait
          
                pi.push_back(pi.back());
             
                auto v=pi.back();
                reserveTable.insert({v.x,v.y,t});
                t++;
            }          
        }  
        timed_paths.push_back(pi);
   }
   old_paths.swap(timed_paths);

}


/**
 * @brief 
 * find a standalone goal, whose degree  should be 0
 * bfs search the nearest start from this standolone goal
 * remove the goal and start, if the found path traverse other goal gj, degree(gj)--
 * @param old_paths 
 */
void FormationControl::update_paths(Paths &old_paths){
    std::cout<<"////////////////updating////////\n";
    int num_agents=old_paths.size();
    auto toPathSet=[](Path &p){
        std::unordered_set<Location> path_set;
        for(auto &vs:p) path_set.insert(vs);
        return path_set;
    };
    
    using PathSet=std::unordered_set<Location>;
    std::unordered_set<Location> goalSet;
  
    std::unordered_set<Location> startSet;
    std::unordered_map<Location,int> degrees;

    auto findStandAloneGoal=[&](){
        for(auto &goal:goalSet){
            if(degrees[goal]<=1) return goal;
        }
        throw std::runtime_error("no standalone goal");
    };

    for(int i=0;i<old_paths.size();i++){
        goalSet.insert(old_paths[i].back());
        startSet.insert(old_paths[i][0]);
        degrees[old_paths[i].back()]=0;
    }

    std::vector<PathSet> path_sets;
    for(int i=0;i<num_agents;i++){
        PathSet pi=toPathSet(old_paths[i]);
        path_sets.push_back(pi);
    }
    for(int i=0;i<num_agents;i++){
        for(auto &v:path_sets[i]){
            if(degrees.find(v)!=degrees.end()) degrees[v]++;
        }
    }

    
    path_sets.clear();
    DAG dag;
    formDAG(old_paths,dag);
    Paths new_paths;
    while(not startSet.empty()){
        auto standAloneGoal=findStandAloneGoal();
        BFS_solver searcher(standAloneGoal);
        searcher.getNeighbors=[&](Location &v){
            auto possible_n=dag[v];
            Configs tmp;
            for(auto &n:possible_n){
                if(n.second>0) tmp.push_back(n.first);
            }
            return tmp;
        };
        searcher.isGoal=[&](Location &v){
            return startSet.find(v)!=startSet.end(); 
        };
        Path pi=searcher.solve();
     
        // std::cout<<pi.back()<<" "<<pi[0]<<" "<<standAloneGoal<<std::endl;
        if(pi.empty())std::cout<<"wrong "<<standAloneGoal<<std::endl;
     
        auto si=pi.back();
        startSet.erase(si);
        goalSet.erase(standAloneGoal);
        std::reverse(pi.begin(),pi.end());
        new_paths.push_back(pi);
        degrees.erase(standAloneGoal);
        auto new_pathSet=toPathSet(pi);
        // for(auto &v:pi){
        //     std::cout<<v<<" ";
        // }
        for(int t=pi.size()-1;t>0;t--){
            auto it1=dag[pi[t]].begin();
            auto it2=dag[pi[t]].end();
            auto cond=[&](WNode &v){
                return v.first==pi[t-1];
            };
            auto it=std::find_if(it1,it2,cond);
            it->second=it->second-1;
        }
        // std::cout<<" for standalone goal "<<standAloneGoal<<std::endl;
        for(auto &v:new_pathSet){
            if(degrees.find(v)!=degrees.end()) degrees[v]--;
        }
        // for(auto &item:degrees){
        //     auto gi=item.first;
        //     if(new_pathSet.find(gi)==new_pathSet.end()) continue;
        //     item.second--;
        // }
    }
    std::cout<<"updated: num_agents="<<new_paths.size()<<std::endl;
    // int k=0;
    // for(auto &p:new_paths){
    //     std::cout<<"agent "<<k<<":";
    //     k++;
    //     for(auto &v:p) std::cout<<v<<" ";
    //     std::cout<<std::endl;
    // }
    old_paths.swap(new_paths);
}

void FormationControl::solve(){
    auto checkValid=[&](Location v){
        return v.x >=0 and v.x<xmax and v.y >=0 and v.y<ymax and obstacles.find(v)==obstacles.end(); 
    };

    getNeighbors=[&](Location v){
        Configs neighbors;
        {
            Location u(v.x,v.y);
            neighbors.push_back(u);
        }
        {
            Location u(v.x+1,v.y);
            if(checkValid(u)) neighbors.push_back(u);
        }
        {
            Location u(v.x-1,v.y);
            if(checkValid(u)) neighbors.push_back(u);
        }
        {
            Location u(v.x,v.y+1);
            if(checkValid(u)) neighbors.push_back(u);
        }
        {
            Location u(v.x,v.y-1);
            if(checkValid(u)) neighbors.push_back(u);
        }
        return neighbors;
    };

    find_initial_paths(result);
  
    update_paths(result);
  
    schedule(result);
    // for(auto &p:result){
    //     for(auto &v:p){
    //         std::cout<<v<<" ";
    //     }
    //     std::cout<<std::endl;
    // }
    // save_paths_as_txt("./test.txt",result,0,true);
    
    
}

Paths FormationControl::get_result(){
    return result;
}



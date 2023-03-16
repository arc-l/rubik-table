/**
 * @file search.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "search.h"

///////////////BFS/////////////////////////////////////////////////////

Path BFS_solver::solve(){
    openList open;
    open.push(start);
    std::unordered_set<Location> closed;
    std::unordered_map<Location,Location> parents;
    bool success=false;
    Location curr;
    while(open.empty()==false){
        auto top=open.front();
        open.pop();
        if(isGoal(top)){
            success=true;
            curr=top;
            break;
        }
        closed.insert(top);
        auto children=getNeighbors(top);
        for(auto &c:children){
            if(closed.find(c)!=closed.end()) continue;
            parents[c]=top;
            open.push(c);
        }

    }
    //retrieve the path
    if(success==false) return {};
    Path result;
    while(curr!=start){
        result.push_back(curr);
        curr=parents[curr];
    }
    result.push_back(start);
    std::reverse(result.begin(),result.end());
    return result;  
}
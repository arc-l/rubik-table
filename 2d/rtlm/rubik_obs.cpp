/**
 * @file rubik_obs.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "rubik_obs.h"
#include<random>
#include "umapf.h"
#include "mp.h"

RTH_obs::RTH_obs(Agents &agents, point2d dims):RUBIK13(agents,dims){
  
    load_data("./local3x3_uniform_obs_db.json");   //load the data
}

void RTH_obs::x_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent :agents){
            if(agent->current.y<=i+2&&agent->current.y>=i) robots.emplace_back(agent);

        }
        HW_MP_obs swapper(robots,{0,xmax-1},{i,i+2},'x');
        // HW_MP swapper(robots,{0,xmax-1},{i,i+2},'x');
        swapper.reconfigure();
    }
    // for(auto &agent:agents){
    //     assert(agent->current==agent->intermediate);
    // }
    fillPaths();
}


void RTH_obs::y_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent:agents){
            if(agent->current.x<=i+2&&agent->current.x>=i) robots.emplace_back(agent);
        }
        HW_MP_obs swapper(robots,{i,i+2},{0,ymax-1},'y');
        swapper.reconfigure();
    }
    fillPaths();

}

HW_MP_obs::HW_MP_obs(Agents &agents,point2d xrange,point2d yrange,char orientation):HW_MP(agents,xrange,yrange,orientation){

    std::ifstream ifs("./local3x3_uniform_obs_db.json");
    data_dict = json::parse(ifs);
}


void RTH_obs::labp_matching_greedy(std::unordered_map<int,Agents> &column_dict,std::unordered_map<int,int>&matching,
        std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row){
    using weighted_edge=std::tuple<int,int,double>;
    std::vector<weighted_edge> costEdge;
    const int max_inf=1e6;
    std::vector<int> column_id;
    for(auto const &column_i:column_dict){
        if(column_i.second.size()!=0)column_id.push_back(column_i.first);
    }
    std::sort(column_id.begin(),column_id.end());  
    // for(auto id:column_id){
    //     std::cout<<id<<std::endl;
    // }
    for(int i=0;i<column_id.size();i++){
        for(int j=0;j<column_id.size();j++){
            bool found=false;
            int min_d=max_inf;
            int col=column_id[i];
            int color=column_id[j];
            Agent_p min_agent;
            for(auto const &agent_i :column_dict[col]){
                if(agent_i->inter_goal.y==color &&abs(agent_i->current.x-row)<min_d){
                    found=true;
                    min_d=abs(agent_i->current.x-row);
                    min_agent=agent_i;
                }
            }
            if(found){
                costEdge.push_back({i,j,min_d});
                // (cost_matrix.back()).push_back(min_d);
                arranged_agents[{col,color}]=min_agent;
                // assert(min_agent->current.y==i);
            }
            else{
                //(cost_matrix.back()).push_back(max_inf);
            }
        }
    }
    
 
    std::vector<int> assignment;
    // double cost=labp_solve(cost_matrix,assignment);
    // for(auto &ce:costEdge){
    //     std::cout<<std::get<0>(ce)<<" "<<std::get<1>(ce)<<" "<<std::get<2>(ce)<<std::endl;
    // }
    // std::cout<<"edge size="<<costEdge.size()<<std::endl;
    double cost=lba_sparse(costEdge,assignment);
    // assert(assignment.size()==column_id.size());
    for(int c=0;c<column_id.size();c++){
        int col=column_id[c];
        int color_id=assignment[c];
        int color=column_id[color_id];
        Agent_p agent=arranged_agents[{col,color}];
    
        auto it = std::find(column_dict[col].begin(), column_dict[col].end(), agent);
        if(it!=column_dict[col].end()){
            column_dict[col].erase(it);
            
        }else{
            throw std::runtime_error("NOT found the agent!");
        }
    }
    for(int c=0;c<column_id.size();c++){
        int col_id=column_id[c];
        int color_id=assignment[c];
        int color=column_id[color_id];
        matching.insert({col_id,color});
    }
}



void RTH_obs::matching(){
    std::unordered_map<int,Agents>column_dict;
    for(int i=0;i<xmax;i++){
        Agents ci;
        column_dict.insert({i,ci});
    }
    for(auto &agent :agents){
        int ci=agent->current.y;
        column_dict[ci].push_back(agent);
    }

  
    for(int i=0;i<int(xmax/3.);i++){
        // std::cout<<"i th matching="<<i<<std::endl;
        std::unordered_map<int,int> matching;
        std::unordered_map<point2d,Agent_p,boost::hash<point2d>> arranged_agents;
        labp_matching_greedy(column_dict,matching,arranged_agents,3*i+1);
        // std::cout<<"matched"<<std::endl;
        for(auto &item:matching){
            int color=item.first;
            int column=item.second;
            arranged_agents[{color,column}]->intermediate=Location(3*i+1,color);
            
            //  std::cout<<"the arranged agent is "<<*(arranged_agents[{color,column}])<<std::endl;
        }
        //debug_print(column_dict);
    }
    if(use_lba)matching_heuristic();

    // for(auto &agent:agents){
    //     std::cout<<*agent<<std::endl;
    // }
    //check_feasible(*agents);
}
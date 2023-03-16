/**
 * @file rubik13.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include"rubik13.h"
#include<iostream>
#include<random>
#include "umapf.h"
#include "mp.h"
#include "labp.h"


RUBIK13::RUBIK13(Agents &agents, point2d dims):RUBIK(agents,dims){
    balanced_starts=starts;
    balanced_goals=goals;
    load_data("./local3x3.json");   //load the data
}



void RUBIK13::load_data(std::string filename){
    std::ifstream ifs(filename);
    data_dict = json::parse(ifs);
}

void RUBIK13::random_to_balanced(){
    Configs bl_config;
    for(int i=0;i<xmax;i+=3){
        for(int j=0;j<ymax;j+=3){
            Configs tmp_list={Location(i+1,j),Location(i+1,j+1),Location(i+1,j+2)};
            bl_config.insert(bl_config.end(),tmp_list.begin(),tmp_list.end());
        }
    }
    auto rng=std::default_random_engine {};
    std::shuffle(bl_config.begin(),bl_config.end(),rng);
    starts.clear();
    goals.clear();
    for(auto agent :agents){
        starts.push_back(agent->start);
        goals.push_back(agent->goal);
    }
    Obstacles obs;
    Paths paths_s=umapf_solve(starts,bl_config,xmax,ymax,obs);// start to balanced
    Paths paths_g=umapf_solve(goals,bl_config,xmax,ymax,obs);   //goal to balanced

    for(int i=0;i<agents.size();i++){
        // agents[i]->start=paths_s[i].back();
        // agents[i]->goal=paths_g[i].back();
        agents[i]->current=paths_s[i].back();
        agents[i]->inter_goal=paths_g[i].back();
        agents[i]->path.insert(agents[i]->path.end(),paths_s[i].begin()+1,paths_s[i].end());
        agents[i]->inter_path_u=paths_g[i];
        std::reverse(agents[i]->inter_path_u.begin(),agents[i]->inter_path_u.end());
    }
}



void RUBIK13::labp_matching_greedy(std::unordered_map<int,Agents> &column_dict,std::unordered_map<int,int>&matching,
        std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row){
    using weighted_edge=std::tuple<int,int,double>;
    std::vector<weighted_edge> costEdge;
    const int max_inf=1e6;
    std::vector<int> column_id;
    for(auto const &column_i:column_dict){
        column_id.push_back(column_i.first);
    }  
    for(auto const &i: column_id){
        for(auto const &j:column_id){
            bool found=false;
            int min_d=max_inf;
            Agent_p min_agent;
            for(auto const &agent_i :column_dict[i]){
                if(agent_i->inter_goal.y==j &&abs(agent_i->current.x-row)<min_d){
                    found=true;
                    min_d=abs(agent_i->current.x-row);
                    min_agent=agent_i;
                }
            }
            if(found){
                costEdge.push_back({i,j,min_d});
                // (cost_matrix.back()).push_back(min_d);
                arranged_agents[{i,j}]=min_agent;
                // assert(min_agent->current.y==i);
            }
            else{
                //(cost_matrix.back()).push_back(max_inf);
            }
        }
    }
    // std::cout<<"arranged agents=================="<<std::endl;
    // for(auto&pair:arranged_agents){
    //     std::cout<<pair.first.first<<" "<<pair.first.second<<"  "<<*(pair.second)<<std::endl;
    // }
    // std::cout<<"aranged agents size="<<arranged_agents.size()<<std::endl;
    // std::cout<<"=================="<<std::endl;
 
    std::vector<int> assignment;
    // double cost=labp_solve(cost_matrix,assignment);
    // for(auto &ce:costEdge){
    //     std::cout<<std::get<0>(ce)<<" "<<std::get<1>(ce)<<" "<<std::get<2>(ce)<<std::endl;
    // }
    double cost=lba_sparse(costEdge,assignment);
    // assert(assignment.size()==column_id.size());
    for(auto c:column_id){
        int color=assignment[c];
        Agent_p agent=arranged_agents[{c,color}];
    
        auto it = std::find(column_dict[c].begin(), column_dict[c].end(), agent);
        if(it!=column_dict[c].end()){
            column_dict[c].erase(it);
            
        }else{
            throw std::runtime_error("NOT found the agent!");
        }
    }
    for(auto c:column_id){
        matching.insert({c,assignment[c]});
    }
}



void RUBIK13::matching_heuristic(){
    std::unordered_map<int,Agents> row_dict;
   
    for(auto &agent:agents){
        if(row_dict.find(agent->intermediate.x)==row_dict.end()){
            Agents as;
            row_dict.insert({agent->intermediate.x,as });
        }
        row_dict[agent->intermediate.x].push_back(agent);
    }

    std::vector<std::vector<double>>cost_matrix;
    std::vector<int> rows;
    for(const auto &pair:row_dict){
        rows.push_back(pair.first);
    }
   
    for(const auto &row:rows){
        // Agents as=row_dict[row];
        std::vector<double> cost_i;
        cost_matrix.push_back(cost_i);
        for(const auto & row2:rows){
            double cost=0;
            for(auto&agent :row_dict[row]){
                cost=std::max(cost,fabs(agent->current.x-row2));  
            }
            cost_matrix.back().push_back(cost);
        }
        // cost_matrix.back.push_back(cost_i);
    }
    // std::cout<<"cost_matrix "<<cost_matrix.size()<<std::endl;
    // for(auto &costs:cost_matrix){
    //     for(auto&cost:costs) {
    //         std::cout<<cost<<" ";
    //     }
    //     std::cout<<std::endl;
    // }

    std::vector<int> assignment;
    labp_solve(cost_matrix,assignment);
    // for(auto &asn:assignment){
    //     std::cout<<asn<<std::endl;
    // }
    for(int i=0;i<rows.size();i++){
        for(auto&agent: row_dict[rows[i]]){
            agent->intermediate.x=rows[assignment[i]];
        }
    }
}

void RUBIK13::matching(){

    // assert(agents.size()==int(1./3*xmax*xmax));
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
        for(int color=0;color<xmax;color++){
            int column=matching[color];
            // std::cout<<column<<"---------"<<color<<std::endl;
            // // assert(arranged_agents.find({color,column})!=arranged_agents.end());
            
            // std::cout<<arranged_agents[{color,column}]->intermediate<<std::endl;
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


void RUBIK13::prepare_cell(int xmin,int ymin,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_start,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_goal){
    std::vector<Location> vertices;
  
    Agents robots_start=agents_start[{int(xmin/3),int(ymin/3)}];
    Agents robots_goal=agents_goal[{int(xmin/3),int(ymin/3)}];
    
    for(int i=xmin;i<xmin+3;i+=1){
        for(int j=ymin;j<ymin+3;j+=1){
            vertices.push_back(Location(i,j));
        }
    }
    
  
    std::sort(robots_start.begin(),robots_start.end(),[](const Agent_p a1,const Agent_p a2){
        return a1->current<a2->current;
    });
    std::sort(robots_goal.begin(),robots_goal.end(),[](const Agent_p a1,const Agent_p a2){
        return a1->goal<a2->goal;
    });
   
 
    std::string start_id;
    start_id="((";
    for(auto &agent:robots_start){
        auto it=std::find(vertices.begin(),vertices.end(),agent->current);
        int id=std::distance(vertices.begin(),it);
        start_id+=std::to_string(id);
        start_id+=", ";
    }
    start_id.pop_back();
    start_id.pop_back();
    start_id+="), ";
    start_id+="'x')";

    // for(auto &agent:robots_start){
    //     std::cout<<*agent<<std::endl;
    // }
    // std::cout<<"=========\n";
    // std::cout<<start_id<<std::endl;
    
    std::vector<std::vector<int>>solution=data_dict[start_id];
    std::string start_id2;
    start_id2="((";
    for(auto &agent:robots_goal){
        auto it=std::find(vertices.begin(),vertices.end(),agent->goal);
        int id=std::distance(vertices.begin(),it);
        start_id2+=std::to_string(id);
        start_id2+=", ";
    }
    start_id2.pop_back();
    start_id2.pop_back();
    start_id2+="), ";
    start_id2+="'x')";

    // for(auto &agent:robots_goal){
    //     std::cout<<*agent<<std::endl;
    // }
    // std::cout<<"=========\n";
    // std::cout<<start_id2<<std::endl;
    // exit(0);
  
    std::vector<std::vector<int>>solution2=data_dict[start_id2];
    for(int i=0;i<robots_start.size();i++){
        for(int j=0;j<solution[i].size();j++){
            robots_start[i]->path.push_back(vertices[solution[i][j]]);
            robots_start[i]->current=robots_start[i]->path.back();
        }
        for(int j=0;j<solution2[i].size();j++){
            robots_goal[i]->inter_goal_path.insert(robots_goal[i]->inter_goal_path.begin(),vertices[solution2[i][j]]);
        }
      
        robots_goal[i]->inter_goal=robots_goal[i]->inter_goal_path[0];
        robots_goal[i]->inter_goal_path.erase(robots_goal[i]->inter_goal_path.begin());
    }
    // for(auto &agent:robots_start){
    //     for(auto v:agent->path){
    //         std::cout<<v<<" ";
    //     }
    //     std::cout<<"\n";
    // }
    // std::cout<<"=====================\n";
    //exit(0);
}


void RUBIK13::x_fitting(){
    for(auto &agent:agents){
        agent->intermediate.set_value(agent->inter_goal.x,agent->current.y);
    }

}

void RUBIK13::y_fitting(){
    for(auto&agent: agents){
        agent->intermediate.set_value(agent->current.x,agent->inter_goal.y);
    }
    //check_feasible(agents);
}


void RUBIK13::fillPaths(){
    int mkpn=0;
    for(auto &agent:agents){
        mkpn=std::max((int)agent->path.size(),mkpn);
    }
    for(auto &agent:agents){
        while(agent->path.size()<mkpn){
            agent->path.emplace_back(agent->path.back());
        }
        agent->current=agent->path.back();
    }
}

void RUBIK13::x_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent :agents){
            if(agent->current.y<=i+2&&agent->current.y>=i) robots.emplace_back(agent);

        }
        HW_MP swapper(robots,{0,xmax-1},{i,i+2},'x');
        swapper.reconfigure();
    }
    // for(auto &agent:agents){
    //     assert(agent->current==agent->intermediate);
    // }
    fillPaths();
}


void RUBIK13::y_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent:agents){
            if(agent->current.x<=i+2&&agent->current.x>=i) robots.emplace_back(agent);
        }
        HW_MP swapper(robots,{i,i+2},{0,ymax-1},'y');
        swapper.reconfigure();
    }
    fillPaths();

}

//three-m shuffle
void RUBIK13::solve(){
    auto t0 = Time::now();
    if(use_umapf==true) random_to_balanced();
    local13_prepare();
    fillPaths();
   
    matching();
    
    x_shuffle();
    
    //check_matching_good(agents);
   // debug_check_agent(agents);
    y_fitting();
   // debug_check_agent(agents);
    y_shuffle();
    // debug_check_paths(agents);
    // exit(0);
    //debug_check_agent(agents);
    x_fitting();
    //debug_check_agent(agents);
    x_shuffle();
    
    for(auto &agent:agents){
        agent->path.insert(agent->path.end(),agent->inter_goal_path.begin(),agent->inter_goal_path.end());
        agent->current=agent->path.back();
        if(use_umapf==true){
            agent->path.insert(agent->path.end(),agent->inter_path_u.begin(),agent->inter_path_u.end());
            agent->current=agent->path.back();
        }
        // assert(agent->current==agent->goal);
    }
    fillPaths();
    auto t1=Time::now();
    fsec fs = t1 - t0;
    runtime=fs.count();
 
}

void RUBIK13::local13_prepare(){
    std::unordered_map<point2d,Agents,boost::hash<point2d>> start_agents;
    std::unordered_map<point2d,Agents,boost::hash<point2d>> goal_agents;
    for(auto &agent :agents){
        point2d start_id={int(agent->current.x/3),int(agent->current.y/3)};
        point2d goal_id={int(agent->goal.x/3),int(agent->goal.y/3)};
        start_agents[start_id].push_back(agent);
        goal_agents[goal_id].push_back(agent);
    }

    for(int i=0;i<xmax;i+=3){
        for(int j=0;j<xmax;j+=3){	
            prepare_cell(i,j,start_agents,goal_agents);
        }
    }
}



///////////////////////////////Improved version 1///////////////////////////////////////////
void IRUBIK13_H1::x_shuffle(){
    for(int i=0;i<xmax;i+=3){
        Agents robots;
        for(auto &agent :agents){
            if(agent->current.y<=i+2&&agent->current.y>=i) robots.emplace_back(agent);
        }
        HW_MP_H1 swapper(robots,{0,xmax-1},{i,i+2},'x');
        swapper.reconfigure();
    }
    // save_results("test.txt");
    // std::cout<<"???????????"<<std::endl;
    // exit(0);
}

void IRUBIK13_H1::y_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent:agents){
            if(agent->current.x<=i+2&&agent->current.x>=i) robots.emplace_back(agent);
        }
        HW_MP_H1 swapper(robots,{i,i+2},{0,ymax-1},'y');
        swapper.reconfigure();
    }
}

///////////////////////////////Improved version 2//////////////////////////////////////////

void IRUBIK13_H2::x_shuffle(){
    for(int i=0;i<xmax;i+=3){
        Agents robots;
        for(auto &agent :agents){
            if(agent->current.y<=i+2&&agent->current.y>=i) robots.emplace_back(agent);
        }
        HW_MP_H2 swapper(robots,{0,xmax-1},{i,i+2},'x');
        swapper.reconfigure();
    }
}

void IRUBIK13_H2::y_shuffle(){
    for(int i=0;i<ymax;i+=3){
        Agents robots;
        for(auto &agent:agents){
            if(agent->current.x<=i+2&&agent->current.x>=i) robots.emplace_back(agent);
        }
        HW_MP_H2 swapper(robots,{i,i+2},{0,ymax-1},'y');
        swapper.reconfigure();
    }
}


#include "rubik12.h"

#include<random>
#include "umapf.h"

RUBIK12::RUBIK12(Agents&agents, point2d dims):RUBIK(agents,dims){
    this->get_data_map();
}

void RUBIK12::solve(){
    auto t0 = Time::now();
    if(use_umapf==true) random_to_balanced();
    local_prepare();
    // random_to_balanced();
    for(auto &robot:agents){
        printf("robot %d is at (%d,%d)\n",robot->id,robot->current.x,robot->current.y);
    }
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
    // append_umapf_paths();
    auto t1=Time::now();
    fsec fs = t1 - t0;
    runtime=fs.count();
}


void RUBIK12::append_umapf_paths(){
    for(auto &a:agents){
        a->path.insert(a->path.begin(),a->inter_path_u.begin(),a->inter_path_u.end());
    }
    fill_paths(agents);
}

void RUBIK12::get_data_map(){
    std::vector<std::vector<int>> pathx(2,std::vector<int>());
    // 0,1
    pathx={{0},{1}};
    data_map[{0,1}]=pathx;

    //0,2
    pathx={{0,1},{2,0}};
    data_map[{0,2}]=pathx;

    //0,3
    pathx={{0,0},{3,1}};
    data_map[{0,3}]=pathx;

    //1,2
    pathx={{1,1},{2,0}};
    data_map[{1,2}]=pathx;

    //1,3
    pathx={{1,0},{3,1}};
    data_map[{1,3}]=pathx;

    //2,3
    pathx={{2,0},{3,1}};
    data_map[{2,3}]=pathx;
}



void RUBIK12::matching(){
    // assert(agents.size()==int(1./3*xmax*xmax));
    std::unordered_map<int,Agents>column_dict;
    for(int i=0;i<ymax;i++){
        Agents ci;
        column_dict.insert({i,ci});
    }
    for(auto &agent :agents){
        int ci=agent->current.y;
        column_dict[ci].push_back(agent);
    }

    std::cout<<"Debug"<<std::endl;
    for(auto &[ci,Ai]:column_dict){
        std::cout<<"column "<<ci<<":";
        for (auto &agent:Ai){
            std::cout<<agent->id<<" ";
        }
        std::cout<<std::endl;
    }

    for(int i=0;i<int(xmax/2.);i++){
        // std::cout<<"i th matching="<<i<<std::endl;
        std::unordered_map<int,int> matching;
        std::unordered_map<point2d,Agent_p,boost::hash<point2d>> arranged_agents;
        labp_matching_greedy(column_dict,matching,arranged_agents,2*i+1);
        // std::cout<<"matched"<<std::endl;
        for(int color=0;color<ymax;color++){
            int column=matching[color];
            // std::cout<<column<<"---------"<<color<<std::endl;
            // // assert(arranged_agents.find({color,column})!=arranged_agents.end());          
            // std::cout<<arranged_agents[{color,column}]->intermediate<<std::endl;
            arranged_agents[{color,column}]->intermediate=Location(2*i+1,color);           
            //  std::cout<<"the arranged agent is "<<*(arranged_agents[{color,column}])<<std::endl;
        }
        //debug_print(column_dict);
    }
    matching_heuristic();

}


void RUBIK12::local_prepare(){
    std::unordered_map<point2d,Agents,boost::hash<point2d>> start_agents;
    std::unordered_map<point2d,Agents,boost::hash<point2d>> goal_agents;
    for(auto &agent :agents){
        point2d start_id={int(agent->current.x/2),int(agent->current.y/2)};
        point2d goal_id={int(agent->goal.x/2),int(agent->goal.y/2)};
        start_agents[start_id].push_back(agent);
        goal_agents[goal_id].push_back(agent);
    }

    for(int i=0;i<xmax;i+=2){
        for(int j=0;j<ymax;j+=2){	
            prepare_cell(i,j,start_agents,goal_agents);
        }
    }
}


void RUBIK12::random_to_balanced(){
    Configs bl_config;
    for(int i=0;i<xmax;i+=2){
        for(int j=0;j<ymax;j+=2){
            Configs tmp_list={Location(i,j),Location(i,j+1)};
            bl_config.insert(bl_config.end(),tmp_list.begin(),tmp_list.end());
        }
    }
    
    Configs bl_config2=bl_config;
    auto rng=std::default_random_engine {};
    


    std::shuffle(bl_config.begin(),bl_config.end(),rng);
    std::shuffle(bl_config2.begin(),bl_config2.end(),rng);
    starts.clear();
    goals.clear();
    for(auto agent :agents){
        starts.push_back(agent->start);
        goals.push_back(agent->goal);
    }
 
    Obstacles obs;
   
    Paths paths_s=umapf_solve(starts,bl_config2,xmax,ymax,obs);// start to balanced
    std::cout<<"solved"<<std::endl;
    Paths paths_g=umapf_solve(goals,bl_config,xmax,ymax,obs);   //goal to balanced
    std::cout<<"solved 2"<<std::endl;
    std::unordered_set<Location> v_set;
    for(auto &p:paths_s){
        v_set.insert(p.back());

    }
    std::cout<<v_set.size()<<" "<<paths_s.size()<<std::endl;
    assert(v_set.size()==paths_s.size());
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



void RUBIK12::prepare_cell(int xmin,int ymin,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_start,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_goal){
    std::vector<Location> vertices;

    auto greaterX=[](Location v1,Location v2){
        return v1.y*1e5+v1.x<v2.y*1e5+v2.x;
    };
    auto greaterY=[](Location v1,Location v2){
        return v1.x*1e5+v1.y<v2.x*1e5+v2.y;
    };
    auto get_index=[](std::vector<Location> &vertices,Location &v){
        auto it=find(vertices.begin(),vertices.end(),v);
        int id=distance(vertices.begin(),it);
        return id;
    };
  
    Agents robots_start=agents_start[{int(xmin/2),int(ymin/2)}];
    Agents robots_goal=agents_goal[{int(xmin/2),int(ymin/2)}];
    
    for(int i=xmin;i<xmin+2;i++){
        for(int j=ymin;j<ymin+2;j++){
            vertices.push_back(Location(i,j));
        }
    }

    // if(orientation=='x') std::sort(vertices.begin(),vertices.end(),greaterX);
    // else std::sort(vertices.begin(),vertices.end(),greaterY);
  
    std::sort(robots_start.begin(),robots_start.end(),[](const Agent_p a1,const Agent_p a2){
        return a1->current<a2->current;
    });
    std::sort(robots_goal.begin(),robots_goal.end(),[](const Agent_p a1,const Agent_p a2){
        return a1->goal<a2->goal;
    });
   
    if(robots_start.size()!=2){
        std::cout<<robots_start.size()<<std::endl;
    }
    assert(robots_start.size()==2);
    assert(robots_goal.size()==2);
    auto id_order1=[&vertices,&get_index](Agent_p a1,Agent_p a2){
        int id1=get_index(vertices,a1->current);
        int id2=get_index(vertices,a2->current);
        return id1<id2;
    };
    std::sort(robots_start.begin(),robots_start.end(),id_order1);
    int id1=get_index(vertices,robots_start[0]->current);
    int id2=get_index(vertices,robots_start[1]->current);
    auto solution=data_map[{id1,id2}];


    auto id_order2=[&vertices,&get_index](Agent_p a1,Agent_p a2){
        int id1=get_index(vertices,a1->goal);
        int id2=get_index(vertices,a2->goal);
        return id1<id2;
    };
    std::sort(robots_goal.begin(),robots_goal.end(),id_order2);
    id1=get_index(vertices,robots_goal[0]->goal);
    id2=get_index(vertices,robots_goal[1]->goal);
    //std::cout<<id1<<"  "<<id2<<std::endl;
    auto solution2=data_map[{id1,id2}];

    for(int i=0;i<robots_start.size();i++){
        for(int j=0;j<solution[i].size();j++){
            robots_start[i]->path.push_back(vertices[solution[i][j]]);
            robots_start[i]->current=robots_start[i]->path.back();
        }
        // assert(vertices[solution2[i][0]]==robots_goal[i]->goal);
        for(int j=0;j<solution2[i].size();j++){
            robots_goal[i]->inter_goal_path.insert(robots_goal[i]->inter_goal_path.begin(),vertices[solution2[i][j]]);
        }
        // assert(robots_goal[i]->inter_goal_path.back()==robots_goal[i]->goal);
        robots_goal[i]->inter_goal=robots_goal[i]->inter_goal_path[0];
        // robots_goal[i]->inter_goal_path.erase(robots_goal[i]->inter_goal_path.begin());
        // assert(robots_goal[i]->inter_goal_path.back()==robots_goal[i]->goal);
    }
}

//using lba to rearrange the assignments
void RUBIK12::matching_heuristic(){
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


void RUBIK12::fillPaths(){
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


void RUBIK12::labp_matching_greedy(std::unordered_map<int,Agents> &column_dict,std::unordered_map<int,int>&matching,
        std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row){
    using weighted_edge=std::tuple<int,int,double>;
    std::vector<weighted_edge> costEdge;
    const int max_inf=1e6;
    std::vector<int> column_id;
    for(auto const &column_i:column_dict){
        column_id.push_back(column_i.first);
        
    }  
    printf("xmax=%d,ymax=%d\n",xmax,ymax);
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
                // printf("(%d--->%d)\n",i,j);
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

void RUBIK12::x_fitting(){
    for(auto &agent:agents){
        agent->intermediate.set_value(agent->inter_goal.x,agent->current.y);
    }
}

void RUBIK12::y_fitting(){
    for(auto&agent: agents){
        agent->intermediate.set_value(agent->current.x,agent->inter_goal.y);
    }
}

void RUBIK12::x_shuffle(){
    for(int i=0;i<ymax;i+=2){
        Agents robots;
        for(auto &agent :agents){
            if(agent->current.y<=i+1&&agent->current.y>=i) robots.emplace_back(agent);

        }
        MergeSorter swapper(robots,{0,xmax-1},{i,i+1},'x');
        swapper.reconfigure();
    }

    fillPaths();
}

void RUBIK12::y_shuffle(){
    for(int i=0;i<xmax;i+=2){
        Agents robots;
        for(auto &agent:agents){
            if(agent->current.x<=i+1&&agent->current.x>=i) robots.emplace_back(agent);
        }
        MergeSorter swapper(robots,{i,i+1},{0,ymax-1},'y');
        swapper.reconfigure();
    }
    fillPaths();
}



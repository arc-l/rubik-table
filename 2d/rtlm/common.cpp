#include "common.h"

#include <cmath>
#include<algorithm>
// #include <yaml-cpp/yaml.h>
#include "json.hpp"




double distance(Location v1,Location v2){
    return abs(v1.x-v2.x)+abs(v1.y-v2.y);
}


// void read_from_yaml(std::string filename,std::vector<Location>& starts,std::vector<Location> &goals,std::unordered_set<Location> &obstacles){
//     YAML::Node config=YAML::LoadFile(filename);
 
//     for(const auto&node:config["map"]["obstacles"]){
//         obstacles.insert(Location(node[0].as<int>(),node[1].as<int>()));
//     }
//     for (const auto&node:config["agents"]){
//         const auto &start=node["start"];
//         const auto &goal=node["goal"];
//         starts.push_back(Location(start[0].as<int>(),start[1].as<int>()));
//         goals.push_back(Location(goal[0].as<int>(),goal[1].as<int>()));
//     }
// }



// Agents agents_from_yaml(std::string filename){
//     YAML::Node config=YAML::LoadFile(filename);
//     Agents agents;
//     for(const auto&node :config["agents"]){
//         const auto &start=node["start"];
//         const auto &goal=node["goal"];
//         std::string name=node["name"].as<std::string>();
//         int group_id;
//         try{
//             group_id=node["group_id"].as<int>();
//         }
//         catch(std::exception &e){
//             group_id=-1;
//         }
        
//         Agent_p agent_i=std::make_shared<Agent>();
//         agent_i->id=std::stoi(name.substr(5));
//         agent_i->group_id=group_id;
//         // agent_i->name=name;
//         agent_i->start.x=start[0].as<int>();
//         agent_i->start.y=start[1].as<int>();
//         agent_i->goal.x=goal[0].as<int>();
//         agent_i->goal.y=goal[1].as<int>();
//         agent_i->current.x=start[0].as<int>();
//         agent_i->current.y=start[1].as<int>();
//         agent_i->path.push_back(agent_i->start);
//         agents.push_back(agent_i);      
//     }
//     return agents;
// }

//only for grid maps that have few obstacles
int evaluate_makespan_lb(Paths &paths){
    int makespan_lb=0;
    for(auto&p:paths){
        auto start=p[0];
        auto goal=p.back();
        makespan_lb=std::max(makespan_lb,(int)distance(start,goal));
    }
    return makespan_lb;
}

//only for grid maps that have few obstacles
int evaluate_soc_lb(Paths &paths){
    int soc_lb=0;
    for(auto&p:paths){
        auto start=p[0];
        auto goal=p.back();
        soc_lb+=(int)distance(start,goal);
    }
    return soc_lb;
}

void save_yaml(std::string filename, Agents &agents,double runtime){
    std::ofstream out(filename);
    out<<"statistics:"<<std::endl;
    // out<<" makespan: "<<evaluate_makespan(agents)<<std::endl;
    out<<" runtime: "<<runtime<<std::endl;
    out<<" schedule: "<<std::endl;
    for (size_t a = 0; a < agents.size(); ++a) {

      out << "  agent" << a << ":" << std::endl;
      for (int i=0;i<agents[a]->path.size();i++) {
        out << "    - x: " << agents[a]->path[i].x << std::endl
            << "      y: " << agents[a]->path[i].y << std::endl
            << "      t: " << i << std::endl;
      }
    }
    out.close();
}

int evaluate_makespan(Paths &paths){
    int makespan=0;
    for(auto &path :paths){
        makespan=std::max(makespan,(int)path.size());
    }   
    return makespan;
}

int evaluate_soc(Paths &paths){
    int soc=0;
    for(auto &path :paths){
        soc+=path.size();
    }   
    return soc;
}


void fill_paths(Agents &agents){
    int makespan=0;
    for(auto &agent:agents){
        makespan=std::max((int)agent->path.size(),makespan);
    }
    for(auto& agent:agents){
        while ((*agent).path.size()<makespan){
            (*agent).path.push_back((*agent).path.back());
        }
    }
}

bool check_feasible(Paths &paths){
    int makespan=evaluate_makespan(paths);

    //vertex conflicts
    for(int t=0;t<makespan;t++){
        std::unordered_set<Location> used_vertices;
        for(auto &path : paths){
            if(used_vertices.find(path[t])!=used_vertices.end()){
                std::cout<<"Vertex conflict happened at time step "<<t<<" at"<<path[t]<<std::endl;
                return false;
            }
            else{
                used_vertices.insert(path[t]);
            }
        }
    }

    //edge conflicts
    for(int t=0;t<makespan-1;t++){
        for(int i=0;i<paths.size();i++){
            for(int j=i+1;j<paths.size();j++){
                if(paths[i][t]==paths[j][t+1]&&paths[i][t+1]==paths[j][t]){
                    std::cout<<"Edge conflict happened"<<std::endl;
                    return false;
                }
            }
        }
    }
    return true;
}

bool check_cycle(Paths &paths){
    using adjList=std::unordered_map<int,int>;
    auto getId=[](Location v){
        return v.x*1000+v.y;
    };
    auto getLocation=[](int vid){
        int x=vid/1000;
        int y=vid%1000;
        return Location(x,y);
    };
    using dfsFunc=std::function<bool(int,adjList&,std::unordered_set<int> &,std::unordered_set<int> &)>;
    dfsFunc DFS_check_v=[& DFS_check_v](int vid,adjList &graph,
        std::unordered_set<int> &visited,std::unordered_set<int>&recStack){
            visited.insert(vid);
            recStack.insert(vid);
            if(graph.find(vid)!=graph.end()){
                auto nextV=graph[vid];
                if(visited.find(nextV)==visited.end()){
                    if(DFS_check_v(nextV,graph,visited,recStack)==true) return true;
                }
                else if(recStack.find(nextV)!=recStack.end()) return true;
            }
            recStack.erase(vid);
            return false;


    };
    auto DFS_check_cycle=[&DFS_check_v](adjList & graph){
        std::unordered_set<int>visited;
        std::unordered_set<int>recStack;
        for(auto &pair:graph){
            auto vid=pair.first;
            if(visited.find(vid)==visited.end()){
                if(DFS_check_v(vid,graph,visited,recStack)==true)
                    return true;
            }
        }
        return false;
    };
    int makespan=evaluate_makespan(paths);
   
    for(int t=0;t<makespan-1;t++){
        adjList graph;
        for(int i=0;i<paths.size();i++){
            int li=paths[i].size();
            if(t+1>=li) continue;
            if(paths[i][t]==paths[i][t+1]) continue;
            int vt=getId(paths[i][t]);
            int vtt=getId(paths[i][t+1]);
            graph[vt]=vtt;
        }
        std::cout<<"checking time step "<<t<<std::endl;

        if(DFS_check_cycle(graph)==true) {
            return true;
        }
    }
    return false;
}


bool check_current(Agents &agents){
    std::unordered_set<Location> used_vertices;
    for(auto &agent:agents){
        if(used_vertices.find((*agent).current)!=used_vertices.end()){
            std::cout<<"Vertex conflict happend"<<std::endl;
            return false;
        }
        else{
            used_vertices.insert((*agent).current);
        }
    }
    return true;
}

void shrink_paths(Paths &paths){
    for(int i=0;i<paths.size();i++){
        if(paths[i].size()<=1) continue;
        while(paths[i][paths[i].size()-1]==paths[i][paths[i].size()-2]){
            paths[i].pop_back();
        }
    }
}

void fill_paths(Paths &paths){
    int makespan=0;
    for(int i=0;i<paths.size();i++){
        makespan=std::max(makespan,(int)paths[i].size());
    }
    for(int i=0;i<paths.size();i++){
        while(paths[i].size()<makespan){
            paths[i].push_back(paths[i][paths[i].size()-1]);
        }
    }
}

void save_result_as_txt(std::string filename,Agents &agents,double runtime,bool save_paths){
    Paths paths;
    for(auto const&agent:agents){
        paths.push_back(agent->path);
    }
    save_paths_as_txt(filename,paths,runtime,save_paths);
}




void save_paths_as_txt(std::string filename,Paths &paths,double runtime,bool save_paths){
    std::ofstream out(filename);
    int makespanLB=evaluate_makespan_lb(paths);
    int socLB=evaluate_soc_lb(paths);
    int soc=evaluate_soc(paths);
    int makespan=evaluate_makespan(paths);
    out<<"soc="<<soc<<std::endl;
    out<<"lb_soc="<<socLB<<std::endl;
    out<<"makespan="<<makespan<<std::endl;
    out<<"lb_makespan="<<makespanLB<<std::endl;
    out<<"comp_time="<<(int)(runtime*1000)<<std::endl;
    // out<<"highLevelExpanded="<<mapf.highLevelExpanded()<<std::endl;
    // out<<"lowLevelExpanded="<<mapf.lowLevelExpanded()<<std::endl;
    if(save_paths==false) return;
    out<<"solution="<<std::endl;
    for(int i=0;i<paths.size();i++){
      out<<i<<":";
      for(const auto& state:paths[i]){
        out<<"("<<state.x<<","<<state.y<<"),";
      }
      out<<std::endl;
    }
}

Problem read_from_txt(std::string map_name,std::string scen_name){
    std::string line;
    std::smatch results;
    std::ifstream map_file(map_name);
    std::ifstream scen_file(scen_name);
    std::regex r_comment=std::regex(R"(#.+)");
    std::regex r_agents=std::regex(R"(agents=(\d+))");
    std::regex r_max_comp_time=std::regex(R"(max_comp_time=(\d+))");
    std::regex r_sg=std::regex(R"((\d+),(\d+),(\d+),(\d+))");
    std::regex r_height=std::regex(R"(height\s(\d+))");
    std::regex r_width=std::regex(R"(width\s(\d+))");
    std::regex r_map=std::regex(R"(map)");
    if(!map_file||!scen_file){
        std::cout<<"File not found"<<std::endl;
        exit(0);
    }
    Configs starts;
    Configs goals;
    double max_comp_time;
    int xmax;
    int ymax;
    Obstacles obstacles;

    while(getline(scen_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();
        //comment
        if(std::regex_match(line,results,r_comment)){
            continue;
        }

        //read map


        //max comp_time
        if(std::regex_match(line,results,r_max_comp_time)){
            max_comp_time=std::stod(results[1].str());
            continue;
        }

        //read initial/goal nodes
        if(std::regex_match(line,results,r_sg)){
            int x_s=std::stoi(results[1].str());
            int y_s=std::stoi(results[2].str());
            int x_g=std::stoi(results[3].str());
            int y_g=std::stoi(results[4].str());
            starts.push_back(Location(x_s,y_s));
            goals.push_back(Location(x_g,y_g));
            continue;
        }
    }

    while(getline(map_file,line)){
        //std::cout<<line<<std::endl;
        if(*(line.end()-1)==0x0d) line.pop_back();
        if(std::regex_match(line,results,r_height)){
            xmax=std::stoi(results[1].str());
            continue;
        }
        if(std::regex_match(line,results,r_width)){
            ymax=std::stoi(results[1].str());
            continue;
        }
        if(std::regex_match(line,results,r_map)){
            break;
        }
    }
  
    assert(xmax>0&&ymax>0);
    int i=0;
    while(getline(map_file,line)){
        if(*(line.end()-1)==0x0d) line.pop_back();
        for(int j=0;j<ymax;j++){
            char s=line[j];
            if(s=='T' or s=='@'){
                obstacles.insert(Location(i,j));
            }
        }
        i++;
    }
    map_file.close();
    scen_file.close();
    Problem new_problem(starts,goals,xmax,ymax,obstacles,max_comp_time);
    return new_problem;
}


void agents_from_txt(std::string scen_name,Agents &agents, int &dim){
    std::string line;
    std::smatch results;
    std::ifstream scen_file(scen_name);
    std::regex r_comment=std::regex(R"(#.+)");
    std::regex r_agents=std::regex(R"(agents=(\d+))");
    std::regex r_max_comp_time=std::regex(R"(max_comp_time=(\d+))");
    std::regex r_sg=std::regex(R"((\d+),(\d+),(\d+),(\d+))");
    std::regex r_height=std::regex(R"(height\s(\d+))");
    std::regex r_width=std::regex(R"(width\s(\d+))");
    std::regex r_map=std::regex(R"(map_file=(\d+)x(\d+).map)");
    if(!scen_file){
        std::cout<<"File not found"<<std::endl;
        exit(0);
    }
    Configs starts;
    Configs goals;
    double max_comp_time;
    // int xmax;
    // int ymax;

    while(getline(scen_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();
        //comment
        if(std::regex_match(line,results,r_comment)){
            continue;
        }

        //read map
        if(std::regex_match(line,results,r_map)){
            dim=std::stoi(results[1].str());
            // std::cout<<dim<<" x "<<dim<<std::endl;
            continue;
        }


        //max comp_time
        if(std::regex_match(line,results,r_max_comp_time)){
            max_comp_time=std::stod(results[1].str());
            continue;
        }

        //read initial/goal nodes
        if(std::regex_match(line,results,r_sg)){
            int x_s=std::stoi(results[1].str());
            int y_s=std::stoi(results[2].str());
            int x_g=std::stoi(results[3].str());
            int y_g=std::stoi(results[4].str());
            starts.push_back(Location(x_s,y_s));
            goals.push_back(Location(x_g,y_g));
            continue;
        }
    }

    scen_file.close();
    agents.clear();
    for(int i=0;i<starts.size();i++){
        Agent_p agent_i=std::make_shared<Agent>();
        agent_i->start=starts[i];
        agent_i->id=i;
        agent_i->goal=goals[i];
        agent_i->current=starts[i];
        agent_i->path.push_back(starts[i]);
        agents.push_back(agent_i);
    }

}


void tokenize(std::string const &s, std::string const & delim,
            std::vector<std::string> &out){
 
    auto start = 0U;
    auto end = s.find(delim);
    std::string token;
    while (end!= std::string::npos) {
        token = s.substr(start, end - start);
        start = end + delim.length();
        // std::cout << token << std::endl;
        out.push_back(token);
        end = s.find(delim, start);
    }
    token = s.substr(start, end - start);
    out.push_back(token);

}

Paths read_paths_from_txt(std::string sol_name,double &comp_time){
    std::ifstream sol_file(sol_name);
    std::string line;
    std::regex r_comp_time=std::regex(R"(comp_time=(\d+))");
    std::smatch results;
    Paths paths;
    int k=0;
    using strings=std::vector<std::string>;
    while(getline(sol_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();

        if(std::regex_match(line,results,r_comp_time)){
            comp_time=std::stod(results[1].str())/1000.;
            continue;
        }
        strings substrs;
        tokenize(line,":",substrs);
        if(substrs.size()<2) continue;
        Path path_i;
        paths.push_back(path_i);
        // for(auto s:substrs) std::cout<<s<<std::endl;
        auto path_string=substrs[1];
        strings vertices;
        tokenize(path_string,"),",vertices);
        for(int i=0;i<vertices.size()-1;i++){
            std::string v=vertices[i];
            // std::cout<<v<<" ";
            v.erase(0,1);
            // std::cout<<v<<" \n";
            strings xy;
            tokenize(v,",",xy);
            // std::cout<<xy[0]<<" ,"<<xy[1]<<std::endl;
            paths[k].emplace_back(Location(std::stoi(xy[0]),std::stoi(xy[1])));
        }
        k++;
    }
    return paths;
}

void assign_tasks(Agents &agents){
    Configs tmp_goal;
    for(auto &agent:agents){
        tmp_goal.emplace_back(agent->goal);
    }
    using weighted_edge=std::tuple<int,int,double>;
    std::vector<weighted_edge>costEdge;
    for(int i=0;i<agents.size();i++){
        for(int j=0;j<agents.size();j++){
            costEdge.push_back({i,j,distance(agents[i]->start,agents[j]->goal)});
        }
    }
    std::vector<int> assignment;
    double cost=lba_sparse(costEdge,assignment);
    for(int i=0;i<agents.size();i++){
        agents[i]->goal=tmp_goal[assignment[i]];
    }

}


void save_paths_as_json(std::string filename,Paths &paths,double runtime){
    nlohmann::json data_json;
    std::vector<std::vector<std::vector<int>>> path_vector(paths.size(),std::vector<std::vector<int>>());
    for(int i=0;i<paths.size();i++){        
        for(auto &v:paths[i]){
            path_vector[i].push_back({v.x,v.y});
        }
    }
    data_json["paths"]=path_vector;
    data_json["runtime"]=runtime;
    std::ofstream file_pointer(filename);
    file_pointer<<data_json<<std::endl;
}
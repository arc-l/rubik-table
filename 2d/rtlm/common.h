#ifndef common_h
#define common_h

/**
 * @file common.h
 * @author Greaten (greaten666@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <tuple>
#include <vector>
#include "labp.h"
#include <limits>
#include <memory>
#include <chrono>
#include <cmath>
#include <regex>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
typedef std::pair<int,int> point2d;


struct Location{
    Location(){x=0;y=0;}
    Location(int x,int y):x(x), y(y){}
    int x;
    int y;
    bool operator<(const Location&other) const{
        return std::tie(x,y)<std::tie(other.x,other.y);
    }
    bool operator==(const Location& other) const{
        return std::tie(x,y)==std::tie(other.x,other.y);
    }
    bool operator!=(const Location &other) const{
        return std::tie(x,y)!=std::tie(other.x,other.y);
    }

    void set_value(int x_,int y_){
        x=x_;
        y=y_;
    }
    friend std::ostream& operator<<(std::ostream&os,const Location &c){
        return os<<"("<<c.x<<","<<c.y<<")";
    }
};



namespace std{
template<>
struct hash<Location>{
    size_t operator()(const Location&s)const{
        size_t seed=0;
        boost::hash_combine(seed,s.x);
        boost::hash_combine(seed,s.y);
        return seed;
    }
};
}

using Configs=std::vector<Location>;
using Path=std::vector<Location>;
using Paths=std::vector<Path>;
using Obstacles=std::unordered_set<Location>;



struct Agent{
    Agent(){};
    Agent(int id,Location &start,Location &goal){
        this->id=id;
        this->start=start;
        this->goal=goal;
        this->current=start;
        this->path.push_back(start);
    }
    // std::string name;
    int id;
    int group_id;
    Location start;
    Location goal;
    Location current;
    Location intermediate;
    Location inter_goal;
    
    bool operator==(const Agent &other)const{
        return this->id==other.id;
    }
    std::vector<Location> path;
    std::vector<Location> inter_path;
    std::vector<Location> inter_goal_path;
    std::vector<Location> inter_path_u;
    Location inter2;
    friend std::ostream& operator<<(std::ostream &os,const Agent &c){
        return os<<"Agent"<<"{"<<"name: agent"<<c.id<<", current:"<<c.current<<", goal:"<<c.goal<<"}";//", intermediate"<<c.intermediate<<", inter_goal"<<c.inter_goal<<"}";
    }
};

using Agent_p=std::shared_ptr<Agent>;
using Agents=std::vector<Agent_p>;

// bool DFS_check_cycle(std::unordered_map<int,int> &adj_list);
bool check_cycle(Paths &paths);
// typedef std::vector<Agent*> Agents;
int evaluate_makespan(Agents &agents);
int evaluate_makespan(Paths&paths);
int evaluate_makespan_lb(Paths &paths);
int evaluate_soc_lb(Paths &paths);
int evaluate_soc(Paths &paths);
void shrink_paths(Paths &paths);
void fill_paths(Paths &paths);
void assign_tasks(Agents &agents);
void save_yaml(std::string filename,Agents &agents,double runtime);
void save_result_as_txt(std::string filename,Agents &agents,double runtime,bool save_paths=false);
void save_paths_as_txt(std::string filename,Paths&paths,double runtime,bool save_paths=false);
void save_paths_as_json(std::string filename,Paths &paths,double runtime=0);

Agents agents_from_yaml(std::string filename);
void agents_from_txt(std::string filename,Agents &,int &xmax);
void read_from_yaml(std::string filename,std::vector<Location>& starts,std::vector<Location> &goals,std::unordered_set<Location> &obstacles,int* dim);
void assign_tasks(Agents& agents);
bool check_feasible(Paths &paths);
double distance(Location v1,Location v2);
void fill_paths(Agents&agents);
bool check_config_valid(Agents &agents);


///////////////////////////read from txt//////////////////////////

class Problem{
public:
    Configs starts;
    Configs goals;
    Obstacles obstacles;
    Problem (Configs &starts,Configs &goals,int xmax,int ymax,Obstacles &obstacles,double max_computation_time=300){
        this->starts=starts;
        this->goals=goals;
        this->obstacles=obstacles;
        this->max_computation_time=max_computation_time;
        this->xmax=xmax;
        this->ymax=ymax;
    }
    int xmax;
    int ymax;
    double max_computation_time;
};

Problem read_from_txt(std::string map_file,std::string scen_file);
Agents read_agents_txt(std::string map_file,std::string scen_file);
Paths read_paths_from_txt(std::string sol_file,double &comp_time);
void tokenize(std::string const &str, std::string const & delim,std::vector<std::string> &out);
/////////////////////////////////////////////////////////////

#endif
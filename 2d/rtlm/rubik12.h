#ifndef RUBIK_12_H
#define RUBIK_12_H


#include"rubik.h"
#include "common.h"
#include "labp.h"
#include "json.hpp"
#include "mp.h"


class RUBIK12:public RUBIK{
using json=nlohmann::json;
public:
    RUBIK12(Agents&agents, point2d dims);

    void solve();
    void prepare_cell(int xmin,int ymin,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_start,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_goal);
    void local_prepare();
    void load_data(std::string filename);
    void random_to_balanced();
    void matching();
    void matching_heuristic();
    void x_shuffle();
    void y_shuffle();
    void x_fitting();
    void y_fitting();
    void fillPaths();
    

    void set_umapf(bool use_umapf){
        this->use_umapf=use_umapf;
    }
   
    void useLba(bool use_lba ){
        this->use_lba=use_lba;
    }


protected:
    void m_perfect_matching(std::unordered_map<int,Agents>&column_dict, std::unordered_map<int,int>&matching,std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row);
    void labp_matching_greedy(std::unordered_map<int,Agents>&column_dict, std::unordered_map<int,int>&matching,std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row);
    json data_dict;
    Configs balanced_starts;
    Configs balanced_goals;
    bool use_umapf=false;
    bool use_lba=true;
    std::unordered_map<point2d,std::vector<std::vector<int>>,boost::hash<point2d>> data_map;
    void get_data_map();

    void append_umapf_paths();
};

#endif
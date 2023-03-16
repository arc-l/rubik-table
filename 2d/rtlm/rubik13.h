#ifndef _rubik13_h
#define _rubik13_h
#include<vector>
#include<iostream>
#include"common.h"
#include "json.hpp"
#include "rubik.h"



//only for square currently
class RUBIK13:public RUBIK{
using json=nlohmann::json;
public:
    RUBIK13(Agents &agents,point2d dims);
    void solve();
    virtual void prepare_cell(int xmin,int ymin,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_start,std::unordered_map<point2d,Agents,boost::hash<point2d>> &agents_goal);
    virtual void local13_prepare();
    void load_data(std::string filename);
    void random_to_balanced();
    virtual void matching();
    void matching_heuristic();
    virtual void x_shuffle();
    virtual void y_shuffle();
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
    virtual void labp_matching_greedy(std::unordered_map<int,Agents>&column_dict, std::unordered_map<int,int>&matching,std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row);
    json data_dict;
    Configs balanced_starts;
    Configs balanced_goals;
    bool use_umapf=false;
    bool use_lba=true;
   
    
};



//the improved version
class IRUBIK13_H1:public RUBIK13{
public :
    IRUBIK13_H1(Agents &agents,point2d dim):RUBIK13(agents,dim){};
    void x_shuffle();
    void y_shuffle();
};


//the improved version 2
class IRUBIK13_H2:public RUBIK13{
public:
    IRUBIK13_H2(Agents &agents,point2d dim):RUBIK13(agents,dim){};
    void x_shuffle();
    void y_shuffle();
};

#endif
/**
 * @file rubikILP.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include "common.h"
#include "rubik13.h"
#include "gurobi_c++.h"

static GRBEnv genv;

class RTH_ILP:public RUBIK13{
public:
    RTH_ILP(Agents &agents, point2d dims):RUBIK13(agents,dims){}   
    void matching(); 
protected:
    using VarMap=std::unordered_map<point2d,GRBVar,boost::hash<point2d>>;
    void prepare_model(GRBModel &model,VarMap &var_map);
};


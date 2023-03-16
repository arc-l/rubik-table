/**
 * @file rubik_obs.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef RTH_OBS_H
#define RTH_OBS_H

#include "common.h"

#include "labp.h"
#include "json.hpp"
#include "rubik13.h"

class RTH_obs:public RUBIK13 {
using json=nlohmann::json;
public:
    RTH_obs(Agents &agents,point2d dims);
    void x_shuffle();
    void y_shuffle();
    void matching();
protected:
    void labp_matching_greedy(std::unordered_map<int,Agents>&column_dict, std::unordered_map<int,int>&matching,std::unordered_map<point2d,Agent_p,boost::hash<point2d>>& arranged_agents,int row);
};

#endif
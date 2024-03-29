/**
 * @file common.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include<iostream>
#include<memory>
#include<unordered_map>
#include<unordered_set>
#include<vector>
#include<queue>
#include<fstream>

#include <chrono>
#include <cmath>
#include <regex>
#include <boost/functional/hash.hpp>
#include"json.hpp"
#include <stack>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

const int cell_size=3;

extern nlohmann::json data2d;

class Location3d{
public:
    Location3d(int _id,int _x,int _y,int _z):x(_x),y(_y),z(_z),id(_id){}
    int x,y,z,id;
    int manhattan_dist(Location3d *other){
        return abs(x-other->x)+abs(y-other->y)+abs(z-other->z);
    }
    std::string print(){
        return "("+std::to_string(x)+","+std::to_string(y)+","+std::to_string(z)+")";
    }
    // std::vector<Location3d*> neighbors;
};

class Grids3d{
public:
    Grids3d(int xmax,int ymax,int zmax);
    ~Grids3d();
    std::vector<Location3d *> getNeighbors(Location3d *);
    Location3d*getVertex(int x,int y, int z);
    Location3d*getVertex(int id);
    int xmax,ymax,zmax;
    int getNodesSize(){
        return nodes.size();
    }
    std::vector<Location3d*> getNodes(){
       return nodes; 
    }

    void add_obstacles(Location3d *obs){
        this->obstacles.insert(obs);
    }
protected:
    std::vector<Location3d *> nodes;
    std::set<Location3d *> obstacles;
};



using Path3d=std:: vector<Location3d*>;
using Paths3d=std::vector<Path3d>;
using Configs=std::vector<Location3d*>;


class Robot{
public:
    Robot(int _id,Location3d* _current,Location3d *_goal):id(_id),current(_current),goal(_goal){
        start=current;
        inter_goal=goal;
        umapf_goal=goal;
        path.push_back(current);
    }
    int id;
    Location3d *start;            //true start
    Location3d *current;        //current state
    Location3d *goal;           //true goal
    Location3d *intermediate=nullptr;   //next config it needs to go
    Location3d *inter_goal; //2d fat column shuffle intermediate goal
    Location3d *inter2=nullptr;  //intermediate in the swapper
    Location3d *umapf_goal;    //umapf goal
    Path3d umapf_goal_path;     //umpaf_goal path
    Path3d path;                //true path
    Path3d inter_path;          //umpaf inter_goal
    bool isVirtual=false;
    
    
};

using Robots=std::vector<Robot*>;

void read_instances(std::string file_name,Robots &robots,Grids3d *&graph);
void save_solutions(std::string file_name,Robots&robots,double runtime,bool save_paths);
void check_feasible(Robots &);
void check_feasible_bruteForce(Robots &);
void format_paths(Paths3d &paths);
void evaluate_result(Robots &robots,int &makespan,int &makespanLB,int &soc,int &socLB);   
void fill_paths(Robots &robots,int makespan=-1);
void shrink_paths(Robots&robots);
void print_one_path(Path3d &path);
void read_starts_goals(std::string filename,Configs &starts,Configs &goals,Grids3d*&graph);
void add_virtual_robots(Robots &robots,Grids3d*graph);
void remove_virtual_robots(Robots &robots);
void construct_building_grids(Robot &robots,Grids3d*&graph);
void save_solutions_as_json(std::string file_name,Robots &robots,double runtime=0,bool save_paths=true);
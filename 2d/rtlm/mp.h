#ifndef MP_H
#define MP_H
#include"common.h"
#include"json.hpp"


class MotionPrimitive{
public:   
    MotionPrimitive(){};
    virtual ~MotionPrimitive(){};
    MotionPrimitive(Agents& agents,point2d xrange,point2d yrange,char orientation='x'){};
   
    void reconfigure();
    void fill_paths();
    virtual void reconfigure_x(){};
    virtual void reconfigure_y(){};
protected:
    Agents agents;
    int xmax;
    int ymax;
    int xmin;
    int ymin;

    char orientation='x';
};


class HW_MP:public MotionPrimitive{
using json=nlohmann::json;
public:
    HW_MP();
    HW_MP(Agents& agents,point2d xrange,point2d yrange,char orientation='x');
    void reconfigure_x();
    void reconfigure_y();
    // void reconfigure();
    // void fill_paths();
    // void assign_tasks();
    
protected:
    void prepare(int xmin,int ymin);
    int get_id(Location vertex);
    std::unordered_map<int,Location> id_vertex_map;
    Location get_vertex(int id);
    int get_makespan();
    json data_dict;

};


//line shuffle with obstacle at the center
class HW_MP_obs:public HW_MP{
using json=nlohmann::json;
public:
    HW_MP_obs(Agents &agents,point2d xrange,point2d yrange,char orientation='x');
    // void reconfigure_x();
    // void reconfigure_y();
protected:
  

};


//line shuffle once it is ready
class HW_MP_H1:public HW_MP{
public:
    HW_MP_H1(Agents &agents,point2d xrange,point2d yrange,char orientation='x')
        :HW_MP(agents,xrange,yrange,orientation){};
    void reconfigure_x();
    void reconfigure_y();
};


//line shuffle using blocks
class HW_MP_H2:public HW_MP{
public:
    HW_MP_H2(Agents &agents,point2d xrange,point2d yrange,char orientation='x')
        :HW_MP(agents,xrange,yrange,orientation){};
    void better_path();
    void sort_agents();
    bool next_move(int t);
    void reconfigure_x();
    void reconfigure_y();
private:
    Agents up_agents,down_agents;
    std::unordered_map<int,int> safe_time;
    std::unordered_map<int,int>cell_safe_time;
    std::unordered_map<int,int> blocked_time;
};

//line sort MP

class MergeSorter:public MotionPrimitive{
using json=nlohmann::json;
public:
    MergeSorter(Agents &agents,point2d xrange,point2d yrange,char orientation='x');
    void reconfigure_x();
    void reconfigure_y();
    void merge(int left,int right);
    void mergeSort(int left,int right);
protected:
    void mergeX(int left,int right);
    void mergeY(int left,int right);
    // void merge();
    
    void prepare(int min_x,int min_y);
    void fill_some_paths(int left,int right);
    // std::unordered_map<int,Location> inter_goals;           //store the intermediate goals
    // void helper(Agents &agents,int min_x,int min_y);
    std::unordered_map<point2d,std::vector<std::vector<int>>,boost::hash<point2d>> data_map;
};




#endif
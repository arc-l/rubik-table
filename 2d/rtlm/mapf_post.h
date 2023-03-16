#pragma once

#include "common.h"
#include<iostream>
#include<memory>
#include<unordered_map>
#include<queue>





struct TPG_node{
    TPG_node(Location u,double t,int id){
        this->u=u;
        this->t=t;
        this->id=id;
    };
    TPG_node(){};
    Location u;             //vertex
    double t;               //time step
    int id;                 //agent id

    std::string toString(){
        return std::to_string(u.x)+" "+std::to_string(u.y)+" "+std::to_string(t)+" "+std::to_string(id);
    }
    bool operator ==(const TPG_node & other)const{
        return u==other.u &&t==other.t &&other.id==id;
    }
};

namespace std{
template<>
struct hash<TPG_node>{
    size_t operator()(const TPG_node&s)const{
        size_t seed=0;
        boost::hash_combine(seed,s.u.x);
        boost::hash_combine(seed,s.u.y);
        boost::hash_combine(seed,s.t);
        boost::hash_combine(seed,s.id);
        return seed;
    }
};
}

class TPG{
    using TPG_node_p=std::shared_ptr<TPG_node>;
    using weighted_edge=std::tuple<TPG_node,TPG_node,double>;
    using Direction=std::pair<int,int>;
    public: 
        TPG();
        TPG(Paths &paths,double rotation_time);
        void solve();
        void build();
        void build_faster();
        void bellman_ford();
        void faster_bellman();
        int get_makespan_post();
        
    private:
        
        
        void compute_directions();
        void shrink();
        void fill_paths();
   
        int num_agents;
        int makespan_old;
        int makespan_post;
        Paths old_paths;
        double rotation_time=1;
        std::vector<std::vector<Direction>> directions;
        TPG_node source;
        
        std::vector<weighted_edge> weighted_edges;
        using child_node=std::tuple<TPG_node,double>;
        using child_nodes=std::vector<child_node>;

        std::unordered_map<TPG_node,child_nodes> adj_list;
        std::unordered_map<TPG_node,child_node> type1_edge;
        // std::unordered_map<TPG_node_p,std::vector<std::tuple<TPG_node_p,double>>> adj_list; //adj list, (node ,cost)
        //std::unordered_map<std::string,TPG_node_p> node_map;
        std::unordered_set<TPG_node> node_set;
};

class MCP{

   
    using Direction=std::pair<int,int>;
   
    enum Action{
        Wait=0, Go=1, Rotate=2
    };

    struct Robot{
        Path plan;
        Action action;
        int rotation_count=0;
        int plan_id;
        Direction orientation;
        Robot(Path &old_path){
            this->plan=old_path;
        }
    };
    using Robot_p=std::shared_ptr<Robot>;
    using Robots=std::vector<Robot_p>;

    
    public:
        MCP(Paths &paths,int rotation_cost=1);
        void solve();
        void init_order();
        

    protected:
        Robots robots;
        int makespan_old;
        int makespan_post;
        int rotation_cost=1;
        void compute_directions(const Paths &paths);
        std::vector<std::vector<Direction>> directions;
        std::unordered_map<Location,std::queue<int>> order_queue;
       
};

// all the agents wait if a delay detected
class SynMove{

    using Direction=std::pair<int,int>;

    enum Action{
        Wait=0, Go=1, Rotate=2
    };

    struct Robot{
        Path plan;
        Action action;
        int rotation_count=0;
        int plan_id;
        Direction orientation;
        Robot(Path &old_path){
            this->plan=old_path;
        }
    };

    using Robot_p=std::shared_ptr<Robot>;
    using Robots=std::vector<Robot_p>;
public:


    SynMove(Paths& paths,int );
    void solve();
    int get_makespan_post(){
        return makespan_post;
    }

private:
    int makespan_old;
    int makespan_post;
    int rotation_cost=1;
    Robots robots;
    std::vector<std::vector<SynMove::Direction>> directions;
    void compute_directions(const Paths &paths);

};

class DependencyMove{
public:
    struct Robot{
        Robot(){};
        Robot(int id,Path &old_path);
        int id;
        Path old_path;
        int plan_id;
        Location goal;
        Location getNextPlan();
        Path execution;
        bool moved=false;
        bool reachedGoal(){
            return plan_id>=old_path.size();
        }
    };
    using Robots=std::vector<Robot*>;
    DependencyMove(Paths &paths);
    void sim();
    void save_results(std::string name,double runtime=0,bool save_paths=false);
    
    
protected:
    int makespan_old;
    int makespan_post;
    int soc_old;
    int soc_post;
    Location getVertex(int id);
    int getId(Location &vertex);
    bool moveOneStep();
    bool moveRobot(Robot* r,std::unordered_map<int,int> &next_occupancy);
    std::unordered_map<int,int> vertex_occupancy;              //store the agent id that occpupies v at time step t
    void findCornerFollowingConflicts(std::vector<int> &delayAgents);       //find the conflicts and delay them
    void findCornerFollowingConflictsFaster(std::vector<int> &delayAgents);       //find the conflicts and delay them
    void findFollowingConflicts(std::vector<int> &delayAgents);             //find the conflicted agents and delay then

    std::unordered_map<int,std::queue<int>> vertex_agent_order;      //store the order of events
    // bool isNextAvailable(int vid,std::unordered_map<int,int>&);               //check if at next timestep, vid is available
    bool isNextAgent(int ai, int vid);          //check if ai should be the agent that occupies vid at next timestep according to the original plan
    Robots robots;
    

} ;


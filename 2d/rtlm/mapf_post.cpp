#include"mapf_post.h"
#include<queue>
const std::pair<int,int> wait={0,0};
const std::pair<int,int> up={0,1};
const std::pair<int,int> down={0,-1};
const std::pair<int,int> left={-1,0};
const std::pair<int,int> right={0,1};


TPG::TPG(Paths &paths, double rotation_time){
    this->old_paths=paths;
    this->rotation_time=rotation_time;
    makespan_old=evaluate_makespan(old_paths);
    num_agents=old_paths.size();
}

void TPG::fill_paths(){
    for(int i=0;i<old_paths.size();i++){
        while(old_paths[i].size()<makespan_old){
            old_paths[i].push_back(old_paths[i].back());
        }
    }
}

void TPG::shrink(){
    for(int i=0;i<old_paths.size();i++){
        if(old_paths[i].size()<=1) continue;
        while(old_paths[i].back()==old_paths[i][old_paths[i].size()-2]){
            old_paths[i].pop_back();
        }
    }
}

void TPG::build(){
    auto direction_dist=[](Direction d1,Direction d2){
        return abs(d1.first-d2.first)+abs(d1.second-d2.second);
    };
    //compute directions
    compute_directions();
    std::cout<<"directions computed"<<std::endl;

    //add vertices and Type 1 edges
    for(int i=0;i<old_paths.size();i++){
        TPG_node v(old_paths[i][0],0,i);
        //node_map.insert({v->toString(),v});
        node_set.insert(v);
        adj_list[v]=child_nodes();
        for(int t=1;t<old_paths[i].size();t++){
            //remove waiting states
           if(!(old_paths[i][t]==old_paths[i][t-1])){
               TPG_node vt(old_paths[i][t],t,i);
               int tj=(int)(vt.t);
               //node_map.insert({vt->toString(),vt});
               node_set.insert(vt);
               int t0=(int)(v.t);
            //    std::cout<<tj<<" "<<t0<<std::endl;
               int direction_cost=direction_dist(directions[i][tj],directions[i][t0]);
               auto cost=std::max(direction_cost*rotation_time+1,(double)tj-t0);
               weighted_edges.emplace_back(std::make_tuple(v,vt,-cost));
               adj_list[v].emplace_back(std::make_tuple(vt,-cost));
               
               v=vt;
           }
        }
    }
    std::cout<<"type 1 edges added"<<std::endl;
    //add type 2 edges
    for(int j=0;j<old_paths.size();j++){
        for(int tj=0;tj<old_paths[j].size();tj++){
            TPG_node v1(old_paths[j][tj],tj,j);
            
            if(node_set.find(v1)!=node_set.end()){
                // TPG_node_p v1=node_map[tmp.toString()];
                for(int k=0;k<old_paths.size();k++){
                    if(k==j) continue;
                    for(int tk=tj+1;tk<old_paths[k].size();tk++){
                        TPG_node v2(old_paths[k][tk],tk,k);
                        if(node_set.find(v2)!=node_set.end() && old_paths[k][tk]==old_paths[j][tj]){
                            //TPG_node_p v2=node_map[tmp2.toString()];
                            weighted_edges.emplace_back(std::make_tuple(v1,v2,0));
                            adj_list[v1].emplace_back(std::make_tuple(v2,0));
                            break;
                        }
                    }
                }
            }
        }
    }
    std::cout<<"type 2 edges added"<<std::endl;

    //start node
    source=TPG_node(Location(-1,-1),-1,-1);
    node_set.insert(source);
    adj_list[source]=child_nodes();
    for(int i=0;i<old_paths.size();i++){
        TPG_node start(old_paths[i][0],0,i);
        weighted_edges.emplace_back(std::make_tuple(source,start,0));
        adj_list[source].emplace_back(std::make_tuple(start,0));
       // break;
    }
    std::cout<<"node size="<<node_set.size()<<std::endl;
    std::cout<<"edge size="<<weighted_edges.size()<<std::endl;
    
}


void TPG::build_faster(){
    auto direction_dist=[](Direction d1,Direction d2){
        return abs(d1.first-d2.first)+abs(d1.second-d2.second);
    };
    //compute directions
    compute_directions();
    std::cout<<"directions computed"<<std::endl;
    using at_pair=std::pair<int,int>;

    std::unordered_map<Location,std::vector<at_pair>> occupied;//( time,agent_id)
    std::unordered_set<at_pair,boost::hash<at_pair>> used;// (agent id, agent id)

    //add vertices and Type 1 edges
    for(int i=0;i<old_paths.size();i++){
        TPG_node v(old_paths[i][0],0,i);
     
        node_set.insert(v);
        adj_list[v]=child_nodes();
        occupied[old_paths[i][0]]={std::make_pair(0,i)};
        
        for(int t=1;t<old_paths[i].size();t++){
            
            //remove waiting states
           if(!(old_paths[i][t]==old_paths[i][t-1])){
               TPG_node vt(old_paths[i][t],t,i);
               int tj=(int)(vt.t);
               //node_map.insert({vt->toString(),vt});
               node_set.insert(vt);
               occupied[old_paths[i][t]].push_back({t,i});
               int t0=(int)(v.t);
            //    std::cout<<tj<<" "<<t0<<std::endl;
               int direction_cost=direction_dist(directions[i][tj],directions[i][t0]);
               auto cost=std::max(direction_cost*rotation_time+1,(double)tj-t0);
               weighted_edges.emplace_back(std::make_tuple(v,vt,-cost));
               adj_list[v].emplace_back(std::make_tuple(vt,-cost));
               type1_edge[v]=std::make_tuple(vt,-cost);
               v=vt;
           }
        }
    }
    std::cout<<"type 1 edges added"<<std::endl;
    for(const auto&node_pair: occupied){
        auto vertex=node_pair.first;
        auto occupied_list=node_pair.second;
        int T=occupied_list.size();
     
        std::sort(occupied_list.begin(),occupied_list.end());
        for(int i=0;i<T;i++){
            used.clear();
            auto pair_i=occupied_list[i];
            int ti=pair_i.first;
            int ai=pair_i.second;
            TPG_node v1(vertex,ti,ai);
            for(int j=i+1;j<T;j++){
                auto pair_j=occupied_list[j];
                int tj=pair_j.first;
                int aj=pair_j.second;
                if(used.find({ai,aj})!=used.end()) continue;
                used.insert({ai,aj});
                
                TPG_node v2(vertex,tj,aj);
                child_node vx=type1_edge[v1];
           
                // std::cout<<std::get<1>(vx)<<std::endl;
                //  weighted_edges.push_back({v1,v2,0});
                // adj_list[v1].emplace_back(std::make_tuple(v2,0));
                weighted_edges.push_back({v1,v2,std::get<1>(vx)});
                adj_list[v1].emplace_back(std::make_tuple(v2,std::get<1>(vx)));
            }
        }
    }
    //add type 2 edges
    // for(int j=0;j<old_paths.size();j++){
    //     for(int tj=0;tj<old_paths[j].size();tj++){
    //         TPG_node v1(old_paths[j][tj],tj,j);
            
    //         if(node_set.find(v1)!=node_set.end()){
    //             // TPG_node_p v1=node_map[tmp.toString()];
    //             for(int k=0;k<old_paths.size();k++){
    //                 if(k==j) continue;
    //                 for(int tk=tj+1;tk<old_paths[k].size();tk++){
    //                     TPG_node v2(old_paths[k][tk],tk,k);
    //                     if(node_set.find(v2)!=node_set.end() && old_paths[k][tk]==old_paths[j][tj]){
    //                         //TPG_node_p v2=node_map[tmp2.toString()];
    //                         weighted_edges.emplace_back(std::make_tuple(v1,v2,0));
    //                         adj_list[v1].emplace_back(std::make_tuple(v2,0));
    //                         break;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    std::cout<<"type 2 edges added"<<std::endl;

    //start node
    source=TPG_node(Location(-1,-1),-1,-1);
    node_set.insert(source);
    adj_list[source]=child_nodes();
    for(int i=0;i<old_paths.size();i++){
        TPG_node start(old_paths[i][0],0,i);
        weighted_edges.emplace_back(std::make_tuple(source,start,0));
        adj_list[source].emplace_back(std::make_tuple(start,0));
       // break;
    }
    std::cout<<"node size="<<node_set.size()<<std::endl;
    std::cout<<"edge size="<<weighted_edges.size()<<std::endl;
    
}

const int max_inf=666666;
void TPG::bellman_ford(){
    std::unordered_map<TPG_node,int> dist;
    std::unordered_map<TPG_node,TPG_node> predecessor;

    //initialize graph
    for(const auto &node:node_set){
        dist[node]=max_inf;
        predecessor[node]=node;
    }
    dist[source]=0;
    std::cout<<"initialized"<<std::endl;

    //step2: relax edges repeatedly
    for(int i=0;i<node_set.size()-1;i++){
        for(auto& edge: weighted_edges){
            auto u=std::get<0>(edge);
            auto v=std::get<1>(edge);
            auto w=std::get<2>(edge);
            if(dist[u]+w<dist[v]){
                dist[v]=dist[u]+w;
                predecessor[v]=u;
            }
        }
    }
    makespan_post=max_inf;
    for(const auto & v_pair: dist){
        makespan_post=std::min(makespan_post,v_pair.second);
    }
    makespan_post=-makespan_post;
    //step3: check for negative-weight cycles (to do) 
    for(auto& edge:weighted_edges){
        auto u=std::get<0>(edge);
        auto v=std::get<1>(edge);
        auto w=std::get<2>(edge);
        if(dist[u]+w<dist[v]){
            throw std::runtime_error("negative weight cycles!");
        }
    }
}


void TPG::faster_bellman(){
    std::unordered_map<TPG_node,int> dist;
    std::unordered_set<TPG_node> in_queue;
    for(const auto &node:node_set){
        dist[node]=max_inf;
    }
    dist[source]=0;
    std::queue<TPG_node> open;
    open.push(source);
    while(open.empty()!=true){
        auto u=open.front();
        open.pop();
        auto it =in_queue.find(u);
        if(it!=in_queue.end()) in_queue.erase(it);
        auto neighbors=adj_list[u];
        for(auto const & neighbor:neighbors){
            auto v=std::get<0>(neighbor);
            auto w=std::get<1>(neighbor);
            if(dist[u]+w<dist[v]){
                dist[v]=dist[u]+w;
            }
            if(in_queue.find(v)==in_queue.end()){
                open.push(v);
                in_queue.insert(v);
            }
        }
    }
    makespan_post=max_inf;
    for(const auto & v_pair: dist){
        makespan_post=std::min(makespan_post,v_pair.second);
    }
    makespan_post=-makespan_post;

    
}

//compute the disired orientations
void TPG::compute_directions(){
    for(int i=0;i<old_paths.size();i++){
        std::vector<Direction> diretion_i;
        directions.emplace_back(diretion_i);
        for(int t=0;t<old_paths[i].size()-1;t++){
            int j;
            for(j=t+1;j<old_paths[i].size()-1;j++){
                if(old_paths[i][t-1]!=old_paths[i][j]) break; //remove waiting states
            }
            if(j==old_paths[i].size()) j=old_paths[i].size()-1;
            auto desired_direction=
            std::pair<int,int>(old_paths[i][j].x-old_paths[i][t].x,old_paths[i][j].y-old_paths[i][t].y);
            directions[i].emplace_back(desired_direction);
        }
        directions[i].emplace_back(directions[i].back());
    }
}


int TPG::get_makespan_post(){
    return makespan_post;
}



SynMove::SynMove(Paths &paths,int rotation_time){
    makespan_old=evaluate_makespan(paths);
    fill_paths(paths);
    for(int i=0;i<paths.size();i++){
        Robot_p robot_i=std::make_shared<Robot>(paths[i]);
        robots.push_back(robot_i);
    }
    this->rotation_cost=rotation_time;
    compute_directions(paths);
    for(int i=0;i<robots.size();i++){
        robots[i]->orientation=directions[i][0];
    }
}

void SynMove::solve(){
    auto direction_dist=[](Direction d1,Direction d2){
        return abs(d1.first-d2.first)+abs(d1.second-d2.second);
    };
    Paths execution(robots.size(),Path());
    for(int i=0;i<robots.size();i++){
        execution[i].emplace_back(robots[i]->plan[0]);
    }
    makespan_post=0;
    int plan_id=0;
    while(true){
        //check if there is delay
        int delay_time=0;
        for(int i=0;i<robots.size();i++){
            if(robots[i]->orientation!=directions[i][plan_id]){
                delay_time=std::max(delay_time,rotation_cost*direction_dist(robots[i]->orientation,directions[i][plan_id]));
            }
        }
        while(delay_time!=0){
            for(int i=0;i<robots.size();i++){
                execution[i].emplace_back(robots[i]->plan[plan_id]);
            }
            delay_time--;
            makespan_post++;
        }
     
        for(int i=0;i<robots.size();i++){
            robots[i]->orientation=directions[i][plan_id];
            execution[i].emplace_back(robots[i]->plan[plan_id+1]);
        }
        makespan_post++;
        plan_id++;
        if(plan_id==makespan_old-1) break; // reached
    }
    
}


void SynMove::compute_directions(const Paths &old_paths){
    for(int i=0;i<old_paths.size();i++){
        std::vector<Direction> diretion_i;
        directions.emplace_back(diretion_i);
        for(int t=0;t<old_paths[i].size()-1;t++){
            int j;
            for(j=t+1;j<old_paths[i].size()-1;j++){
                if(old_paths[i][t-1]!=old_paths[i][j]) break; //remove waiting states
            }
            if(j==old_paths[i].size()) j=old_paths[i].size()-1;
            auto desired_direction=
            std::pair<int,int>(old_paths[i][j].x-old_paths[i][t].x,old_paths[i][j].y-old_paths[i][t].y);
            directions[i].emplace_back(desired_direction);
        }
        directions[i].emplace_back(directions[i].back());
    }
}


//MCP like execution policy
DependencyMove::Robot::Robot(int _id,Path &_path): id(_id),old_path(_path){
    plan_id=0;
    goal=old_path.back();
}


Location DependencyMove::Robot::getNextPlan(){
    if(plan_id>=old_path.size()) return old_path.back();
    else return old_path[plan_id];
}

int DependencyMove::getId(Location &vertex){
    return vertex.x*1000+vertex.y;
}

Location DependencyMove::getVertex(int id){
    int x,y;
    x=id/1000;
    y=id%1000;
    if(x>=30 or y>=30) std::cout<<"vid="<<id<<std::endl;
    return Location(x,y);
}

DependencyMove::DependencyMove(Paths &paths){
    //create robots
    for(int i=0;i<paths.size();i++){
        Robot*ri=new Robot(i,paths[i]);
        ri->execution.push_back(paths[i][0]);// start
        ri->plan_id=1;
        robots.push_back(ri);
    }

    //create vertex agent_order
    int makespan=0;
    for(auto &p:paths) makespan=std::max((int)p.size(),makespan);

    for(int t=0;t<makespan;t++){
        for(int i=0;i<paths.size();i++){
            int li=paths[i].size();
            int ti=std::min(li-1,t);
            int tt=std::min(li-1,t-1);
            int vid=getId(paths[i][ti]);
            
            if(t>0 and paths[i][ti]==paths[i][tt])
                continue;
            // if(t==6 and i==107) std::cout<<"debug  "<<paths[i][t]<<std::endl;
            vertex_agent_order[vid].push(i);
            
        }
    }
    // Location v=Location(5,19);
    // Location u=Location(4,19);
    // int vid=getId(v);
    // auto q=vertex_agent_order[vid];
    // while(q.empty()==false){
    //     auto ai=q.front();
    //     q.pop();
    //     std::cout<<ai<<" ";
    // }
    // std::cout<<"\n";
    // vid=getId(u);
    // q=vertex_agent_order[vid];
    // while(q.empty()==false){
    //     auto ai=q.front();
    //     q.pop();
    //     std::cout<<ai<<" ";
    // }
    // exit(0);

    // debug vertex agent order
    // for(auto &pair:vertex_agent_order){
    //     auto vid=pair.first;
    //     auto q=pair.second;
    //     std::cout<<getVertex(vid)<<":";
    //     while(q.empty()==false){
    //         auto top=q.front();
    //         std::cout<<top<<" ";
    //         q.pop();
    //     }
    //     std::cout<<"\n";

        
    // }
    // exit(0);
}

void DependencyMove::sim(){
    bool reachedGoal=false;
    makespan_post=0;
    for(auto &r:robots){
        int start=getId(r->old_path[0]);
        vertex_occupancy[start]=r->id;
        vertex_agent_order[start].pop();
    }
    while(reachedGoal==false){
        std::cout<<"timeStep="<<makespan_post<<std::endl;
        reachedGoal= moveOneStep();
        makespan_post++;
        // if(makespan_post>100) break;
        // std::unordered_set<int> occupied;
        // for(auto &r:robots){
        //     int current=getId(r->execution.back());
        //     if(occupied.find(current)!=occupied.end()) {
        //         throw std::runtime_error("wrong");
        //     }
        //     else occupied.insert(current);
        // }
    }
}


bool DependencyMove::moveOneStep(){
    for(auto &ri:robots) ri->moved=false;
    
    std::vector<int> delayedRobots;
    bool reachedGoal=true;
    // auto t0=Time::now();
    findCornerFollowingConflictsFaster(delayedRobots);
    // auto t1=Time::now();

    // std::cout<<"following conflicts detected: ";
    // for(auto &id:delayedRobots) std::cout<<id<<" ";
    // std::cout<<std::endl;
    //delay conflicted robots
     
    std::unordered_map<int,int> next_occupancy;
    for(auto ai:delayedRobots){
        auto ri=robots[ai];
        auto current=ri->execution.back();
        ri->execution.push_back(current);    //wait
        // std::cout<<ri->id<<current<<std::endl;
        // if(next_occupancy.find(getId(current))!=next_occupancy.end()){
        //     throw std::runtime_error("infeasible");
        // }
        next_occupancy[getId(current)]=ri->id;
        reachedGoal=false;
        ri->moved=true;
    }
    // auto t2=Time::now();
    for(auto  &ri:robots){
       if(moveRobot(ri,next_occupancy)==false) {
        //    std::cout<<"robot "<<ri->id<<" not reached goal"<<std::endl;
           reachedGoal=false;
       }
    }
    // auto t3=Time::now();
    // fsec dt1=t1-t0;
    // fsec dt2=t2-t1;
    // fsec dt3=t3-t2;
    // std::cout<<dt1.count()<<" "<<dt2.count()<<" "<<dt3.count()<<std::endl;
    vertex_occupancy.swap(next_occupancy);
    return reachedGoal;
}

bool DependencyMove::moveRobot(Robot* ri,std::unordered_map<int,int>&next_occupancy){
   
    if(ri->moved==true) return ri->reachedGoal();
    // if(ri->id==5){
    //     std::cout<<"debug\n";
    //     std::cout<<ri->execution.back()<<" "<<ri->getNextPlan()<<std::endl;
    // }
    auto current=ri->execution.back();
    auto nextV=ri->getNextPlan();
    // assert(vertex_occupancy.find(getId(current))!=vertex_occupancy.end());
    if(current==nextV){
        ri->execution.push_back(current);
        ri->plan_id++;
        ri->moved=true; 

        next_occupancy[getId(current)]=ri->id;
        
    }
    else{
     
        
        if(vertex_occupancy.find(getId(nextV))!=vertex_occupancy.end()){//nextV is occupied
            int nid=getId(nextV);
            int sizex=vertex_occupancy.size();
            // std::cout<<"size="<<sizex<<" ";
            int aj=vertex_occupancy[getId(nextV)];
            
            auto next_j=robots[aj]->getNextPlan();
            // if(next_j==current and robots[aj]->moved==false){
            //     std::cout<<ri->id<<" "<<current<<" -> "<<nextV<<std::endl;
            //     std::cout<<aj<<" "<<nextV<<"  -> "<<next_j<<std::endl;  //unexpected edge conflict occured
            //     assert(false);
            // }

            // if(nextV==Location(19,18)){
            //     std::cout<<"debug "<<aj<<" "<<ri->id<<std::endl;
            // }
            moveRobot(robots[aj],next_occupancy);
            // if(next_occupancy.find(getId(nextV))!=next_occupancy.end()){    //delayed
            //     ri->execution.push_back(current);
            //     ri->plan_id++;
            //     ri->moved=true;
                
            //     if(next_occupancy.find(getId(current))!=next_occupancy.end()){
            //         throw std::runtime_error("Cycle detected 2");
            //     }
            //     next_occupancy[getId(current)]=ri->id;
            // }
        }
        
        // if(ri->id==5){
        //     std::cout<<"debug\n";
        //     std::cout<<isNextAgent(ri->id,getId(nextV))<<" "<<nextV<<std::endl;
        //     std::cout<<"next agent should be "<<vertex_agent_order[getId(nextV)].front()<<std::endl;
        // }
        if(isNextAgent(ri->id,getId(nextV)) and next_occupancy.find(getId(nextV))==next_occupancy.end()){
          
            ri->execution.push_back(nextV);
            ri->plan_id++;
            ri->moved=true;
            // std::cout<<"move to next vertex  "<<std::endl;
            next_occupancy[getId(nextV)]=ri->id;
            // if(nextV==Location(5,19)) std::cout<<ri->id<<" enters (5,19)"<<std::endl;
            // if(nextV==Location(4,19)) std::cout<<ri->id<<" enters (4,19)"<<std::endl;
            vertex_agent_order[getId(nextV)].pop();
            // if(ri->id==7){
            //     std::cout<<nextV<<" poped"<<std::endl;
            // }
        }
        //delay
        else{
            // std::cout<<isNextAgent(ri->id,getId(nextV))<<" "<<(next_occupancy.find(getId(nextV))==next_occupancy.end())<<std::endl;
            ri->execution.push_back(current);
            ri->moved=true;
            
            // if(next_occupancy.find(getId(current))!=next_occupancy.end()){
            //     std::cout<<current<<" "<<nextV<<std::endl;
            //     std::cout<<robots[98]->execution.back()<<" "<<robots[98]->execution[(robots[98]->execution.size())-2]<<std::endl;
            //     std::cout<<"agent ai "<<ri->id<<" "<<current<<" is occupied by agent "<<next_occupancy[getId(current)]<<std::endl;
            //     throw std::runtime_error("Cycle detected 3?");
            // }
            next_occupancy[getId(current)]=ri->id;
            
        }
    }        
    
    return ri->reachedGoal();
}


void DependencyMove::findCornerFollowingConflicts(std::vector<int> &delayAgents){
    for(int j=0;j<robots.size();j++){
        for(int i=0;i<robots.size();i++){
            if(i==j) continue;
            auto ui=robots[i]->execution.back();
            auto vi=robots[i]->getNextPlan();
            auto uj=robots[j]->execution.back();
            auto vj=robots[j]->getNextPlan();
            if(vj==ui){
                int product=(ui.x-vi.x)*(uj.y-vj.y)+(ui.y-vi.y)*(uj.x-vj.x);
                if(product!=0){
                    delayAgents.push_back(j);
                    break;
                } 
            }

        }
    }
}


void DependencyMove::findCornerFollowingConflictsFaster(std::vector<int> &delayAgents){
    std::unordered_map<int,int> next_map;
    for(int j=0;j<robots.size();j++){
        auto v=robots[j]->getNextPlan();
        next_map[getId(v)]=j;
    }
    for(int i=0;i<robots.size();i++){
        auto vi=robots[i]->getNextPlan();
        auto ui=robots[i]->execution.back();
        if(vi==ui) continue;
        if(next_map.find(getId(ui))==next_map.end()) continue;
        int j=next_map[getId(ui)];
        auto uj=robots[j]->execution.back();
        auto vj=robots[j]->getNextPlan();
        int product=(ui.x-vi.x)*(uj.y-vj.y)+(ui.y-vi.y)*(uj.x-vj.x);
        if(product!=0) delayAgents.push_back(j);
    }
    // for(int j=0;j<robots.size();j++){
    //     for(int i=0;i<robots.size();i++){
    //         if(i==j) continue;
    //         auto ui=robots[i]->execution.back();
    //         auto vi=robots[i]->getNextPlan();
    //         auto uj=robots[j]->execution.back();
    //         auto vj=robots[j]->getNextPlan();
    //         if(vj==ui){
    //             int product=(ui.x-vi.x)*(uj.y-vj.y)+(ui.y-vi.y)*(uj.x-vj.x);
    //             if(product!=0){
    //                 delayAgents.push_back(j);
    //                 break;
    //             } 
    //         }

    //     }
    // }
}

void DependencyMove::findFollowingConflicts(std::vector<int> &delayAgents){
    for(int i=0;i<robots.size();i++){
        for(int j=0;j<robots.size();j++){
            if(i==j) continue;
            auto ui=robots[i]->execution.back();
            auto vi=robots[i]->getNextPlan();
            auto uj=robots[j]->execution.back();
            auto vj=robots[j]->getNextPlan();
            if(vj==ui){
               delayAgents.push_back(j);
            }
        }
    }
}


bool DependencyMove::isNextAgent(int ai,int vid){
    if(vertex_agent_order[vid].empty()) return true;
    auto av=vertex_agent_order[vid].front();
    return ai==av;
}

void DependencyMove::save_results(std::string name,double runtime,bool save_paths){
    int makespan=0;
    int soc=0;
    int makespanLB=0;
    int socLB=0;
    Paths paths;
    for(auto &robot:robots){
        paths.push_back(robot->execution);
    }
    // shrink_paths(paths);
    save_paths_as_txt(name,paths,runtime,save_paths);

}
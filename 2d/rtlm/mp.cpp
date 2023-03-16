#include "mp.h"



void debug_checker(Agents & agents){
    //fill pahts
    int makespan=0;
    for(const auto &agent:agents){
        makespan=std::max(makespan,(int)agent->path.size());
    }

    //check vertex conflicts
    for(int t=0;t<makespan;t++){
        std::unordered_set<Location> occupied;
        for(const auto &agent:agents){  
            if(occupied.find(agent->path[t])!=occupied.end()){
                throw std::runtime_error("find vertex conflicts");
            }
            else occupied.insert(agent->path[t]);
        }
    }

    //check consistency
    for(const auto &agent:agents){
        int T=agent->path.size();
        for(int t=0;t<T-1;t++){
            if(distance(agent->path[t],agent->path[t+1])>1){
                throw std::runtime_error("path in-consistency");
            }
        }
    }
}


HW_MP::HW_MP(Agents &agents, point2d xrange, point2d yrange, char orientation)
{
    (this->agents) = agents;
    this->xmin = xrange.first;
    this->xmax = xrange.second;
    this->ymin = yrange.first;
    this->ymax = yrange.second;
    this->orientation = orientation;
    // std::cout<<"here are "<<agents.size()<<" agents"<<std::endl;
    // for(auto &agent:agents){
    //     std::cout<<*agent<<std::endl;
    // }
    // std::cout<<xmin<<" "<<xmax<<" "<<ymin<<" "<<ymax<<" orientation="<<orientation<<std::endl;
    id_vertex_map = {{0, Location(0, 0)}, {1, Location(1, 0)}, {2, Location(2, 0)}, {3, Location(0, 1)}, {4, Location(1, 1)}, {5, Location(2, 1)}, {6, Location(0, 2)}, {7, Location(1, 2)}, {8, Location(2, 2)}};
    std::ifstream ifs("./local3x3.json");
    data_dict = json::parse(ifs);
}

void MotionPrimitive::fill_paths()
{
    int makespan = 0;
    for (auto &agent : agents)
    {
        makespan = std::max((int)agent->path.size(), makespan);
    }
    for (auto &agent : agents)
    {
        while (agent->path.size() < makespan)
        {
            agent->path.emplace_back(agent->path.back());
            
        }
        agent->current = agent->path.back();
    }
    
}

void HW_MP::prepare(int min_x, int min_y)
{
    //using for soritng the vertices
    int max_x = min_x + 2;
    int max_y = min_y + 2;
    auto greaterX = [](Location v1, Location v2) {
        return v1.y * 1e5 + v1.x < v2.y * 1e5 + v2.x;
    };
    auto greaterY = [](Location v1, Location v2) {
        return v1.x * 1e5 + v1.y < v2.x * 1e5 + v2.y;
    };
    auto get_index = [](std::vector<Location> &vertices, Location &v) {
        auto it = find(vertices.begin(), vertices.end(), v);
        int id = distance(vertices.begin(), it);
        return id;
    };
    // std::cout<<min_x<<" "<<max_x<<" "<<min_y<<" "<<max_y<<std::endl;
    ////////////////////////////////////////////
    Agents robots;  //starts robots
    Agents robots2;     //goals robots
    std::vector<Location> vertices;
    for (int i = min_x; i <= max_x; i++)
    {
        for (int j = min_y; j <= max_y; j++)
        {
            vertices.push_back(Location(i, j));
        }
    }

    if (orientation == 'x')
        std::sort(vertices.begin(), vertices.end(), greaterX);
    else
        std::sort(vertices.begin(), vertices.end(), greaterY);
    for (const auto &agent : agents)
    {
        if (std::find(vertices.begin(), vertices.end(), agent->current) != vertices.end())
        {
            robots.push_back(agent);
        }
        if (std::find(vertices.begin(), vertices.end(), agent->intermediate) != vertices.end())
        {
            robots2.push_back(agent);
        }
    }

    //////////////////////////////////////////////retrieve the data from json//////////////////////////////////
    std::string start_id1, start_id2;

    if (robots.size() != 0)
    {
        auto id_order = [&vertices, &get_index](Agent_p a1, Agent_p a2) {
            int id1 = get_index(vertices, a1->current);
            int id2 = get_index(vertices, a2->current);
            return id1 < id2;
        };
        start_id1 = "((";
        std::sort(robots.begin(), robots.end(), id_order);
        for (auto &agent : robots)
        {
            int id = get_index(vertices, agent->current);
            start_id1 += std::to_string(id);
            start_id1 += ", ";
        }
        start_id1.pop_back();
        start_id1.pop_back();
        start_id1 += "), ";
        start_id1 += "'x')";
        // std::cout<<start_id1<<std::endl;
        std::vector<std::vector<int>> solution = data_dict[start_id1];
        for (int i = 0; i < robots.size(); i++)
        {
            Path new_path;
            for (auto vid : solution[i])
            {
                new_path.push_back(vertices[vid]);
            }
            robots[i]->path.insert(robots[i]->path.end(), new_path.begin(), new_path.end());
            robots[i]->current = new_path.back();
        }
    }

    if (robots2.size() != 0)
    {
        // assert(robots2.size()<=3);
        auto id_order2 = [&vertices, &get_index](Agent_p a1, Agent_p a2) {
            int id1 = get_index(vertices, a1->intermediate);
            int id2 = get_index(vertices, a2->intermediate);
            return id1 < id2;
        };
        start_id2 = "((";
        std::sort(robots2.begin(), robots2.end(), id_order2);
        for (auto &agent : robots2)
        {
            int id = get_index(vertices, agent->intermediate);
            start_id2 += std::to_string(id);
            start_id2 += ", ";
        }
        start_id2.pop_back();
        start_id2.pop_back();
        start_id2 += "), ";
        start_id2 += "'x')";
        // std::cout<<start_id2<<std::endl;
        std::vector<std::vector<int>> solution = data_dict[start_id2];
        for (int i = 0; i < robots.size(); i++)
        {
            Path new_path;
            for (auto vid : solution[i])
            {
                new_path.push_back(vertices[vid]);
            }
            // assert(new_path[0]==robots2[i]->intermediate);
            robots2[i]->inter2 = new_path.back();
            new_path.pop_back();
            std::reverse(new_path.begin(), new_path.end());
            robots2[i]->inter_path = new_path;
   
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////
}

void HW_MP::reconfigure_x()
{
    if ((agents).size() == 0)
        return;
    for (int i = xmin; i < xmax + 1; i += 3)
    {
        prepare(i, ymin);
    }
    fill_paths();

    for (auto &agent : agents)
    {
        int xs = agent->current.x;
        int ys = agent->current.y;
        int xg = agent->inter2.x;
        int yg = agent->inter2.y;
        if (xg > xs)
        {
            for (int x = xs; x < xg + 1; x++)
            {
                agent->path.emplace_back(Location(x, ys + 1));
            }
            agent->path.emplace_back(Location(xg, yg));
        }
        else
        {
            for (int x = xs; x > xg - 1; x--)
            {
                agent->path.emplace_back(Location(x, ys - 1));
            }
            agent->path.emplace_back(Location(xg, yg));
        }
    }
    fill_paths();
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
        // assert(agent->current==agent->intermediate);
    }
    fill_paths();
}

void HW_MP::reconfigure_y()
{
    if ((agents).size() == 0)
        return;
    for (int i = ymin; i < ymax + 1; i += 3)
    {
        prepare(xmin, i);
    }
    fill_paths();

    for (auto &agent : agents)
    {
        int xs = agent->current.x;
        int ys = agent->current.y;
        int xg = agent->inter2.x;
        int yg = agent->inter2.y;
        if (yg > ys)
        {
            for (int y = ys; y < yg + 1; y++)
            {
                agent->path.emplace_back(Location(xs + 1, y));
            }
            agent->path.emplace_back(Location(xg, yg));
        }
        else
        {
            for (int y = ys; y > yg - 1; y--)
            {
                agent->path.emplace_back(Location(xs - 1, y));
            }
            agent->path.emplace_back(Location(xg, yg));
        }
    }
    fill_paths();
    // return;
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
        // if(agent->current!=agent->intermediate) std::cout<<*agent<<"  "<<agent->inter2<<std::endl;
        // assert(agent->current==agent->intermediate);
    }
    fill_paths();
}

void MotionPrimitive::reconfigure()
{
    if (orientation == 'x')
        reconfigure_x();
    else
        reconfigure_y();
}

int HW_MP::get_id(Location vertex)
{
    return vertex.x + vertex.y * 3;
}

Location HW_MP::get_vertex(int id)
{
    return id_vertex_map[id];
}

///////////////////////////////////////////////////////////////
//Motion primitive improved version 1

void HW_MP_H1::reconfigure_x()
{
    if (agents.size() == 0)
        return;
    for (int i = xmin; i < xmax + 1; i += 3)
    {
        prepare(i, ymin);
    }
    fill_paths();
    int start_time = 0;
    std::unordered_map<int, int> safe_time;
    for (auto &agent : agents)
    {
        int xs = agent->current.x;
        int ys = agent->current.y;
        int xg = agent->inter2.x;
        int yg = agent->inter2.y;
        if (xg > xs)
        {
            for (int x = xs; x < xg + 1; x++)
            {
                agent->path.push_back(Location(x, ys + 1));
                int cell_id = (int)x / 3;
                if (safe_time.find(cell_id) == safe_time.end())
                {
                    safe_time[cell_id] = 0;
                }
                safe_time[cell_id] = std::max((int)agent->path.size(), safe_time[cell_id]);
            }
            agent->path.push_back(Location(xg, yg));
        }
        else
        {
            for (int x = xs; x > xg - 1; x--)
            {
                agent->path.push_back(Location(x, ys - 1));
                int cell_id = (int)x / 3;
                if (safe_time.find(cell_id) == safe_time.end())
                {
                    safe_time[cell_id] = 0;
                }
                safe_time[cell_id] = std::max((int)agent->path.size(), safe_time[cell_id]);
            }
            agent->path.push_back(Location(xg, yg));
        }
    }
    for (auto &agent : agents)
    {
        Location vg = agent->path.back();
        int cell_id = (int)(vg.x / 3);
        int safe_time_i;
        if (safe_time.find(cell_id) != safe_time.end())
        {
            safe_time_i = safe_time[cell_id] + 1;
        }
        else
        {
            safe_time_i = agent->path.size();
        }
        while (agent->path.size() < safe_time_i)
            agent->path.emplace_back(agent->path.back());
    }
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
    }
}

void HW_MP_H1::reconfigure_y()
{
    if (agents.size() == 0)
        return;
    for (int i = ymin; i < ymax + 1; i += 3)
    {
        prepare(xmin, i);
    }
    fill_paths();
    int start_time = 0;
    std::unordered_map<int, int> safe_time;
    for (auto &agent : agents)
    {
        int xs = agent->current.x;
        int ys = agent->current.y;
        int xg = agent->inter2.x;
        int yg = agent->inter2.y;
        if (yg > ys)
        {
            for (int y = ys; y < yg + 1; y++)
            {
                agent->path.push_back(Location(xs + 1, y));
                int cell_id = (int)y / 3;
                if (safe_time.find(cell_id) == safe_time.end())
                {
                    safe_time[cell_id] = 0;
                }
                safe_time[cell_id] = std::max((int)agent->path.size(), safe_time[cell_id]);
            }
            agent->path.push_back(Location(xg, yg));
        }
        else
        {
            for (int y = ys; y > yg - 1; y--)
            {
                agent->path.push_back(Location(xs - 1, y));
                int cell_id = (int)y / 3;
                if (safe_time.find(cell_id) == safe_time.end())
                {
                    safe_time[cell_id] = 0;
                }
                safe_time[cell_id] = std::max((int)agent->path.size(), safe_time[cell_id]);
            }
            agent->path.push_back(Location(xg, yg));
        }
    }
    for (auto &agent : agents)
    {
        Location vg = agent->path.back();
        int cell_id = (int)(vg.y / 3);
        int safe_time_i;
        if (safe_time.find(cell_id) != safe_time.end())
        {
            safe_time_i = safe_time[cell_id] + 1;
        }
        else
        {
            safe_time_i = agent->path.size();
        }
        while (agent->path.size() < safe_time_i)
            agent->path.emplace_back(agent->path.back());
    }
  
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
    }
}


/////////////////////////////////////Motion primitive improved version 2///////////////////////////////////////
void HW_MP_H2::better_path(){
    for(auto &agent:agents){
        while(agent->path.size()<safe_time[agent->id]){
            agent->path.emplace_back(agent->path.back());
        }
    }
}

void HW_MP_H2::sort_agents(){
  
    
    if (orientation=='x'){
        for(auto &agent:agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(xg>xs) up_agents.emplace_back(agent);
            else down_agents.emplace_back(agent);
            int cell_id=xs/3;
            blocked_time[cell_id]=std::max((int)agent->path.size(),blocked_time[cell_id]+1);
        }
        std::sort(up_agents.begin(),up_agents.end(),[&](Agent_p a1,Agent_p a2){return a1->current.x>a2->current.x;});
        std::sort(down_agents.begin(),down_agents.end(),[&](Agent_p a1,Agent_p a2){return a1->current.x<a2->current.x;});
    }
    else{
        for(auto &agent:agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(yg>ys) up_agents.emplace_back(agent);
            else down_agents.emplace_back(agent);
            int cell_id=ys/3;
            blocked_time[cell_id]=std::max((int)agent->path.size(),blocked_time[cell_id]+1);
        }
        std::sort(up_agents.begin(),up_agents.end(),[&](Agent_p a1,Agent_p a2){return a1->current.y>a2->current.y;});
        std::sort(down_agents.begin(),down_agents.end(),[&](Agent_p a1,Agent_p a2){return a1->current.y<a2->current.y;});
    }
}


bool HW_MP_H2::next_move(int t){
    std::unordered_map<point2d,int,boost::hash<point2d>> occupied;
    bool all_reached=true;
    if(orientation=='x'){
        for(auto &agent:up_agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(xs==xg and ys==yg){
                occupied[{xs,ys}]=1;
                continue;
            }
            all_reached=false;
            int tmp_time=blocked_time[xs/3];
            if(tmp_time>=t) continue;
            point2d next_v;
            if(ys==yg) next_v={xs,ys+1};
            else if(xs==xg) next_v={xg,yg};
            else {
                point2d nv={xs+1,ys};
                int cell_id=nv.first/3;
                if(blocked_time[cell_id]<t and occupied.find(nv)==occupied.end()) next_v=nv;
                else next_v={xs,ys};
            }
            occupied[next_v]=1;
            agent->path.emplace_back(Location(next_v.first,next_v.second));
            agent->current=Location(next_v.first,next_v.second);
            int cell_id=next_v.first/3;
            cell_safe_time[cell_id]=std::max((int)agent->path.size(),cell_safe_time[cell_id]);
        }

        for(auto &agent:down_agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(xs==xg and ys==yg){
                occupied[{xs,ys}]=1;
                continue;
            }
            all_reached=false;
            if(blocked_time[xs/3]>=t) continue;
            point2d next_v;
            if(ys==yg) next_v={xs,ys-1};
            else if(xs==xg) next_v={xg,yg};
            else {
                point2d nv={xs-1,ys};
                int cell_id=nv.first/3;
                if(blocked_time[cell_id]<t and occupied.find(nv)==occupied.end()) next_v=nv;
                else next_v={xs,ys};
            }
            occupied[next_v]=1;
            agent->path.emplace_back(Location(next_v.first,next_v.second));
            agent->current=Location(next_v.first,next_v.second);
            int cell_id=next_v.first/3;
            cell_safe_time[cell_id]=std::max((int)agent->path.size(),cell_safe_time[cell_id]);

        }
    }
    else{
        for(auto &agent:up_agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(xs==xg and ys==yg){
                occupied[{xs,ys}]=1;
                continue;
            }
            all_reached=false;
            if(blocked_time[ys/3]>=t) continue;
            point2d next_v;
            if(xs==xg) next_v={xs+1,ys};
            else if(ys==yg) next_v={xg,yg};
            else {
                point2d nv={xs,ys+1};
                int cell_id=nv.second/3;
                if(blocked_time[cell_id]<t and occupied.find(nv)==occupied.end()) next_v=nv;
                else next_v={xs,ys};
            }
            occupied[next_v]=1;
            agent->path.emplace_back(Location(next_v.first,next_v.second));
            agent->current=Location(next_v.first,next_v.second);
            int cell_id=next_v.second/3;
            cell_safe_time[cell_id]=std::max((int)agent->path.size(),cell_safe_time[cell_id]);
        }

        for(auto &agent:down_agents){
            int xs=agent->current.x;
            int ys=agent->current.y;
            int xg=agent->inter2.x;
            int yg=agent->inter2.y;
            if(xs==xg and ys==yg){
                occupied[{xs,ys}]=1;
                continue;
            }
            all_reached=false;
            if(blocked_time[ys/3]>=t) continue;
            point2d next_v;
            if(xs==xg) next_v={xs-1,ys};
            else if(ys==yg) next_v={xg,yg};
            else {
                point2d nv={xs,ys-1};
                int cell_id=nv.second/3;
                if(blocked_time[cell_id]<t and occupied.find(nv)==occupied.end()) next_v=nv;
                else next_v={xs,ys};
            }
            occupied[next_v]=1;
            agent->path.emplace_back(Location(next_v.first,next_v.second));
            agent->current=Location(next_v.first,next_v.second);
            int cell_id=next_v.second/3;
            cell_safe_time[cell_id]=std::max((int)agent->path.size(),cell_safe_time[cell_id]);

        }
    }
    return all_reached;
}

void HW_MP_H2::reconfigure_x(){
    if(agents.size()==0) return;
    for(int i=xmin;i<=xmax;i+=3) prepare(i,ymin);
    int t0=1000000;
    for (auto &agent:agents){
        t0=std::min((int)agent->path.size(),t0);

    }
    t0++;
    sort_agents();
    safe_time=blocked_time;
    while(true){
        bool all_reached=next_move(t0);
        if(all_reached) break;
        t0++;
    }
    for (auto &agent:agents){
        auto vg=agent->path.back();
        int cell_id=vg.x/3;
        if(cell_safe_time.find(cell_id)!=cell_safe_time.end()) safe_time[agent->id]=cell_safe_time[cell_id]+1;
        else safe_time[agent->id]=agent->path.size();
    }

    better_path();
    for(auto &agent:agents){
        agent->path.insert(agent->path.end(),agent->inter_path.begin(),agent->inter_path.end());
        agent->current=agent->path.back();
    }
}

void HW_MP_H2::reconfigure_y(){
    if(agents.size()==0) return;
    for(int i=ymin;i<=ymax;i+=3) prepare(xmin,i);
    int t0=1000000;
    for (auto &agent:agents){
        t0=std::min((int)agent->path.size(),t0);

    }
    t0++;
    sort_agents();
    safe_time=blocked_time;
    while(true){
        bool all_reached=next_move(t0);
        if(all_reached) break;
        t0++;
    }
    for (auto &agent:agents){
        auto vg=agent->path.back();
        int cell_id=vg.y/3;
        if(cell_safe_time.find(cell_id)!=cell_safe_time.end()) safe_time[agent->id]=cell_safe_time[cell_id]+1;
        else safe_time[agent->id]=agent->path.size();
    }

    better_path();
    for(auto &agent:agents){
        agent->path.insert(agent->path.end(),agent->inter_path.begin(),agent->inter_path.end());
        agent->current=agent->path.back();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Line  merge sort///////////////////////////////////////////////////////////

MergeSorter::MergeSorter(Agents &agents, point2d xrange, point2d yrange, char orientation)
{
    (this->agents) = agents;
    this->xmin = xrange.first;
    this->xmax = xrange.second;
    this->ymin = yrange.first;
    this->ymax = yrange.second;
    this->orientation = orientation;

    /////////////////////////////////
    std::vector<std::vector<int>> pathx(2, std::vector<int>());

    // 0,1
    pathx = {{0}, {1}};
    data_map[{0, 1}] = pathx;

    //0,2
    pathx = {{0, 1}, {2, 0}};
    data_map[{0, 2}] = pathx;

    //0,3
    pathx = {{0, 0}, {3, 1}};
    data_map[{0, 3}] = pathx;

    //1,2
    pathx = {{1, 1}, {2, 0}};
    data_map[{1, 2}] = pathx;

    //1,3
    pathx = {{1, 0}, {3, 1}};
    data_map[{1, 3}] = pathx;

    //2,3
    pathx = {{2, 0}, {3, 1}};
    data_map[{2, 3}] = pathx;
}

void MergeSorter::prepare(int min_x, int min_y)
{
    int max_x = min_x + 1;
    int max_y = min_y + 1;

    auto greaterX = [](Location v1, Location v2) {
        return v1.y * 1e5 + v1.x < v2.y * 1e5 + v2.x;
    };
    auto greaterY = [](Location v1, Location v2) {
        return v1.x * 1e5 + v1.y < v2.x * 1e5 + v2.y;
    };
    auto get_index = [](std::vector<Location> &vertices, Location &v) {
        auto it = find(vertices.begin(), vertices.end(), v);
        int id = distance(vertices.begin(), it);
        return id;
    };
    Agents robots1;
    Agents robots2;

    std::vector<Location> vertices;
    for (int i = min_x; i <= max_x; i++)
    {
        for (int j = min_y; j <=max_y; j++)
        {
            vertices.push_back(Location(i, j));
        }
    }
    // assert(vertices.size()==4);

    if (orientation == 'x')
        std::sort(vertices.begin(), vertices.end(), greaterX);
    else
        std::sort(vertices.begin(), vertices.end(), greaterY);

    for (auto &agent : agents)
    {
        if (std::find(vertices.begin(), vertices.end(), agent->current) != vertices.end())
        {
            robots1.push_back(agent);
        }
        if (std::find(vertices.begin(), vertices.end(), agent->intermediate) != vertices.end())
        {
            robots2.push_back(agent);
        }
    }

    // assert(robots1.size()==2);
    // assert(robots2.size()==2);
    if (robots1.empty() == false)
    {
        auto id_order = [&vertices, &get_index](Agent_p a1, Agent_p a2) {
            int id1 = get_index(vertices, a1->current);
            int id2 = get_index(vertices, a2->current);
            return id1 < id2;
        };
        std::sort(robots1.begin(), robots1.end(), id_order);
        int id1 = get_index(vertices, robots1[0]->current);
        int id2 = get_index(vertices, robots1[1]->current);
        
        auto solution = data_map[{id1, id2}];
        for (int i = 0; i < robots1.size(); i++)
        {
            Path new_path;
            for (auto vid : solution[i])
            {
                new_path.push_back(vertices[vid]);
            }
            robots1[i]->path.insert(robots1[i]->path.end(), new_path.begin() + 1, new_path.end());
            robots1[i]->current = new_path.back();
            // assert(robots1[i]->current.y==ymin);
        }
    }

    if (robots2.empty() != true)
    {
        auto id_order = [&vertices, &get_index](Agent_p a1, Agent_p a2) {
            int id1 = get_index(vertices, a1->intermediate);
            int id2 = get_index(vertices, a2->intermediate);
            return id1 < id2;
        };
        std::sort(robots2.begin(), robots2.end(), id_order);
        int id1 = get_index(vertices, robots2[0]->intermediate);
        int id2 = get_index(vertices, robots2[1]->intermediate);
        auto solution = data_map[{id1, id2}];
        for (int i = 0; i < robots2.size(); i++)
        {
            Path new_path;
            for (auto vid : solution[i])
            {
                new_path.push_back(vertices[vid]);
            }
            // assert(new_path[0]==robots2[i]->intermediate);
            robots2[i]->inter2 = new_path.back();
            // assert(robots2[i]->inter2.y==ymin);
            new_path.pop_back();
            std::reverse(new_path.begin(), new_path.end());
            robots2[i]->inter_path = new_path;
        }
    }
}

// void MergeSorter::helper(Agents &agents, int min_x,int min_y){
//     std::vector<Location> vs;
//     if(orientation=='x'){
//         vs={Location(min_x,min_y),Location(min_x+1,min_y)};
//     }
//     else{
//         vs=
//     }
// }

void MergeSorter::merge(int left, int right)
{
    if (orientation == 'x')
        mergeX(left, right);
    else
        mergeY(left, right);
}

void MergeSorter::mergeX(int left, int right)
{
    auto order = [](Agent_p a1, Agent_p a2) {
        return a1->inter2.x < a2->inter2.x;
    };
    auto order2 = [](Location v1, Location v2) {
        return v1.x < v2.x;
    };

    //fake merge sort
   
    std::vector<Location> vertices;
    std::unordered_map<Location, int> safe_time;
    // std::cout<<left<<" "<<right<<"!!!!!!!!!!!"<<std::endl;
    // for(int i=left;i<right;i++){
    //     std::cout<<*agents[i]<<std::endl;
    // }
     std::sort(agents.begin() + left, agents.begin() + right, order);
    for (int i = left; i < right; i++)
    {
        vertices.push_back(agents[i]->current);
        safe_time[agents[i]->current] = 0;
        // assert(agents[i]->current.x>=left &&agents[i]->current.x<right);
    }

    std::sort(vertices.begin(), vertices.end(), order2);

    for (int i = left; i < right; i++)
    {
        int xs, xg, ys, yg;
        auto v_des = vertices[i - left]; //agent i should go to this vertex
        xs = agents[i]->current.x;
        ys = agents[i]->current.y;
  
        xg = v_des.x;
        yg = v_des.y;
        if (xg == xs)
            continue;
        if (xg > xs)
        {
            agents[i]->path.push_back(Location(xs, ys + 1));
            for (int x = xs; x <= xg; x++)
            {
                agents[i]->path.push_back(Location(x, ys + 1));
            }
        }
        else
        {
            for (int x = xs; x >= xg; x--)
            {
                agents[i]->path.emplace_back(Location(x, ys));

                safe_time[Location(x, ys)] = std::max(safe_time[Location(x, ys)], (int)agents[i]->path.size());
            }
        }
    }

    for (int i = left; i < right; i++)
    {
        int xs, xg, ys, yg;
        auto v_des = vertices[i - left];
        xs = agents[i]->current.x;
        ys = agents[i]->current.y;
        xg = v_des.x;
        yg = v_des.y;
        if (xs < xg)
        {
            // std::cout<<"attention: "<<*agents[i]<<" move to"<<v_des<<std::endl;
            auto safe_t = safe_time[v_des];
            
            while (agents[i]->path.size() < safe_t)
            {
                agents[i]->path.emplace_back(agents[i]->path.back());
            }
            agents[i]->path.emplace_back(v_des);
            if(agents[i]->path.size()<=safe_time[v_des]){
                std::cout<<agents[i]->path.size()<<" "<<safe_time[v_des]<<std::endl;
            }
            // assert(agents[i]->path.size()>safe_time[v_des]);
            // assert(agents[i]->path.back()==v_des);
        }
    }
    // std::cout << "=============" << std::endl;
    fill_some_paths(left,right);
    // for(int i=left;i<right;i++){
    //     std::cout << *agents[i]<<" desired: "<<vertices[i-left] << std::endl;
    // }
    // std::cout << "=============" << std::endl;
}

void MergeSorter::mergeY(int left, int right)
{
    auto order = [](Agent_p a1, Agent_p a2) {
        return a1->inter2.y < a2->inter2.y;
    };

    auto order2 = [](Location v1, Location v2) {
        return v1.y < v2.y;
    };

    //fake merge sort
    std::sort(agents.begin() + left, agents.begin() + right, order);
    std::vector<Location> vertices;
    std::unordered_map<Location, int> safe_time;

    for (int i = left; i < right; i++)
    {
        vertices.push_back(agents[i]->current);
        safe_time[agents[i]->current] = 0;
    }

    std::sort(vertices.begin(), vertices.end(), order2);

    for (int i = left; i < right; i++)
    {
        int xs, xg, ys, yg;
        auto v_des = vertices[i - left];
        xs = agents[i]->current.x;
        ys = agents[i]->current.y;
        xg = v_des.x;
        yg = v_des.y;
        if (yg == ys)
            continue;
        if (yg > ys)
        {
            agents[i]->path.push_back(Location(xs + 1, ys));
            for (int y = ys; y <= yg; y++)
            {
                agents[i]->path.push_back(Location(xs + 1, y));
            }
        }
        else
        {
            for (int y = ys; y >= yg ; y--)
            {
                agents[i]->path.emplace_back(Location(xs, y));
                // assert(safe_time.find(Location(xs, y)) != safe_time.end());
                safe_time[Location(xs, y)] = std::max(safe_time[Location(xs, y)], (int)agents[i]->path.size());
            }
        }
    }

    for (int i = left; i < right; i++)
    {
        int xs, xg, ys, yg;
        auto v_des = vertices[i - left];

        xs = agents[i]->current.x;
        ys = agents[i]->current.y;
        xg = v_des.x;
        yg = v_des.y;
        if (ys < yg)
        {
            auto safe_t = safe_time[Location(xg, yg)];
            while (agents[i]->path.size() < safe_t)
            {
                agents[i]->path.emplace_back(agents[i]->path.back());
            }
            agents[i]->path.emplace_back(v_des);
        }
    }

    fill_some_paths(left,right);
}

void MergeSorter::reconfigure_x()
{
    // std::cout<<"reconfigure!"<<std::endl;
    if ((agents).size() == 0)
        return;
    for (int i = xmin; i < xmax + 1; i += 2)
    {
        prepare(i, ymin);
    }
    fill_paths();
    
    auto compare=[](Agent_p a1,Agent_p a2){
        return a1->current.x<a2->current.x;
    };
    std::sort(agents.begin(),agents.end(),compare);
    //debug
    // for(auto &agent:agents){
    //     assert(agent->current.y==ymin);
    // }
    ////////////////////////////////////
    // debug_checker(agents);
    // std::cout<<xmax<<std::endl;
    mergeSort(xmin, xmax + 1);
   
    // debug_checker(agents);
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
        // assert(agent->current==agent->intermediate);
    }
    fill_paths();
}

void MergeSorter::reconfigure_y()
{
    if ((agents).size() == 0)
        return;
    for (int i = ymin; i < ymax + 1; i += 2)
    {
        prepare(xmin, i);
    }
    fill_paths();
    auto compare=[](Agent_p a1,Agent_p a2){
        return a1->current.y<a2->current.y;
    };
    std::sort(agents.begin(),agents.end(),compare);
     //debug
    // for(auto &agent:agents){
    //     assert(agent->current.x==xmin);
    // }
    mergeSort(ymin, ymax + 1);
    for (auto &agent : agents)
    {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
        assert(agent->current==agent->intermediate);
    }
    fill_paths();
}

void MergeSorter::mergeSort(int left, int right)
{
    if (right - left > 1)
    {
        auto middle = left + (right - left) / 2;
        mergeSort(left, middle);
        mergeSort(middle, right);
        fill_some_paths(left,right);

        // debug_checker(agents);
        merge(left, right);
    }
}

void MergeSorter::fill_some_paths(int left,int right)
{
    int makespan = 0;
    for (int i=left;i<right;i++)
    {
        makespan = std::max((int)agents[i]->path.size(), makespan);
    }
    for (int i=left;i<right;i++)
    {
        while (agents[i]->path.size() < makespan)
        {
            agents[i]->path.emplace_back(agents[i]->path.back());
            
        }
        agents[i]->current =agents[i]->path.back();
    }
}
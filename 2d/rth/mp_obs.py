import numpy as np
import networkx as nx
import problem_generator as pg
import json
import yaml
import itertools
import functools
from common import *
from LBAP import *

f = open ('./local3x3_uniform_obs_db.json', "r")
local3x3_data=json.load(f)

def get_vertex(id):
    vertex_id_map={0:(0,0),1:(1,0),2:(2,0),3:(0,1),4:(1,1),5:(2,1),6:(0,2),7:(1,2),8:(2,2)}
    return vertex_id_map[id]

def get_id(vertex):
    return vertex[0]+vertex[1]*3


class Local13MP(object):
    def __init__(self,agents,dim,orientation):
        self.agents=agents
        self.orientation=orientation
        self.xmin,self.xmax,self.ymin,self.ymax=dim
     
        # mixed labeled version
        self.assign_tasks()
        
    
    def lbap_assign(self,agents):
       
        tmp_goal=[agent.intermediate for agent in agents]
        inter_goals=[agent.inter_goal for agent in agents]
        final_goals=[agent.goal for agent in agents]
        cost_matrix=[[distance(agents[i].current,agents[j].intermediate) for j in range(0,len(agents))]for i in range(len(agents))]
        row_ind,col_ind,mk=labp_solve(cost_matrix)
        # print("makespan",mk,'----------')
        for i in row_ind:
            agents[i].intermediate=tmp_goal[col_ind[i]]
            agents[i].inter_goal=inter_goals[col_ind[i]]
            agents[i].goal=final_goals[col_ind[i]]
            # print(agents[i],agents[i].intermediate,agents[i].current,distance(agents[i].intermediate,agents[i].current),int(agents[i].current[0]/3))
   


    def assign_tasks(self):
        id_dict=dict()
        for agent in self.agents:
            if agent.group_id not in id_dict:
                id_dict[agent.group_id]=[]
            id_dict[agent.group_id].append(agent)

        for id,groups in id_dict.items():
            self.lbap_assign(groups)


    def greaterX(self,v1,v2):
        """
        Custom sorting function for vertices
        """
        if v1[1] * 10000+ v1[0] > v2[1] * 10000 + v2[0]:
            return 1
        return -1

    def greaterY(self,v1,v2):
        if v1[0] * 10000 + v1[1] > v2[0] * 10000 + v2[1]:
            return 1
        return -1

    def get_id(self,v):
        if self.orientation=='x':
            return v[1]*10000+v[0]
        else:
            return v[0]*10000+v[1]

    def refine_paths(self):
        makespan=max(len(agent.path) for agent in self.agents)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1])       

    def reconfigure_x(self):
        for i in range(self.xmin,self.xmax+1,3):
            self.prepare(i,self.ymin,orientation='x')
        self.refine_paths()
        assert(check_valid(self.agents))
        for agent in self.agents:
            xs,ys=agent.current
            xg,yg=agent.inter2
            if xg>xs:
                # agent.path.append((xs,ys+1))
                for x in range(xs,xg+1):
                    agent.path.append((x,ys+1))
                agent.path.append((xg,yg))
            elif xg<xs:
                # agent.path.append((xs,ys-1))
                for x in reversed(range(xg,xs+1)):
                    agent.path.append((x,ys-1))
                agent.path.append((xg,yg))
        self.refine_paths()
        for agent in self.agents:
            agent.path.extend(agent.inter_path)
            agent.current=agent.path[-1]
         
        self.refine_paths()
        
            

    def reconfigure_y(self):
        # 'y'
        for i in range(self.ymin,self.ymax+1,3):
            self.prepare(self.xmin,i,orientation='y')   
        self.refine_paths()  
     
        assert(check_valid(self.agents) )
        # reconfigure between two m-configs:
        for agent in self.agents:
            xs,ys=agent.current
            xg,yg=agent.inter2
            if yg>ys:
                # agent.path.append((xs+1,ys))
                for y in range(ys,yg+1):
                    agent.path.append((xs+1,y))
                agent.path.append((xg,yg))
            elif yg<ys:
                # agent.path.append((xs-1,ys))
                for y in reversed(range(yg,ys+1)):
                    agent.path.append((xs-1,y))
                agent.path.append((xg,yg))  
        self.refine_paths()
        assert(check_valid(self.agents)) 
        for agent in self.agents:
            agent.path.extend(agent.inter_path)
            agent.current=agent.path[-1] 
           
            
        self.refine_paths() 
    
        check_valid(self.agents) 

    def manhattan_distance(self,v1,v2):
        return abs(v1[0]-v2[0])+abs(v1[1]-v2[1])

    def prepare(self,xmin,ymin,orientation='x'):
        xmax=xmin+2
        ymax=ymin+2
        robots=[]
        robots2=[]
        vertices=itertools.product([xmin,xmin+1,xmin+2],[ymin,ymin+1,ymin+2])
        if orientation=='x':
            vertices=sorted(vertices,key=functools.cmp_to_key(self.greaterX))
        else:
            vertices=sorted(vertices,key=functools.cmp_to_key(self.greaterY))
        for a in self.agents:
            if a.current in vertices:
                robots.append(a)
            if a.intermediate in vertices:
                robots2.append(a)
        # if not len(robots)==3:
        #     print(len(robots),xmin,ymin)
        # assert(len(robots)==3)
        # assert(len(robots2)==3)
        robots.sort(key=lambda x:vertices.index(x.current))
        robots2.sort(key=lambda x:vertices.index(x.intermediate))
        start_id=tuple([vertices.index(r.current) for r in robots])
        start_id2=tuple([vertices.index(r.intermediate) for r in robots2])
        key=str((start_id,'x'))
        solutions=local3x3_data[key]

        key2=str((start_id2,'x'))
        solutions2=local3x3_data[key2]
        #print(key,solutions)
        
        for i in range(0,len(robots)):
            path=[vertices[v] for v in solutions[i]]
            robots[i].path.extend(path[1:])
            robots[i].current=robots[i].path[-1]
            path2=[vertices[v] for v in solutions2[i]]
            path2.reverse()
            robots2[i].inter_path=path2[1:]
            robots2[i].inter2=path2[0]




class Local13MP_uniform_obs(object):
    def __init__(self,agents,dim,orientation):
        self.agents=agents
        self.orientation=orientation
        self.xmin,self.xmax,self.ymin,self.ymax=dim
     
        # mixed labeled version
        
        
    
    def lbap_assign(self,agents):
       
        tmp_goal=[agent.intermediate for agent in agents]
        inter_goals=[agent.inter_goal for agent in agents]
        final_goals=[agent.goal for agent in agents]
        cost_matrix=[[distance(agents[i].current,agents[j].intermediate) for j in range(0,len(agents))]for i in range(len(agents))]
        row_ind,col_ind,mk=labp_solve(cost_matrix)
        # print("makespan",mk,'----------')
        for i in row_ind:
            agents[i].intermediate=tmp_goal[col_ind[i]]
            agents[i].inter_goal=inter_goals[col_ind[i]]
            agents[i].goal=final_goals[col_ind[i]]
            # print(agents[i],agents[i].intermediate,agents[i].current,distance(agents[i].intermediate,agents[i].current),int(agents[i].current[0]/3))
   


    def assign_tasks(self):
        id_dict=dict()
        for agent in self.agents:
            if agent.group_id not in id_dict:
                id_dict[agent.group_id]=[]
            id_dict[agent.group_id].append(agent)

        for id,groups in id_dict.items():
            self.lbap_assign(groups)


    def greaterX(self,v1,v2):
        """
        Custom sorting function for vertices
        """
        if v1[1] * 10000+ v1[0] > v2[1] * 10000 + v2[0]:
            return 1
        return -1

    def greaterY(self,v1,v2):
        if v1[0] * 10000 + v1[1] > v2[0] * 10000 + v2[1]:
            return 1
        return -1

    def get_id(self,v):
        if self.orientation=='x':
            return v[1]*10000+v[0]
        else:
            return v[0]*10000+v[1]

    def refine_paths(self):
        makespan=max(len(agent.path) for agent in self.agents)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1])       

    def reconfigure_x(self):
        for i in range(self.xmin,self.xmax+1,3):
            self.prepare(i,self.ymin,orientation='x')
        self.refine_paths()
        assert(check_valid(self.agents))
        for agent in self.agents:
            xs,ys=agent.current
            xg,yg=agent.inter2
            if xg>xs:
                # agent.path.append((xs,ys+1))
                for x in range(xs,xg+1):
                    agent.path.append((x,ys+1))
                agent.path.append((xg,yg))
            elif xg<xs:
                # agent.path.append((xs,ys-1))
                for x in reversed(range(xg,xs+1)):
                    agent.path.append((x,ys-1))
                agent.path.append((xg,yg))
        self.refine_paths()
        for agent in self.agents:
            agent.path.extend(agent.inter_path)
            agent.current=agent.path[-1]
         
        self.refine_paths()
        
            

    def reconfigure_y(self):
        # 'y'
        for i in range(self.ymin,self.ymax+1,3):
            self.prepare(self.xmin,i,orientation='y')   
        self.refine_paths()  
     
        assert(check_valid(self.agents) )
        # reconfigure between two m-configs:
        for agent in self.agents:
            xs,ys=agent.current
            xg,yg=agent.inter2
            if yg>ys:
                # agent.path.append((xs+1,ys))
                for y in range(ys,yg+1):
                    agent.path.append((xs+1,y))
                agent.path.append((xg,yg))
            elif yg<ys:
                # agent.path.append((xs-1,ys))
                for y in reversed(range(yg,ys+1)):
                    agent.path.append((xs-1,y))
                agent.path.append((xg,yg))  
        self.refine_paths()
        # assert(check_valid(self.agents)) 
        for agent in self.agents:
            agent.path.extend(agent.inter_path)
            agent.current=agent.path[-1] 
           
            
        self.refine_paths() 
    
        # check_valid(self.agents) 

    def manhattan_distance(self,v1,v2):
        return abs(v1[0]-v2[0])+abs(v1[1]-v2[1])

    def prepare(self,xmin,ymin,orientation='x'):
        xmax=xmin+2
        ymax=ymin+2
        robots=[]
        robots2=[]
        vertices=itertools.product([xmin,xmin+1,xmin+2],[ymin,ymin+1,ymin+2])
        if orientation=='x':
            vertices=sorted(vertices,key=functools.cmp_to_key(self.greaterX))
        else:
            vertices=sorted(vertices,key=functools.cmp_to_key(self.greaterY))
        for a in self.agents:
            if a.current in vertices:
                robots.append(a)
            if a.intermediate in vertices:
                robots2.append(a)
        # if not len(robots)==3:
        #     print(len(robots),xmin,ymin)
        # assert(len(robots)==3)
        # assert(len(robots2)==3)
        robots.sort(key=lambda x:vertices.index(x.current))
        robots2.sort(key=lambda x:vertices.index(x.intermediate))
        start_id=tuple([vertices.index(r.current) for r in robots])
        start_id2=tuple([vertices.index(r.intermediate) for r in robots2])
        key=str((start_id,'x'))
        solutions=local3x3_data[key]

        key2=str((start_id2,'x'))
        solutions2=local3x3_data[key2]
        #print(key,solutions)
        
        for i in range(0,len(robots)):
            path=[vertices[v] for v in solutions[i]]
            robots[i].path.extend(path[1:])
            robots[i].current=robots[i].path[-1]
            path2=[vertices[v] for v in solutions2[i]]
            path2.reverse()
            robots2[i].inter_path=path2[1:]
            robots2[i].inter2=path2[0]

def test_case1():
    #test x-reconfig
    l=9
    mid_v=[(x,1) for x in range(0,l)]
   
    starts=mid_v.copy()
    goals=mid_v.copy()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    print(starts)
    print(goals)
    agents=[Agent(i,starts[i],goals[i],intermediate=goals[i]) for i in range(0,l)]
    swapper=Local13MP(agents,(0,l-1,0,2),'x')
    swapper.reconfigure_x()
    check_valid(agents)
   
    
def test_case2():
    l=30
    mid_v=[(1,y) for y in range(0,l)]

    starts=mid_v.copy()
    goals=mid_v.copy()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    print(starts)
    print(goals)
    agents=[Agent(i,starts[i],goals[i],intermediate=goals[i]) for i in range(0,l)]
    swapper=Local13MP(agents,(0,2,0,l-1),'y')
    swapper.reconfigure_y()
    check_valid(agents)
    # for agent in agents:
    #     print(agent.path)

def test_case3():
    # x line
    l=30
    starts=[]
    goals=[]
    for i in range(0,l,3):
        v=[(i,0),(i,1),(i,2),(i+1,0),(i+1,1),(i+1,2),(i+2,0),(i+2,1),(i+2,2)]
        vs=v.copy()
        vg=v.copy()
        np.random.shuffle(vg)
        np.random.shuffle(vs)
        starts.extend(vs[0:3])
        goals.extend(vg[0:3])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    agents=[Agent(i,starts[i],goals[i],intermediate=goals[i]) for i in range(0,l)]
    swapper=Local13MP(agents,(0,l-1,0,2),'x')
    swapper.reconfigure_x()
    check_valid(agents)
    
        
def test_case4():
    # x line
    l=30
    starts=[]
    goals=[]
    for i in range(0,l,3):
        v=[(0,i),(1,i),(2,i),(0,i+1),(1,i+1),(2,i+1),(0,i+2),(1,i+2),(2,i+2)]
        vs=v.copy()
        vg=v.copy()
        np.random.shuffle(vg)
        np.random.shuffle(vs)
        starts.extend(vs[0:3])
        goals.extend(vg[0:3])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    agents=[Agent(i,starts[i],goals[i],intermediate=goals[i]) for i in range(0,l)]
    swapper=Local13MP(agents,(0,2,0,l-1),'y')
    swapper.reconfigure_y()
    check_valid(agents)


    


        
if __name__=='__main__':
    test_case4()

    



        
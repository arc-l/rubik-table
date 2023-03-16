
import itertools
from motion_primitive_old import Local13MP
from typing import List
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import problem_generator as pg
import json
import argparse
import time
import gurobipy as gp
from gurobipy import GRB
from common import *
from LBAP import *
from umapf import *
'''
phase 0: start--->local 1/3 start--->prepare (start_inter)
phase 1: n/3 matching
phase 2: row fitting
phase 3: column fitting
phase 4: column fitting (goal_inter) -->prepare-->local 1/3 goal--> goal
'''

f = open ('./local3x3.json', "r")
local3x3_data=json.load(f)


def draw_bi_graph(B:nx.Graph):
    l, r = nx.bipartite.sets(B)
    pos = {}
    # Update position for node from each group
    pos.update((node, (1, index)) for index, node in enumerate(l))
    pos.update((node, (2, index)) for index, node in enumerate(r))
    nx.draw(B,with_labels = True,pos=pos)


class Rb_problem(object):
    def __init__(self,dim,agents):
        self.dim=dim
        self.agents=agents
        self.starts=[agent.current for agent in agents]
        self.goals=[agent.goal for agent in agents]
        self.using_h=True
     
        self.all_agents=[]
        self.colors=dict() #used for visualization
  
        for i in range(0,self.dim[1]):
            c=tuple(np.random.random(3))
            self.colors[i]=c

    def generate_local_13(self,m):
        config=[]
        for i in range(0,m,3):
            for j in range(0,m,3):
                #tmp_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
                # tmp_list=[(i,j+1),(i+1,j+1),(i+2,j+1)]
                tmp_list=[(i+1,j),(i+1,j+1),(i+1,j+2)]
                #np.random.shuffle(tmp_list)
                #print(len(tmp_list[0:3]))
                config.extend(tmp_list[0:3])
        #print(config)
        assert(len(config)==m*m/3)
        np.random.shuffle(config)
     
        return config

    def random_to_local(self):
        local=self.generate_local_13(self.dim[0])
        starts=[agent.start for agent in self.all_agents]
        goals=[agent.goal for agent in self.all_agents]
        assert(len(goals)==len(local))
        umapf=UMAPF_OR(pg.generate_full_graph(self.dim[0],self.dim[1]),starts,local)
        paths1,tasks1,dt1=umapf.solve()
        umapf=UMAPF_OR(pg.generate_full_graph(self.dim[0],self.dim[1]),goals,local)
        paths2,tasks2,dt2=umapf.solve()
        # for i in range(0,len(self.agents)):
        #     self.agents[i].start=tasks1[i]
        #     self.agents[i].goal=tasks2[i]
        #     self.agents[i].current=tasks1[i]
        #     self.agents[i].inter_goal=tasks2[i]
        for i in range(0,len(self.all_agents)):
            self.all_agents[i].start=tasks1[i]
            self.all_agents[i].goal=tasks2[i]
            self.all_agents[i].current=tasks1[i]
            self.all_agents[i].inter_goal=tasks2[i]
        return paths1+paths2,dt1+dt2
  

    def draw_configuration(self):
        data_x=[]
        data_y=[]
        data_color=[]
        for agent in self.agents:
            data_x.append(agent.current[0])
            data_y.append(agent.current[1])
            data_color.append(self.colors[agent.goal[1]])
            plt.text(agent.current[0],agent.current[1],agent.id)
        plt.scatter(data_x,data_y,color=data_color,marker='o',s=40)
        
        plt.show()

        

    def evaluate_quality(self,agents):
        costs=[abs(agent.start[0]-agent.intermediate[0])+abs(agent.inter_goal[0]-agent.intermediate[0]) for agent in agents]
        cost_phase_1=[abs(agent.start[0]-agent.intermediate[0]) for agent in agents]
        cost_phase_3=[abs(agent.inter_goal[0]-agent.intermediate[0]) for agent in agents]
        makespan=max(costs)
        makespan_phase1=max(cost_phase_1)
        makespan_phase3=max(cost_phase_3)
        sum_of_costs=sum(costs)
        print(makespan, makespan_phase1,makespan_phase3,sum_of_costs)
      
   
    def check_feasible(self, agents):
        config=[agent.intermediate for agent in agents]
        config_set=set(config)
        if not len(config_set)==len(config):
            print(len(config_set))
            print(len(config))
        assert(len(config_set)==len(config))   

    def local13_prepare(self):
        agents_start_dict=dict()
        agents_goal_dict=dict()
        # for agent in self.agents:
        for agent in self.all_agents:
            start_id=(int(agent.current[0]/3),int(agent.current[1]/3))
            goal_id=(int(agent.goal[0]/3),int(agent.goal[1]/3))
            if start_id not in agents_start_dict:
                agents_start_dict[start_id]=[]
            if goal_id not in agents_goal_dict:
                agents_goal_dict[goal_id]=[]
            agents_start_dict[start_id].append(agent)
            agents_goal_dict[goal_id].append(agent)
     

        for i in range(0,self.dim[0],3):
            for j in range(0,self.dim[1],3):
                self.prepare_cell(i,j,agents_start_dict,agents_goal_dict)
        #prepare for inter_start and inter_goal

    def cal_cost(self,agents,opt=0):
        costs=dict()
        for row in range(0,int(self.dim[0]/3)):
            for agent in agents:
                variable=(row,agent)
                if opt==0:
                    cost=abs(3*row+1-agent.inter_goal[0])#abs(row-agent.start[0])
                else:
                    cost=abs(3*row+1-agent.current[0])
                costs[variable]=cost
        return costs
        
    def prepare_model(self,agents):
        model = gp.Model('MRMP')
        model.Params.LogToConsole = 0
        model.Params.TimeLimit = 300
        objective_expression = gp.LinExpr()
        variable_keys=[]
        agent_row_dict=dict()
        row_color_dict=dict()
        vertex_dict=dict()
        for r in range(0,int(self.dim[0]/3)):
            if r not in row_color_dict:
                row_color_dict[r]=dict()
            for a in agents:
                var=(r,a)
                variable_keys.append(var)
                if a.inter_goal[1] not in row_color_dict[r]:
                    row_color_dict[r][a.inter_goal[1]]=[]
                row_color_dict[r][a.inter_goal[1]].append(var)
                if a not in agent_row_dict:
                    agent_row_dict[a]=[]
                agent_row_dict[a].append(var)
                if (a.current[1],r) not in vertex_dict:
                    vertex_dict[(a.current[1],r)]=[]
                vertex_dict[(a.current[1],r)].append(var)
        grb_variables = model.addVars(variable_keys,vtype=GRB.BINARY)


        #add constraints
        # agent is assigned to one row only:
        for agent in agents:
            variables=[grb_variables[i] for i in agent_row_dict[agent]]
            constraint_expr=gp.LinExpr([1 for i in variables],variables)
            model.addLConstr(constraint_expr, GRB.EQUAL, 1)
        #each row  the colors are different
        for row in range(0,int(self.dim[0]/3)):
            for color in  range(0,self.dim[1]):
                variables=[grb_variables[i] for i in row_color_dict[row][color]]
                constraint_expr=gp.LinExpr([1 for i in variables],variables)
                model.addLConstr(constraint_expr, GRB.EQUAL, 1)

        # each vertex must be occupied by one agent:
        for column in range(0,self.dim[1]):
            for row in range(0,int(self.dim[0]/3)):
                variables=[grb_variables[i] for i in vertex_dict[(column,row)]]
                constraint_expr=gp.LinExpr([1 for i in variables],variables)
                model.addLConstr(constraint_expr, GRB.EQUAL, 1)

        # objective
        costs1=self.cal_cost(agents,opt=0)
        #costs2=self.cal_cost(agents,opt=1)
        maxobj1= model.addVar()
        #maxobj2 =model. addVar()
        for key in variable_keys:
            model.addConstr(costs1[key]*grb_variables[key]<=maxobj1)
            #model.addConstr(costs2[key]*grb_variables[key]<=maxobj2)
        
        objective_expression=gp.LinExpr([1],[maxobj1])
        model.setObjective(objective_expression,GRB.MINIMIZE)
        model.update()
        return model, grb_variables

    def ilp_optim_match(self,agents):
        model,grb_variables=self.prepare_model(self.agents)
        model.optimize()
        obj=model.getObjective()
        #retrieve solutions
        print("optimized!")
        for agent in agents:
            for row in range(0,int(self.dim[0]/3)):
                if abs(grb_variables[(row,agent)].x-1)<1e-3:
                    agent.intermediate=(3*row+1,agent.current[1])
                    break

    def prepare_cell(self,xmin,ymin,agents_start_dict,agents_goal_dict):
        vertices=list(itertools.product(np.arange(xmin,xmin+3),np.arange(ymin,ymin+3)))
        robots=agents_start_dict[(int(xmin/3),int(ymin/3))]
        robots2=agents_goal_dict[(int(xmin/3),int(ymin/3))]
        # robots=[]
        # robots2=[]
        # for agent in self.agents:
        #     if agent.current in vertices:
        #         robots.append(agent)
        #     if agent.goal in vertices:
        #         robots2.append(agent)
        robots.sort(key=lambda x:vertices.index(x.current))
        robots2.sort(key=lambda x:vertices.index(x.goal))
        start_id=tuple([vertices.index(r.current) for r in robots])
        start_id2=tuple([vertices.index(r.goal) for r in robots2])
        key=str((start_id,'x'))
        if key not in local3x3_data:
            print(xmin,ymin,key,vertices)
        solutions=local3x3_data[key]
        key=str((start_id2,'x'))
        solutions2=local3x3_data[key]
        for i in range(len(robots)):
            path=[vertices[v] for v in solutions[i]]
            robots[i].path.extend(path)
            robots[i].current=robots[i].path[-1]
            path2=[vertices[v] for v in solutions2[i]]
            path2.reverse()
            robots2[i].inter_goal_path=path2[1:]
            robots2[i].inter_goal=path2[0]


    def local13_bimatching(self,column_dict,row=None):
        B=nx.Graph()
        bottom_nodes=list(range(0,self.dim[1]))
        top_nodes=list(column_dict.keys())
        B.add_nodes_from(top_nodes, bipartite=0)
        B.add_nodes_from(bottom_nodes, bipartite=1)
        edges=[]
        arranged_agents=dict()
        for i in top_nodes:
            for j in range(self.dim[1]):
                for agent in column_dict[i]:
                    if agent.inter_goal[1]==j:
                        edges.append((i,j))
                        #remove the agent
                        arranged_agents[(i,j)]=agent               
                        break
                
        B.add_edges_from(edges)
        #assert(nx.is_bipartite(B))
        perfect=nx.algorithms.bipartite.matching.hopcroft_karp_matching(B, top_nodes)

        for col in top_nodes:
            color=perfect[col]
            agent=arranged_agents[(col,color)]
            column_dict[col].remove(agent)
        return perfect,arranged_agents

    def matching_heuristic(self,agents):
        #agents already perfect matching
        row_dict=dict()
        for agent in agents:
            if agent.intermediate[0] not in row_dict:
                row_dict[agent.intermediate[0]]=[]
            row_dict[agent.intermediate[0]].append(agent)
        
        cost_matrix=[]
        keys=list(row_dict.keys())
        for i in range(0,len(keys)):
            r=keys[i]
            agent_r=row_dict[r]
            costs=[]
            for j in range(0,len(keys)):
                cost_l=max([abs(keys[j]-agent.current[0]) for agent in agent_r])
                costs.append(cost_l)
            cost_matrix.append(costs)
        row_ind,col_ind,makespan=labp_solve(cost_matrix)
        
        for i in range(0,len(keys)):
            r=keys[i]
            row=r
            agent_r=row_dict[row]
            c=col_ind[i]
            new_row=keys[c]
            for agent in agent_r:
                assert(agent.intermediate[0]==row)
                agent.intermediate=(new_row,agent.intermediate[1])

    def labp_matching_greedy(self,column_dict,row=0):
        cost=[]
        arranged_agents = dict()
        c_keys=list(column_dict.keys())
        
        for i in c_keys:
            #i===> column
            cost_i=[]
            for j in c_keys:
                #j===> color
                found=False
                min_d=900000
                min_agent=None
                for agent in column_dict[i]:
                    if agent.inter_goal[1] == j and abs(agent.current[0]-row)<min_d:
                        found=True
                        min_d=abs(agent.current[0]-row)
                        min_agent=agent
                if found:
                    cost_i.append(min_d)
                        # remove the agent
                    arranged_agents[(i, j)] = min_agent
                else:
                    cost_i.append(np.inf)
            cost.append(cost_i)
        col,colors,_=labp_solve(cost)
        for c in col:
            color =c_keys[colors[c]]
            agent = arranged_agents[(c_keys[c], color)]
            column_dict[c_keys[c]].remove(agent)
        perfect=dict()
        for c in col:
            perfect[c_keys[colors[c]]]=c_keys[c]
       
        return perfect, arranged_agents



    def local13_matching(self):
        # a goal vertex is a tuple (v1,v2) v2---> color, v1--->index
        # assert(len(self.agents)<=1/3*self.dim[0]*self.dim[1])
        #declare there local-1/3 is satisfied
    
        # all_agents=self.agents.copy()
        column_dict=dict()
        for agent in self.all_agents:
            if agent.current[1] not in column_dict:
                column_dict[agent.current[1]]=[]
            column_dict[agent.current[1]].append(agent)
            #column_dict[str(i)].sort(key=lambda x:x.inter_goal[0])
            # fill with virtual agents
        #the colors in column i
  
        for i in range(0,int(self.dim[0]/3)):
            # matching,arranged_agents=self.local13_bimatching(column_dict)
            matching,arranged_agents=self.labp_matching_greedy(column_dict,row=3*i+1)
            for color in list(column_dict.keys()):
                column=matching[color]
                agent=arranged_agents[(column,color)]
         
                agent.intermediate=(3*i+1,int(column))  
        self.matching_heuristic(self.all_agents)
        # self.evaluate_quality(self.agents)      
        # self.check_feasible(self.agents)
        
        
    def local13_rubik(self):
       
        self.local13_prepare()
       
        self.local13_matching()
        #self.ilp_optim_match(self.agents)
        self.local13_x_shuffle()
        print('matching done!')
        
        # assert(check_valid(self.agents))
        self.local13_y_fitting()
        self.local13_y_shuffle()
        # save_output(self.agents,computation_time=0,filename='./demo/output_random_small.yaml',save_path=True)
      
        print('y fitting done!')

 
        self.local13_x_fitting()
        self.local13_x_shuffle()

    
        self.refine_paths()
   
        # assert(check_paths_feasibility([agent.path for agent in self.agents] ))
        # assert(check_valid(self.agents))
        for agent in self.agents:
            agent.path.extend(agent.inter_goal_path)
            agent.current=agent.path[-1]
        print('x fitting done!')
       
        # for agent in self.agents:
        #     print(agent.path)
        # assert(check_agents_feasibility(self.agents))
        # self.refine_paths()
       
        #self.check_feasible(self.agents)
        # makespan=max(len(agent.path) for agent in self.agents)
        
        # print(makespan)
        # assert(check_valid(self.agents))

    def refine_paths(self):
        makespan=max(len(agent.path) for agent in self.agents)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1]) 
        
        
    def local13_y_fitting(self):
        #row shuffling
        #agent are now in the correct row, we need to put them in the correct columns.
        for agent in self.agents:
            agent.intermediate=(agent.current[0],agent.inter_goal[1])
            #agent.path.append(agent.current)
        self.check_feasible(self.agents)   

    def local13_x_fitting(self):
        for agent in self. agents:
            agent.intermediate=(agent.inter_goal[0],agent.current[1])
            #agent.path.append(agent.current)
        self.check_feasible(self.agents)   
        
    def route_to_inter(self):
        for agent in self.agents:
            agent.current=agent.intermediate




    


    def local13_y_shuffle(self):
        for i in range(0,self.dim[0],3):
            robots=[]
            for agent in self.agents:
                if agent.current[0]<=i+2 and agent.current[0]>=i:
                    robots.append(agent)
            swapper=Local13MP(robots,(i,i+2,0,self.dim[1]-1),orientation='y')
            swapper.reconfigure_y()
        # self.refine_paths()

    def local13_x_shuffle(self):
        for i in range(0,self.dim[1],3):
            robots=[]
            for agent in self.agents:
                if agent.current[1]<=i+2 and agent.current[1]>=i:
                    robots.append(agent)
            swapper=Local13MP(robots,(0,self.dim[0]-1,i,i+2),orientation='x')
            swapper.reconfigure_x()
        # self.refine_paths()
        
        




def check_feasible(starts):
    used=dict()
    for s in starts:
        if s not in used:
            used[s]=1
        else:
            return False
    return True   


    



def generate_toy_instance():
    graph=pg.generate_full_graph(60,60)
    starts=[]
    goals=[]
    for i in range(1,20,3):
        for j in range(1,20,3):
            starts.append((i,j))
            goals.append((i,j))
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts, goals






def test_nx():
    B = nx.Graph()
    # Add nodes with the node attribute "bipartite"
    top_nodes = [1, 2, 3]
    bottom_nodes = ["A", "B", "C"]
    B.add_nodes_from(top_nodes, bipartite=0)
    B.add_nodes_from(bottom_nodes, bipartite=1)
    # Add edges only between nodes of opposite node sets
    B.add_edges_from([(1, "A"), (1, "B"), (2, "B"), (2, "C"), (3, "A"), (3, "C")])
    l, r = nx.bipartite.sets(B)
    pos = {}
    pos.update((node, (1, index)) for index, node in enumerate(l))
    pos.update((node, (2, index)) for index, node in enumerate(r))
    nx.draw(B,with_labels = True,pos=pos)
    plt.show(B)




def generate_demo():
    filename='./demo/demo_random_small.yaml'
    agents,graph=pg.read_agents_from_yaml(filename)
    starts=[agent.start for agent in agents]
    goals=[agent.goal for agent in agents]
    m=30
    local=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            local.extend([(i+1,j),(i+1,j+1),(i+1,j+2)])
    umapf1=UMAPF_OR(graph,starts,local)
    paths1,tasks1,_=umapf1.solve()
    umapf2=UMAPF_OR(graph,goals,local)
    paths2,tasks2,_=umapf2.solve()
    for i in range(0,len(agents)):
        agents[i].start=tasks1[i]
        agents[i].current=tasks1[i]
        agents[i].goal=tasks2[i]
        agents[i].path=paths1[i]
        agents[i].inter_paths_u=paths2[i]
        agents[i].inter_paths_u.reverse()
    
    p=Rb_problem([m,m],agents)
    p.all_agents=agents
   
        # p.random_to_local()
    t1=time.clock()
    p.local13_rubik()
    t2=time.clock()
    
    for agent in agents:
        agent.path.extend(agent.inter_paths_u[1:])
    assert(check_paths_feasibility([agent.path for agent in agents] ))
  
    save_output(agents,computation_time=t2-t1,filename='./demo/output_random_small_better.yaml',save_path=True)



def generate_rec_demo():
    filename='./demo/demo_rth_36x48.yaml'
    agents,graph=pg.read_agents_from_yaml(filename)
    starts=[agent.start for agent in agents]
    goals=[agent.goal for agent in agents]
    xmax = max([x for x, y in graph.nodes()]) + 1
    ymax = max([y for x, y in graph.nodes()]) + 1
    print(starts)
    local=[]
    # exit(0)
    for i in range(0,xmax,3):
        for j in range(0,ymax,3):
            local.extend([(i+1,j),(i+1,j+1),(i+1,j+2)])
    umapf1=UMAPF_OR(graph,starts,local)
    paths1,tasks1,_=umapf1.solve()
    print("solved")
    umapf2=UMAPF_OR(graph,goals,local)
    paths2,tasks2,_=umapf2.solve()
    print("solved")
    for i in range(0,len(agents)):
        agents[i].start=tasks1[i]
        agents[i].current=tasks1[i]
        agents[i].goal=tasks2[i]
        agents[i].path=paths1[i]
        agents[i].inter_paths_u=paths2[i]
        agents[i].inter_paths_u.reverse()
    
    p=Rb_problem([xmax,ymax],agents)
    p.all_agents=agents
   
        # p.random_to_local()
    t1=time.time()
    p.local13_rubik()
    t2=time.time()
    
    for agent in agents:
        agent.path.extend(agent.inter_paths_u[1:])
    assert(check_paths_feasibility([agent.path for agent in agents] ))
  
    save_output(agents,computation_time=t2-t1,filename='./demo/demo_rth_36x48.path',save_path=True)
    # save_as_txt(agents,computation_time=t2-t1,filename='./demo_rth_36x48.path',save_path=True)

def generate_column_demo():
    filename='./demo/demo_column_large.yaml'
    agents,graph=pg.read_agents_from_yaml(filename)
    starts=[agent.start for agent in agents]
    goals=[agent.goal for agent in agents]
    m=60
    local=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            local.extend([(i,j+1),(i+1,j+1),(i+2,j+1)])
    umapf1=UMAPF_OR(graph,starts,local)
    paths1,tasks1,_=umapf1.solve()
    # umapf2=UMAPF_OR(graph,goals,local)
    # paths2,tasks2,_=umapf2.solve()
    for i in range(0,len(agents)):
        agents[i].start=tasks1[i]
        agents[i].current=tasks1[i]
        # agents[i].goal=tasks2[i]
        agents[i].path=paths1[i]
        # agents[i].inter_paths_u=paths2[i]
        # agents[i].inter_paths_u.reverse()
    assign_tasks(agents)
    p=Rb_problem([m,m],agents)
    p.all_agents=agents
   
        # p.random_to_local()
    t1=time.clock()
    p.local13_rubik()
    t2=time.clock()
    assert(check_paths_feasibility([agent.path for agent in agents] ))
    # for agent in agents:
    #     agent.path.extend(agent.inter_paths_u[1:])
    
    save_output(agents,computation_time=t2-t1,filename='./demo/output_column_large.yaml',save_path=True)

def test_density():
    num_agents=[1600,1920,2240,2560,2880,3200,3520,3840,4480,4800]
    for num_agent in num_agents:
        for k in range(5):
            instance_name="./data/instance/density/instances/"+str(num_agent)+'/'+str(k)+'.yaml'
            path_name="./data/instance/density/solutions/RTH_paths/"+str(num_agent)+"/"+str(k)+".json"
            graph,starts,goals=pg.read_from_yaml(instance_name)
            agents=agents_from_starts_and_goals(starts,goals)
            
            xmax,ymax=120,120
            p=Rb_problem([xmax,ymax],agents)
            virtual_agents=[]
        
            starts_res=[v for v in graph.nodes if v not in p.starts]
            np.random.shuffle(starts_res)
            goals_res=[v for v in graph.nodes if v not in p.goals]
            np.random.shuffle(goals_res)
            for i in range(0,int(xmax*xmax/3)-len(p.starts)):
                agent=Agent(len(p.starts)+i+1,starts_res[i],goals_res[i],virtual=True)
                virtual_agents.append(agent)
            p.all_agents=p.agents+virtual_agents   
            p.random_to_local()
            # ymax = max([x for x, y in graph.nodes()]) + 1
            # xmax = max([y for x, y in graph.nodes()]) + 1
            
            
            t1=time.time()

            
            p.local13_rubik()
            t2=time.time()
            save_result_as_json(agents,0,path_name,True,t2-t1)


def test_ll_rthLBA():
    grid_size=[30,60,75,120,150,240,300]
    # ymax_=[int(90000/x) for x in xmax]
    for x in grid_size:
        xmax=x
        ymax=int(90000/x)
        print("grid size=",x,'===========')
        for k in range(3):
            print("k=",k)
            instance_name='./data/instance/new/instances/'+str(x)+'/'+str(k)+'.yaml'
            path_name='./data/instance/new/solutions/30/LL/'+str(x)+'/'+str(k)+'.json'
            graph,starts,goals=pg.read_from_yaml(instance_name)
            agents=agents_from_starts_and_goals(starts,goals)
            p=Rb_problem([xmax,ymax],agents)
            p.all_agents=agents
            t1=time.time()
            p.local13_rubik()
            t2=time.time()
            save_result_as_json(agents,0,path_name,True,t2-t1)



def test_ss_rthLBA():
    grid_size=[30,60,75,120,150,240,300]
    # ymax_=[int(90000/x) for x in xmax]
    for x in grid_size:
        xmax=x
        ymax=int(90000/x)
        xmax,ymax=ymax,xmax
        print("grid size=",x,'===========')
        for k in range(3):
            
            instance_name='./data/instance/new/instances/'+str(x)+'/'+str(k)+'.yaml'
            path_name='./data/instance/new/solutions/SS/30/'+str(x)+'/'+str(k)+'.json'
            graph,starts,goals=pg.read_from_yaml(instance_name)
            print("k=",k,"num agents=",len(starts))
            starts=[(s[1],s[0]) for s in starts]
            goals=[(g[1],g[0]) for g in goals]
            agents=agents_from_starts_and_goals(starts,goals)
            p=Rb_problem([xmax,ymax],agents)
            p.all_agents=agents
            t1=time.time()
            p.local13_rubik()
            t2=time.time()
            save_result_as_json(agents,0,path_name,True,t2-t1)



def test_rectangle():
    agents,graph=pg.read_agents_from_yaml('./data/instance/rectangle/45/1.yaml')
    ymax = max([x for x, y in graph.nodes()]) + 1
    xmax = max([y for x, y in graph.nodes()]) + 1
    print(xmax,ymax)
    p=Rb_problem([xmax,ymax],agents)
    
    p.all_agents=agents
    p.local13_rubik()
    save_output(p.agents,computation_time=0,filename='./test.yaml',save_path=False)

def generate_rth_lba_paths():
    grid_size=[15,30,45,60,75,90,120,150,180,210,240,270,300,330,360]
    for m in grid_size:
        for k in range(5):
            # try:
                instance_name="./data/instance/local13/instances/"+str(m)+"/"+str(k)+".yaml"
                path_name="./data/instance/local13/solutions/RTH_paths/"+str(m)+"/"+str(k)+".json"
                graph,starts,goals=pg.read_from_yaml(instance_name)
                ymax = max([x for x, y in graph.nodes()]) + 1
                xmax = max([y for x, y in graph.nodes()]) + 1
                agents=agents_from_starts_and_goals(starts,goals)
                # local=[]
                # for i in range(0,m,3):
                #     for j in range(0,m,3):
                #         local.extend([(i+1,j),(i+1,j+1),(i+1,j+2)])
                t1=time.time()
                # umapf1=UMAPF_OR(graph,starts,local)
                # paths1,tasks1,_=umapf1.solve()
                # umapf2=UMAPF_OR(graph,goals,local)
                # paths2,tasks2,_=umapf2.solve()
                # for i in range(0,len(agents)):
                #     agents[i].start=tasks1[i]
                #     agents[i].current=tasks1[i]
                #     agents[i].goal=tasks2[i]
                #     agents[i].path=paths1[i]
                #     agents[i].inter_paths_u=paths2[i]
                #     agents[i].inter_paths_u.reverse()
                p=Rb_problem([xmax,ymax],agents)
                p.all_agents=agents
                p.local13_rubik()
                # for agent in agents:
                #     agent.path.extend(agent.inter_paths_u[1:])
                t2=time.time()
                save_result_as_json(agents,0,path_name,True,t2-t1)
            # except:
                pass
            # exit(0)
           


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--f", help="filename")
    parser.add_argument("--o",help='output')
    parser.add_argument("--w",help="heuristic")
    args = parser.parse_args()
    
    if args.f:
        graph,starts,goals=pg.read_from_yaml(args.f)
        # for i in range(0,len(starts)):
        #     starts[i]=starts[i][1],starts[i][0]
        #     goals[i]=goals[i][1],goals[i][0]
        # agents,graph=pg.read_agents_from_yaml(args.f)
        ymax = max([x for x, y in graph.nodes()]) + 1
        xmax = max([y for x, y in graph.nodes()]) + 1
        print(xmax,ymax)
        pg.save_instance_txt(graph,starts,goals,"./48x48.map","demo_block.scen")
        agents=agents_from_starts_and_goals(starts,goals)
        # assign_tasks(agents)
        p=Rb_problem([xmax,ymax],agents)
        # if args.w==0:
        #     p.using_h=False
        p.all_agents=agents
        # virtual_agents=[]
     
        # starts_res=[v for v in graph.nodes if v not in p.starts]
        # np.random.shuffle(starts_res)
        # goals_res=[v for v in graph.nodes if v not in p.goals]
        # np.random.shuffle(goals_res)
        # for i in range(0,int(xmax*xmax/3)-len(p.starts)):
        #     agent=Agent(len(p.starts)+i+1,starts_res[i],goals_res[i],virtual=True)
        #     virtual_agents.append(agent)
        # p.all_agents=p.agents+virtual_agents   
        # print(len(p.all_agents))
        # extra_steps,extra_time=p.random_to_local()
       
        t1=time.clock()
        p.local13_rubik()
        t2=time.clock()
        save_as_txt(agents,computation_time=t2-t1,filename=args.o,save_path=True)
        # save_output(p.agents,computation_time=t2-t1,filename=args.o,save_path=False)#,exra_step=extra_steps,extra_time=extra_time)
        
    else:
        graph,starts,goals=pg.read_from_yaml('./15.yaml')
        xmax = max([x for x, y in graph.nodes()]) + 1
        ymax = max([y for x, y in graph.nodes()]) + 1
        agents=agents_from_starts_and_goals(starts,goals)  
        p=Rb_problem([xmax,ymax],agents)
        #p.three_n_shuffle_rubik()
        #p.local13_rubik()
        # p.random_to_local()
        p.local13_rubik()
        #save_output(p.agents,computation_time=0,filename='./output.yaml',save_path=True)
        #graph=pg.generate_full_graph(12,2)
        #starts,goals=pg.generate_instance(graph,24)
if __name__=="__main__":
    # main()
    # generate_rth_lba_paths()
    # test_ll_rthLBA()
    test_ss_rthLBA()
    # test_density()
    # generate_rec_demo()
    # test_rectangle()
    #generate_column_demo()
    # generate_demo()







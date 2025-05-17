from hashlib import new
from random import random
from networkx.classes import graph
# from ruamel import yaml
import numpy as np
from collections import OrderedDict 
import problem_generator as pg
import argparse
import os
import errno
import itertools
from common import *
import json



w=120
h=120
fi=""
def get_id(x,y):
    return x*h+y


def check_feasible(starts):
    used=dict()
    for s in starts:
        if s in used:
            return False
        used[s]=1
    return True

def test():
    parser = argparse.ArgumentParser()
    parser.add_argument("--w", help="width")
    parser.add_argument("--h", help="height")
    parser.add_argument("--numAgents", help="numAgents")
    parser.add_argument("--fi", help="store file")
    args = parser.parse_args()
    if args.w:
        w=int(args.w)
        h=int(args.h)
        numAgents=int(args.numAgents)
        fi=args.fi
    else:
        fi="./test.yaml"
        numAgents=100
    
    contents=dict() 
    agents=[]
    obstacles=0
   
    usedStart=[]
    usedGoal=[]
    for i in range(0,numAgents):
        agent=dict() 
        sx=np.random.randint(0,w)
        sy=np.random.randint(0,h)
        while get_id(sx,sy) in usedStart:
            sx=np.random.randint(0,w)
            sy=np.random.randint(0,h)
        usedStart.append(get_id(sx,sy))
        
        gx=np.random.randint(0,w)
        gy=np.random.randint(0,h)
        while get_id(gx,gy) in usedGoal:
            gx=np.random.randint(0,w)
            gy=np.random.randint(0,h)
        usedGoal.append(get_id(gx,gy))
        agent["goal"]=yaml.comments.CommentedSeq([gx,gy])
        agent["name"]="agent"+str(i)
        agent["start"]=yaml.comments.CommentedSeq([sx,sy])
        agents.append(agent)
    contents["agents"]=agents
    
    maps=dict()
    
    maps["dimensions"]=[w,h]
    
   
    maps["obstacles"]=[]
    
    contents["map"]=maps
    
    
    
    with open(fi, 'w') as nf:
        yaml.dump(contents, nf, Dumper=yaml.RoundTripDumper)
    print("OK")

def test3():
    graph=pg.generate_full_graph(2,20)
    starts,goals=pg.generate_instance(graph,40)
    instance=pg.Instance(starts,goals,graph)
    save_yaml(instance)

def test_row():
    graph=pg.generate_full_graph(2,100)
    numAgents=[20,40,60,80,100,120]
    for numAgent in numAgents:
        for i in range(0,20):
            starts,goals=pg.generate_instance(graph,numAgent)
            instance=pg.Instance(starts,goals,graph)
            filename='./data/instance/2x100/'+str(numAgent)+'/'+str(i)+'.yaml'

            save_yaml(instance,filename)

def generate_local_13(m):
  
    config=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            tmp_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            np.random.shuffle(tmp_list)
            #print(len(tmp_list[0:3]))
            config.extend(tmp_list[0:3])
    #print(config)
    assert(len(config)==m*m/3)
    np.random.shuffle(config)
    assert(check_feasible(config))
    return config


def generate_local_19_agents_19_obs(m):
    start_config=[]
    goal_config=[]
    obs=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            # obs_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            # np.random.shuffle(obs_list)
            obs_i=(i+1,j+1)
            tmp_start_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            tmp_start_list.remove(obs_i)
            tmp_goal_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            tmp_goal_list.remove(obs_i)
            np.random.shuffle(tmp_start_list)
            np.random.shuffle(tmp_goal_list)
            start_i=tmp_start_list.pop(0)
            goal_i=tmp_goal_list.pop(0)
            start_config.append(start_i)
            goal_config.append(goal_i)
            obs.append(obs_i)
    return start_config,goal_config,obs

def generate_local_19_agents_29_obs(m):
    start_config=[]
    goal_config=[]
    obs=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            # obs_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            # np.random.shuffle(obs_list)
            obs_i=(i+1,j+1)
            tmp_start_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            tmp_start_list.remove(obs_i)
            tmp_goal_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            tmp_goal_list.remove(obs_i)
            np.random.shuffle(tmp_start_list)
            np.random.shuffle(tmp_goal_list)
         
            start_config.extend(tmp_start_list[0:2])
            goal_config.extend(tmp_goal_list[0:2])
            obs.append(obs_i)
    return start_config,goal_config,obs


    


def test_13():
    for s in [15,30,45,60,75,90]:
        graph=pg.generate_full_graph(s,s)
        for i in range(0,20):
            starts=generate_local_13(s)
            goals=generate_local_13(s)
            filename='./data/instance/local13/instances/'+str(s)+'/'+str(i)+'.yaml'
            instance=pg.Instance(starts,goals,graph)
            save_yaml(instance,filename)


def test_local_13():
    for s in [270,300,330,360,390]:
        graph=pg.generate_full_graph(s,s)
        for i in range(20):
            # graph=pg.generate_general_graph(s,s,0.05)
        
            starts=generate_local_13(s)
            goals=generate_local_13(s)
            # starts,goals=pg.generate_instance(graph,int(s*s/9),seed=np.random.randint(100))
            filename='./data/instance/local13/instances/'+str(s)+'/'+str(i)+'.yaml'
            instance=pg.Instance(starts,goals,graph)
            save_yaml(instance,filename)
            print("ok")
   


def generate_local_rectangle(m1,m2):
    config=[]
    for i in range(0,m1,3):
        for j in range(0,m2,3):
            tmp_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            np.random.shuffle(tmp_list)
            #print(len(tmp_list[0:3]))
            config.extend(tmp_list[0:3])
    #print(config)
    # assert(len(config)==m1*m2/3)
    np.random.shuffle(config)
    # assert(check_feasible(config))
    return config


def test_local_rectangle():
    m1=30
    for s in [30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]:
        graph=pg.generate_full_graph(m1,s)
        for i in range(20):
            # graph=pg.generate_general_graph(s,s,0.05)
        
            starts=generate_local_rectangle(m1,s)
            goals=generate_local_rectangle(m1,s)
            # starts,goals=pg.generate_instance(graph,int(s*s/9),seed=np.random.randint(100))
            filename='./data/instance/retangle/'+str(s)+'/'+str(i)+'.yaml'
            instance=pg.Instance(starts,goals,graph)
            save_yaml(instance,filename)
        print("ok")    

def test_random_13():
    for s in [15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]:
        graph=pg.generate_full_graph(s,s)
        #graph=pg.generate_general_graph(s,s,0.05)
        #for i in range(0,20):
            #starts=generate_local_13(s)
            #goals=generate_local_13(s)
        starts,goals=pg.generate_instance(graph,int(s*s/3),seed=np.random.randint(100))
        filename='./data/instance/random13/test/'+str(s)+'.yaml'
        instance=pg.Instance(starts,goals,graph)
        save_yaml(instance,filename)
        print("ok")

def generate_colored_two_phases():
    #local
    for s in [15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]:
        for k in range(20):
            starts=generate_local_13(s)
            goals=generate_local_13(s)
            agents=[]
            for i in range(0,len(starts)):
                agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i],group_id=goals[i][1])
                agents.append(agent)
            filename='./data/instance/colored/instances/two_phases/'+str(s)+'/'+str(k)+'.yaml'
            #instance=pg.Instance(starts,goals,graph)
            save_yaml_agents([s,s],agents,[],filename)
            print('ok')


def generate_obs_agents(xmax=100,ymax=100):
    graph=pg.generate_general_graph(xmax,ymax,0.1, np.random.randint(100))
    for numAgents in [1000,1500,2000,2500,3000]:
        starts,goals=pg.generate_instance(graph,numAgents)
        filename='./data/instance/random13_obs/'+str(numAgents)+'.yaml'
        obstacles=[(i,j) for i in range(0,xmax) for j in range(0,ymax) if (i,j) not in list(graph.nodes())]
        instance=pg.Instance(starts,goals,graph)
        save_yaml(instance,filename,obstacles)
        print("ok")


def generate_colored_local():
    #local
    for s in [15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]:
        for k in range(0,20):
            starts=generate_local_13(s)
            goals=generate_local_13(s)
            agents=[]
            colors=[goal[1] for goal in goals]
            np.random.shuffle(colors)
            for i in range(0,len(starts)):
                agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i],group_id=colors.pop())
                agents.append(agent)
            filename='./data/instance/colored/instances/local/'+str(s)+'/'+str(k)+'.yaml'
            #instance=pg.Instance(starts,goals,graph)
            save_yaml_agents([s,s],agents,[],filename)
        print('ok')

def generate_local19_19_obs():
    #local
    for s in [15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]:
    
        starts,goals,obs=generate_local_19_agents_19_obs(s)
        np.random.shuffle(goals)
        np.random.shuffle(starts)
        agents=[]
    
        for i in range(0,len(starts)):
            agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
            agents.append(agent)
        filename='./data/instance/local19_obs/test/'+str(s)+'.yaml'
        #instance=pg.Instance(starts,goals,graph)
        save_yaml_agents([s,s],agents,obs,filename)
        print('ok')

def generate_local29_19_obs():
    for s in [15,30,45,60,75,90,120,150,180]:
        for k in range(20):
            starts,goals,obs=generate_local_19_agents_29_obs(s)
            np.random.shuffle(goals)
            np.random.shuffle(starts)
            save_sg_json(starts, goals)
            print(len(starts),len(goals))
            # for i in range(0,len(starts)):
            #     agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
            #     agents.append(agent)
            # filename='./data/instance/sorting_center_new/instances/'+str(s)+'/'+str(k)+'.yaml'
            # #instance=pg.Instance(starts,goals,graph)
            # save_yaml_agents([s,s],agents,obs,filename)
        print('ok')

def generate_fully_sorted_local13(m):
    agents=[]
    starts=[]
    goals=[]
    ids=[]
    for i in range(0,m,3):
        for j in range(0,m,3):
            tmp_start_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            tmp_goal_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]            
            np.random.shuffle(tmp_start_list)
            np.random.shuffle(tmp_goal_list)
            colors=[j,j+1,j+2]
            np.random.shuffle(colors)
            goals.extend(tmp_goal_list[0:3])
            starts.extend(tmp_start_list[0:3])
            ids.extend(colors)
    np.random.shuffle(starts)
    for i in range(0,len(starts)):
        agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i],group_id=ids[i])
        agents.append(agent)
    return agents
            #print(len(tmp_list[0:3]))
            # config.extend(tmp_list[0:3])

def fully_sorted_local13():
    for s in [15,30,45,60,75,90,120,150,180,210,240]:
        for k in range(20):
            agents=generate_fully_sorted_local13(s)
            filename='./data/instance/colored/instances/sorted/'+str(s)+'/'+str(k)+'.yaml'
            #instance=pg.Instance(starts,goals,graph)
            save_yaml_agents([s,s],agents,[],filename)
        print('ok')

def generate_random_local29():
    num_agents=[15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]
    
    for s in num_agents:
        starts=[]
        goals=[]
        for i in range(0,s,3):
            for j in range(0,s,3):
                vs=list(itertools.product([i,i+1,i+2],[j,j+1,j+2]))
                vg=list(itertools.product([i,i+1,i+2],[j,j+1,j+2]))
             
                np.random.shuffle(vs)
                np.random.shuffle(vg)
                starts.extend(vs[0:2])
                goals.extend(vg[0:2])
        np.random.shuffle(starts)
        np.random.shuffle(goals)
        agents=[]
        # print(starts)
        for i in range(0,len(starts)):
            agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
            agents.append(agent)
        filename='./data/instance/local_29/test/'+str(s)+'.yaml'
        save_yaml_agents([s,s],agents,[],filename)
        print('OK')


def generate_random_local19():
    num_agents=[15,30,45,60,75,90,120,150,180,210,240,270,300,330,360,390]
    
    for s in num_agents:
        starts=[]
        goals=[]
        for i in range(0,s,3):
            for j in range(0,s,3):
                vs=list(itertools.product([i,i+1,i+2],[j,j+1,j+2]))
                vg=list(itertools.product([i,i+1,i+2],[j,j+1,j+2]))
             
                np.random.shuffle(vs)
                np.random.shuffle(vg)
                starts.extend(vs[0:1])
                goals.extend(vg[0:1])
        np.random.shuffle(starts)
        np.random.shuffle(goals)
        agents=[]
        # print(starts)
        for i in range(0,len(starts)):
            agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
            agents.append(agent)
        filename='./data/instance/local_19/test/'+str(s)+'.yaml'
        save_yaml_agents([s,s],agents,[],filename)
        print('OK')

def generate_random_360_360():
    graph=pg.generate_full_graph(120,120)
    
    for density in range(1600,4800+1,320):
        for s in range(0,20):
            agents=[]
            starts,goals=pg.generate_instance(graph,density,np.random.randint(3000))
            for i in range(0,len(starts)):
                agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
                agents.append(agent)
            filename='./data/instance/density/instances/'+str(density)+'/'+str(s)+'.yaml'
            save_yaml_agents([120,120],agents,[],filename)
        print("OK")
    

def generate_teams_colored():

    obstacles=[(i,j) for i in range(1,120,3) for j in range(1,120,3)]
    agents=[]
    k=0
    starts=[]
    goals=[]
    for i in range(0,120,3):
        for j in range(0,120,3):
            start_v=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            goal_v=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
            np.random.shuffle(start_v)
            np.random.shuffle(goal_v)
            starts.extend(start_v[0:2])
            goals.extend(goal_v[0:2])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
 
    group_dict=dict()
    for i in range(0,len(starts)):
        gi=goals[i]
        group_id=int(gi[0]/3)*40+int(gi[1]/3)
        if group_id not in group_dict:
            group_dict[group_id]=[]
        print(gi,group_id)
        agent=Agent(i,starts[i],goals[i],False,starts[i],group_id=group_id)
        group_dict[group_id].append(agent)
        agents.append(agent)

    for key,value in group_dict.items():
  
        assert(len(value)==2)
    filename='./data/instance/colored_teams/test/'+'1600_groups'+'.yaml'
    save_yaml_agents([120,120],agents,obstacles,filename)
    

def save_sg_yaml(starts,goals,filename,dim):
    agents=[]
    for i in range(0,len(starts)):
        agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
        agents.append(agent)
    save_yaml_agents([dim,dim],agents,[],filename)


def save_sg_json(starts, goals, filename):
    data_dict=dict()
    data_dict["starts"]=starts
    data_dict["goals"]=goals
    with open(filename, "w") as f:
        json.dump(data_dict, f)
    
        
  
def generate_sort_center_9x9():
    obstacles=[(i,j) for i in range(1,9,3) for j in range(1,9,3)]
    agents=[]
    starts=[]
    goals=[]
    s=9
    for i in range(0,s,3):
        for j in range(0,s,3):
            vs=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+2,j+1),(i+2,j),(i+2,j+1),(i+2,j+2)]
            vg=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+2,j+1),(i+2,j),(i+2,j+1),(i+2,j+2)]      
            np.random.shuffle(vs)
            np.random.shuffle(vg)
            starts.extend(vs[0:2])
            goals.extend(vg[0:2])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    agents=[]
    # print(starts)
    for i in range(0,len(starts)):
        agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i])
        agents.append(agent)
    filename='./test.yaml'
    save_yaml_agents([s,s],agents,obstacles,filename)
    print('OK')


def sorting_center_demo():
    xmax=48
    ymax=72
    obstacles=[(i,j) for i in range(1,xmax,3) for j in range(1,ymax,3)]
    # agents=[]
    starts=[]
    goals=[]
    n=int(xmax*ymax*2/9)
    graph=pg.generate_graph(xmax,ymax,obstacles)
    print(graph.nodes)
    starts,goals=pg.generate_instance(graph,n,np.random.randint(1000))
    filename='./demo_obs_48x72.yaml'
    agents=agents_from_starts_and_goals(starts,goals)
    save_yaml_agents([xmax,ymax],agents,obstacles,filename)
    print("demo yaml finished")


def random_demo():
    m=60

    starts=[]
    goals=[]
    n=int(m*m/3)
    graph=pg.generate_graph(m,m,[])
    starts,goals=pg.generate_instance(graph,n,np.random.randint(1000))
    filename='./demo_random.yaml'
    agents=agents_from_starts_and_goals(starts,goals)
    save_yaml_agents([m,m],agents,[],filename)
    print("demo yaml finished")


def generate_one_row_column(m):
    row=list(range(0,m))
    column=list(range(0,m))
    np.random.shuffle(column)
    rc_map= dict(zip(row, column))
    starts=generate_local_13(m)
    row_dict=dict()
    for s in starts:
        if s[0] not in row_dict:
            row_dict[s[0]]=[]
        row_dict[s[0]].append(s)
    final_starts=[]
    final_goals=[]
    ids=[]
    for row,row_starts in row_dict.items():
        c=rc_map[row]
        possible_row=list(range(1,m,3))
        np.random.shuffle(possible_row)
        for s in row_starts:
            goal_row=possible_row.pop()
            final_starts.append(s)
            final_goals.append((goal_row,c))
            ids.append(row)
    # assert(len(set(final_goals))==len(final_goals))
    # assert(len(set(final_starts))==len(final_starts))
    # assert(len(final_starts)==len(final_goals))
    return final_starts,final_goals,ids  


def generate_sorting(m):
    starts=[]
    goals=[]
    ids=[]
    for i in range(0,m):
        for j in range(1,m,3):
            starts.append((j,i))
            goals.append((i,j))
            ids.append(j)
    return starts,goals,ids


def row_column_sort():
    s=[15,30,45,60,75,90,120,150,180,210,240,270,300,330,360]
    for m in s:
        for k in range(0,10):
            agents=[]
            starts,goals,ids=generate_one_row_column(m)
            for i in range(len(starts)):
                agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i],group_id=ids[i])
                agents.append(agent)
            
            filename='./data/instance/colored/instances/row_column/'+str(m)+'/'+str(k)+'.yaml'
            save_yaml_agents([m,m],agents,[],filename)
        print(m,"OK")


def row_column_sortx():
    s=[15,30,45,60,75,90,120,150,180,210,240,270,300,330,360]
    for m in s:
        agents=[]
        starts,goals,ids=generate_sorting(m)
        for i in range(len(starts)):
            agent=Agent(i,starts[i],goals[i],virtual=False,current=starts[i],group_id=ids[i])
            agents.append(agent)
        
        filename='./data/instance/colored/instances/row_column/test/'+str(m)+'.yaml'
        save_yaml_agents([m,m],agents,[],filename)
        print(m,"OK")

def generate_circle():
    for m in range(30,361,30):
        starts=[]
        goals=[]
        for i in range(1,m,3):
            low_x=i
            high_x=m-1-i
            low_y=i
            high_y=m-1-i
            start_set=set()
            start_set.update([(i,low_y) for i in range(low_x,high_x+1)])
            start_set.update([(i,high_y) for i in range(low_x,high_x+1)])
            start_set.update([(low_x,i) for i in range(low_y,high_y+1)])
            start_set.update([(high_x,i) for i in range(low_y,high_y+1)])
            
            starts.extend(list(start_set))
        # starts.extend([(6,7),(8,7)])
        for start in starts:
            
            goal=(m-1-start[0],m-1-start[1])
            goals.append(goal)
        print(len(starts),len(set(starts)),len(set(goals)),m)
        filename='./data/instance/ring_json/'+str(m)+"x"+str(m)+'_'+str(0)+'.json'
        # save_sg_yaml(starts,goals,filename,m)
        save_sg_json(starts,goals,filename)
    # return starts,goals


def generate_ring():
    m=48
    starts=[]
    goals=[]
    for i in range(1,m,3):
        low_x=i
        high_x=m-1-i
        low_y=i
        high_y=m-1-i
        start_set=set()
        start_set.update([(i,low_y) for i in range(low_x,high_x+1)])
        start_set.update([(i,high_y) for i in range(low_x,high_x+1)])
        start_set.update([(low_x,i) for i in range(low_y,high_y+1)])
        start_set.update([(high_x,i) for i in range(low_y,high_y+1)])
        
        starts.extend(list(start_set))
    # starts.extend([(6,7),(8,7)])
    for start in starts:
        
        goal=(m-1-start[0],m-1-start[1])
        goals.append(goal)
    print(len(starts),len(set(starts)),len(set(goals)),m)
    filename='./demo_ring'+'.yaml'
    save_sg_yaml(starts,goals,filename,m)

def generate_block_demo():
    d=12
    m=48
    starts=generate_local_13(m)
    goals=generate_local_13(m)
    md=int(m/d)
    ids=[]
    start_ids=[(i,j) for i in range(0,md) for j in range(0,md)]
    goal_ids=[(i,j) for i in range(0,md) for j in range(0,md)]
    np.random.shuffle(goal_ids)
    #find_agents:
    agents_groups=dict()
    for goal in goals:
        goal_id=(int(goal[0]/d),int(goal[1]/d))
        if goal_id not in agents_groups:
            agents_groups[goal_id]=[]
        agents_groups[goal_id].append(goal)
    new_goals=[]
    for start in starts:
        start_id=(int(start[0]/d),int(start[1]/d))
        goal_id=goal_ids[start_ids.index(start_id)]
        new_goal=agents_groups[goal_id].pop()
        ids.append(start_ids.index(start_id))
        new_goals.append(new_goal)
    print(len(starts),len(set(starts)),len(set(new_goals)),m)
    filename='./demo_block_48.yaml'
    agents=[]

    for i in range(len(starts)):
        agent=Agent(i,starts[i],new_goals[i],virtual=False,current=starts[i],group_id=ids[i])
        agents.append(agent)
    save_yaml_agents([m,m],agents,[],filename)
    

def generate_blocks():
    d=15
    for m in range(30,361,30):
        for k in range(0,20):
            starts=generate_local_13(m)
            goals=generate_local_13(m)
            md=int(m/d)
            ids=[]
            start_ids=[(i,j) for i in range(0,md) for j in range(0,md)]
            goal_ids=[(i,j) for i in range(0,md) for j in range(0,md)]
            np.random.shuffle(goal_ids)
            #find_agents:
            agents_groups=dict()
            for goal in goals:
                goal_id=(int(goal[0]/d),int(goal[1]/d))
                if goal_id not in agents_groups:
                    agents_groups[goal_id]=[]
                agents_groups[goal_id].append(goal)
            new_goals=[]
            for start in starts:
                start_id=(int(start[0]/d),int(start[1]/d))
                goal_id=goal_ids[start_ids.index(start_id)]
                new_goal=agents_groups[goal_id].pop()
                ids.append(start_ids.index(start_id))
                new_goals.append(new_goal)
            print(len(starts),len(set(starts)),len(set(new_goals)),m)
            filename='./data/instance/blocks_json/'+str(m)+"x"+str(m)+'_'+str(k)+'.json'
            # agents=[]
      
            # for i in range(len(starts)):
            #     agent=Agent(i,starts[i],new_goals[i],virtual=False,current=starts[i],group_id=ids[i])
            #     agents.append(agent)
            # save_yaml_agents([m,m],agents,[],filename)
            # save_sg_yaml(starts,new_goals,filename,m)
            save_sg_json(starts,goals,filename)

def generate_column_demo(m):
    graph=pg.generate_full_graph(m,m)
    starts,goals=pg.generate_instance(graph,int(m*m/3))
    new_goals=[(i,j) for i in range(0,m,3) for j in range(0,m)]
    ids=[]
    for goal in new_goals:
        ids.append(int(goal[0]/3))
    agents=[]
    for i in range(0,len(starts)):
        agent=Agent(i,starts[i],new_goals[i],virtual=False,current=starts[i],group_id=ids[i])
        agents.append(agent)
    save_yaml_agents([m,m],agents,[],filename='./demo_column_large.yaml')
    
def generate_19_density():
    
    def generate_local_19(m):
        starts=[]
        goals=[]
        for i in range(0,m,3):
            for j in range(0,m,3):
                tmp_start_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
                tmp_goal_list=[(i,j),(i+1,j),(i+2,j),(i,j+1),(i+1,j+1),(i+2,j+1),(i,j+2),(i+1,j+2),(i+2,j+2)]
                np.random.shuffle(tmp_start_list)
                np.random.shuffle(tmp_goal_list)
                #print(len(tmp_list[0:3]))
                starts.append(tmp_start_list[0])
                goals.append(tmp_goal_list[0])
        return starts,goals

    for s in range(30,301,30):
        graph=pg.generate_full_graph(s,s)
        # graph=pg.generate_general_graph(s,s,0.05)
        starts=generate_local_19(s)
        goals=generate_local_19(s)
        # starts,goals=pg.generate_instance(graph,int(s*s/9),seed=np.random.randint(100))
        filename='./data/instance/local19/instances/'+str(s)+'/'+str(i)+'.yaml'
        instance=pg.Instance(starts,goals,graph)
        save_yaml(instance,filename)
        print("ok")
    graph=pg.generate_graph(s,s)

def generate_full():
    def generate_one_full(m,n):
        starts=[]
        goals=[]
        starts=[(i,j) for i in range(m) for j in range(n)]
        goals=starts.copy()
        np.random.shuffle(starts)
        np.random.shuffle(goals)
        return starts,goals
    
    size=[12,18,24,30,36,42,48,54,60,66]
    for m in size:
        graph=pg.generate_full_graph(m,5*m)
        for k in range(10):
            starts,goals=generate_one_full(m,5*m)
            filename='./data/instance/1-5/instances/'+str(m)+"/"+str(k)+'.ymal'
            instance=pg.Instance(starts,goals,graph)
            save_yaml(instance,filename)
        print("ok")
        
def generate_full_demo():
    def generate_one_full(m,n):
        starts=[]
        goals=[]
        starts=[(i,j) for i in range(m) for j in range(n)]
        goals=starts.copy()
        np.random.shuffle(starts)
        np.random.shuffle(goals)
        return starts,goals
    graph=pg.generate_full_graph(6,6)
    starts,goals=generate_one_full(6,6)
    filename='./demo_full_6x6.yaml'
    instance=pg.Instance(starts,goals,graph)
    save_yaml(instance,filename)
    print('ok')

def generate_fixed():
    num=90000
    size=[30,60,75,120,150,240,300] 
    for m1 in size:
        m2=int(num/m1)
        graph=pg.generate_full_graph(m1,m2)
        for k in range(20):
            
           starts=generate_local_rectangle(m1,m2)
           goals=generate_local_rectangle(m1,m2)  
           filename='./data/instance/new/instances/'+str(m1)+"/"+str(k)+'.yaml'
           instance=pg.Instance(starts,goals,graph)
           print(filename)
           save_yaml(instance,filename)
        print("ok")
               
            

#   goals=generate_local_13(s)
def generate_random_36x54():
    graph=pg.generate_full_graph(36,48)
    num=int(36*48/3)
    starts,goals=pg.generate_instance(graph,num)
    instance=pg.Instance(starts,goals,graph)
    save_yaml(instance,'./demo_rth_36x48.yaml')
    


if __name__ == "__main__":
    #test_row()
    
    #instance=pg.Instance(starts,goals,graph)
    # save_yaml_agents([6,6],agents,[],filename)
    # generate_colored_local()
    #generate_local29_19_obs()
    #fully_sorted_local13()
    #test_random_13()
    # generate_random_local29()
    # generate_colored_two_phases()
    # generate_random_360_360()
    # generate_teams_colored()
    #test_local_13()
    # generate_colored_two_phases()
    # generate_sort_center_9x9()
    # generate_local29_19_obs()
    # sorting_center_demo()
    # generate_ring()
    #test_local_rectangle()
    generate_full_demo()
    # generate_random_36x54()
    # generate_blocks()
    # generate_ring()
    # generate_circle()
    #generate_fixed()
    # random_demo()
    # generate_one_row_column(60)
    # row_column_sortx()
    # generate_column_demo(60)
    # generate_block_demo()
    # generate_local29_19_obs()

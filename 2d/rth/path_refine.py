from os import path
from common import *
import yaml
import numpy as np
from collections import OrderedDict
import copy

def remove_redundant(paths):
    
    new_paths=paths.copy()
    for i in range(0,len(new_paths)):
        t=0
        while t<len(new_paths[i])-1:
            if new_paths[i][t]==new_paths[i][t+1]:
                new_paths[i].pop(t)
            else:
                t=t+1
        assert(new_paths[0]==paths[0])
        assert(new_paths[-1]==paths[-1])
    return new_paths

def test_remove():
    paths=read_output('output_block_small.yaml')
    makespan=max([len(p) for p in paths])
    new_paths=remove_redundant(paths)
    new_makespan=max([len(p) for p in new_paths])
    print(makespan,new_makespan)
    # print(count_conficts(new_path))
    agents=[]
    starts=[p[0] for p in new_paths]
    goals=[p[-1] for p in new_paths]
    for i in range(0,len(paths)):
        agent=Agent(i,starts[i],goals[i])
        agent.path=new_paths[i]
        agents.append(agent)
    save_output(agents,0,'./demo/test.yaml',True)

def shrink_paths():
    paths=read_output('output_block_small.yaml')
    makespan=max([len(p) for p in paths])
    new_paths=remove_redundant(paths)
    conflicts=dict()
    numConflicts=0
    for p in new_paths:
        for t in range(0,len(p)):
            if (p[t],t) not in conflicts:
                conflicts[(p[t],t)]=0
            else:
                numConflicts=numConflicts+1
                print(numConflicts,"found conflicts at t=",t)

class CBS_node(object):
    def __init__(self,paths) -> None:
        self.paths=copy.deepcopy(paths)
        self.makespan=max([len(p) for p in paths])
        self.soc=sum([len(p) for p in paths])
    
    def find_first_conflict(self):
        conflicts=dict()
        for t in range(self.makespan):
            for p in range(self.paths):
                tj=t
                if len(p)<self.makespan:
                    tj=len(p)-1
                if (p[tj],t) not in conflicts:
                    conflicts[(p[tj],t)]=0
                else:
                    return ()

                    


        
               


def count_conficts(paths):
    conflicts=dict()
    for p in paths:
        for t in range(len(p)):
            if (p[t],t) not in conflicts:
                conflicts[(p[t],t)]=0
            else:
                conflicts[(p[t],t)]=conflicts[(p[t],t)]+1
    numConflicts=0
    for key,value in conflicts.items():
        numConflicts=numConflicts+value
    return numConflicts

if __name__=="__main__":
    shrink_paths()
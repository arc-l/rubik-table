#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math

scale=10
Colors = ['orange']#, 'blue', 'green']
def color_for_goal(g,m):
    start_color=[1,0,0]
    start_color[1]=start_color[1]+g[0]/m
    start_color[2]=start_color[2]+g[1]/m
    start_color[0]=start_color[0]-(g[0]+g[1])/(2.5*m)
    
    return np.array(start_color)
    

class Animation:
  def __init__(self, map, schedule):
    self.map = map
    self.schedule = schedule
    m=map["map"]["dimensions"][0]
    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
    sizex=16
    self.fig = plt.figure(frameon=False, figsize=(sizex * aspect, sizex))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] - 0.5
    ymax = map["map"]["dimensions"][1] - 0.5

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')
    print("starting!")
   
    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='red'))
    #self.arrange_block_colors()
    #self.arrange_column_colors()
    self.arrange_random_colors(m)
    #self.colors = np.random.rand(len(map["agents"]),3)
    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):

      self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=self.colors[i%len(self.colors)], edgecolor='black', alpha=0.15))
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]

      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.3, facecolor=self.colors[i%len(self.colors)], edgecolor='black')
      self.agents[name].original_face_color = self.colors[i%len(self.colors)]
      self.patches.append(self.agents[name])
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      #self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''),fontsize=10)
      #self.agent_names[name].set_horizontalalignment('center')
      #self.agent_names[name].set_verticalalignment('center')
      #self.artists.append(self.agent_names[name])

    print("agents created!")

    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()
    self.init_func()
    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * scale,
                               interval=scale,
                               blit=True,repeat=False)
    self.anim.save('example.mp4',fps=20)
    
  def arrange_square_colors(self):
    self.colors=[]
    color_dict=dict()
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
        si=(d["start"][0],d["start"][1])
        gi=(d["goal"][0],d["goal"][1])
        if si not in color_dict:
            color_dict[si]=np.random.rand(3)
            color_dict[gi]=np.random.rand(3)
        self.colors.append(color_dict[si])
        
  def arrange_block_colors(self):
       m=30
       dd=6
       self.colors=[]
       color_dict=dict()
       for d, i in zip(map["agents"], range(0, len(map["agents"]))):
           si=(d["start"][0],d["start"][1])
           s_id=(int(si[0]/dd),int(si[1]/dd))
           if s_id not in color_dict:
               color_dict[s_id]=np.random.rand(3)
           self.colors.append(color_dict[s_id])
        
           
  def arrange_column_colors(self):
      
      self.colors=[]
      color_dict=dict()
      for d, i in zip(map["agents"], range(0, len(map["agents"]))):
          si=(d["start"][0],d["start"][1])
          gi=(d["goal"][0],d["goal"][1])
          g_id=int(gi[0]/3)
          if g_id not in color_dict:
              color_dict[g_id]=np.random.rand(3)
          self.colors.append(color_dict[g_id])
   
        
  def arrange_random_colors(self,m=30):
      
      self.colors=[]


      for d, i in zip(map["agents"], range(0, len(map["agents"]))):
          si=(d["start"][0],d["start"][1])
          gi=(d["goal"][0],d["goal"][1])
         
          self.colors.append(color_for_goal(gi,m))     
        
  
  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=800),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for agent_name in self.schedule["schedule"]:
      agent = schedule["schedule"][agent_name]
      pos = self.getState(i / scale, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      #self.agent_names[agent_name].set_position(p)

    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.7:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file,Loader=yaml.Loader)

  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file,Loader=yaml.Loader)

  animation = Animation(map, schedule)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()

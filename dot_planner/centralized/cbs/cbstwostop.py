#!/usr/bin/env python3
"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""
import sys
import rospy
sys.path.insert(0, '../')
import argparse
import yaml
import tf
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16
from math import fabs
from itertools import combinations
from copy import deepcopy

from cbs.a_star import AStar

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors


    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()
    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan

path_pub1 = rospy.Publisher('/dot4/path', Path, queue_size=0,latch=True) 
path_pub2=rospy.Publisher('/dot2/path', Path, queue_size=0,latch=True) 
path_pub3=rospy.Publisher('/dot3/path', Path, queue_size=0,latch=True) 
path_pub4=rospy.Publisher('/dot1/path', Path, queue_size=0,latch=True)
path_pub5=rospy.Publisher('/dot5/path', Path, queue_size=0,latch=True)
path_pub6=rospy.Publisher('/dot6/path', Path, queue_size=0,latch=True)
servo_pub1=rospy.Publisher('/dot4/drop', String, queue_size=1,latch=True)
servo_pub2=rospy.Publisher('/dot2/drop', String, queue_size=1,latch=True)
        
def publish_plan(solution,ag):
        """
        publish the global plan
        """
        
        global coords,coords2
          
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for p in solution[ag]:
            pose = PoseStamped()
            pose.header.frame_id='map'
            pose.header.stamp=rospy.Time.now()
            coor=coords2[p['x']][p['y']]
            pose.pose.position.x = coor[0]
            pose.pose.position.y = coor[1]
            print(p['x'],p['y'])
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            msg.poses.append(pose)
                       
        rospy.loginfo("Publiing Plan...")
        # print(msg)
        return msg
        

gc=[(4,1),(6,1)]
ptg={
'm':[(2,1),(3,2),(1,2),(2,3)],
'd':[(2,3),(3,4),(1,4),(2,5)],
'k':[(2,5),(3,6),(1,6),(2,7)],
'c':[(4,1),(5,2),(3,2),(4,3)],
'b':[(4,3),(5,4),(3,4),(4,5)],
'h':[(4,5),(5,6),(3,6),(4,7)],
'p':[(6,1),(5,2),(7,2),(6,3)],
'a':[(6,3),(5,4),(7,4),(6,5)],
'j':[(6,5),(5,6),(7,6),(6,7)]
     }
pkg1=['a','p','m','d','j','m','k','b','k','d','j','d','c','h','d']
pkg2=['a','b','k','d','d','b','a','c','h','c','h','b','b','a','b']
destbot=['c','p']
tp1=len(pkg1)
tp2=len(pkg2)
acp={'m':(2,2),'d':(2,4),'k':(2,6),'c':(4,2),'b':(4,4),'h':(4,6),'p':(6,2),'a':(6,4),'j':(6,6)}
botid=[4,2]
st=[0,0]
tc=[(6,3),(6,7)]
f=0
bots=2
ind1=PoseStamped()
ind1.header.frame_id='map'
ind1.pose.position.x=-1.17
ind1.pose.position.y=0.380
ind1.pose.orientation.w=1
ind2=PoseStamped()
ind2.header.frame_id='map'
ind2.pose.position.x=-1.15
ind2.pose.position.y=-0.354
ind2.pose.orientation.w=1
ind11=PoseStamped()
ind11.header.frame_id='map'
ind11.pose.position.x=-1.105
ind11.pose.position.y=0.380
ind11.pose.orientation.w=1
ind22=PoseStamped()
ind22.header.frame_id='map'
ind22.pose.position.x=-1.105
ind22.pose.position.y=-0.354
ind22.pose.orientation.w=1
indy=PoseStamped()
indx1=PoseStamped()
indx1.header.frame_id='map'
indx1.pose.position.x=-0.899
indx1.pose.position.y=0.380
indx1.pose.orientation.w=1
indy1=PoseStamped()
indy1.header.frame_id='map'
indy1.pose.position.x=-0.910
indy1.pose.position.y=-0.360
indy1.pose.orientation.w=1

def trajer1(data):
    global re
    re[0]=1
def trajer2(data):
    global re
    re[1]=1
def dropper1(data):
    global dropping
    if data.data==1:
      print("Dropping0 updated 1")
      dropping[0]=1
    else:
        dropping[0]=0
def dropper2(data):
    print(data)
    global dropping
    if data.data==1:
      dropping[1]=1
      print("Dropping1 updated 1")
    else:
        dropping[1]=0
# def trajer3(data):
#     global re
#     re[2]=1
# def trajer4(data):
#     global re
#     re[3]=1
# def trajer5(data):
#     global re
#     re[4]=1
# def trajer6(data):
#     global re
#     re[5]=1
def sgn(x):
    if x==0:
        return 0
    elif x>0:
        return 1
    else:
        return -1
def mapper(x1,y1):
   xd=round((y1)/0.3048)
   yd=round((x1)/0.3048)
   gx6=4-xd
   gy6=4+yd
   return (gx6,gy6)

def imap(x1,y1):
#    return (-1.143+y1*0.1524,1.143-x1*0.1524)
    return coords2[x1][y1]
 
def mark(m1,m2):
    global st,gc,tc,re,pmsg,bots,minlen,dropping,pkg1,pkg2,destbot,path_pub1,path_pub2,servo_pub1,servo_pub2,ind1,ind2,acp,ind11,ind22,indx1,indy1
    pathinitialise()
    re=[0,0]
    dropping=[1,1]
    s=0
    rate=rospy.Rate(20)
    msgl=[m1,m2]
    st=[0,0]
    path_pub=[path_pub1,path_pub2]
    servo_pub=[servo_pub1,servo_pub2]
    nopub=[0,0]
    lm=[len(m1.poses),len(m2.poses)]
    for i in range(bots):
        offx=0
        offy=0
        if not induct(tc[i][0],tc[i][1]):
            destin=acp[destbot[i]]
            dirx=destin[1]-tc[i][1]
            diry=destin[0]-tc[i][0]
            print(dirx,diry)
            if dirx!=0:
                offx=sgn(dirx)*0.075
                if (tc[i][0]==2 and tc[i][1]==4) or (tc[i][0]==2 and tc[i][1]==6):
                    offx=sgn(dirx)*0.09
                print("Offsetting x=",offx)
            if diry!=0:
                offy=-1*sgn(diry)*0.075
                if (tc[i][0]==6 and tc[i][1]==4):
                    offy=-1*sgn(diry)*0.09
                print("Offseting y=",offy)
            msgl[i].poses[lm[i]-1].pose.position.x+=offx
            msgl[i].poses[lm[i]-1].pose.position.y+=offy
    for i in range(bots):
       if lm[i]==1:
          if gc[i][0]==3 and gc[i][1]==1 and not pkg1:
              lm[i]=500
          if gc[i][0]==5 and gc[i][1]==1 and not pkg2:
              lm[i]=500
    # print(lm)
    print("least length=",min(lm))
    if min(lm)<500:
     for i in range(1,min(lm)+1):
      s=0
      re=[0,0]
      dropping=[1,1]
      for j in range(bots):
         if(i==lm[j]):
            st[j]=1
      pathinitialise()
      for j in range(bots):
        if st[j]==0:
          pmsg[j].poses.append(msgl[j].poses[i-1])
          pmsg[j].poses.append(msgl[j].poses[i])
          mapped=mapper(msgl[j].poses[i].pose.position.x,msgl[j].poses[i].pose.position.y)
          gc[j]=(mapped[0],mapped[1])
        else:
           imapped=imap(gc[j][0],gc[j][1])
           if (gc[j][0]==3 and gc[j][1]==1):
              pmsg[j].poses.append(msgl[j].poses[i-1])
              pmsg[j].poses.append(indx1)
              pmsg[j].poses.append(ind11)
              pmsg[j].poses.append(ind11)
              pmsg[j].poses.append(ind1)
              pmsg[j].poses.append(ind1)
              pmsg[j].poses.append(ind11)
              pmsg[j].poses.append(ind11)
              pmsg[j].poses.append(indx1)
              pmsg[j].poses.append(msgl[j].poses[i-1])
           elif gc[j][0]==5 and gc[j][1]==1:
              pmsg[j].poses.append(msgl[j].poses[i-1])
              pmsg[j].poses.append(indy1)
              pmsg[j].poses.append(ind22)
              pmsg[j].poses.append(ind22)
              pmsg[j].poses.append(ind2)
              pmsg[j].poses.append(ind2)
              pmsg[j].poses.append(ind22)
              pmsg[j].poses.append(ind22)
              pmsg[j].poses.append(indy1)
              pmsg[j].poses.append(msgl[j].poses[i-1])
           else:
             nopub[j]=1
             dropping[j]=0
            #  servo_pub[j].publish(destbot[j]+str(botid[j]))
            #  print("Servo activated",botid[j])
      for j in range(bots):
          if not nopub[j]:
            path_pub[j].publish(pmsg[j])  
          else:
              print("Servo activated",botid[j])
              servo_pub[j].publish(destbot[j]+str(botid[j]))
    #   print("s=",s)
      while s!=bots and not rospy.is_shutdown():
          s=0
        #   print("Dropping",dropping)
        #   print(re)
          for i in range(bots):
              if dropping[i]!=1:
                  continue
              s+=re[i]
          rate.sleep()
      rospy.sleep(3)
    #   print(s)
     if not rospy.is_shutdown():
        print("REcalculating")
        targetpt()
    else:
        print("All delivered")

def dis(x1,y1,x2,y2):
      return abs(x2-x1)+abs(y2-y1)
def induct(x,y):
   if (x==3 and y==1) or (x==5 and y==1):
       return True
   else:
     return False
def targetpt():
    global gc,tc,st,ptg,pkg1,pkg2,bots,f,tp1,tp2
    ipl=[]
    spl=[]
    # print("f=",f)
    if f==0:
        for i in range(bots):
            ipl.append((gc[i],i))
        f=1
    for i in range(bots):
      if st[i]==1:
        if induct(gc[i][0],gc[i][1]):
           spl.append((gc[i],i))
        else:
           ipl.append((gc[i],i))
           
    inductpts=[(3,1),(5,1)]
    for p in inductpts:
      mini=500
      g=0
      if tc.count(p)==0:  
        for i in ipl:
            disi=dis(i[0][0],i[0][1],p[0],p[1])
            if disi<mini:
               posi=i[1]
               posj=i
               mini=disi
               g=1
        if g==1:
         tc[posi]=p
         ipl.remove(posj)
       
    for i in spl:
       if i[0][0]==3 and i[0][1]==1:
           if pkg1:
              dest=pkg1[0]
              print("Picked up package no",tp1-len(pkg1)+1,"from induct 1,Destination =",pkg1[0])
              destlist=ptg[dest]
              for j in destlist:
                  if tc.count(j)==0:
                     targ=j
                     break
              pkg1.pop(0)
              destbot[i[1]]=dest
           else:
             targ=(3,1)
           tc[i[1]]=targ
       if i[0][0]==5 and i[0][1]==1:
          if pkg2:
             dest=pkg2[0]
             print("Picked up package no",tp2-len(pkg2)+1,"from induct 2,Destination =",pkg2[0])
             destlist=ptg[dest]
             for j in destlist:
                if tc.count(j)==0:
                   targ=j
                   break
             pkg2.pop(0)
             destbot[i[1]]=dest
          else:
             targ=(5,1)
          tc[i[1]]=targ
          
    cbscalc()

def statics():
      tfl=tf.TransformListener()
      stat=[]
      for i in range(12,54):
          stat.append(i)
      for x in stat:
          tfl.waitForTransform('map','tag_'+str(x),rospy.Time.now(),rospy.Duration(5.0))
          (trans,rot)=tfl.lookupTransform('map','tag_'+str(x),rospy.Time(0))
          stadic[x]=(trans[0],trans[1])
def mapping():
    global coords2,coords
    coords2=[]
    coords=[]
    for i in range(0,9):
          col=[]
          for j in range(0,9):
             col.append((-1.143+2*j*0.1524,1.143-2*i*0.1524))
          coords.append(col)

    tag=14
    for i in range(1,8):
      col=[]
      for j in range(1,8):
         if(i%2==0 and j%2==0) or (i==0) or (j==0) or (i==8) or (j==8):
             col.append((0,0))
             continue
        #  print(tag)
         coords[i][j]=(stadic[tag][0],stadic[tag][1])
         tag+=1

    coords2=coords
def cbscalc():
    global gc,tc,bots
    print("Cbs start=",gc)
    print("Targets=",tc)
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']
    
    for i in range(bots):
    #    print(agents[0]['start'])
       agents[i]['start']=[gc[i][0],gc[i][1]]
       agents[i]['goal']=[tc[i][0],tc[i][1]]
#    print(agents)
    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    msg1=publish_plan(solution,'agent0')
    msg2=publish_plan(solution,'agent1')
    # msg3=publish_plan(solution,'agent2')
    # msg4=publish_plan(solution,'agent3')
    # msg5=publish_plan(solution,'agent4')
    # msg6=publish_plan(solution,'agent5')
    mark(msg1,msg2)
    if not solution:
        print(" Solution not found" )
def pathinitialise():
      global pmsg,bots
      for i in range(bots):
        pa = Path()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        pmsg[i]=pa
    # print("Pmsg",pmsg[0])
def main():
   global f
   traj_call1=rospy.Subscriber('/dot4/trajectory_finished',Bool,trajer1,queue_size=10)
   traj_call2=rospy.Subscriber('/dot2/trajectory_finished',Bool,trajer2,queue_size=10)
   drop_call1=rospy.Subscriber('/dot4/dropdone',Int16,dropper1,queue_size=10)
   drop_call2=rospy.Subscriber('/dot2/dropdone',Int16,dropper2,queue_size=10)
#    traj_call3=rospy.Subscriber('/dot3/trajectory_finished',Bool,trajer3,queue_size=10)
#    traj_call4=rospy.Subscriber('/dot4/trajectory_finished',Bool,trajer4,queue_size=10)
#    traj_call5=rospy.Subscriber('/dot5/trajectory_finished',Bool,trajer5,queue_size=10)
#    traj_call6=rospy.Subscriber('/dot6/trajectory_finished',Bool,trajer6,queue_size=10)
   pathinitialise()
   statics()
   mapping()
   if f==0:
      targetpt()

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)
    stadic={}
    pa1=Path()
    pa1.header.frame_id = "map"
    pa1.header.stamp = rospy.Time.now()
    pa2=Path()
    pa2.header.frame_id="map"
    pa2.header.stamp=rospy.Time.now()
    pmsg=[pa1,pa2]
    dropping=[1,1]
    coords=[[(0.0,0.0)]*9 for i in range(9)]
    main()
    rospy.spin()

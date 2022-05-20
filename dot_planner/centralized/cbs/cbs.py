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
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8
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

path_pub1 = rospy.Publisher('/dot1/path', Path, queue_size=10) 
path_pub2=rospy.Publisher('/dot2/path', Path, queue_size=10) 
path_pub3=rospy.Publisher('/dot3/path', Path, queue_size=10) 
path_pub4=rospy.Publisher('/dot4/path', Path, queue_size=10)
path_pub5=rospy.Publisher('/dot5/path', Path, queue_size=10)
path_pub6=rospy.Publisher('/dot6/path', Path, queue_size=10)
        
def publish_plan(solution,ag):
        """
        publish the global plan
        """
        #print(coords)
        global coords
        coords=[]
        for i in range(0,16):
          col=[]
          for j in range(0,16):
             col.append((-1.143+j*0.1524,1.143-i*0.1524))
          coords.append(col)
          
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for p in solution[ag]:
            pose = PoseStamped()
            coor=coords[p['x']][p['y']]
            pose.pose.position.x = coor[0]
            pose.pose.position.y = coor[1]
#            print(p['x'],p['y'])
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
            msg.poses.append(pose)
                       
        rospy.loginfo("Publishing Plan...")
        return msg
        

gc=[(1,10),(5,0),(10,0),(9,7),(4,5),(3,1)]
ptg={
'm':[(2,2),(3,2),(4,2),(5,2),(5,3),(5,4),(5,5),(4,5),(3,5),(2,5),(2,4),(2,3)],
'd':[(2,6),(3,6),(4,6),(5,6),(5,7),(5,8),(5,9),(4,9),(3,9),(2,9),(2,8),(2,7)],
'k':[(2,10),(3,10),(4,10),(5,10),(5,11),(5,12),(5,13),(4,13),(3,13),(2,13),(2,12),(2,11)],
'c':[(6,2),(7,2),(8,2),(9,2),(9,3),(9,4),(9,5),(8,5),(8,4),(8,3),(8,2),(7,2)],
'b':[(6,6),(7,6),(8,6),(9,6),(9,7),(9,8),(9,9),(8,9),(7,9),(6,9),(6,8),(6,7)],
'h':[(6,10),(7,10),(8,10),(9,10),(9,11),(9,12),(9,13),(8,13),(7,13),(6,13),(6,12),(6,11)],
'p':[(10,2),(11,2),(12,2),(13,2),(13,3),(13,4),(13,5),(12,5),(11,5),(10,5),(10,4),(10,3)],
'a':[(10,6),(11,6),(12,6),(13,6),(13,7),(13,8),(13,9),(12,9),(12,8),(12,7),(12,6),(11,6)],
'j':[(10,10),(11,10),(12,10),(13,10),(13,11),(13,12),(13,13),(12,13),(11,13),(10,13),(10,12),(10,11)]
     }
pkg1=['c','b','k','m','m','m','c','k','k']
pkg2=['c','b','k','m','c','b','m','a','p']
st=[0,0,0,0,0,0]
tc=[(6,3),(6,7),(5,0),(10,0),(3,1),(12,1)]
f=0
mark1_pub=rospy.Publisher("/dot1/marker",Marker,queue_size=1)
mark2_pub=rospy.Publisher("/dot2/marker",Marker,queue_size=1)
mark3_pub=rospy.Publisher("/dot3/marker",Marker,queue_size=1)
mark4_pub=rospy.Publisher("/dot4/marker",Marker,queue_size=1)
mark5_pub=rospy.Publisher("/dot5/marker",Marker,queue_size=1)
mark6_pub=rospy.Publisher("/dot6/marker",Marker,queue_size=1)
    

def mapper(x1,y1):
   xd=round((y1-0.1524/2)/0.1524)
   yd=round((x1-0.1524/2)/0.1524)
   gx6=7-xd
   gy6=8+yd
   return [gx6,gy6]

def imap(x1,y1):
   return (-1.143+y1*0.1524,1.143-x1*0.1524)
 
def mark(m1,m2,m3,m4,m5,m6):
    global st,mark1_pub,mark2_pub,mark3_pub,mark4_pub,mark5_pub,mark6_pub,gc,tc
    marker1=Marker()
    marker1.header.frame_id = "map"
    marker1.header.stamp = rospy.Time.now()
    marker1.ns = "/"
    marker1.id = 0
    marker1.type = Marker.SPHERE
    marker1.action=0
    marker1.scale.x=0.04
    marker1.scale.y=0.04
    marker1.scale.z=0.01
    marker1.color.a = 1.0
    marker1.color.r = 0.0
    marker1.color.g = 1.0
    marker1.color.b = 0.0
    marker1.pose.position.z=0
    marker1.pose.orientation.x = 0.0
    marker1.pose.orientation.y = 0.0
    marker1.pose.orientation.z = 0.0
    marker1.pose.orientation.w = 1.0
    
    marker2=Marker()
    marker2.header.frame_id = "map"
    marker2.header.stamp = rospy.Time.now()
    marker2.ns = "/"
    marker2.id = 0
    marker2.type = Marker.SPHERE
    marker2.action=0
    marker2.scale.x=0.04
    marker2.scale.y=0.04
    marker2.scale.z=0.01
    marker2.color.a = 1.0
    marker2.color.r = 0.0
    marker2.color.g = 0.0
    marker2.color.b = 1.0
    marker2.pose.position.z=0
    marker2.pose.orientation.x = 0.0
    marker2.pose.orientation.y = 0.0
    marker2.pose.orientation.z = 0.0
    marker2.pose.orientation.w = 1.0
    
    marker3=Marker()
    marker3.header.frame_id = "map"
    marker3.header.stamp = rospy.Time.now()
    marker3.ns = "/"
    marker3.id = 0
    marker3.type = Marker.SPHERE
    marker3.action=0
    marker3.scale.x=0.04
    marker3.scale.y=0.04
    marker3.scale.z=0.01
    marker3.color.a = 1.0
    marker3.color.r = 1.0
    marker3.color.g = 0.0
    marker3.color.b = 0.0
    marker3.pose.position.z=0
    marker3.pose.orientation.x = 0.0
    marker3.pose.orientation.y = 0.0
    marker3.pose.orientation.z = 0.0
    marker3.pose.orientation.w = 1.0
    
    marker4=Marker()
    marker4.header.frame_id = "map"
    marker4.header.stamp = rospy.Time.now()
    marker4.ns = "/"
    marker4.id = 0
    marker4.type = Marker.SPHERE
    marker4.action=0
    marker4.scale.x=0.04
    marker4.scale.y=0.04
    marker4.scale.z=0.01
    marker4.color.a = 1.0
    marker4.color.r = 0.0
    marker4.color.g = 1.0
    marker4.color.b = 1.0
    marker4.pose.position.z=0
    marker4.pose.orientation.x = 0.0
    marker4.pose.orientation.y = 0.0
    marker4.pose.orientation.z = 0.0
    marker4.pose.orientation.w = 1.0
    
    marker5=Marker()
    marker5.header.frame_id = "map"
    marker5.header.stamp = rospy.Time.now()
    marker5.ns = "/"
    marker5.id = 0
    marker5.type = Marker.SPHERE
    marker5.action=0
    marker5.scale.x=0.04
    marker5.scale.y=0.04
    marker5.scale.z=0.01
    marker5.color.a = 1.0
    marker5.color.r = 1.0
    marker5.color.g = 0.0
    marker5.color.b = 1.0
    marker5.pose.position.z=0
    marker5.pose.orientation.x = 0.0
    marker5.pose.orientation.y = 0.0
    marker5.pose.orientation.z = 0.0
    marker5.pose.orientation.w = 1.0
    
    marker6=Marker()
    marker6.header.frame_id = "map"
    marker6.header.stamp = rospy.Time.now()
    marker6.ns = "/"
    marker6.id = 0
    marker6.type = Marker.SPHERE
    marker6.action=0
    marker6.scale.x=0.04
    marker6.scale.y=0.04
    marker6.scale.z=0.01
    marker6.color.a = 1.0
    marker6.color.r = 1.0
    marker6.color.g = 1.0
    marker6.color.b = 0.0
    marker6.pose.position.z=0
    marker6.pose.orientation.x = 0.0
    marker6.pose.orientation.y = 0.0
    marker6.pose.orientation.z = 0.0
    marker6.pose.orientation.w = 1.0
    
    st=[0,0,0,0,0,0]
    msgl=[m1,m2,m3,m4,m5,m6]
    markl=[marker1,marker2,marker3,marker4,marker5,marker6]
    mark_publ=[mark1_pub,mark2_pub,mark3_pub,mark4_pub,mark5_pub,mark6_pub]
    lm=[len(m1.poses),len(m2.poses),len(m3.poses),len(m4.poses),len(m5.poses),len(m6.poses)]
    for i in range(6):
       if lm[i]==1:
          lm[i]=500
    print(lm)
    print("least=",min(lm))
    le=min(lm)
    for i in range(1,le+1):
      for j in range(0,6):
         if(i==lm[j] or lm[j]==500):
            st[j]=1
      for j in range(0,6):
        if st[j]==0:
          markl[j].pose.position.x=msgl[j].poses[i].pose.position.x
          markl[j].pose.position.y=msgl[j].poses[i].pose.position.y
          mapped=mapper(msgl[j].poses[i].pose.position.x,msgl[j].poses[i].pose.position.y)
          gc[j]=(mapped[0],mapped[1])
        else:
           imapped=imap(gc[j][0],gc[j][1])
           markl[j].pose.position.x=imapped[0]
           markl[j].pose.position.y=imapped[1]
      for j in range(0,6):
          mark_publ[j].publish(markl[j])
      rospy.sleep(0.5)
    print("st=",st)
    targetpt()

def dis(x1,y1,x2,y2):
      return abs(x2-x1)+abs(y2-y1)
def induct(x,y):
   if (x==5 and y==0) or (x==10 and y==0):
       return True
   else:
     return False
def targetpt():
    global gc,tc,st,ptg,pkg1,pkg2
    ipl=[]
    spl=[]
#    tc=[(1,14),(3,14),(5,14),(7,14),(9,14),(11,14)]
    for i in range(6):
      if st[i]==1:
        if induct(gc[i][0],gc[i][1]):
           spl.append((gc[i],i))
        else:
           ipl.append((gc[i],i))
           
    inductpts=[(5,0),(10,0),(4,1),(11,1),(3,1),(12,1)]
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
       if i[0][0]==5 and i[0][1]==0:
           if pkg1:
              dest=pkg1[0]
              print(pkg1[0])
              destlist=ptg[dest]
              for j in destlist:
                  if tc.count(j)==0:
                     targ=j
                     break
              pkg1.pop(0)
           else:
             targ=(5,0)
           tc[i[1]]=targ
       if i[0][0]==10 and i[0][1]==0:
          if pkg2:
             dest=pkg2[0]
             print(pkg2[0])
             destlist=ptg[dest]
             for j in destlist:
                if tc.count(j)==0:
                   targ=j
                   break
             pkg2.pop(0)
          else:
             targ=(10,0)
          tc[i[1]]=targ
          
    print(tc)
    cbscalc()
   
def cbscalc():
    global gc,tc
    print("Cbs start=",gc)
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
    
    for i in range(0,6):
       print(agents[0]['start'])
       agents[i]['start']=[gc[i][0],gc[i][1]]
       agents[i]['goal']=[tc[i][0],tc[i][1]]
#    print(agents)
    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    msg1=publish_plan(solution,'agent0')
    msg2=publish_plan(solution,'agent1')
    msg3=publish_plan(solution,'agent2')
    msg4=publish_plan(solution,'agent3')
    msg5=publish_plan(solution,'agent4')
    msg6=publish_plan(solution,'agent5')
    path_pub1.publish(msg1)
    path_pub2.publish(msg2)
    path_pub3.publish(msg3)
    path_pub4.publish(msg4)
    path_pub5.publish(msg5)
    path_pub6.publish(msg6)
    mark(msg1,msg2,msg3,msg4,msg5,msg6)
    if not solution:
        print(" Solution not found" )
  
def main():
    global f
#    path_pub = rospy.Publisher('/dot/path', Path, queue_size=1)
#    odom_call=rospy.Subscriber('/dot/odom',Odometry,odom_callback,queue_size=1)  
#    m1=rospy.Subscriber('/dot1/marker',Marker,m1_callback,queue_size=1)
#    m2=rospy.Subscriber('/dot2/marker',Marker,m2_callback,queue_size=1)
#    m3=rospy.Subscriber('/dot3/marker',Marker,m3_callback,queue_size=1)
#    m4=rospy.Subscriber('/dot4/marker',Marker,m4_callback,queue_size=1)
#    m5=rospy.Subscriber('/dot5/marker',Marker,m5_callback,queue_size=1)
#    m6=rospy.Subscriber('/dot6/marker',Marker,m6_callback,queue_size=1)
#    recalc=rospy.Subscriber('/recalc',Int8,targetpt,queue_size=1)
    if f==0:
      targetpt()
      cbscalc()
      f=1

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)
    main()
    rospy.spin()

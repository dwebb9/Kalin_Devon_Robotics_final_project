import kinematics as kin  #this is your kinematics file that you've been developing all along
import numpy as np
from queue import PriorityQueue

#TODO: 
# Find path with square obs
# define and measure metrics for comparing path planners. 

block_size = 0.5
ik_type = "p_inv"
K = 0.4*np.eye(3,3)

# create test evn to verify A_star with

def find_walkable(current, currentq, grid_size, obs_points, arm):
      # print("currentq: ", currentq)
      walk = []

      xmin = -grid_size[0]/2
      xmax = grid_size[0]/2
      ymin = -grid_size[1]/2
      ymax = grid_size[1]/2
      zmax = grid_size[2]/2
      zmin = -grid_size[2]/2

      if current[0] < xmax - 1:
            walk.append([current[0] + 1, current[1], current[2]])
      if current[0] > xmin:
            walk.append([current[0] - 1, current[1], current[2]])
      if current[1] < ymax:
            walk.append([current[0], current[1] + 1, current[2]])
      if current[1] > ymin:
            walk.append([current[0], current[1] - 1, current[2]])
      if current[2] < zmax:
            walk.append([current[0], current[1], current[2] + 1])
      if current[2] > zmin:
            walk.append([current[0], current[1], current[2] - 1])
      out = []
      for i in walk:
            (q, ef, count, flag, message) = arm.ik_position([i[0]*block_size, i[1]*block_size, i[2]*block_size], currentq, ik_type, K=K)
            col = check_collision(q, obs_points, arm)
            if not col:
                  out.append((i,q))
      return out

def get_obs_poinst(obs_list):
      # TODO: may need to adjust to deal with obs in the negative quadrants. 
      obs_pnts = []

      for o in obs_list:
            corner = o[0]
            size = o[1]
            for x in range(0, int(size/block_size)):
                  for y in range(0, int(size/block_size)):
                        for z in range(0, int(size/block_size)):
                              obs_pnts.append([corner[0]/block_size + x, corner[1]/block_size + y, corner[2]/block_size + z])
      return obs_pnts

class Node():
      def __init__(self, loc, q, g, h):
            self.loc = loc
            self.q = q
            self.g = g
            self.h = h
            self.total_cost = g + h
            self.prev = None

      def __lt__(self, other):
            return self.total_cost < other.total_cost

      def __hash__(self):
            return hash((self.loc[0], self.loc[1], self.loc[2]))

      def __eq__(self, other):
            return (self.loc[0], self.loc[1], self.loc[2]) == (other.loc[0], other.loc[1], other.loc[2])

def goal_reach_test(current, goal, buffer=0.75):
      if current[0] <= (goal[0] + buffer) and current[0] >= (goal[0] - buffer) and current[1] <= (goal[1] + buffer) and current[1] >= (goal[1] - buffer) and current[2] <= (goal[2] + buffer) and current[2] >= (goal[2] - buffer):
            return True
      else:
            return False


def check_collision(q, obs_points, arm, test=False):
      joint_locs = []
      # NOTE: CHANGE 0 TO 3 FOR 3D
      for i in range(0, arm.n+1):
            T = arm.fk(q,i)
            joint_locs.append(T[0:3,3])

      tip_T = arm.fk(q, arm.n, tip=True)
      joint_locs.append(tip_T[0:3,3])
      if test: print("joint_locs: \n", joint_locs)

      beam_locs = []
      for i in range(0, len(joint_locs) - 1):
            curr = joint_locs[i]
            next = joint_locs[i+1]

            x_dist = np.abs(curr[0] - next[0])
            y_dist = np.abs(curr[1] - next[1])
            z_dist = np.abs(curr[2] - next[2])

            x = min([curr[0], next[0]]) + x_dist/2.0
            y = min([curr[1], next[1]]) + y_dist/2.0
            z = min([curr[2], next[2]]) + z_dist/2.0

            beam_locs.append([x,y,z])

      for b in beam_locs: joint_locs.append(b)
      
      for j in joint_locs:
            j_grid = [j[0]/block_size, j[1]/block_size, j[2]/block_size]
            if test: print("j_grid: \n", j_grid)
            for o in obs_points:
                  if goal_reach_test(j_grid, o, buffer=0.5):
                        return True
      return False

# EXAMPLE A* CODE:
 #Find best path
# t = 0
# Start_node = Node(agent_start, 0, 0)
# toSearch = PriorityQueue()
# toSearch.put((Start_node.total_cost ,Start_node))

# processed = dict()
# last = Node([0,0],0, 0)

# while not toSearch.empty():
#       temp = toSearch.get()
#       current = temp[1]

#       if current.time == number_steps - 1:
#       last = current
#       break

#       if not contain_test(processed, current):
#       processed[current] = True
#       walkable_points = find_walkable(current, grid_size, grid_obs_points)
#       for i in walkable_points:
#             grid_i = [int(i[0]), int(i[1])]
#             newNode = Node(np.array([i[0]*block_size, i[1]*block_size]), current.time + 1, current.total_cost + grid_cost[grid_i[0]][grid_i[1]][current.time + 1] + heuristic[current.time + 1])
#             newNode.prev = current
#             newNode.current_cost = grid_cost[grid_i[0]][grid_i[1]][current.time + 1]
#             toSearch.put((newNode.total_cost, newNode))
            
#       if t < current.time + 1:
#       t = current.time + 1
#       # print("last time: ", t)
#       toc = time.perf_counter()
#       # print("current path planning time: ", toc - tic)

# final_path = []
# current = last
# sum_cost = 0
# while current is not None:
#       sum_cost += current.current_cost
#       final_path.append(current.loc)
#       current = current.prev


def get_astar_path(q0, obs_list, grid_size, goal, arm):
      #inputs: q0, obs_list(list of obstacles), grid_size, goal
      #ouput: q_list(list of iterative q values)

      obs_pnts = get_obs_poinst(obs_list)
      start_end_pos = arm.fk(q0, arm.n)[0:3,3]
      goal_grid_cell = [goal[0]/block_size, goal[1]/block_size, goal[2]/block_size]
      start_grid_cell = [start_end_pos[0]/block_size, start_end_pos[1]/block_size, start_end_pos[2]/block_size]

      # print("goal grid cell: ", goal_grid_cell)

      start_g = 0.0
      start_h = np.abs(start_grid_cell[0] - goal_grid_cell[0]) + np.abs(start_grid_cell[1] - goal_grid_cell[1]) + np.abs(start_grid_cell[2] - goal_grid_cell[2]) 
      # start_h = np.linalg.norm(np.array(start_grid_cell) - np.array(goal_grid_cell))

      Start_node = Node(start_end_pos, q0, start_g, start_h)
      toSearch = PriorityQueue()
      toSearch.put((Start_node.total_cost ,Start_node))


      goal_node = A_star(toSearch, goal_grid_cell, obs_pnts, arm, grid_size)

      q_list = []
      # print("found path: ")
      dist = goal_node.g
      while goal_node is not None:
            q_list.append(goal_node.q)
            # print("loc: ", goal_node.loc)
            # print("h: ", goal_node.h)
            # print("g: ", goal_node.g)
            # print("total cost: ", goal_node.total_cost)
            # print(goal_node.q)
            # check_collision(goal_node.q, obs_pnts, arm, test=True)
            goal_node = goal_node.prev

      q_list.reverse()

      return q_list, dist


def A_star(toSearch, goal, obs_points, arm, grid_size):
      proccessed = []
      min_h = 100
      # print("obs points: \n", obs_points)

      while not toSearch.empty():
            current = toSearch.get()[1]
            # print("loc: ", current.loc)
            # print("proccessed: ", proccessed) 

            # print(current.loc)
            if current.loc not in proccessed:
                  
                  proccessed.append([current.loc[0], current.loc[1], current.loc[2]])
                  
                  walkable_points = find_walkable(current.loc, current.q, grid_size, obs_points, arm)

                  for w in walkable_points:
                        # h = np.linalg.norm(np.array(goal) - np.array(w[0]))
                        h = np.abs(goal[0] - w[0][0]) + np.abs(goal[1] - w[0][1]) + np.abs(goal[2] - w[0][2]) 
                        # (q, ef, count, flag, message) = arm.ik_position([w[0]*block_size, w[1]*block_size, w[2]*block_size], current.q, ik_type, K=K)
                        q = w[1]
                        newNode = Node(w[0], q, current.g + 1, h)
                        newNode.prev = current
                        toSearch.put((newNode.total_cost, newNode))
                        if h < min_h:
                              min_h = h
                              # print("min h: ", min_h)
                              if goal_reach_test(newNode.loc, goal):
                                    return newNode

                  # if not toSearch.qsize() % 100:
                  #       print(toSearch.qsize())
      
      # for p in proccessed:
      #       if int(p[0]) == 0:
      #             if int(p[1]) == 4:
      #                   print("p: ", p)

      print("A Start failed to find path")
      return current

if __name__ == "__main__":
      from visualization import VizScene # this is the newest visualization file updated on Oct 12
      import time
      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      goal = [-4, 4, 4]
      obst_position = [-5, 2, 0]
      obs2_pos = [-9, 2, 0]
      obs3_pos = [-3,2,0]
      obs4_pos = [-1,2,0]
      obs5_pos = [-1,4,0]
      obs6_pos = [-1,6,0]
      obs7_pos = [-1,7,0]
      obs8_pos = [-7,-4,0]
      obs9_pos = [-7,-6,0]
      obst_rad = 2

      # if you just want to check if you have your code set up correctly, you can uncomment the next three lines and run this file
      # using either vs code or the terminal (and running "python3 midterm_2022.py"). None of the next three lines are needed
      # for your solution, they may just help you check your visualization before you get going. It will just display 100 different
      # random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      q_ik_slns = np.random.uniform(size=(100,3))
      q_ik_slns = q_ik_slns.tolist()

      #2D arm
      # q_0 = [np.pi, 0, 0]
      # dh = [[0, 0, 4, 0],
      #       [0, 0, 4, 0],
      #       [0, 0, 4, 0]]
      # arm = kin.SerialArm(dh)

      # q_0 = [-np.pi/2, 0, -np.pi/2, 0]
      # grid_size = (40,40,10)
      # dh = [[0, 0, 4, 0],
      #       [0, 0, 4, 0],
      #       [0, 0, 4, 0],
      #       [0, 0, 4, 0]]
      # arm = kin.SerialArm(dh)
      # obs_list = [(obst_position, obst_rad), 
      #             (obs2_pos, obst_rad),
      #             (obs3_pos, obst_rad),
      #             (obs4_pos, obst_rad),
      #             (obs5_pos, obst_rad),
      #             (obs6_pos, obst_rad),
      #             (obs7_pos, obst_rad)]

      # print("arm end location", arm.fk([0, 0,0], arm.n)[0:3,3])

      # 3D arm
      # q_0 = [0, 0, 0, 0]
      # goal = [-4, 4, 4]
      # grid_size = (40,40,40)
      # dh = [[np.pi/2.0, 4, 0, np.pi/2.0],
      #       [np.pi/6.0, 0, 0, np.pi/2.0],
      #       [0, 4.031, 0, np.pi/2.0],
      #       [np.pi/6.0, 0, 2, np.pi/2.0]]
      # arm = kin.SerialArm(dh)
      # obst_position = [0,0,0]

      # obs_list = []
      # obs_list = [([-3,3,2], obst_rad)]


      #Kalin's rob
      q_0 = np.array([0,0,0,0,0,0,0])
      grid_size = (15,15,15)
      goal = [1,2,1]
      dh = np.array([[0, 1, 0, np.pi / 2],
                        [np.pi / 2, 0, 1, -np.pi / 2],
                        [np.pi / 2, 1, 0, np.pi / 2],
                        [0, 0, 0, np.pi / 2],
                        [0, -1, 0, -np.pi / 2],
                        [np.pi / 2, 0, 0, np.pi / 2],
                        [0, 0, 1, 0]])
      
      arm = kin.SerialArm(dh)
            
      obs1_rad = 0.75 * 2
      obs1_pos = [0.25, 1.25 ,1.25]

      obs2_pos = [-1.5, 0.5, 0.75]
      obs2_rad = 0.5 * 2

      obs3_pos = [-1.5, 0.5, 2.25]
      obs3_rad = 0.5 * 2

      obs4_pos = [-1.5, 1, 0.5]
      obs4_rad = 0.5 * 2

      obs5_pos = [-3.5, 0.25, 2]
      obs5_rad = 0.5 * 2

      obs6_pos = [-3.5, 0.25, 1.5]
      obs6_rad = 0.5 * 2

      obs7_pos = [-3.5, 0.25, 1]
      obs7_rad = 0.5 * 2

      obs_list = [(obs1_pos, obs1_rad), 
                  (obs2_pos, obs2_rad),
                  (obs3_pos, obs3_rad),
                  (obs4_pos, obs4_rad),
                  (obs5_pos, obs5_rad),
                  (obs6_pos, obs6_rad),
                  (obs7_pos, obs7_rad)]

      q_ik_slns, dist = get_astar_path(q_0, obs_list, grid_size, goal, arm)

      print("arm end location", arm.fk(q_0, arm.n)[0:3,3])

      path_points = []
      
      for q in q_ik_slns:
            path_points.append( arm.fk(q, arm.n)[0:3,3])

      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()

      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, size=20)

      red = [0.7, 0, 0, 1]
      for p in path_points:
            viz.add_marker(p, color=red, size = 10)
      for o in obs_list:
            viz.add_obstacle(o[0], rad=o[1], square=True)
      # viz.add_obstacle(obs2_pos, rad=obst_rad, square=True)

      # viz.update(qs=[np.pi, -np.pi/2, -np.pi/2])
      viz.update(qs=[q_0])

      # viz.hold()
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.5)
            # time.sleep(0.05)
      viz.hold()
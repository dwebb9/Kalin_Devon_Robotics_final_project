import kinematics as kin  #this is your kinematics file that you've been developing all along
import numpy as np
from queue import PriorityQueue

#TODO: 
# Find path with square obs
# define and measure metrics for comparing path planners. 
# create test evn to verify A_star with

def find_walkable(current, grid_size, obs_points):
      pnts = []
      return pnts

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

def check_collision(q, obs_loc, obs_rad, arm):
      joint_locs = []
      for i in range(3, arm.n+1):
            T = arm.fk(q,i)
            joint_locs.append(T[0:3,3])

      center = np.array(obs_loc)
      for j in joint_locs:
            # print("test location: ", j)
            # print("dist: ", np.linalg.norm(j - center))
            if np.linalg.norm(j - center) <= obs_rad:
                  # print("broke on : ", j)
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

      start_end_pos = arm.fk(q0, arm.n)[0:3,3]
      start_g = 0.0
      start_h = np.linalg.norm(start_end_pos - goal)

      Start_node = Node(start_end_pos, q0, start_g, start_h)
      toSearch = PriorityQueue()
      toSearch.put((Start_node.total_cost ,Start_node))

      #ouput: q_list(list of iterative q values)
      q_list = []
      # strat: similar to RRT solver. Find min path of tip of robot through environment, checking for colisions at each step. 


      return q_list

if __name__ == "__main__":
      from visualization import VizScene # this is the newest visualization file updated on Oct 12
      import time
      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      goal = [0, 4, 0]
      obst_position = [0, 1, 0]
      obst_rad = 2.5

      # if you just want to check if you have your code set up correctly, you can uncomment the next three lines and run this file
      # using either vs code or the terminal (and running "python3 midterm_2022.py"). None of the next three lines are needed
      # for your solution, they may just help you check your visualization before you get going. It will just display 100 different
      # random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      q_ik_slns = np.random.uniform(size=(100,3))
      q_ik_slns = q_ik_slns.tolist()

      #2D arm
      q_0 = [0, 0, 0]
      dh = [[0, 0, 4, 0],
            [0, 0, 4, 0],
            [0, 0, 4, 0]]
      arm = kin.SerialArm(dh)

      print("arm end location", arm.fk([0, np.pi/2, np.pi/2], arm.n)[0:3,3])

      #3D arm
      # q_0 = [0, 0, 0, 0]
      # dh = [[np.pi/2.0, 4, 0, np.pi/2.0],
      #       [np.pi/6.0, 0, 0, np.pi/2.0],
      #       [0, 4.031, 0, np.pi/2.0],
      #       [np.pi/6.0, 0, 2, np.pi/2.0]]
      # arm = kin.SerialArm(dh)

      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()

      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, size=20)
      viz.add_obstacle(obst_position, rad=obst_rad, square=True)

      viz.update(qs=[0, np.pi/2, np.pi/2])

      # viz.hold()
      # for q in q_ik_slns:
      #       viz.update(qs=[q])

      #       # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
      #       time.sleep(0.1)
      #       time.sleep(0.05)
      viz.hold()
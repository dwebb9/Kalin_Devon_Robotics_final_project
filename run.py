# %% [markdown]
# # Midterm 2022
# * Copy this file to your homework workspace to have access to your other kinematic functions
# * Make sure to also copy a new "visualization.py" file to replace the old file (this new file can plot obstacles to scale)

# %%
# To test your setup, after defining the robot arm as described below, (but nothing else)
# you can run this file directly to make sure it is plotting the arm, obstacle, and goal 
# as expected. 

import kinematics as kin  #this is your kinematics file that you've been developing all along
from visualization import VizScene # this is the newest visualization file updated on Oct 12
import time
import numpy as np


# Define your kinematics and an "arm" variable here using DH parameters so they
# are global variables that are available in your function below:

dh = [[np.pi/2.0, 4, 0, np.pi/2.0],
      [np.pi/6.0, 0, 0, np.pi/2.0],
      [0, 4.031, 0, np.pi/2.0],
      [np.pi/6.0, 0, 2, np.pi/2.0]]
arm = kin.SerialArm(dh)


# let's also plot robot to make sure it matches what we think it should
# (this will look mostly like the pictures on part 1 if your DH parameters
# are correct
# viz_check = VizScene()
# viz_check.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
# viz_check.update(qs = [[0, 0, 0, 0]])
# viz_check.hold()

class Node():
      def __init__(self, loc, q):
            self.loc = loc
            self.q = q
            self.prev = None

def check_collision(q, obs_loc, obs_rad, testing=False):
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

def get_random_point():
      # defining workspace as a sphere about (0,0,4) with radius 6
      theta = np.random.random_sample()*2*np.pi
      phi = np.random.random_sample()*np.pi
      r = np.random.random_sample()*6
      x = r * np.sin(phi) * np.cos(theta)
      y = r * np.sin(phi) * np.sin(theta)
      z = r * np.cos(phi) + 4
      return [x,y,z]

def find_closest_point(p, tree):
      # print("p: ", p)
      # print("tree: \n")
      # for i in tree:
      #       print(i.loc)

      dist = 1000
      closest = tree[0]
      for i in tree:
            newdist = np.linalg.norm(np.array(p) - np.array(i.loc))
            if newdist < dist:
                  closest = i
                  dist = newdist
      # print('closest: ', closest.loc)
      return closest

def compute_robot_path(q_init, goal, obst_location, obst_radius):
      # this can be similar to your IK solution for HW 6, but will require modifications
      # to make sure you avoid the obstacle as well.

      #IK parameters
      K = 0.4*np.eye(3,3)
      ik_type = "p_inv"

      # this is an empty list that we'll use to return our solutions so they can be plotted
      q_s = []
      searching = True
      rrt_tree = []
      starting_loc_T = arm.fk(q_init, arm.n)
      starting_loc = starting_loc_T[0:3,3]
      # starting_loc = np.array([0,0,4])

      print("starting loc: ", starting_loc)
      print("starting q: ", q_init)
      rrt_tree.append(Node(starting_loc, q_init))
      goalNode = Node(starting_loc, q_init)
      segment_length = 0.4
      last_jump = 0.6 #segment_length * 2
      prog_dist = 10000
      test_itt = 0
      # check_collision(q_init, testing=True)
      while searching: # set your st opping criteria here
            p = get_random_point()
            # p = np.array([0,2,4])
            test_itt += 1
            v_star = find_closest_point(p, rrt_tree) #closes point in tree
            v_plus = (p - v_star.loc)*segment_length/np.linalg.norm(np.array(v_star.loc) - np.array(p)) + v_star.loc
            (v_plus_q, ef, count, flag, message)  = arm.ik_position(v_plus, v_star.q, ik_type, K=K)
            # print("\np: ", p)
            # print("v_star: ", v_star.loc)
            # print("v_plus: ", v_plus)
            if not check_collision(v_plus_q, obst_location, obst_rad):
                  # print("good points")
                  newNode = Node(v_plus, v_plus_q)
                  newNode.prev = v_star
                  rrt_tree.append(newNode)
                  # print("tree length: ", len(rrt_tree))
                  dist_to_goal = np.linalg.norm(np.array(v_plus) - np.array(goal))
                  if prog_dist > dist_to_goal:
                        print("\nnew dist to goal: ", dist_to_goal)
                        print("at : ", v_plus)
                        prog_dist = dist_to_goal
                  if dist_to_goal < last_jump:
                        (goal_q, ef, count, flag, message)  = arm.ik_position(goal, newNode.q, ik_type, K=K)
                        goalNode = Node(goal, goal_q)
                        goalNode.prev = newNode
                        rrt_tree.append(newNode)
                        searching = False
      
      current = goalNode
      print("path: ")
      while current is not None:
            q_s.append(current.q)
            print(current.loc)
            current = current.prev
      
      q_s.reverse()
      return q_s

if __name__ == "__main__":

      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      q_0 = [0, 0, 0, 0]
      goal = [0, 2, 4]
      obst_position = [0, 3, 2]
      obst_rad = 1.0

      q_ik_slns = compute_robot_path(q_0, goal, obst_position, obst_rad)


      # if you just want to check if you have your code set up correctly, you can uncomment the next three lines and run this file
      # using either vs code or the terminal (and running "python3 midterm_2022.py"). None of the next three lines are needed
      # for your solution, they may just help you check your visualization before you get going. It will just display 100 different
      # random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      # q_ik_slns = np.random.uniform(size=(100,4))
      # q_ik_slns = q_ik_slns.tolist()




      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()
      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, size=20)
      viz.add_obstacle(obst_position, rad=obst_rad)
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.1)
            # time.sleep(0.05)
      # viz.hold()

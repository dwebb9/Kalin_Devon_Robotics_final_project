import kinematics as kin  #this is your kinematics file that you've been developing all along
import numpy as np

#TODO: 
# Find path with square obs
# define and measure metrics for comparing path planners. 
# create test evn to verify A_star with

class Node():
      def __init__(self, loc, q):
            self.loc = loc
            self.q = q
            self.prev = None

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

def get_astar_path():
      #inputs: q0, obs_list(list of obstacles), grid_size, end_point

      #ouput: q_list(list of iterative q values)
      q_list = []
      # strat: similar to RRT solver. Find min path of tip of robot through environment, checking for colisions at each step. 


      return q_list

if __name__ == "__main__":
      from visualization import VizScene # this is the newest visualization file updated on Oct 12
      import time
      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      q_0 = [0, 0, 0, 0]
      goal = [0, 2, 4]
      obst_position = [0, 3, 2]
      obst_rad = 1.0

      # if you just want to check if you have your code set up correctly, you can uncomment the next three lines and run this file
      # using either vs code or the terminal (and running "python3 midterm_2022.py"). None of the next three lines are needed
      # for your solution, they may just help you check your visualization before you get going. It will just display 100 different
      # random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      q_ik_slns = np.random.uniform(size=(100,3))
      q_ik_slns = q_ik_slns.tolist()

      #2D arm
      dh = [[0, 0, 4, 0],
            [0, 0, 4, 0],
            [0, 0, 4, 0]]
      arm = kin.SerialArm(dh)

      #3D arm
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
      viz.add_obstacle(obst_position, rad=obst_rad)

      # viz.hold()
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.1)
            time.sleep(0.05)
      viz.hold()
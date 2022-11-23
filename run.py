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

if __name__ == "__main__":

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
      q_ik_slns = np.random.uniform(size=(100,4))
      q_ik_slns = q_ik_slns.tolist()




      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()
      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, size=20)
      viz.add_obstacle(obst_position, rad=obst_rad, square=True)
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.1)
            # time.sleep(0.05)
      # viz.hold()

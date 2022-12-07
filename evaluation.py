import numpy as np
import time

import kinematics as kin
from potential_fields import ObstFields
from potential_fields import run as run_pf


def eval_2d(num_trials):
    # Initialize arrays to store results
    times_pf = np.zeros(num_trials)
    dists_pf = np.zeros(num_trials)
    times_as = np.zeros(num_trials)
    dists_as = np.zeros(num_trials)
    # Create arm, obstacles, and target
    dh = np.array([[0, 0, 1, 0],
                   [0, 0, 1, 0],
                   [0, 0, 1, 0]])
    arm = kin.SerialArm(dh)
    obst = ObstFields(np.array([[2, 2, 0.5], [0, 2, 0.5]]), 0.01, 0.2)
    target = np.array([0, 2.5])
    # Run trials
    for i in range(num_trials):
        # Run potential fields
        start = time.time()
        qs, dist = run_pf(arm, obst, target)
        end = time.time()
        times_pf[i] = end - start
        dists_pf[i] = dist
        # Run A*
        # TODO - Add in A* code
        
    # Print mean values with 5 decimal places
    print(f'PF 2D: {np.mean(times_pf):.5f} s, {np.mean(dists_pf):.5f} m')
    print(f'AS 2D: {np.mean(times_as):.5f} s, {np.mean(dists_as):.5f} m')
    print() 

def eval_3d(num_trials):
    # Initialize arrays to store results
    times_pf = np.zeros(num_trials)
    dists_pf = np.zeros(num_trials)
    times_as = np.zeros(num_trials)
    dists_as = np.zeros(num_trials)
    # Create arm, obstacles, and target
    dh = np.array([[0, 1, 0, np.pi / 2],
                   [np.pi / 2, 0, 1, -np.pi / 2],
                   [np.pi / 2, 1, 0, np.pi / 2],
                   [0, 0, 0, np.pi / 2],
                   [0, -1, 0, -np.pi / 2],
                   [np.pi / 2, 0, 0, np.pi / 2],
                   [0, 0, 1, 0]])
    arm = kin.SerialArm(dh)
    obst = ObstFields(np.array([[1, -2, 2, 0.75], 
                                [-1, -1, 1.25, 0.5], 
                                [-1, -1, 2.75, 0.5], 
                                [-1, -1.5, 1, 0.5], 
                                [-3, -0.75, 2.5, 0.5], 
                                [-3, -0.75, 2, 0.5], 
                                [-3, -0.75, 1.5, 0.5]]), 0.01, 0.2)
    target = np.array([1, -2, 1])
    # Run trials
    for i in range(num_trials):
        # Run potential fields
        start = time.time()
        _, dist = run_pf(arm, obst, target)
        times_pf[i] = time.time() - start
        dists_pf[i] = dist.copy()
        # Run A*
        # TODO - Add in A* code
    
    # Print mean values with 5 decimal places
    print(f'PF 3D: {np.mean(times_pf):.5f} s, {np.mean(dists_pf):.5f} m')
    print(f'AS 3D: {np.mean(times_as):.5f} s, {np.mean(dists_as):.5f} m')
    print() 
    
if __name__ == "__main__":
    num_trials = 10
    print(f'Testing 2D with {num_trials} trials')
    eval_2d(num_trials)
    print(f'Testing 3D with {num_trials} trials')
    eval_3d(num_trials)
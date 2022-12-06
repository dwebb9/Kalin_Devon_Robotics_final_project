import numpy as np
import kinematics as kin

class ObstFields:
    def __init__(self, obstacles, eta, rho0) -> None:
        '''Class initialization
        
        Parameters:
        obstacles   - nx3 or nx4 array : Obstacles, where n is the number of obstacles, and the columns correspond to x and y (and optionally z) coordinates of the center, and the radius
        eta         - float     : Multiplier in potential field calculation for obstacles
        rho0        - float     : Distance threshold from obstacle before potential field begins to take effect
        '''
        if len(obstacles.shape) < 2:
            raise ValueError("'obstacles' must be a two-dimensional array, even if only one obstacle is present")
        if obstacles.shape[1] == 3:
            # If no z coordinates are provided, add a column of zeros
            self.obstacles = np.hstack((obstacles[:, 0][None].T, obstacles[:, 1][None].T, np.zeros((obstacles.shape[0], 1)), obstacles[:, 2][None].T))
            if len(self.obstacles.shape) < 2:
                self.obstacles = np.reshape(self.obstacles, (1, 4))
                print(self.obstacles.shape)
        elif obstacles.shape[1] == 4: 
            self.obstacles = obstacles
        else:
            raise ValueError("'obstacles' must have 3 or 4 columns, corresponding to x, y, z, and radius, with z being optional")
        self.num_obstacles = obstacles.shape[0]
        self.eta = eta
        self.rho0 = rho0
        
    def _dist(self, x1, y1, z1, x2, y2, z2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    
    def _dist_and_deriv(self, x1, y1, z1, x2, y2, z2):
        dist = self._dist(x1, y1, z1, x2, y2, z2)
        dx1 = -(x2 - x1) / dist
        dy1 = -(y2 - y1) / dist
        dz1 = -(z2 - z1) / dist
        return dist, np.array([dx1, dy1, dz1])
        
        
    def _f_psp(self, pt, obst):
        # Extract coordinates of the point
        xp, yp, zp = pt
        # Extract obstacle location and radius
        xo, yo, zo, ro = obst
        # Calculate function output, based on eq. 18 in the paper
        dist, dist_deriv = self._dist_and_deriv(xp, yp, zp, xo, yo, zo)
        rho = dist - ro
        if rho > self.rho0:
            return np.zeros(3)
        else:
            return self.eta * (1 / rho - 1 / self.rho0) * 1 / rho**2 * dist_deriv
        
    def _find_closest_point_to_obst(self, pt1, pt2, obst):
        # Find the vector from pt1 to pt2
        v = pt2 - pt1
        # Find the vector from pt1 to the obstacle
        w = obst[:3] - pt1
        # Find the projection of w onto v
        proj = np.dot(w, v) / np.dot(v, v)
        if np.any(np.isnan(proj)):
            proj = 0
        # If the projection is less than 0, the closest point is pt1
        if proj < 0:
            return pt1
        # If the projection is greater than 1, the closest point is pt2
        elif proj > 1:
            return pt2
        # Otherwise, the closest point is on the line segment
        else:
            return pt1 + proj * v
        
    def get_obst_forces(self, arm: kin.SerialArm, q):
        obst_force = np.zeros(3)
        # Iterate through each obstacle
        for i in range(self.num_obstacles):
            link_start = np.zeros(3)
            # Iterate through each link in robotic arm
            for j in range(arm.n):
                link_end = arm.fk(q, j+1)[:3, 3]
                # Find closest point on link to the obstacle
                cp = self._find_closest_point_to_obst(link_start, link_end, self.obstacles[i])
                # Calculate the potential field at the closest point
                of = self._f_psp(cp, self.obstacles[i])
                # Add the potential field to the total obstacle force
                obst_force += of
                # Update the link start point in preparation for the next link calculation
                link_start = link_end.copy()
        return obst_force
    
def target_pot_field(kp, x, xd, kv, xdot):
    return -kp * (x - xd) - kv * xdot

def run(arm: kin.SerialArm, obst: ObstFields, target, max_iter=1000):
    if len(target) == 2:
        # Ensure that the target is a 3D vector (assume x and y if only two values are provided, and set z = 0)
        target = np.hstack((target, np.zeros(1)))
    q = np.zeros(arm.n)
    qs = [q]
    cur_pos = arm.fk(q)[:3, 3]
    prev_pos = cur_pos.copy()
    init_dist = obst._dist(cur_pos[0], cur_pos[1], cur_pos[2], target[0], target[1], target[2])
    count = 0
    total_dist = 0
    while not np.isclose(cur_pos, target, rtol=1e-1).all():
        # Calculate the target potential field
        pos_dot = cur_pos - prev_pos
        cur_dist = obst._dist(cur_pos[0], cur_pos[1], cur_pos[2], target[0], target[1], target[2])
        kp = max(0.2, 1 - cur_dist / init_dist)
        target_force = target_pot_field(kp, cur_pos, target, 1, pos_dot)
        # Calculate the obstacle potential field
        obst_force = obst.get_obst_forces(arm, q)
        # Calculate the total force
        total_force = target_force + obst_force
        # Calculate the joint velocities
        qdot = arm.jacob(q)[:3, :].T @ total_force
        # Calculate the new joint angles
        q = q + qdot * 0.01
        qs.append(q)
        # Calculate the new end effector position
        prev_pos = cur_pos.copy()
        cur_pos = arm.fk(q)[:3, 3]
        # Add on the distance traveled
        total_dist += obst._dist(*cur_pos, *prev_pos)
        count += 1
        if max_iter is not None and count >= max_iter:
            break
    # Return list of joint angles, and the total distance travelled
    return np.array(qs), total_dist
    
if __name__ == "__main__":
    import visualization as vis
    import time
    
    # # 2D robot arm (only moves in the XY plane)
    # dh = np.array([[0, 0, 1, 0],
    #             #    [0, 0, 1, 0],
    #             #    [0, 0, 1, 0],
    #             #    [0, 0, 1, 0],
    #                [0, 0, 1, 0],
    #                [0, 0, 1, 0]])
    
    # 3D robot arm
    dh = np.array([[0, 1, 0, np.pi / 2],
                   [np.pi / 2, 0, 1, -np.pi / 2],
                   [np.pi / 2, 1, 0, np.pi / 2],
                   [0, 0, 0, np.pi / 2],
                   [0, -1, 0, -np.pi / 2],
                   [np.pi / 2, 0, 0, np.pi / 2],
                   [0, 0, 1, 0]])
    
    arm = kin.SerialArm(dh)
    
    # vis.ArmPlayer(arm)
    # exit()
    
    # obst = ObstFields2D(np.array([[2, 2, 0.5], [0, 2, 0.5]]), 0.01, 0.2)
    obst = ObstFields(np.array([[1, -2, 2, 0.75], [-1, -1, 1.25, 0.5], [-1, -1, 2.75, 0.5], [-1, -1.5, 1, 0.5], [-3, -0.75, 2.5, 0.5], [-3, -0.75, 2, 0.5], [-3, -0.75, 1.5, 0.5]]), 0.01, 0.2)
    
    # qs = run(arm, obst, np.array([0, 2.5]))
    qs, end_dist = run(arm, obst, np.array([1, -2, 1]))
    print(end_dist)
    print(len(qs))
    
    v = vis.VizScene()
    v.window.setCameraPosition(distance=4, elevation=10, azimuth=-90)
    v.add_arm(arm)
    for i in range(obst.num_obstacles):
        obst_pos = np.zeros(3)
        obst_pos = obst.obstacles[i, :3]
        obst_rad = obst.obstacles[i, 3]
        v.add_obstacle(obst_pos, rad=obst_rad)
            
    for q in qs:
        v.update(q)
        time.sleep(1 / 60)
    v.close_viz()
        
    
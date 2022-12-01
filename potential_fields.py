import numpy as np

class ObstFields2D:
    def __init__(self, obstacles, eta, rho0) -> None:
        '''Class initialization
        
        Parameters:
        obstacles   - nx3 array : Obstacles, where n is the number of obstacles, and the columns correspond to x and y coordinates of the center, and the radius
        eta         - float     : Multiplier in potential field calculation for obstacles
        rho0        - float     : Distance threshold from obstacle before potential field begins to take effect
        '''
        if len(obstacles.shape < 1):
            raise ValueError("'obstacles' must be a two-dimensional array, even if only one obstacles is present")
        if obstacles.shape[1] != 3:
            raise ValueError("'obstacles' must be an nx3 array")
        self.num_obstacles = obstacles.shape[0]
        self.obstacles = obstacles
        self.eta = eta
        self.rho0 = rho0
        
    def _dist(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def _dist_and_deriv(self, x1, y1, x2, y2):
        dist = self._dist(x1, y1, x2, y2)
        dx1 = -(x2 - x1) / dist
        dy1 = -(y2 - y1) / dist
        return dist, np.array([dx1, dy1])
        
        
    def _f_psp(self, pt, obst):
        # Extract coordinates of the point
        xp, yp = pt
        # Extract obstacle location and radius
        xo, yo, ro = obst
        # Calculate function output, based on eq. 18 in the paper
        dist, dist_deriv = self._dist_and_deriv(xp, yp, xo, yo)
        rho = dist - ro
        if rho > self.rho0:
            return np.zeros(2)
        else:
            return self.eta * (1 / rho - 1 / self.rho0) * 1 / rho**2 * dist_deriv
        
    def _find_closest_point_to_obst(self, pt1, pt2, obst):
        # Find the vector from pt1 to pt2
        v = pt2 - pt1
        # Find the vector from pt1 to the obstacle
        w = obst[:2] - pt1
        # Find the projection of w onto v
        proj = np.dot(w, v) / np.dot(v, v)
        # If the projection is less than 0, the closest point is pt1
        if proj < 0:
            return pt1
        # If the projection is greater than 1, the closest point is pt2
        elif proj > 1:
            return pt2
        # Otherwise, the closest point is on the line segment
        else:
            return pt1 + proj * v
        
    def get_obst_forces()
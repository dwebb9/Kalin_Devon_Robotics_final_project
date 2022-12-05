"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022
"""

from transforms import *

eye = np.eye(4)
pi = np.pi


class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts one link of dh parameters and returns a function "f" that will generate a
    homogeneous transform "A" given "q" as an input. A represents the transform from 
    link i to link i+1

    Parameters:
    dh - 1 x 4 list or iterable of floats, dh parameter table for one transform from link i to link i+1,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the transform from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            def A(q):
                theta = dh[0] + q
                d = dh[1]
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                return np.array([[cos(theta), -sin(theta)*cos(alpha), sin(alpha)*sin(theta), a*cos(theta)],
                                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha) , a*sin(theta)],
                                 [0, sin(alpha), cos(alpha), d],
                                 [0, 0, 0, 1]])


        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                theta = dh[0]
                d = dh[1] + q
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                return np.array([[cos(theta), -sin(theta)*cos(alpha), sin(alpha)*sin(theta), a*cos(theta)],
                                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha) , a*sin(theta)],
                                 [0, sin(alpha), cos(alpha), d],
                                 [0, 0, 0, 1]])

        self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """


    def __init__(self, dh, jt=None, base=eye, tip=eye, joint_limits=None):
        """
        arm = SerialArm(dh, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list or iterable of length 4 list or iterables representing dh parameters, [d theta a alpha]
        :param jt: n length list or iterable of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy or sympy array representing SE3 transform from world frame to frame 0
        :param tip: 4x4 numpy or sympy array representing SE3 transform from frame n to tool frame
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        self.dh = dh
        self.n = len(dh)

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][0]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[0], dh[2]]))

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # generating the function A(q) for each set of DH parameters
        for i in range(self.n):
            # TODO use the class definition above (dh2AFunc), and the dh parameters and joint type to
            # make a function and then append that function to the "transforms" list. 
            f = dh2AFunc(self.dh[i], jt=self.jt[i])
            self.transforms.append(f.A)

        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits

    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string


    def fk(self, q, index=None, base=False, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index - integer or list of two integers. If a list of two integers, the first integer represents the starting JOINT 
                    (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                    the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        # the following lines of code are data type and error checking. You don't need to understand
        # all of it, but it is helpful to keep. 

        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None

        # TODO complete each of the different cases below. If you don't like the 
        # current setup (in terms of if/else statements) you can do your own thing.
        # But the functionality should be the same. 
        if base and start_frame == 0:
            T = self.base
        else:
            T = np.eye(4)
        for i in range(start_frame, end_frame):
            T = T@self.transforms[i](q[i])

        if tip and end_frame == self.n:
            T = T@self.tip

        return T


    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the end effector frame of the arm in a given configuration

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """

        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        # TODO - start by declaring the correct size for the Jacobian
        J = np.zeros((6, self.n))
        # J = np.zeros((self.n,6))

        # TODO - find the current position of the gripper "pe" using your fk function
        # this will likely require additional intermediate variables than what is shown here. 
        Te = self.fk(q, (0,self.n), base, tip)
        # print("Te: \n", Te)
        pe = Te[0:3,3]
        # print("pe: \n", pe)

        # TODO - calculate all the necessary values using your "fk" function, and fill every column
        # of the jacobian using this "for" loop. Functions like "np.cross" may also be useful. 
        for i in range(index):
            Ti = self.fk(q, (0,i), base, tip)
            Zi = Ti[0:3,0:3]@np.array([0,0,1])
            # print("Ti for ", i)
            # print(Ti)
            # check if joint is revolute
            if self.jt[i] == 'r':
                pi = Ti[0:3,3]
                J_v = np.cross(Zi, (pe-pi))
                J[:,i] = np.array([J_v[0], J_v[1], J_v[2] ,Zi[0], Zi[1], Zi[2]])
            # if not assume joint is prismatic
            else:
                J[:,i] = np.array([Zi[0], Zi[1], Zi[2], 0,0,0])
            
        # print(J)
        return J

    # insert this function into your SerialArm class and complete it. 
    # Please keep the function definition, and what it returns the same. 
    def ik_position(self, target, q0=None, method='J_T', force=True, tol=1e-4, K=None, kd=0.001, max_iter=100):
        """
        (qf, ef, iter, reached_max_iter, status_msg) = arm.ik2(target, q0=None, method='jt', force=False, tol=1e-6, K=None)
        Description:
            Returns a solution to the inverse kinematics problem finding
            joint angles corresponding to the position (x y z coords) of target

        Args:
            target: 3x1 numpy array that defines the target location. 

            q0: length of initial joint coordinates, defaults to q=0 (which is
            often a singularity - other starting positions are recommended)

            method: String describing which IK algorithm to use. Options include:
                - 'pinv': damped pseudo-inverse solution, qdot = J_dag * e * dt, where
                J_dag = J.T * (J * J.T + kd**2)^-1
                - 'J_T': jacobian transpose method, qdot = J.T * K * e

            force: Boolean, if True will attempt to solve even if a naive reach check
            determines the target to be outside the reach of the arm

            tol: float, tolerance in the norm of the error in pose used as termination criteria for while loop

            K: 3x3 numpy matrix. For both pinv and J_T, K is the positive definite gain matrix used for both. 

            kd: is a scalar used in the pinv method to make sure the matrix is invertible. 

            max_iter: maximum attempts before giving up.

        Returns:
            qf: 6x1 numpy matrix of final joint values. If IK fails to converge the last set
            of joint angles is still returned

            ef: 3x1 numpy vector of the final error

            count: int, number of iterations

            flag: bool, "true" indicates successful IK solution and "false" unsuccessful

            status_msg: A string that may be useful to understanding why it failed. 
        """
        # Fill in q if none given, and convert to numpy array 
        # print("q0: ", q0)
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        # Try basic check for if the target is in the workspace.
        # Maximum length of the arm is sum(sqrt(d_i^2 + a_i^2)), distance to target is norm(A_t)
        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)

        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"
        # else:
        #     if target_distance > maximum_reach:
        #         print("Target out of workspace, but finding closest solution anyway")
        #     else:
        #         print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
        #             float(target_distance), float(maximum_reach)))

        if not isinstance(K, np.ndarray):
            return q, error, count, False,  "No gain matrix 'K' provided"

        # you may want to define some functions here to help with operations that you will 
        # perform repeatedly in the while loop below. Alternatively, you can also just define 
        # them as class functions and use them as self.<function_name>.

        # for example:
        def get_error(q):
            # print("q: ", q)
            T = self.fk(q, [0,self.n], tip=True)
            # print("T: ", T)
            cur_position = T[0:3,3]
            e = target - cur_position
            return e

        error = get_error(q)
        time_step = 2

        while np.linalg.norm(error) > tol and count < max_iter:
        
        # In this while loop you will update q for each iteration, and update, then
        # your error to see if the problem has converged. You may want to print the error
        # or the "count" at each iteration to help you see the progress as you debug. 
        # You may even want to plot an arm initially for each iteration to make sure 
        # it's moving in the right direction towards the target.
            J_full = self.jacob(q, self.n, tip=True)
            J = J_full[0:3, 0:self.n]
            if  method == "p_inv":
                J_cross = J.T @ np.linalg.inv(J@J.T + kd*np.eye(3,3))
                q_dot = J_cross @ K @ error
            else:
                q_dot = J.T @ K @ error

            q = q + q_dot*time_step
            error = get_error(q)    
            count += 1
        # when "while" loop is done, return the relevant info. 

        return (q, error, count, count < max_iter, 'No errors noted')

if __name__ == "__main__":
    from visualization import VizScene
    import time

    # Defining a table of DH parameters where each row corresponds to another joint.
    # The order of the DH parameters is [theta, d, a, alpha] - which is the order of operations. 
    # The symbolic joint variables "q" do not have to be explicitly defined here. 
    # This is a two link, planar robot arm with two revolute joints. 
    dh = [[0, 0, 0.3, 0],
          [0, 0, 0.3, 0]]

    # make robot arm (assuming all joints are revolute)
    arm = SerialArm(dh)

    # defining joint configuration
    q = [pi/4.0, pi/4.0]  # 45 degrees and 45 degrees

    # show an example of calculating the entire forward kinematics
    Tn_in_0 = arm.fk(q)
    print("Tn_in_0:\n", Tn_in_0, "\n")
    #show_frame('0', '2', Tn_to_0) # this will only work if all of values are numeric

    # show an example of calculating the kinematics between frames 0 and 1
    T1_in_0 = arm.fk(q, index=[0,1])
    print("T1_in 0:\n", T1_in_0, "\n")
    #show_frame('0', '1', T1_to_0)

    print(arm)

    viz = VizScene()

    viz.add_frame(arm.base, label='base')
    viz.add_frame(Tn_in_0, label="Tn_in_0")
    viz.add_frame(T1_in_0, label="T1_in_0")

    time_to_run = 30
    refresh_rate = 60

    for i in range(refresh_rate * time_to_run):
        viz.update()
        time.sleep(1.0/refresh_rate)
    
    viz.close_viz()
    
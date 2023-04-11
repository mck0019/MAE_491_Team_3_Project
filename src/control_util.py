# control_util.py
# Last Updated: April 10th, 2023
# Authors: Michael Key, Ian Holbrook, John Thorne

# imports
import math

# defines
CONTROLLER_TIME_STEP = 0.1 # [s]

# remaps a value from one range to a new range
def remap(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

# convertes degrees to radians
def deg_to_rad(degrees):
    return degrees * (math.pi / 180.0)

# converts radians to degrees
def rad_to_deg(radians):
    return radians * (180.0 / math.pi)

# matrix class
class matrix:
    
    # matrix constructor
    def __init__(self, data):
        # handle empty data
        if not data:
            self.rows = 0
            self.cols = 0
            self.data = []
        # handle nested list input
        elif isinstance(data, list) and all(isinstance(row, list) for row in data):
            self.rows = len(data)
            self.cols = len(data[0])
            self.data = data
        # handle single-row list input
        elif isinstance(data, list):
            self.rows = 1
            self.cols = len(data)
            self.data = [data]
        # handle single-element input
        elif isinstance(data, (int, float)):
            self.rows = 1
            self.cols = 1
            self.data = [[data]]
        else:
            raise ValueError("Invalid input data")
        
    # create an empty matrix
    def zeros(rows, cols):
        data = [[0] * cols for _ in range(rows)]
        return matrix(data)
    
    # get an element of the matrix
    def __getitem__(self, index):
        return self.data[index]    
    
    # matrix string conversion
    def __str__(self):
        return ("\n".join([str(row) for row in self.data])) + "\n(" + str(self.rows) + "x" + str(self.cols) + ")\n"

    # matrix addition
    def __add__(self, other):
        
        # error checking
        if not isinstance(other, matrix):
            raise TypeError("Addition not supported between matrix and " + str(type(other)))
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Incompatible shapes for matrix addition")
        
        result_data = [[self.data[i][j] + other.data[i][j] for j in range(self.cols)] for i in range(self.rows)]
        
        return matrix(result_data)

    # matrix subtraction
    def __sub__(self, other):
        
        # error checking
        if not isinstance(other, matrix):
            raise TypeError("Subrtaction not supported between matrix and " + str(type(other)))
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Incompatible shapes for matrix subtraction")
        
        result_data = [[self.data[i][j] - other.data[i][j] for j in range(self.cols)] for i in range(self.rows)]
        
        return matrix(result_data)


    # matrix multiplication
    def __mul__(self, other):
        
        # matrix-matrix multiplication
        if isinstance(other, matrix):
            
            # error checking
            if self.cols != other.rows:
                raise ValueError("Incompatible shapes for matrix multiplication")
            
            # compute result
            result_data = [[0 for _ in range(other.cols)] for _ in range(self.rows)]
            for i in range(self.rows):
                for j in range(other.cols):
                    for k in range(self.cols):
                        result_data[i][j] += self.data[i][k] * other.data[k][j]
            
            return matrix(result_data)
        
        # matrix-number multiplication
        elif isinstance(other, (int, float)):
            
            # computer result
            result_data = [[self.data[i][j] * other for j in range(self.cols)] for i in range(self.rows)]
            return matrix(result_data)
        
        # unsupported multiplication
        else:
            raise TypeError("Multiplication not supported between matrix and " + str(type(other)))
    
    # absolute value
    def abs(self):
        abs_data = [[abs(col) for col in row] for row in self.data]
        return matrix(abs_data)
    
    # maximum
    def max(self):
        max_val = None
        for row in self.data:
            for col in row:
                if max_val is None or col > max_val:
                    max_val = col
        return max_val

    # matrix transpose 
    def transpose(self):
        # create a new matrix with swapped rows and columns
        transposed_data = [[self.data[j][i] for j in range(self.rows)] for i in range(self.cols)]
        return matrix(transposed_data)

    # matrix inverse
    def inverse(self):
        if self.rows != self.cols:
            raise ValueError("Matrix must be square to find its inverse")
        
        # create an augmented matrix [A | I]
        augmented_data = [self.data[i] + [1 if i == j else 0 for j in range(self.cols)] for i in range(self.rows)]
        augmented_matrix = matrix(augmented_data)
        
        # perform Gaussian elimination to transform A into the identity matrix
        for i in range(self.rows):
            pivot_row = i
            while pivot_row < self.rows and augmented_matrix.data[pivot_row][i] == 0:
                pivot_row += 1
            if pivot_row == self.rows:
                raise ValueError("Matrix is singular and has no inverse")
            if pivot_row != i:
                augmented_matrix.data[i], augmented_matrix.data[pivot_row] = augmented_matrix.data[pivot_row], augmented_matrix.data[i]
            pivot_value = augmented_matrix.data[i][i]
            for j in range(i, 2 * self.cols):
                augmented_matrix.data[i][j] /= pivot_value
            for k in range(self.rows):
                if k != i:
                    multiple = augmented_matrix.data[k][i]
                    for j in range(i, 2 * self.cols):
                        augmented_matrix.data[k][j] -= multiple * augmented_matrix.data[i][j]
        
        # extract the inverse matrix from the right half of the augmented matrix
        inverse_data = [augmented_matrix.data[i][self.cols:] for i in range(self.rows)]
        inverse_matrix = matrix(inverse_data)
        return inverse_matrix
    
    # isclose method
    def isclose(self, other, rel_tol=1e-9, abs_tol=0.0):
        # check that other is a matrix with the same dimensions as self
        if not isinstance(other, matrix) or self.rows != other.rows or self.cols != other.cols:
            return False
        
        # check that the absolute difference between corresponding elements is within tolerances
        for i in range(self.rows):
            for j in range(self.cols):
                if not math.isclose(self.data[i][j], other.data[i][j], rel_tol=rel_tol, abs_tol=abs_tol):
                    return False
        
        # if all element pairs pass the test, the matrices are close enough
        return True

    # matrix reduced row echelon form
    def rref(self):
        A = self.data
        lead = 0
        row_count = len(A)
        column_count = len(A[0])
        for r in range(row_count):
            if lead >= column_count:
                return
            i = r
            while A[i][lead] == 0:
                i += 1
                if i == row_count:
                    i = r
                    lead += 1
                    if column_count == lead:
                        return
            A[i], A[r] = A[r], A[i]
            lv = A[r][lead]
            A[r] = [mrx / float(lv) for mrx in A[r]]
            for i in range(row_count):
                if i != r:
                    lv = A[i][lead]
                    A[i] = [iv - lv*rv for rv,iv in zip(A[r],A[i])]
            lead += 1
        return matrix(A)

# linear-quadratic regulator
def lqr(A, B, Q, R):
    #
    # A = [n x n]
    # B = [n x m]
    # Q = [n x n]
    # R = [m x m]
    #
    # K = [m x n]
    #
    
    n = A.rows
    m = B.cols
    P = matrix.zeros(n,n)
    K = matrix.zeros(m,n)
    
    max_iter = 100
    eps = 1e-6
    
    for i in range(max_iter):
        K_prev = K
        A_T = A.transpose()
        B_T = B.transpose()
        
        P = Q + A_T * P * A - A_T * P * B * (R + (B_T * P * B)).inverse() *  B_T * P * A
        K = (R + B_T * P * B).inverse() * B_T * P * A
        
        if ((K - K_prev).abs()).max() < eps:
            break
        
    return K


# full state space controller
class fss_controller:
    
    # controller constructor
    def __init__(self, K_matrix, target_state):
        self.K_matrix = K_matrix # k matrix (1x2)
        self.r_t = target_state # target state (2x1)
        
        self.min_thrust = 0.05 # minimum force
        self.max_thrust = 2.5 # maximum force
        self.f_avg = (self.min_thrust + self.max_thrust) / 2 # force average
        
        self.I = 0.044 # moment of inertia - kg*m^2
        self.K = 0.12 # spring constant N*m/rad
        self.C = 0.2 # damping friction N*m*s/rad
        
    def set_desired_angle(self, value):
        self.r_t = matrix([deg_to_rad(value), 0.0])
        
    def update(self, theta, theta_dot):
        
        # implement control law
        z_t = matrix([[0, theta_dot],[theta, 0]]) # z(t) [2x2]
        u_t = self.r_t - self.K_matrix * z_t # u(t) = r(t) - K * z(t) [1x2]
        forces = matrix([[0.5, 0.5, self.f_avg], [0.180975, -0.180975, u_t[0][0]]]) # required force [2x3]
        thrusts = forces.rref() # required thrust matrix [2x3]
        
        # convert to pressure
        p_top = max(self.min_thrust, min(self.max_thrust, thrusts[0][2])) * 22.5537 - 3.1155 # pressure required top
        p_bot = max(self.min_thrust, min(self.max_thrust, thrusts[1][2])) * 22.5537 - 3.1155 # pressure required bottom
        
        return (p_top, p_bot)
    
    
# full state feedback controller with an integrator 
class fss_controller_w_int:
    
    # controller constructor
    def __init__(self, K, K_i):
        self.K = K # k matrix (1x2)
        self.x0 = matrix([0.0, 0.0]) # target state (1x2)
        self.K_i = K_i # integrator gain coefficient
        self.first_iter = True # first iteration flag
        
        self.min_thrust = 0.05 # minimum force
        self.max_thrust = 2.5 # maximum force
        self.f_avg = (self.min_thrust + self.max_thrust) / 2 # force average
        self.df = 0.180975 # moment arm from center of rotation to nozzle, in meters
        
        self.cumul_err = 0 # cumulative error
        self.curr_err = 0 # current error
        self.prev_err = 0 # previous error
        
        self.min_thrust = 0.1
        self.max_thrust = 2.7
        self.force_avg = (self.min_thrust + self.max_thrust) / 2.0
        
    # set the target angle
    def set_target_angle(self, value):
        self.x0 = matrix([deg_to_rad(value), 0.0]) # set x0 matrix [rad]
        
    # update function
    def update(self, theta, theta_dot):
        
        x = matrix([ [0.0, theta_dot], [theta, 0.0] ]) # state (2x2)
        cmd = self.x0 - self.K * x # target_state - k_matrix * state (1x2)
        control_torque = cmd[0][0]
        
        max_control_torque = (self.max_thrust - self.min_thrust)*self.df
        
        if control_torque > max_control_torque:
            control_torque = max_control_torque
        elif control_torque < -max_control_torque:
            control_torque = -max_control_torque
        
        #F = matrix([[0.5, 0.5, self.force_avg], [0.180975, -0.180975, cmd[0][0]]])
        #T = F.rref() # required thrust of the system
        
        self.err = self.x0[0][0] - theta # compute error between desired and current angle
        
        if self.first_iter: # if we're on the first loop, derivative and integral errors are undefined
            self.last_err = self.err
            self.cumul_err = 0 # set both equal to zero
            self.first_iter = False # update first iteration flag
        else:
            self.cumul_err += self.err * CONTROLLER_TIME_STEP
            
        if (self.err > 0 and self.last_err < 0) or (self.err < 0 and self.last_err > 0) or (abs(self.err) < deg_to_rad(2)):
            self.cumul_err = 0
        
        F_top = self.f_avg - (control_torque/(2*self.df)) - self.K_i * self.cumul_err # LEFT nozzle
        F_bot = self.f_avg + (control_torque/(2*self.df)) + self.K_i * self.cumul_err # RIGHT nozzle
        
        p_top = max(self.min_thrust, min(self.max_thrust, F_top)) * 22.5537 - 3.1155 # pressure required top
        p_bot = max(self.min_thrust, min(self.max_thrust, F_bot)) * 22.5537 - 3.1155 # pressure required bottom
        
        self.last_err = self.err
        return (p_top, p_bot)

# PID Controller (just in case ;))
class pid_controller:
    
    # PID controller constructor 
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p # proportional gain coefficient
        self.K_i = K_i # integral gain coefficient
        self.K_d = K_d # derivative gain coefficient
        self.x0 = 0 # initial state
        
        self.first_iter = True # first interaction flag
        
        self.min_thrust = 0.05 # minimum force
        self.max_thrust = 2.7 # maximum force
        self.f_avg = (self.min_thrust + self.max_thrust) / 2 # force average
        self.df = 0.180975 # moment arm from center of rotation to nozzle, in meters
        
        self.cumul_err = 0 # cumulative error
        self.d_err = 0 # derivative error
        self.last_err = 0 # last error
        self.err = 0
        
    # set the target angle
    def set_target_angle(self, value):
        self.x0 = deg_to_rad(value) # set desired angle [rad]
        
    # update function
    def update(self, theta):
        self.err = (self.x0 - theta) # compute error between desired and current angle [rad]

        if self.first_iter: # if we're on the first loop, derivative and integral errors are undefined
            self.cumul_err = 0 # set both equal to zero
            self.d_err = 0
            self.first_iter = False # update flag
        else: # otherwise compute integral and derivative errors as normal
            self.cumul_err += self.err * CONTROLLER_TIME_STEP # rectangular approximation of integral error for this update
            self.d_err = (self.err - self.last_err) / CONTROLLER_TIME_STEP # first order finite difference approximation of derivative error
        
        if (self.err > 0 and self.last_err < 0) or (self.err < 0 and self.last_err > 0) or (abs(self.err) < deg_to_rad(2)) or (self.d_err > 0.4):
            self.cumul_err = 0
        
        control_torque = (self.K_p * self.err) + (self.K_i * self.cumul_err) + (self.K_d * self.d_err) # compute control signal
        max_control_torque = (self.max_thrust - self.min_thrust)*self.df
        
        if control_torque > max_control_torque:
            control_torque = max_control_torque
        elif control_torque < -max_control_torque:
            control_torque = -max_control_torque
        
        F_top = self.f_avg - (control_torque/(2*self.df)) # LEFT nozzle
        F_bot = self.f_avg + (control_torque/(2*self.df)) # RIGHT nozzle
        
        p_top = max(self.min_thrust, min(self.max_thrust, F_top)) * 22.5537 - 3.1155 # pressure required top
        p_bot = max(self.min_thrust, min(self.max_thrust, F_bot)) * 22.5537 - 3.1155 # pressure required bottom
        
        self.last_err = self.err # set last error equal to this loop's error
        
        return p_top, p_bot# return required nozzle pressures

class logger:
    
    def __init__(self, header):
        self.log_file = open("log_file.csv", "w")
        self.log_file.write(header)
        
    def write(self, *args):
        for arg in args[0:-1]:
            self.log_file.write(str(arg) + ", ")
        self.log_file.write(str(args[-1]) + "\n")
        
    def close(self):
        self.log_file.close()


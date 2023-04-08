# control_util.py
# April 5th, 2023

# imports
import math

# angle functions
def deg_to_rad(degrees):
    return degrees * (math.pi / 180.0)

def rad_to_deg(radians):
    return radians * (180.0 / math.pi)

# math functions
def remap(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min


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
    
    # matrix reduced row echelon form
    def rref(self):
        A = self.data
        lead = 0
        rowCount = len(A)
        columnCount = len(A[0])
        for r in range(rowCount):
            if lead >= columnCount:
                return
            i = r
            while A[i][lead] == 0:
                i += 1
                if i == rowCount:
                    i = r
                    lead += 1
                    if columnCount == lead:
                        return
            A[i], A[r] = A[r], A[i]
            lv = A[r][lead]
            A[r] = [mrx / float(lv) for mrx in A[r]]
            for i in range(rowCount):
                if i != r:
                    lv = A[i][lead]
                    A[i] = [iv - lv*rv for rv,iv in zip(A[r],A[i])]
            lead += 1
        return matrix(A)

class fss_controller:
    
    def __init__(self, K_matrix, target_state):
        self.K_matrix = K_matrix # k matrix (1x2)
        self.r_t = target_state # target state (2x1)
        
        self.min_thrust = 0.05
        self.max_thrust = 2.5
        self.f_avg = (self.min_thrust + self.max_thrust) / 2
        
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
    
    
class fss_controller_real:
    
    def __init__(self, K, Ki, updateInt):
        self.K = K # k matrix (1x2)
        self.x0 = matrix([deg_to_rad(30), 0.0]) # target state (1x2)
        self.cumulErr = 0
        self.Ki = Ki
        self.updateInt = updateInt/1000 # update interval for the controller. pass in milliseconds
        self.lastErr = 0
        self.loopCnt = 0 # loop counter
        self.err = 0
        
    def set_desired_angle(self, value):
        self.x0 = matrix([deg_to_rad(value), 0.0])
        
    def update(self, theta, theta_dot):
        x = matrix([ [0.0, theta_dot], [theta, 0.0] ]) # state (2x2)
        cmd = self.x0 - self.K * x # target_state - k_matrix * state (1x2)
        F = matrix([[0.5, 0.5, (0.05 + 2.5) / 2.0], [0.180975, -0.180975, cmd[0][0]]])
        T = F.rref() # required thrust of the system
        
        self.err = self.x0[0][0] - (-theta) # compute error between desired and current angle
        
        if self.loopCnt == 0: # if we're on the first loop, derivative and integral errors are undefined
            self.lastErr = self.err
            self.cumulErr = 0 # set both equal to zero
            self.loopCnt = 1 # update logical variable
        else:
            self.cumulErr += self.err*self.updateInt
            
        if (self.err > 0 and self.lastErr < 0) or (self.err < 0 and self.lastErr > 0) or (abs(self.err) < deg_to_rad(2)):
            self.cumulErr = 0
        
        Ftop = T[0][2] - self.Ki*self.cumulErr
        Fbot = T[1][2] + self.Ki*self.cumulErr
        
        print(self.err, self.cumulErr, Ftop, Fbot)
        
        p_top = max(0.05, min(2.5, Ftop)) * 22.5537 - 3.1155 # pressure required top
        p_bot = max(0.05, min(2.5, Fbot)) * 22.5537 - 3.1155 # pressure required bottom
        
        self.lastErr = self.err
        return (p_top, p_bot)

# PID Controller (just in case ;))
class pid_controller:
    
    # Kp = 3, Ki = 5, Kd = 2 from MATLAB

    def __init__(self,Kp,Ki,Kd,x0,updateInt):
        self.Kp = Kp # proportional gain
        self.Ki = Ki # integral gain
        self.Kd = Kd # derivative gain
        self.x0 = x0 # target ANGLE
        self.updateInt = updateInt/1000 # update interval for the controller. pass in milliseconds
        self.loopCnt = 0 # loop counter
        self.cumulErr = 0
        self.dErr = 0
        self.lastErr = 0
        
    def set_desired_angle(self,value):
        self.x0 = value # set desired angle
        
    def update(self, theta):
        err = (self.x0 - theta) # compute error between desired and current angle

        if self.loopCnt == 0: # if we're on the first loop, derivative and integral errors are undefined
            self.cumulErr = 0 # set both equal to zero
            self.dErr = 0
            self.loopCnt = 1 # update logical variable
        else: # otherwise compute integral and derivative errors as normal
            self.cumulErr += err*self.updateInt # rectangular approximation of integral error for this update
            self.dErr = (err-self.lastErr)/self.updateInt # first order finite difference approximation of derivative error
        
        controlTorque = (self.Kp*err) + (self.Ki*self.cumulErr) + (self.Kd*self.dErr) # compute control signal
        
        F = matrix([[0.5, 0.5, (0.05 + 2.5) / 2.0], [0.180975, -0.180975, controlTorque]]) # split torque into forces
        T = F.rref() # required thrust of the system
        
        print(T[1][2], T[0][2])
        
        p_bot = max(0.05, min(2.7, T[0][2])) * 22.5537 - 3.1155 # pressure required top
        p_top = max(0.05, min(2.7, T[1][2])) * 22.5537 - 3.1155 # pressure required bottom
        
        #print(p_top, p_bot)
        
        self.lastErr = err # set last error equal to this loop's error
        
        return (p_top, p_bot) # return required nozzle pressures

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


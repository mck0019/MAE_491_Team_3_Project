# control_util.py
# April 4th, 2023

# imports
import math

# angle functions
def deg_to_rad(degrees):
    return degrees * math.pi / 180.0

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

# controller class
class controller:
    
    def __init__(self):
        
        
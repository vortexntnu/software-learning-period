import numpy as np

class Matrix:
    def __init__(self, rows, columns, elements=None):
        self.rows = rows
        self.columns = columns
        if elements is None:
            self.elements = np.zeros((rows, columns))
        else:
            self.elements = np.array(elements)

    def __str__(self):
        return str(self.elements)

    def __add__(self, other):
        if self.rows != other.rows or self.columns != other.columns:
            raise ValueError("The dimensions of the matrices must be the same.")
        return Matrix(self.rows, self.columns, self.elements + other.elements)
    
    def __mul__(self, other):
        if isinstance(other, Matrix):
            if self.columns != other.rows:
                raise ValueError("The number of columns in the first matrix must be the same as the number of rows in the second matrix.")
            result = self.elements @ other.elements
            return Matrix(self.rows, other.columns, result)
        else:
            return Matrix(self.rows, self.columns, self.elements * other)

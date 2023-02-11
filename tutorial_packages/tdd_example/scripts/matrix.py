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
        return
    
    def __mul__(self, other):
        return
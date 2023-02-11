from scripts.matrix import Matrix

import numpy as np
import pytest

def test_init():
    m = Matrix(3, 4)
    assert m.rows == 3
    assert m.columns == 4
    assert np.array_equal(m.elements, np.zeros((3, 4)))

    m = Matrix(2, 2, [[1, 2], [3, 4]])
    assert m.rows == 2
    assert m.columns == 2
    assert np.array_equal(m.elements, np.array([[1, 2], [3, 4]]))

def test_add():
    m1 = Matrix(2, 2, [[1, 2], [3, 4]])
    m2 = Matrix(2, 2, [[5, 6], [7, 8]])
    m3 = m1 + m2
    assert np.array_equal(m3.elements, np.array([[6, 8], [10, 12]]))

    with pytest.raises(ValueError) as exc_info:
        m1 + Matrix(3, 3)
    assert str(exc_info.value) == "The dimensions of the matrices must be the same."

def test_mul():
    m1 = Matrix(2, 3, [[1, 2, 3], [4, 5, 6]])
    m2 = Matrix(3, 2, [[7, 8], [9, 10], [11, 12]])
    m3 = m1 * m2
    assert np.array_equal(m3.elements, np.array([[58, 64], [139, 154]]))

    m4 = m1 * 2
    assert np.array_equal(m4.elements, np.array([[2, 4, 6], [8, 10, 12]]))

    with pytest.raises(ValueError) as exc_info:
        m1 * Matrix(2, 3)
    assert str(exc_info.value) == "The number of columns in the first matrix must be the same as the number of rows in the second matrix."

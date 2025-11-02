import numpy as np

def euler_discretise(A, B, C, D, ts=1.0):
    """Discretise continuous-time state-space using Euler method.
    """
    Ad = np.eye(A.shape[0]) + A*ts
    Bd = B*ts
    Cd = C
    Dd = D

    return Ad, Bd, Cd, Dd

def exact_discretise(A, B, C, D, ts=1.0):
    n = A.shape[0]
    m = B.shape[1]

    
import numpy as np
import scipy.integrate as integrate
import scipy.linalg as la

def euler_discretise(Ac, Bc, Cc, Dc, ts=1.0):
    """Discretise continuous-time state-space using Euler method.
    """
    Ad = np.eye(Ac.shape[0]) + Ac*ts
    Bd = Bc*ts
    Cd = Cc
    Dd = Dc

    return Ad, Bd, Cd, Dd

def exact_discretise(Ac, Bc, Cc, Dc, ts=1.0):
    n = Ac.shape[0]
    m = Bc.shape[1]

    M = np.zeros((n+m, n+m))
    M[0:n, 0:n] = Ac
    M[0:n, n:n+m] = Bc
    expMts = la.expm(M*ts)

    Ad = expMts[0:n, 0:n]
    Bd = expMts[0:n, n:n+m]
    Cd = Cc
    Dd = Dc

    return Ad, Bd, Cd, Dd



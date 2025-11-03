import numpy as np
import matplotlib.pyplot as plt
import stabilisation as st

def system_step(A, B, C, D, x_k, u_k):
    """Perform one step of the discrete state-space system.
    """
    x_next = A @ x_k + B @ u_k
    y_k = C @ x_k + D @ u_k
    return x_next, y_k

def simulate_system(A, B, C, D, U, x0=None):
    """Simulate the discrete state-space system.
    """
    n_steps, m = U.shape
    n = A.shape[0]
    p = C.shape[0]

    if x0 is None:
        x_k = np.zeros((n, 1))
    else:
        x_k = x0
    
    Y = np.zeros((n_steps, p))
    X = np.zeros((n_steps, n))

    X[0, :] = x_k.flatten()

    for k in range(n_steps):
        u_k = U[k, :].reshape(m, 1)
        x_next, y_k = system_step(A, B, C, D, x_k, u_k)
        if k+1 < n_steps:
            X[k+1, :] = x_next.flatten()
        Y[k, :] = y_k.flatten()
        x_k = x_next

    return X, Y

def simulate_mpc_stabilisation(A, B, C, D, Q, R, ts, N_horizon, x0, n_steps, M=None):
    """Simulate the system under unconstrained MPC stabilisation.
    """
    n = A.shape[0]
    m = B.shape[1]

    if M is None:
        M = N_horizon

    X_history = np.zeros((n_steps + 1, n))
    U_history = np.zeros((n_steps, m))

    xt_k = x0
    X_history[0, :] = xt_k.flatten()

    for k in range(n_steps):
        u_k = st.unconstrained_stabilisation(A, B, C, D, Q, R, ts, N_horizon, M, xt_k)
        U_history[k, :] = u_k.flatten()
        xt_k = A @ xt_k + B @ u_k
        X_history[k + 1, :] = xt_k.flatten()

    return X_history, U_history

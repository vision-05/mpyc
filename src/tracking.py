import numpy as np
import scipy.linalg as la

def unconstrained_tracking_law(A, B, C, D, Q, R, ts, N, M, xt0, rt):
    """Control law gain for unconstrained tracking MPC.
       xt0 is current state vector must be a column vector
       rt is reference trajectory over horizon N (shape: (n, N))
    """
    if M > N:
        raise ValueError("Control horizon M cannot be greater than time horizon N.")

    n = A.shape[0]
    m = B.shape[1]
    X_t = np.zeros((N, n))
    U_t = np.zeros((N, m))
    F = np.zeros((n*N, n))
    phi = np.zeros((n*N, m*N))
    Q_Q = np.kron(np.eye(N), Q)
    R_R = np.kron(np.eye(M), R)

    F[0:n, :] = A
    phi[0:n, 0:m] = B

    for i in range(1,N):
        F[i*n : (i+1)*n, :] = F[(i-1)*n : i*n, :] @ A
        
        phi[i*n : (i+1)*n, 0:m] = F[(i-1)*n : i*n, :] @ B
        
        for j in range(1, i+1):
             phi[i*n : (i+1)*n, j*m : (j+1)*m] = phi[(i-1)*n : i*n, (j-1)*m : j*m]


    phi_M = phi[:, 0:m*M]
    i_M = np.zeros((m, m*M))
    i_M[:, 0:m] = np.eye(m)

    Hessian = np.transpose(phi_M) @ Q_Q @ phi_M + R_R
    Cross_term = np.transpose(phi_M) @ Q_Q
    K_full = la.solve(Hessian, Cross_term) 

    kmpc = i_M @ K_full
    return kmpc
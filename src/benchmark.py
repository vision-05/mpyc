import time
import numpy as np
from discretisation import euler_discretise, exact_discretise
import statespace as s
import stabilisation as st
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt

def euler_vs_exact_discretisation():
    m=1
    k=1
    b=2
    Ac = np.array([[0, 1], [-k/m, -b/m]])
    Bc = np.array([[0], [1/m]])
    Cc = np.array([[1, 0]])
    Dc = np.array([[0]])
    ts = 0.1  # Sampling time

    tstart = time.time()
    Ad_euler, Bd_euler, Cd_euler, Dd_euler = euler_discretise(Ac, Bc, Cc, Dc, ts) 
    tend = time.time()
    print(f"Euler discretisation time: {tend - tstart:.6f} seconds") 
    # Discretise the system using Exact method
    tstart = time.time()
    Ad_exact, Bd_exact, Cd_exact, Dd_exact = exact_discretise(Ac, Bc, Cc, Dc, ts)
    tend = time.time()
    print(f"Exact discretisation time: {tend - tstart:.6f} seconds")


def euler_vs_exact_mpc():
    # Continuous-time state-space matrices
    m=1
    k=1
    b=2
    Ac = np.array([[0, 1], [-k/m, -b/m]])
    Bc = np.array([[0], [1/m]])
    Cc = np.array([[1, 0]])
    Dc = np.array([[0]])
    ts = 0.1  # Sampling time

    # Discretise the system using Euler method
    Ad_euler, Bd_euler, Cd_euler, Dd_euler = euler_discretise(Ac, Bc, Cc, Dc, ts)  
    # Discretise the system using Exact method
    Ad_exact, Bd_exact, Cd_exact, Dd_exact = exact_discretise(Ac, Bc, Cc, Dc, ts)

    # Simulation parameters
    simtime = np.arange(0, 50, ts)
    n_steps = len(simtime)
    # Stabilisation using unconstrained MPC
    Q = np.array([[100, 0], [0, 1]])
    R = np.array([[10]])

    tstart = time.time()
    X_euler_history, U_euler_history = s.simulate_mpc_stabilisation(
        Ad_euler, Bd_euler, Cd_euler, Dd_euler,
        Q, R, ts, N_horizon=50, x0=np.array([[5], [0]]), n_steps=n_steps, M=10
    )
    tend = time.time()
    print(f"Euler MPC simulation time: {tend - tstart:.4f} seconds")

    tstart = time.time()
    X_exact_history, U_exact_history = s.simulate_mpc_stabilisation(
        Ad_exact, Bd_exact, Cd_exact, Dd_exact,
        Q, R, ts, N_horizon=50, x0=np.array([[5], [0]]), n_steps=n_steps, M=10
    )
    tend = time.time()
    print(f"Exact MPC simulation time: {tend - tstart:.4f} seconds") 

    # Plot results
    plt.figure()
    plt.plot(simtime, X_euler_history[:-1, 0], label='Euler Discretisation')
    plt.plot(simtime, X_exact_history[:-1, 0], label='Exact Discretisation', linestyle='--')
    plt.title('MPC Stabilisation: Euler vs Exact Discretisation')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (x1)')
    plt.legend()
    plt.grid()
    plt.savefig('../mpc_stabilisation_euler_vs_exact.png')

if __name__ == "__main__":
    euler_vs_exact_discretisation()
    euler_vs_exact_mpc()
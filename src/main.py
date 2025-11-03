import statespace as s
import discretisation as d
import stabilisation as st
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt

def main():
    # Continuous-time state-space matrices
    # mass spring damper system
    m=1
    k=1
    b=2
    Ac = np.array([[0, 1], [-k/m, -b/m]])
    Bc = np.array([[0], [1/m]])
    Cc = np.array([[1, 0]])
    Dc = np.array([[0]])
    ts = 0.1  # Sampling time

    # Discretise the system using Euler method
    Ad_euler, Bd_euler, Cd_euler, Dd_euler = d.euler_discretise(Ac, Bc, Cc, Dc, ts)

    # Discretise the system using Exact method
    Ad_exact, Bd_exact, Cd_exact, Dd_exact = d.exact_discretise(Ac, Bc, Cc, Dc, ts)

    # Initial state
    x0 = np.array([[0], [0]])

    # Simulation parameters
    simtime = np.arange(0, 50, ts)
    n_steps = len(simtime)
    U = np.ones((n_steps, 1))  # Zero input

    # Simulate both systems
    #X_eu, Y_eu = s.simulate_system(Ad_euler, Bd_euler, Cd_euler, Dd_euler, U, x0)
    #X_ex, Y_ex = s.simulate_system(Ad_exact, Bd_exact, Cd_euler, Dd_euler, U, x0)
    # Plot results
    #plt.figure()
    #plt.plot(simtime, Y_eu[:, 0], label='Euler Discretisation')
    #plt.plot(simtime, Y_ex[:, 0], label='Exact Discretisation', linestyle='--')
    #plt.title('System Response Comparison')
    #plt.xlabel('Time (s)')
    #plt.ylabel('Output')
    #plt.legend()
    #plt.grid()
    #plt.savefig('../system_response_comparison.png')

    # Stabilisation using unconstrained MPC
    Q = np.array([[100, 0], [0, 1]])
    R = np.array([[10]])

    X_stab_history, U_stab_history = s.simulate_mpc_stabilisation(
        Ad_exact, Bd_exact, Cd_exact, Dd_exact,
        Q, R, ts, N_horizon=50, x0=np.array([[5], [0]]), n_steps=n_steps, M=10
    )

    
    plt.figure()
    plt.plot(simtime, X_stab_history[:-1, 0], label='Position (x1)')
    plt.plot(simtime, X_stab_history[:-1, 1], label='Velocity (x2)', linestyle='--')
    plt.title('State Trajectories under Unconstrained MPC Stabilisation')
    plt.xlabel('Time (s)')
    plt.ylabel('States')
    plt.legend()
    plt.grid()
    plt.savefig('../state_trajectories_stabilisation.png')

    # (Optional) Plot the control inputs
    plt.figure()
    plt.plot(simtime[:-1], U_stab_history[:-1, 0], '.-', label='Control Input (u)')
    plt.title('Control Input Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Input Force')
    plt.legend()
    plt.grid()
    plt.savefig('../control_input_stabilisation.png')

    X_eu, Y_eu = s.simulate_system(Ad_euler, Bd_euler, Cd_euler, Dd_euler, U_stab_history, np.array([[5], [0]]))
    X_ex, Y_ex = s.simulate_system(Ad_exact, Bd_exact, Cd_euler, Dd_euler, U_stab_history, np.array([[5], [0]]))
    #Plot results
    plt.figure()
    plt.plot(simtime, Y_eu[:, 0], label='Euler Discretisation')
    plt.plot(simtime, Y_ex[:, 0], label='Exact Discretisation', linestyle='--')
    plt.title('System Response Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid()
    plt.savefig('../system_response_comparison.png')
    

if __name__ == "__main__":
    main()
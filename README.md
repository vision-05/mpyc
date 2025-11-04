# mpyc
MPC module from scratch in Python with numpy, an application of my control 2 module

# Discretisation

## Euler discretisation
$$\dot{x}(t) = A_cx(t)+B_cu(t)$$

$$y(t) = C_cx(t) + D_cu(t)$$

We would like to discretise this system using the Euler discretisation. The euler discretisation uses a forward divided difference and so it is computationally cheap, although it requires <assumptions> to be accurate enough

Assume that we only have an integer timestep so we have $t_s$ as our timestep length and $k$ as our integer multiplier

$$\dot{x}(kt_s) = A_cx(kt_s) + B_cu(kt_s)$$

$$\dot{x}(kt_s) \approx \frac{x((k+1)t_s) - x(kt_s)}{t_s}$$

$$\frac{x((k+1)t_s) - x(kt_s)}{t_s} = A_cx(kt_s) + B_cu(kt_s)$$

$$x((k+1)t_s) - x(kt_s) = t_sA_cx(kt_s) + t_sB_cu(kt_s)$$

$$x((k+1)t_s) = x(kt_s) + t_sA_cx(kt_s) + t_sB_cu(kt_s)$$

$$x((k+1)t_s) = (I+t_sA_c)x(kt_s) + t_sB_cu(kt_s)$$

$$x(k+1) = Ax(k) + Bu(k)$$

$$y(k) = C(k) + D(k)$$
Where

$$A = I+t_sA_c$$

$$B = t_sB_c$$

$$C = C_c$$

$$D = D_c$$

Giving us the discretised system for a state space description

## Exact discretisation
Take the state space description

$$\dot{x}(t) = A_cx(t)+B_cu(t)$$

$$y(t) = C_cx(t) + D_cu(t)$$

Let us find the general solution to a state space system, which is the standard steps for solving homogeneous linear first order ODE

$$\dot{x}(t) = ax(t)+bu(t)\\$$

Multiply by integrating factor

$$e^{-at}\dot{x}(t) = e^{-at}ax(t) + e^{-at}bu(t)$$

Rearrange to have all x(t) terms on one side

$$-ae^{-at}x(t) + e^{-at}\dot{x}(t) = e^{-at}bu(t)$$

Through product rule

$$\frac{d}{dt}(e^{-at}x(t)) = (\frac{d}{dt}e^{-at})x(t) + e^{-at}(\frac{d}{dt}x(t))$$

$$\frac{d}{dt}(e^{-at}x(t)) = -ae^{-at}x(t) + e^{-at}\dot{x}(t)$$

Substitute in this equality

$$\frac{d}{dt}(e^{-at}x(t)) = e^{-at}bu(t)$$

Then integrate both sides, from time $t$ to time $t_0$. We introduce a new variable $\tau$ for this

$$e^{-at}x(t) - e^{-at_0}x(t_0) = \int_{t_0}^te^{-a\tau}bu(\tau)d\tau$$

Rearrange

$$e^{-at}x(t) = e^{-at_0}x(t_0) + \int_{t_0}^te^{-a\tau}bu(\tau)d\tau$$

Divide through by $e^{-at}$ (as this cannot be 0)

$$x(t) = e^{a(t-t_0)}x(t_0) + \int_{t_0}^te^{a(t-\tau)}bu(\tau)d\tau$$

Now substitute our matrix values in again for a standard response

$$x(t) = e^{A_c(t-t_0)}x(t_0) + \int_{t_0}^te^{A_c(t-\tau)}B_cu(\tau)d\tau$$

Now we have the standard format, we will use this to write our solutions for our discrete timesteps $x(kt_s)$ and $x((k+1)t_s)$, and setting $t_0 = 0$

$$x((k+1)t_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$$

$$x(kt_s) = e^{A_ckt_s}x(0) + \int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau$$

Then as

$$e^{A_ckt_s} \cdot e^{A_ct_s} = e^{A_c(k+1)t_s}$$

We multiply both sides by $e^{A_ct_s}$

$$e^{A_ct_s}x(kt_s) = e^{A_ct_s}e^{A_ckt_s}x(0) + e^{A_ct_s}\int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau$$

Simplify

$$e^{A_ct_s}x(kt_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$$

Then substitute

$$e^{A_c(k+1)t_s}x(0) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$$

$$e^{A_ct_s}x(kt_s) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$$

Rearrange

$$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau - \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau$$

By rule of integral linearity

$$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$$

Then because of our discretisation, in the step between $kt_s$ and $(k+1)t_s$ $u(\tau)$ must be constant so we take it out of the integral

$$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}d\tau$$

Then take new variable $\lambda = (k+1)t_s - \tau$, (so we flip the sign of the integral as $\lambda$ is related to $-\tau$)

$$x((k+1)t_s) = e^{A_ct_s}x(kt_s) - B_cu(kt_s)\int_{t_s}^{0}e^{A_c\lambda}d\lambda$$

Then we can restate this as $\tau = -\lambda$. Yes I know I'm redefining and this is mathematically incorrect, but I am an engineer and I already used tau but in the powerpoint slide for the final equation I wanted, my professor used tau :)

$$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{0}^{t_s}e^{A_c\tau}d\tau$$

Finally we will redefine our equation as

$$x((k+1)t_s) = Ax(kt_s) + Bu(kt_s)$$

Where

$$A = e^{A_ct_s}$$

$$B = B_c\int_{0}^{t_s}e^{A_c\tau}d\tau$$

The output equations always remain the same so

$$y(kt_s) = Cx(kt_s) + Du(kt_s)$$

Where

$$C = C_c$$

$$D = D_c$$

As python does not support integration that returns a matrix, we instead create a new matrix

$$M = \begin{bmatrix} A_c & B_c \\\ 0 & 0 \end{bmatrix}$$

Then by a long taylor expansion of the matrix exponential

$$e^{Mt_s} = \begin{bmatrix} e^{A_ct_s} & \int_{0}^{t_s}e^{A_c\tau}d\tau \\\ 0 & I \end{bmatrix}$$

Thus rather than compute integrals, and take many matrix exponentials, we only need to take one matrix exponential and then extract our terms.

# MPC

MPC is an advanced control strategy that uses a mathematical model to predict future states of the system. It then uses these predictions to come up with a sequence of the best control inputs it can to reach a desired output. It looks into the future for a fixed amount of time, called the time horizon, and it creates a control sequence for a separate fixed amount of time, called the control horizon. The control horizon is always shorter than the time horizon.

The thing is, this method is not robust, as if we followed this sequence of inputs blindly, if an unexpected disturbance appeared, our system would be derailed entirely and not reach our target output or state. So what we do is only execute the first input of our optimal input sequence, and then recompute our inputs for the next time horizon again.

This has 2 distinct advantages:
The system can adapt to overcome disturbances as they happen, and using the model the disturbance can be observed and "rejected" in real time
The policy can be subject to constraints. Because we are dealing with a constantly computed optimisation problem, unlike traditional robust control methodologies, we can apply multiple constraints in the controller design itself

This also has 2 distinct disadvantages:
Computationally expensive, for long time horizons and with terminal cost especially. This means that for particularly quickly sampled states, or for low powered devices this technique won't be very effective
Requires a good model, so this will not work for a black box system, and is not robust to unmodelled dynamics.

## Unconstrained stabilisation with QP
We predict $x_{t+k}$ states, where $1 \leq k \leq N$ and $N$ is the number of steps in our time horizon 
$$x_t = x(t) \\
x_{t+1} = Ax_t + Bu_t \\
x_{t+2} = Ax_{t+1} + Bu_{t+1}
        = A^2x_t + ABu_t + Bu_{t+1} \\
x_{t+N-1} = A^{N-1}x_t + A^{N-2}Bu_t + A^{N-3}Bu_{t+1}+ ... + Bu_{t+N-2}$$

Then we would like to state our states as a function of our initial state,
$$X_t = Fx(t) + \phi U_t$$

so

$$ \begin{bmatrix} x_t \\\ x_{t+1} \\\ x_{t+2} \\\ . \\\ . \\\ x_{t + N - 2} \\\ x_{t + N -1} \end{bmatrix} 
= \begin{bmatrix} 1 \\\ A \\\ A^2 \\\ . \\\ . \\\ A^{N-2} \\\ A^{N-1} \end{bmatrix} x(t) + 
\begin{bmatrix} 0 & 0 & 0 & . & . & 0 & 0 \\\
                B & 0 & 0 & . & . & 0 & 0 \\\ 
                AB & B & 0 & . & . & 0 & 0 \\\
                . & . & . & . & . & . & . \\\
                . & . & . & . & . & . & . \\\
                A^{N-3}B & A^{N-4}B & A^{N-5}B & . & . & 0 & 0 \\\
                A^{N-2}B & A^{N-3}B & A^{N-4}B & . & . & B & 0
\end{bmatrix} \begin{bmatrix} u_t \\\ u_{t+1} \\\ u_{t+2} \\\ . \\\ . \\\ u_{t + N - 2} \\\ u_{t + N - 1} \end{bmatrix}$$

This equation allows us to work out all of our states for n steps into the future, given our initial state at time t and a sequence of inputs. We will use a quadratic program to work out the optimal set of states $U_t^*$ given the current state, as well as our discrete-time A and B vectors.

In order to find our $U_t^*$ we must come up with an optimal control problem.

$$ J(U_t) = \sum_{k=0}^{N-1}{x_{t+k}^T Q x_{t+k} + u_{t+k}^T R u_{t+k}} $$

Where

$$Q \geq 0$$

$$R \geq 0$$

$Q$ and $R$ are tunable, where higher $Q$ penalises state deviation from 0, and greater $R$ penalises higher control effort. Think about it, your controller wants to use the minimal amount of effort to get your state to 0. But sometimes you dont need to limit your control effort as much, and that gives you a quicker response. But sometimes having a lower effort is more important than quick response, such as autopilot car acceleration.

So our Quadratic Program is as follows:

$$ U_t^* = \argmin_{U_t} \sum_{k=0}^{N-1}{x_{t+k}^T Q x_{t+k} + u_{t+k}^T R u_{t+k}} $$

Subject to

$$ x_t = x(t)$$
   
$$x_{t+k+1} = Ax_{t+k} + Bu_{t+k}$$

$$U_t = \set{u_t, u_{t+1}, ..., u_{t+N-1}}$$

For this case, we can find a closed form solution:

The cost is

$$J_t = \sum_{k=0}^{N-1}{x_{t+k}^T Q x_{t+k} + u_{t+k}^T R u_{t+k}}$$

Then sub in our states

$$\begin{equation}J_t = \begin{bmatrix} x_t^T & x_{t+1}^T & ... & x_{t+N-1}^T \end{bmatrix} Q_Q \begin{bmatrix} x_t^T \\\ x_{t+1}^T \\\ . \\\ . \\\ . \\\ x_{t+N-1}^T \end{bmatrix} + \begin{bmatrix} u_t^T & u_{t+1}^T & ... & u_{t+N-1}^T \end{bmatrix} R_R \begin{bmatrix} u_t^T \\\ u_{t+1}^T \\\ . \\\ . \\\ . \\\ u_{t+N-1}^T \end{bmatrix}\end{equation}$$
$$\begin{equation}J_t = X_t^T Q_Q X_t + U_t^T R_R U_t\end{equation}$$

where 
$$ Q_Q = diag(Q,Q,...,Q) \in \R^{Nn \times Nn}$$

$$R_R = diag(R,R,...,R) \in \R^{Nm \times Nm}$$

and

$$ x \in \R^n$$

$$u \in \R^m$$

Then

$$\begin{equation}J_t = (Fx(t) + \phi U_t)^T Q_Q (Fx(t) + \phi U_t) + U_t^T R_R U_t\end{equation}$$

$$\begin{equation}J_t = U_t^T(\phi^T Q_Q \phi + R_R)U_t + 2 U_t^T \phi^T Q_Q Fx(t) + x(t)^T F^T Q_Q F x(t)\end{equation}$$

Then from here we want to minimise the cost, so we take the gradient with respect to $U_t$

$$\begin{equation}\frac{\delta J_t}{\delta U_t} = 2(\phi^T Q_Q \phi + R_R)U_t + 2 \phi^T Q_Q Fx(t)\end{equation}$$

Then set this to zero

$$\begin{equation}2(\phi^T Q_Q \phi + R_R)U_t^* + 2 \phi^T Q_Q Fx(t)=0\end{equation}$$

$$\begin{equation}U_t^* = \frac{-\phi^T Q_Q Fx(t)}{\phi^T Q_Q\phi + R_R}\end{equation}$$

Which is in the form

$$\begin{equation}U_t^* = \begin{bmatrix} u_t^* \\\ u_{t+1}^* \\\ . \\\ . \\\ . \\\ u_{t+N-1}^* \end{bmatrix}\end{equation}$$

Then to obtain only the first control input, we set

$$\begin{equation}u(t) = \begin{bmatrix} 1 & 0 & ... & 0 \end{bmatrix} U_t^* \end{equation}$$

$$\begin{equation}u(t) = -I_N \frac{\phi^T Q_Q F}{\phi^T Q_Q\phi + R_R}x(t)\end{equation}$$

Then define the control law

$$\begin{equation}k_{mpc} = -I_N \frac{\phi^T Q_Q F}{\phi^T Q_Q\phi + R_R}\end{equation}$$

and

$$\begin{equation}u(t) = k_{mpc} x(t)\end{equation}$$

And that concludes our unconstrained MPC for stabilisation, or setting a state to 0

## Unconstrained tracking with QP

Track output to a reference $r(t)$.

We assume the reference is a constant, why? We implement only a single control action in our sequence, so for the time $t_s$ our reference doesn't change.

So we can say $r(t) = r_c$

We want to minimise the difference between the output and our reference, so this time we need to get the next output in terms of our previous ouptut and the changes in state and changes in input. These represent the state and input added from a single timestep.

Thus we will need a new set of equations, starting with state equations that contain a constant unknown disturbance

Fill in here

$$\begin{equation}\Delta x(t) = x(t) - x(t-1)\end{equation}$$

$$\begin{equation}\Delta x(t+1) = x(t+1) - x(t)\end{equation}$$

$$\begin{equation}\Delta u(t) = u(t) - u(t-1)\end{equation}$$

$$\begin{equation}\Delta x(t+1) = A \Delta x(t) + B \Delta u(t)\end{equation}$$

Next we must find the equation for $y(t+1)$ in terms of $y(t)$

$$\begin{equation}y(t) = Cx(t) + Du(t)\end{equation}$$

$$\begin{equation}y(t+1) = Cx(t+1) + Du(t+1)\end{equation}$$

Assume the inputs are equal???? IDK why

$$\begin{equation}\Delta y(t+1) = y(t+1) - y(t)\end{equation}$$

$$\begin{equation}\Delta y(t+1) = C(x(t+1)-x(t))\end{equation}$$

$$\begin{equation}\Delta y(t+1) = C \Delta x(t+1)\end{equation}$$

$$\begin{equation}\Delta y(t+1) = CA \Delta x(t) + CB \Delta u(t)\end{equation}$$

$$\begin{equation}y(t+1) = y(t) + \Delta y(t+1)\end{equation}$$

$$\begin{equation}y(t+1) = y(t) + CA \Delta x(t) + CB \Delta u(t)\end{equation}$$

So

$$\begin{equation}\begin{bmatrix} \Delta x(t+1) \\\ y(t+1) \end{bmatrix}
= \begin{bmatrix} A & 0 \\\ CA & I \end{bmatrix}
\begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix} + 
\begin{bmatrix} B \\\ CB\end{bmatrix} \Delta u(t)\end{equation}$$

And then our new output is 
$$\begin{equation}y(t) = \begin{bmatrix} 0 & I \end{bmatrix}
\begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix}\end{equation}$$

This gives us our augmented state space, so we have

$$\begin{equation}x_a = \begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix}\end{equation}$$

$$\begin{equation}u_a = \Delta u(t)\end{equation}$$

$$\begin{equation}\dot{x}_a = A_a x_a + B_a u_a\end{equation}$$

$$\begin{equation}y_a = C_c x_a\end{equation}$$

Where

$$A_a = \begin{bmatrix} A & 0 \\\ CA & I \end{bmatrix}$$

$$B_a = \begin{bmatrix} B \\\ CB \end{bmatrix}$$

$$C_a = \begin{bmatrix} 0 & I \end{bmatrix}$$

Our next step is to turn this into a QP problem to solve, so we must come up with a quadratic cost function

$$\begin{equation}J(U_t) = \sum_{k=0}^{N-1}(y(t+k)-r_c)^T Q_Q (y(t+k)-r_c) + \Delta u(t+k)^T R_R \Delta u(t+k)\end{equation}$$

So Our optimisation problem is

$$\begin{equation}\min_{\Delta U_t} \sum_{k=0}^{N-1}(y(t+k)-r_c)^T Q_Q (y(t+k)-r_c) + \Delta u(t+k)^T R_R \Delta u(t+k)\end{equation}$$

And then we form our equations for the states

$$\begin{equation}\begin{bmatrix} y_{a,t} \\\ y_{a,t+1} \\\ . \\\ . \\\ . \\\ y_{a,t+N-2} \\\ y_{a,t+N-1} \end{bmatrix} =
\begin{bmatrix} 1 \\\ A_a \\\ . \\\ . \\\ . \\\ A_a^{N-2} \\\ A_a^{N-1} \end{bmatrix} x_a(t) + 
\begin{bmatrix}
0 & 0 & . & . & . & 0 & 0 \\\
B_a & 0 & . & . & . & 0 & 0 \\\
. & . & . & . & . & . &. \\\
. & . & . & . & . & . &. \\\
. & . & . & . & . & . &. \\\
A_a^{N-3}B_a & A_a^{N-4}B_a & . & . & . & 0 & 0 \\\
A_a^{N-2}B_a & A_a^{N-3}B_a & . & . & . & B_a & 0
\end{bmatrix}
\begin{bmatrix} \Delta u_t \\\ \Delta u_t+1 \\\ . \\\ . \\\ . \\\ \Delta u_{t+N-2} \\\ \Delta u_{t+N-1} \end{bmatrix}\end{equation}$$

Obtaining the equations in the form

$$\begin{equation}Y_t = F x_a(t) + \phi \Delta U_t\end{equation}$$

So

$$\begin{equation}J_t = (Y_t-R_c)^T Q_Q (Y_t-R_c) + \Delta U_t^T R_R \Delta U_t\end{equation}$$

$$\begin{equation}J_t = (F x_a(t) + \phi \Delta U_t-R_c)^T Q_Q (F x_a(t) + \phi \Delta U_t-R_c) + \Delta U_t^T R_R \Delta U_t\end{equation}$$

$$\begin{equation}J_t = \Delta U_t^T(\phi^T Q_Q \phi + R_R)\Delta U_t + 2 \Delta U_t^T\phi^T Q_Q (F x_a(t) - R_c) + ...\end{equation}$$

Then take derivative

$$\begin{equation} \frac{\delta J_t}{\delta \Delta U_t} = 2 (\phi^T Q_Q \phi + R_R) \Delta U_t + 2\phi^T Q_Q (F x_a(t) - R_c)\end{equation}$$

Set to 0 and solve

$$\begin{equation} 2 (\phi^T Q_Q \phi + R_R) \Delta U_t + 2\phi^T Q_Q (F x_a(t) - R_c) = 0\end{equation}$$

$$\begin{equation} 2 (\phi^T Q_Q \phi + R_R) \Delta U_t = - 2\phi^T Q_Q (F x_a(t) - R_c)\end{equation}$$

$$\begin{equation} \Delta U_t^* = - (\phi^T Q_Q \phi + R_R)^{-1} \phi^T Q_Q (F x_a(t) - R_c)\end{equation}$$

And thus we have our optimal $\Delta U_t^*$ sequence for our horizon of length N for our tracking problem.

Again we apply our receding horizon, so multiplying by $I_N$ (or with a control horizon size $M$ then $I_M$)

$$\begin{equation} \Delta u_t^* = \begin{bmatrix} \end{bmatrix} \end{equation}$$
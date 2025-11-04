# mpyc
A practical guide to controls engineering for Robotics and AI, building up to an implementation of Model Predictive Control. 

# Control engineering for Robotics and AI

## Introduction to control engineering:
Control engineering is the field of engineering concerned with "controlling" systems. But what does this actually mean? 

The dictionary definition of control is to "determine the behaviour or supervise the running of". In control engineering I think it is more apt to change the definition to "determine the behaviour and supervise the running of". Think of an engineering problem and how you might apply this concept.

This might include making an aeroplane fly. The first step is to determine the aeroplane's behaviour. If an extraterrestrial being has never seen an aeroplane before and is just told to "make it fly", the result will probably not be good. You might end up with the being deconstructing it and throwing the individual pieces, disintegrating it with a laser gun and spreading the dust in front of a fan, or something entirely more absurd.

We humans know that the plane has certain mechanics, or rules for its motion. We know that if we make it move fast enough, it will start to generate lift on its wings, and elevate into the sky. This is an example of a plane's behaviour. Once we have determined this, we can focus on controlling the flight. Without having determined this behaviour, we will not be able to use a logical method to control the flight. We might get lucky with trial and error, but that is not what engineers do. Without determining this relationship between horizontal speed and vertical lift, we might spend entire lifetimes driving our planes at 10mph and wondering when we will fly. (If you look up the history of the aeroplane, you will see why taking the "determining behaviour approach" through mechanics first might save a lot of time and lives)

The next step is supervising the running of the system. In the case of an aeroplane, we want to make sure that when the pilot sets a speed and direction, the plane will maintain that speed.

Sometimes we will encounter disturbances, such as turbulence. A good control system will be __robust__ to disturbances, meaning that they will be able to quickly and smoothly recover from or reject them.

# Control Systems

That might mean controlling the speed of a car to a desired setpoint, the orientation of a satellite in the direction of the Earth, etc, etc.

Take the example of a car's cruise control. How can we make a cruise control that keeps the car's speed at our desired setpoint? 

First let us define our system. The car's speed, what we want to control, is our __output__. This is the main value we care about.

To change the __output__ directly, we have to control an __input__ (or set of inputs). In our case, this will be the engine's throttle. Open it more, and more fuel or air enters the engine, making the engine work harder and eventually driving the car to move faster. Close the throttle, and less fuel and air get to mix and combust, and we end up with a slower speed.

So in this case our __input__ might be the "openness" of our throttle.

Then we have to consider the current __state__ of the system. What is our actual speed right now? The car's speed therefore is one of our states. But in order to see this state, we need to either measure or estimate this. We use a __sensor__ (or __observer__), or a set of sensors for this. In our case, this is the car's speedometer.

You might notice that I mentioned speed being our output and a state. This is because the output is defined as a linear combination of our states and inputs. The states are the core values of the system, and our output tends to be the states we care to control, or some combination of the states we care to control.

Finally, we need a __controller__. This controller will be what decides how much to change our input, to reach our desired output. In our case, the car computer will decide how much to open or close the engine throttle to maintain our desired speed.

The controller works by finding out how far away from the setpoint our current output is. Let us call the setpoint our __reference__ from now on. There is a lot of nomenclature in control, sorry. It's best to get used to using these words now before you end up all confused in a conference of control engineers discussing MPCs and PIDs and SSE.



# Generic control strategies

## Bang bang

## PID

# State Space Modelling

## Discretisation

### Euler discretisation
$$\dot{x}(t) = A_cx(t)+B_cu(t)$$

$$y(t) = C_cx(t) + D_cu(t)$$

We would like to discretise this system using the Euler discretisation. The euler discretisation uses a forward divided difference and so it is computationally cheap, although it requires <assumptions> to be accurate enough

Assume that we only have an integer timestep so we have $t_s$ as our timestep length and $k$ as our integer multiplier

$$\begin{equation}\dot{x}(kt_s) = A_cx(kt_s) + B_cu(kt_s)\end{equation}$$

$$\begin{equation}\dot{x}(kt_s) \approx \frac{x((k+1)t_s) - x(kt_s)}{t_s}\end{equation}$$

$$\begin{equation}\frac{x((k+1)t_s) - x(kt_s)}{t_s} = A_cx(kt_s) + B_cu(kt_s)\end{equation}$$

$$\begin{equation}x((k+1)t_s) - x(kt_s) = t_sA_cx(kt_s) + t_sB_cu(kt_s)\end{equation}$$

$$\begin{equation}x((k+1)t_s) = x(kt_s) + t_sA_cx(kt_s) + t_sB_cu(kt_s)\end{equation}$$

$$\begin{equation}x((k+1)t_s) = (I+t_sA_c)x(kt_s) + t_sB_cu(kt_s)\end{equation}$$

$$\begin{equation}x(k+1) = Ax(k) + Bu(k)\end{equation}$$

$$\begin{equation}y(k) = C(k) + D(k)\end{equation}$$
Where

$$A = I+t_sA_c$$

$$B = t_sB_c$$

$$C = C_c$$

$$D = D_c$$

Giving us the discretised system for a state space description

### Exact discretisation
Take the state space description

$$\begin{equation}\dot{x}(t) = A_cx(t)+B_cu(t)\end{equation}$$

$$\begin{equation}y(t) = C_cx(t) + D_cu(t)\end{equation}$$

Let us find the general solution to a state space system, which is the standard steps for solving homogeneous linear first order ODE

$$\begin{equation}\dot{x}(t) = ax(t)+bu(t)\end{equation}$$

Multiply by integrating factor

$$\begin{equation}e^{-at}\dot{x}(t) = e^{-at}ax(t) + e^{-at}bu(t)\end{equation}$$

Rearrange to have all x(t) terms on one side

$$\begin{equation}-ae^{-at}x(t) + e^{-at}\dot{x}(t) = e^{-at}bu(t)\end{equation}$$

Through product rule

$$\begin{equation}\frac{d}{dt}(e^{-at}x(t)) = (\frac{d}{dt}e^{-at})x(t) + e^{-at}(\frac{d}{dt}x(t))\end{equation}$$

$$\begin{equation}\frac{d}{dt}(e^{-at}x(t)) = -ae^{-at}x(t) + e^{-at}\dot{x}(t)\end{equation}$$

Substitute in this equality

$$\begin{equation}\frac{d}{dt}(e^{-at}x(t)) = e^{-at}bu(t)\end{equation}$$

Then integrate both sides, from time $t$ to time $t_0$. We introduce a new variable $\tau$ for this

$$\begin{equation}e^{-at}x(t) - e^{-at_0}x(t_0) = \int_{t_0}^te^{-a\tau}bu(\tau)d\tau\end{equation}$$

Rearrange

$$\begin{equation}e^{-at}x(t) = e^{-at_0}x(t_0) + \int_{t_0}^te^{-a\tau}bu(\tau)d\tau\end{equation}$$

Divide through by $e^{-at}$ (as this cannot be 0)

$$\begin{equation}x(t) = e^{a(t-t_0)}x(t_0) + \int_{t_0}^te^{a(t-\tau)}bu(\tau)d\tau\end{equation}$$

Now substitute our matrix values in again for a standard response

$$\begin{equation}x(t) = e^{A_c(t-t_0)}x(t_0) + \int_{t_0}^te^{A_c(t-\tau)}B_cu(\tau)d\tau\end{equation}$$

Now we have the standard format, we will use this to write our solutions for our discrete timesteps $x(kt_s)$ and $x((k+1)t_s)$, and setting $t_0 = 0$

$$\begin{equation}x((k+1)t_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

$$\begin{equation}x(kt_s) = e^{A_ckt_s}x(0) + \int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

Then as

$$\begin{equation}e^{A_ckt_s} \cdot e^{A_ct_s} = e^{A_c(k+1)t_s}\end{equation}$$

We multiply both sides by $e^{A_ct_s}$

$$\begin{equation}e^{A_ct_s}x(kt_s) = e^{A_ct_s}e^{A_ckt_s}x(0) + e^{A_ct_s}\int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

Simplify

$$\begin{equation}e^{A_ct_s}x(kt_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

Then substitute

$$\begin{equation}e^{A_c(k+1)t_s}x(0) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

$$\begin{equation}e^{A_ct_s}x(kt_s) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

Rearrange

$$\begin{equation}x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau - \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau\end{equation}$$

By rule of integral linearity

$$\begin{equation}x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau\end{equation}$$

Then because of our discretisation, in the step between $kt_s$ and $(k+1)t_s$ $u(\tau)$ must be constant so we take it out of the integral

$$\begin{equation}x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}d\tau\end{equation}$$

Then take new variable $\lambda = (k+1)t_s - \tau$, (so we flip the sign of the integral as $\lambda$ is related to $-\tau$)

$$\begin{equation}x((k+1)t_s) = e^{A_ct_s}x(kt_s) - B_cu(kt_s)\int_{t_s}^{0}e^{A_c\lambda}d\lambda\end{equation}$$

Then we can restate this as $\tau = -\lambda$. Yes I know I'm redefining and this is mathematically incorrect, but I am an engineer and I already used tau but in the powerpoint slide for the final equation I wanted, my professor used tau :)

$$\begin{equation}x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{0}^{t_s}e^{A_c\tau}d\tau\end{equation}$$

Finally we will redefine our equation as

$$\begin{equation}x((k+1)t_s) = Ax(kt_s) + Bu(kt_s)\end{equation}$$

Where

$$A = e^{A_ct_s}$$

$$B = B_c\int_{0}^{t_s}e^{A_c\tau}d\tau$$

The output equations always remain the same so

$$y(kt_s) = Cx(kt_s) + Du(kt_s)$$

Where

$$C = C_c$$

$$D = D_c$$

As python does not support integration that returns a matrix, we instead create a new matrix

$$\begin{equation}M = \begin{bmatrix} A_c & B_c \\\ 0 & 0 \end{bmatrix}\end{equation}$$

Then by a long taylor expansion of the matrix exponential

$$\begin{equation}e^{Mt_s} = \begin{bmatrix} e^{A_ct_s} & \int_{0}^{t_s}e^{A_c\tau}d\tau \\\ 0 & I \end{bmatrix}\end{equation}$$

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

$$\begin{equation} x(t+1) = Ax(t) + Bu(t) + B_d d_c \end{equation}$$

$$\begin{equation} x(t) = Ax(t-1) + Bu(t-1) + B_d d_c \end{equation}$$

$(1)$ - $(2)$

$$\begin{equation} \Delta x(t+1) = A(x(t) - x(t-1)) + B(u(t) - u(t-1)) \end{equation}$$

Removing effect of disturbance

$$\begin{equation} \Delta x(t) = x(t) - x(t-1) \end{equation}$$

$$\begin{equation}\Delta x(t+1) = x(t+1) - x(t)\end{equation}$$

$$\begin{equation}\Delta u(t) = u(t) - u(t-1)\end{equation}$$

Substitute in to (3)

$$\begin{equation}\Delta x(t+1) = A \Delta x(t) + B \Delta u(t)\end{equation}$$

Next we must find the equation for $y(t+1)$ in terms of $y(t)$

$$\begin{equation}y(t) = Cx(t) + Du(t)\end{equation}$$

$$\begin{equation}y(t+1) = Cx(t+1) + Du(t+1)\end{equation}$$

Subtract $(9)$ - $(8)$

$$\begin{equation}\Delta y(t+1) = y(t+1) - y(t)\end{equation}$$

$$\begin{equation}\Delta y(t+1) = C(x(t+1)-x(t)) + D(u(t+1)-u(t))\end{equation}$$

$$\begin{equation} \Delta y(t+1) = C\Delta x(t+1) + D\Delta u(t+1) \end{equation}$$

Then

$$\begin{equation} \Delta y(t+1) = CA \Delta x(t) + CB \Delta u(t) + D\Delta u(t+1) \end{equation}$$


$$\begin{equation}y(t+1) = y(t) + \Delta y(t+1)\end{equation}$$

$$\begin{equation}y(t+1) = y(t) + CA \Delta x(t) + CB \Delta u(t) + D\Delta u(t+1)\end{equation}$$

$$\begin{equation}\Delta y(t+1) = C(x(t+1)-x(t))\end{equation}$$

$$\begin{equation}\Delta y(t+1) = C \Delta x(t+1)\end{equation}$$

$$\begin{equation}\Delta y(t+1) = CA \Delta x(t) + CB \Delta u(t)\end{equation}$$

$$\begin{equation}y(t+1) = y(t) + \Delta y(t+1)\end{equation}$$

$$\begin{equation}y(t+1) = y(t) + CA \Delta x(t) + CB \Delta u(t)\end{equation}$$

So

$$\begin{equation}\begin{bmatrix} \Delta x(t+1) \\\ y(t+1) \end{bmatrix}
= \begin{bmatrix} A & 0 \\\ CA & I \end{bmatrix}
\begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix} + 
\begin{bmatrix} B & 0 \\\ CB & D \end{bmatrix} \begin{bmatrix} \Delta u(t) \\\ \Delta u(t+1) \end{bmatrix}\end{equation}$$

And then our new output is 
$$\begin{equation}y_a(t) = \begin{bmatrix} 0 & I \end{bmatrix}
\begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix}\end{equation}$$

This gives us our augmented state space, so we have

$$\begin{equation}x_a = \begin{bmatrix} \Delta x(t) \\\ y(t) \end{bmatrix}\end{equation}$$

$$\begin{equation}u_a = \Delta u(t)\end{equation}$$

$$\begin{equation}\dot{x}_a = A_a x_a + B_a u_a\end{equation}$$

$$\begin{equation}y_a = C_c x_a\end{equation}$$

Where

$$A_a = \begin{bmatrix} A & 0 \\\ CA & I \end{bmatrix}$$

$$B_a = \begin{bmatrix} B & 0 \\\ CB & D \end{bmatrix}$$

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
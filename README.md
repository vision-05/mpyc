# mpyc
MPC module from scratch in Python with numpy, an application of my control 2 module

# Euler discretisation
$\dot{x}(t) = A_cx(t)+B_cu(t)\\$
$y(t) = C_cx(t) + D_cu(t)$

We would like to discretise this system using the Euler discretisation. The euler discretisation uses a forward divided difference and so it is computationally cheap, although it requires <assumptions> to be accurate enough

Assume that we only have an integer timestep so we have $t_s$ as our timestep length and $k$ as our integer multiplier

$\dot{x}(kt_s) = A_cx(kt_s) + B_cu(kt_s)\\$
$\dot{x}(kt_s) \approx \frac{x((k+1)t_s) - x(kt_s)}{t_s}\\$
$\frac{x((k+1)t_s) - x(kt_s)}{t_s} = A_cx(kt_s) + B_cu(kt_s)\\$
$x((k+1)t_s) - x(kt_s) = t_sA_cx(kt_s) + t_sB_cu(kt_s)\\$
$x((k+1)t_s) = x(kt_s) + t_sA_cx(kt_s) + t_sB_cu(kt_s)\\$
$x((k+1)t_s) = (I+t_sA_c)x(kt_s) + t_sB_cu(kt_s)\\$
$x(k+1) = Ax(k) + Bu(k)\\$
$y(k) = C(k) + D(k)$
Where

$A = I+t_sA_c\\$
$B = t_sB_c\\$
$C = C_c\\$
$D = D_c$

Giving us the discretised system for a state space description

# Exact discretisation
Take the state space description

$\dot{x}(t) = A_cx(t)+B_cu(t)\\$
$y(t) = C_cx(t) + D_cu(t)$

Let us find the general solution to a state space system, which is the standard steps for solving homogeneous linear first order ODE

$\dot{x}(t) = ax(t)+bu(t)\\$

Multiply by integrating factor

$e^{-at}\dot{x}(t) = e^{-at}ax(t) + e^{-at}bu(t)$

Rearrange to have all x(t) terms on one side

$-ae^{-at}x(t) + e^{-at}\dot{x}(t) = e^{-at}bu(t)$

Through product rule

$\frac{d}{dt}(e^{-at}x(t)) = (\frac{d}{dt}e^{-at})x(t) + e^{-at}(\frac{d}{dt}x(t))\\$
$\frac{d}{dt}(e^{-at}x(t)) = -ae^{-at}x(t) + e^{-at}\dot{x}(t)$

Substitute in this equality

$\frac{d}{dt}(e^{-at}x(t)) = e^{-at}bu(t)$

Then integrate both sides, from time $t$ to time $t_0$. We introduce a new variable $\tau$ for this

$e^{-at}x(t) - e^{-at_0}x(t_0) = \int_{t_0}^te^{-a\tau}bu(\tau)d\tau$

Rearrange

$e^{-at}x(t) = e^{-at_0}x(t_0) + \int_{t_0}^te^{-a\tau}bu(\tau)d\tau$

Divide through by $e^{-at}$ (as this cannot be 0)

$x(t) = e^{a(t-t_0)}x(t_0) + \int_{t_0}^te^{a(t-\tau)}bu(\tau)d\tau$

Now substitute our matrix values in again for a standard response

$x(t) = e^{A_c(t-t_0)}x(t_0) + \int_{t_0}^te^{A_c(t-\tau)}B_cu(\tau)d\tau$

Now we have the standard format, we will use this to write our solutions for our discrete timesteps $x(kt_s)$ and $x((k+1)t_s)$, and setting $t_0 = 0$

$x((k+1)t_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$

$x(kt_s) = e^{A_ckt_s}x(0) + \int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau$

Then as

$e^{A_ckt_s} \cdot e^{A_ct_s} = e^{A_c(k+1)t_s}$

We multiply both sides by $e^{A_ct_s}$

$e^{A_ct_s}x(kt_s) = e^{A_ct_s}e^{A_ckt_s}x(0) + e^{A_ct_s}\int_{0}^{kt_s}e^{A_c(kt_s-\tau)}B_cu(\tau)d\tau$

Simplify

$e^{A_ct_s}x(kt_s) = e^{A_c(k+1)t_s}x(0) + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$

Then substitute

$e^{A_c(k+1)t_s}x(0) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$

$e^{A_ct_s}x(kt_s) = x((k+1)t_s) - \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau + \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$

Rearrange

$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{0}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau - \int_{0}^{kt_s}e^{A_c((k+1)t_s-\tau)}Bu(\tau)d\tau$

By rule of integral linearity

$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + \int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}B_cu(\tau)d\tau$

Then because of our discretisation, in the step between $kt_s$ and $(k+1)t_s$ $u(\tau)$ must be constant so we take it out of the integral

$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{kt_s}^{(k+1)t_s}e^{A_c((k+1)t_s-\tau)}d\tau$

Then take new variable $\lambda = (k+1)t_s - \tau$, (so we flip the sign of the integral as $\lambda$ is related to $-\tau$)

$x((k+1)t_s) = e^{A_ct_s}x(kt_s) - B_cu(kt_s)\int_{t_s}^{0}e^{A_c\lambda}d\lambda$

Then we can restate this as $\tau = -\lambda$. Yes I know I'm redefining and this is mathematically incorrect, but I am an engineer and I already used tau but in the powerpoint slide for the final equation I wanted, my professor used tau :)

$x((k+1)t_s) = e^{A_ct_s}x(kt_s) + B_cu(kt_s)\int_{0}^{t_s}e^{A_c\tau}d\tau$

Finally we will redefine our equation as

$x((k+1)t_s) = Ax(kt_s) + Bu(kt_s)$

Where

$A = e^{A_ct_s}\\$
$B = B_c\int_{0}^{t_s}e^{A_c\tau}d\tau$

The output equations always remain the same so

$y(kt_s) = Cx(kt_s) + Du(kt_s)$

Where

$C = C_c\\$
$D = D_c$
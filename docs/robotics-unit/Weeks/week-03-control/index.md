# Week 3 Robot control

Last week we talked about **how to model our robot**. This week we ask, how do we actually make it move?

More specifically:

> how do we choose the control inputs $u$ that make the robot do what we want?


## Open loop control

Let’s start with the simplest idea. We compute a sequence of controls ahead of time:

$$
u_0, u_1, u_2, \dots
$$

and then just execute them. No checking. No correction. Just vibes. This is *open loop control*. This might work if the world is perfectly predictable and nothing disturbs the system. Then we can plan once and execute.

In fact, this is basically what a robot *planner* gives us, a trajectory and corresponding actions to follow.

**Why does this fail?** Because the real world exists. Wheels slip, sensors are noisy, models are wrong, humans do unpredictable things. So what actually happens is:
> small errors → become big errors → robot ends up somewhere random

## Feedback control loops

So instead of blindly executing commands, we do something smarter. We measure what actually happened, and correct for it. This gives us a **feedback loop**:

Assume we have some desired state  or reference $x_d$ and the actual state $x$, we compute error, apply control $u$ and repeat until $x_d \rightarrow x$. If $x_d$ is a point, we call this *setpoint* control or regulation, if $x_d$ is a trajectory, we call this trajectory tracking.

![A feedback control loop. Given a goal and an measured output, a controller computes a control to send to a plant which produces an output, that we sense and compare to our goal again.](/img/week-02/feedbackcontrol.png)


## PD control

The most common starting point is **Proportional-Derivative (PD) control**.

We define an error:

$$
e(t) = x_d(t) - x(t)
$$

and then choose control as:

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$

The
- **Proportional term**:  pushes toward the goal, bigger error → bigger correction. 
- **Derivative term**:  damps motion by penalising the rate of change, prevents overshooting  

![PD control of robot position to a goal. PD results in a smooth response, while P only can oscillate or become unstable.](/img/week-02/stepresponse.png)


So PD is basically go toward the goal, but don’t be too aggressive. We *tune* a controller by selecting the *gains* to get the best response for a given application.

---

### Example: driving to a goal

For a simple robot:

- state: position $(x, y)$, orientation $\theta$
- control: velocity $v$, angular velocity $\omega$

We can:

1. compute the angle to the goal  
2. compute the distance to the goal  
3. apply proportional control

$$
\omega_k = K_{p\theta} (\theta_g - \theta_k)
$$

$$
v_k = K_{p} \sqrt{(x_g - x_k)^2 + (y_g - y_k)^2}
$$

This is a surprisingly effective approach. PD is simple, fast and works surprisingly well. But, it has no notion of optimality, requires tuning gain, struggles with complex systems where there is coupling between dimensions, and does not explicitly handle constraints (eg. follow this trajectory while maintaining a safe level of torque.).

---

## LQR (Linear Quadratic Regulator)

Now we step up a level. Instead of saying push toward the goal, we say choose controls that **minimise a cost function**. We have already seen this formulation in the unit overview. Lets assume our robot is a linear system:

$$
x_{k+1} = A x_k + B u_k
$$

We define a cost:
$$
J = \sum_{k=0}^{\infty} \left[(x_k - x_g)^T Q (x_k - x_g) + u_k^T R u_k \right]
$$


### What does this mean?

We are penalising:

- state error → $Q$  
- control effort → $R$

So we are balancing getting to the goal vs don’t use ridiculous control inputs. The easiest way to interpret this is to imagine $Q$ and $R$ are identity. Then we are basically penalising Euclidean distance to a desired state and the Euclidean norm of the control effort we use. This is a *convex* optimisation problem

We can prove that the optimal controller for a system like this takes the form:
$$
u_k = -K(x_k - x_g)
$$

So it still looks like feedback. But now the gains $K$ come from solving an optimisation problem called the algebraic Ricatti equation. The main difference is that in PD control we picked gains and hoped for good behaviour, while LQR defines an objective and then derives optimal gains.

> Kalman, R. E. (1960). "Contributions to the Theory of Optimal Control." *Boletin de la Sociedad Matematica Mexicana*, 5(2), 102–119.  


## Iterative LQR (iLQR)

So LQR is great, but only works for **linear systems**. Robots are almost never linear, but maybe they are locally linear. For a nonlinear system:
$$
x_{k+1} = f(x_k, u_k)
$$

1. start with a guess for controls  
2. simulate the trajectory  
3. locally approximate the system as linear  
4. solve an LQR  
5. update controls  
6. repeat  

This is **iterative LQR (iLQR)**. It has some limitations, it depends on your initial guess, can get stuck in local minima because the original problem is now potentially non-convex, and can be slow, so often computed offline. We may still need a PD control loop to handle disturbances.

> Li, W. and Todorov, E. (2004). "Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems." In *Proceedings of the 1st International Conference on Informatics in Control, Automation and Robotics (ICINCO)*, pp. 222–229.

## MPC (Model Predictive Control)

Now we take one more step. Instead of solving once and committing, we solve → act → re-solve → act → repeat. At each timestep we will

1. plan over a short horizon  
2. compute optimal controls  
3. execute only the first control  
4. shift horizon forward  
5. repeat  

This is **Model Predictive Control (MPC)**. This is powerful because it naturally handles disturbances, adapts to new information and can incorporate constraints, but it's more computationally expensive, requires solving optimisation problems repeatedly. Most robot control systems are some form of MPC under the hood.

> Mayne, D. Q., Rawlings, J. B., Rao, C. V., and Scokaert, P. O. M. (2000). "Constrained model predictive control: Stability and optimality." *Automatica*, 36(6), 789–814. 

## Key Papers

> Kalman, R. E. (1960). "Contributions to the Theory of Optimal Control." *Boletin de la Sociedad Matematica Mexicana*, 5(2), 102–119.

> Li, W. and Todorov, E. (2004). "Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems." In *Proceedings of ICINCO*, pp. 222–229.

> Mayne, D. Q., Rawlings, J. B., Rao, C. V., and Scokaert, P. O. M. (2000). "Constrained model predictive control: Stability and optimality." *Automatica*, 36(6), 789–814.

# Coming up next

We have been assuming we can measure the state of the robot perfectly. Next, we look at what happens when we can't.

→ [Week 4: State Estimation](../week-04-state-estimation/)

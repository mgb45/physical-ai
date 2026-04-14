# Week 8: Articulated robots and dynamics

Last week we covered [kinematics](../week-07-kinematics/): the geometry of how joint angles map to end-effector poses. We knew where the arm was, and we knew how to get the tip to a target. But we were ignoring something important.

> what actually makes the arm move?

Kinematics tells you *where* you can go. Dynamics tells you *how* you get there — and what it costs in terms of forces and torques. To command a real robot arm, you need to think about **mass**, **inertia**, and **forces**.

This week we add the physics.


## The manipulator equations

Every robot arm obeys Newton's laws. We can derive the equations of motion for an $n$-DOF arm by applying the Euler-Lagrange equations to the system's kinetic and potential energy. The result is a compact and elegant expression:

$$
M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}
$$

These are the **manipulator equations** — also called the **equations of motion** of the robot.

Let's unpack each term:

| Term | Name | What it captures |
|------|------|-----------------|
| $M(\mathbf{q})$ | **mass matrix** | inertial resistance to acceleration; depends on the current configuration |
| $C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ | **Coriolis and centripetal** | forces that arise from the robot already being in motion |
| $\mathbf{g}(\mathbf{q})$ | **gravity vector** | torques needed just to hold the arm up against gravity |
| $\boldsymbol{\tau}$ | **joint torques** | what we actually command from the actuators |

This is the robot's version of $F = ma$.

The mass matrix $M(\mathbf{q})$ is **configuration-dependent**: the same arm is harder to move when its links are stretched out than when they are folded in. This is not something you get from kinematics — it requires knowing the mass and inertia of every link.

The Coriolis and centripetal term $C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ captures the forces that appear when joints are moving. These terms couple the joints: moving one joint can induce forces at others. At low speeds they are negligible, but at high speeds they become significant.

The gravity vector $\mathbf{g}(\mathbf{q})$ is the torque each joint must exert just to hold the arm in place. Even a stationary arm requires non-zero torques if it is not perfectly balanced.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.


## Forward and inverse dynamics

Just as we had forward and inverse kinematics, we have two directions for the manipulator equations.

**Forward dynamics**: given the joint torques, what acceleration results?

$$
\ddot{\mathbf{q}} = M(\mathbf{q})^{-1} \left[ \boldsymbol{\tau} - C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) \right]
$$

This is what a simulator uses. You command torques, and you integrate forward to get the motion. It is the robot equivalent of simulating a ball rolling under gravity.

**Inverse dynamics**: given a desired acceleration, what torques do we need?

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})
$$

This is what a controller uses. Given a trajectory — a sequence of desired positions, velocities, and accelerations — compute the torques required to follow it. This is the direction that matters for real hardware.

The recursive Newton-Euler algorithm provides an efficient way to compute inverse dynamics in $O(n)$ time, which makes real-time control feasible.

> Luh, J. Y. S., Walker, M. W., and Paul, R. P. C. (1980). "On-line computational scheme for mechanical manipulators." *Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69–76.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.


## Inverse dynamics control

Recall from [Week 3](../week-03-control/) that feedback control tries to correct errors. But for a robot arm, simple PID control on joint angles has to fight the coupled dynamics — inertia, Coriolis forces, gravity — without any model of them. The result is imprecise and sluggish.

A much better approach is to use our model of the dynamics directly.

**Computed torque control** (also called **inverse dynamics control**) uses the manipulator equations to cancel out the arm's dynamics:

$$
\boldsymbol{\tau} = M(\mathbf{q}) \mathbf{a}_d + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})
$$

where $\mathbf{a}_d$ is a desired acceleration. If our model is perfect, this reduces the closed-loop system to:

$$
\ddot{\mathbf{q}} = \mathbf{a}_d
$$

a set of decoupled, linear double integrators. We have exactly cancelled out all the nonlinear dynamics. Now a simple PD controller on each joint works independently and perfectly.

In practice we set:

$$
\mathbf{a}_d = \ddot{\mathbf{q}}_d + K_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p (\mathbf{q}_d - \mathbf{q})
$$

where $\mathbf{q}_d$ is the desired trajectory. The gains $K_p$ and $K_d$ can now be tuned on a linear system — something we know how to do well.

> Sciavicco, L. and Siciliano, B. (2000). *Modelling and Control of Robot Manipulators* (2nd ed.). Springer.

The key assumption is: **our dynamics model is accurate**. If it is not, the cancellation is incomplete and we are left with residual nonlinear terms. This brings us to the central practical problem.


## System identification: the dynamics as an estimation problem

Computed torque control requires $M(\mathbf{q})$, $C(\mathbf{q}, \dot{\mathbf{q}})$, and $\mathbf{g}(\mathbf{q})$. These depend on the physical parameters of every link: mass, centre of mass, and inertia tensor. For a real robot, we cannot derive these from the CAD model alone — manufacturing tolerances, cables, and end-effectors all shift the real values away from the nominal design.

**System identification** is the problem of estimating these dynamic parameters from measurements.

The key observation: the manipulator equations are **linear in the dynamic parameters**. We can rearrange them as:

$$
\boldsymbol{\tau} = Y(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}) \, \boldsymbol{\phi}
$$

where $Y$ is a matrix of known functions (called the **regressor**) and $\boldsymbol{\phi}$ is a vector of the unknown physical parameters (masses, inertias, centres of mass). We measure torques and joint motion on the real robot, build up many rows of this equation, and then solve for $\boldsymbol{\phi}$.

This is a **least squares estimation problem** — exactly the class of problems we studied in [Week 4](../week-04-state-estimation/). We have a linear model, noisy observations, and we want the best estimate of unknown parameters. The same Bayesian perspective applies: if we have a prior belief about the parameters (say, from the CAD model), we can regularise the estimate accordingly.

The quality of the estimate depends on the **excitation** of the trajectory used for identification. A trajectory that only moves one joint, or moves slowly, will not excite all the relevant dynamics and will give a poor estimate of some parameters. Designing **persistently exciting trajectories** — motions that cover the full dynamic range — is an important part of system identification.

> Khosla, P. and Kanade, T. (1985). "Parameter identification of robot dynamics." In *Proceedings of the 24th IEEE Conference on Decision and Control*, pp. 1754–1760.

> Gautier, M. and Khalil, W. (1990). "Direct calculation of minimum set of inertial parameters of serial robots." *IEEE Transactions on Robotics and Automation*, 6(3), 368–373.


## Robot calibration: geometry as an estimation problem

System identification estimates the dynamic parameters. But there is a parallel problem for the geometric parameters: the DH parameters we wrote down in [Week 7](../week-07-kinematics/) are also never exactly right.

The nominal DH parameters come from the robot's design drawings. But real manufacturing introduces small errors in link lengths, joint offsets, and twist angles. These errors accumulate along the chain and can produce end-effector position errors of several millimetres — unacceptable for precision tasks.

**Robot calibration** is the problem of estimating the true geometric parameters from measurements. A common approach is to measure the actual end-effector position (using a tracking system or a fixed reference point) at many different joint configurations, and then minimise the error between the predicted position (from the nominal DH model) and the measured position.

This is again an **estimation problem**, exactly as in [Week 4](../week-04-state-estimation/): we have a parametric model (the kinematic chain), measurements (end-effector positions), and we want to find the parameter values that best explain the data. The model from [Week 2](../week-02-modelling/) gives us the structure; the estimation machinery from Week 4 gives us the tools.

> Hollerbach, J., Khalil, W., and Gautier, M. (2016). "Model identification." In Siciliano, B. et al. (Eds.), *Springer Handbook of Robotics* (2nd ed.), pp. 113–138. Springer.

The connection is worth emphasising:

> robot modelling ([Week 2](../week-02-modelling/)) tells you the structure of the equations  
> state estimation ([Week 4](../week-04-state-estimation/)) gives you the tools to fit them to reality

System identification and robot calibration are both instances of this pattern. You write down a model with unknown parameters, collect data, and estimate the parameters. The physics changes, but the estimation framework is the same.

## Contact modelling

Everything so far has assumed the robot moves freely through space. But robots exist to interact with the world — to grasp objects, push surfaces, walk on floors. The moment a robot makes or breaks contact, the dynamics change fundamentally.

Contact introduces three things that free-space dynamics does not have:

- **Normal forces**: a surface pushes back when penetrated
- **Friction forces**: tangential forces that resist sliding
- **Unilateral constraints**: contact can only push, never pull

The simplest model is **rigid body contact with Coulomb friction**. Two conditions must hold simultaneously at each contact point:

$$
f_n \geq 0, \quad \delta \geq 0, \quad f_n \delta = 0
$$

where $f_n$ is the normal contact force and $\delta$ is the gap between surfaces. This is a **complementarity condition**: either the surfaces are touching ($\delta = 0$) with a positive normal force, or they are separated ($\delta > 0$) with no force. Never both at once.

Friction is governed by the **Coulomb friction cone**:

$$
\| \mathbf{f}_t \| \leq \mu f_n
$$

where $\mathbf{f}_t$ is the tangential friction force and $\mu$ is the coefficient of friction. When the contact is sticking, the friction force lies strictly inside the cone; when it is sliding, the force is at the boundary and opposes motion.

Combining the manipulator equations with contact forces and these constraints gives a **Linear Complementarity Problem (LCP)** or a **Quadratic Program** at each timestep. These are more expensive to solve than free-space dynamics, but remain tractable.

> Stewart, D. E. and Trinkle, J. C. (1996). "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and Coulomb friction." *International Journal for Numerical Methods in Engineering*, 39(15), 2673–2691.

> Posa, M., Cantu, C., and Tedrake, R. (2014). "A direct method for trajectory optimisation of rigid bodies through contact." *The International Journal of Robotics Research*, 33(1), 69–81.


### Contact in simulation

Real physics engines — MuJoCo, PyBullet, Isaac Gym — handle contact differently, making different tradeoffs between accuracy and speed.

**Rigid contact** enforces complementarity exactly. It is physically correct but numerically challenging: rigid impacts produce instantaneous impulses, and the solver must handle discontinuities carefully.

**Soft contact** is the common engineering approximation. Instead of enforcing non-penetration exactly, small penetrations are permitted and contact forces are modelled as a spring-damper:

$$
f_n = k \max(0, -\delta) + b \max(0, -\dot{\delta})
$$

where $k$ is contact stiffness and $b$ is damping. This is smooth and differentiable, which makes numerical integration stable and fast.

MuJoCo uses a variant of this soft contact approach. It trades exact rigid-body semantics for numerical robustness and speed, which is why it has become the default simulator for training reinforcement learning policies.

> Todorov, E., Erez, T., and Tassa, Y. (2012). "MuJoCo: A physics engine for model-based control." In *Proceedings of IROS*, pp. 5026–5033.


### The sim-to-real gap starts here

The gap between simulation and reality is not only about inertial parameters. It is, in large part, about contact. A policy trained in simulation experiences one contact model; the real robot experiences the true physics of surfaces, materials, and deformable objects.

Small differences in friction coefficient, contact stiffness, or collision geometry propagate into large differences in robot behaviour. This is the **sim-to-real gap** that will be central to [Week 9](../week-09-rl/), and it is one of the hardest unsolved problems in applying learning to real robots.

One practical mitigation is **domain randomisation**: during training, randomise the contact parameters — friction, stiffness, damping — so the policy learns to handle a range of contact conditions rather than one specific simulation. Policies trained this way generalise better to the real world, though the fundamental challenge of approximate contact models remains.

> Andrychowicz, M., Baker, B., Chociej, M., et al. (2020). "Learning dexterous in-hand manipulation." *The International Journal of Robotics Research*, 39(1), 3–20.

---

## Final thought

Dynamics connects kinematics to the real world.

Kinematics tells you what configurations are reachable and how joints map to end-effector pose. Dynamics tells you what forces are required to achieve them — and gives you the tools to design controllers that actually work on physical hardware.

The manipulator equations are the heart of it:

> $M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$

With this model, and with accurately estimated parameters, you can design controllers that compensate for everything the arm is doing — gravity, inertia, coupling — and achieve precise, fast, repeatable motion.

Without it, you are fighting the physics without understanding it.

---

## Key Papers

> Luh, J. Y. S., Walker, M. W., and Paul, R. P. C. (1980). "On-line computational scheme for mechanical manipulators." *Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69–76.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

> Khosla, P. and Kanade, T. (1985). "Parameter identification of robot dynamics." In *Proceedings of the 24th IEEE Conference on Decision and Control*, pp. 1754–1760.

> Gautier, M. and Khalil, W. (1990). "Direct calculation of minimum set of inertial parameters of serial robots." *IEEE Transactions on Robotics and Automation*, 6(3), 368–373.

> Hollerbach, J., Khalil, W., and Gautier, M. (2016). "Model identification." In Siciliano, B. et al. (Eds.), *Springer Handbook of Robotics* (2nd ed.), pp. 113–138. Springer.

> Stewart, D. E. and Trinkle, J. C. (1996). "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and Coulomb friction." *International Journal for Numerical Methods in Engineering*, 39(15), 2673–2691.

> Todorov, E., Erez, T., and Tassa, Y. (2012). "MuJoCo: A physics engine for model-based control." In *Proceedings of IROS*, pp. 5026–5033.

> Posa, M., Cantu, C., and Tedrake, R. (2014). "A direct method for trajectory optimisation of rigid bodies through contact." *The International Journal of Robotics Research*, 33(1), 69–81.

> Andrychowicz, M., Baker, B., Chociej, M., et al. (2020). "Learning dexterous in-hand manipulation." *The International Journal of Robotics Research*, 39(1), 3–20.

---

# Coming up next

We have now covered two broad approaches to making robots behave: design a model of the physics and invert it, or design a controller and tune it.

But what if we don't have a model, and tuning by hand is impractical?

→ [Week 9: Reinforcement learning](../week-09-rl/)

# Chapter 7: Articulated robots and dynamics

Last chapter we covered [kinematics](../chapter-06-kinematics/index.md): the geometry of how joint angles map to end-effector poses. Kinematics tells us where the robot is and where the end-effector can go, but it deliberately ignores an important question:

> what actually makes the arm move?

Dynamics answers that question. It tells us how motion changes in response to forces and torques, and what it costs to move a real robot with mass, inertia, gravity and contact. In Chapter 2, we introduced motion models in a fairly abstract way. This chapter, we return to robot modelling and add the physics needed to command articulated robots.

## The manipulator equations

Every robot arm obeys Newton's laws. For an $n$-degree-of-freedom manipulator, the equations of motion can be derived from the Euler-Lagrange equations by writing down the system's kinetic and potential energy. The result is a compact expression known as the **manipulator equations**:

$$
M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}.
$$

This is the robot-arm version of $F = ma$. The vector $\mathbf{q}$ contains the joint positions, $\dot{\mathbf{q}}$ the joint velocities, and $\ddot{\mathbf{q}}$ the joint accelerations. The vector $\boldsymbol{\tau}$ contains the torques applied by the actuators.

The matrix $M(\mathbf{q})$ is the **mass matrix**. It describes the inertial resistance of the robot to acceleration. Unlike the scalar mass in $F = ma$, the mass matrix depends on the robot's current configuration. The same arm can be much harder to accelerate when it is stretched out than when it is folded close to its base. This term depends on the mass, centre of mass and inertia tensor of every link.

The term $C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ captures **Coriolis and centripetal effects**. These are velocity-dependent forces that appear when the robot is already moving. They also couple the joints together: motion at one joint can create torques at other joints. At low speeds, these effects may be small enough to ignore. At high speeds, they become important.

The vector $\mathbf{g}(\mathbf{q})$ is the **gravity vector**. It gives the torques required just to hold the arm still against gravity. Even when $\dot{\mathbf{q}} = 0$ and $\ddot{\mathbf{q}} = 0$, a robot arm usually needs non-zero torque simply to maintain its pose.

Putting these pieces together, the manipulator equations tell us that actuator torque is used to accelerate the links, overcome velocity-dependent coupling effects and compensate for gravity.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

## Forward and inverse dynamics

Just as kinematics has a forward and inverse direction, dynamics can also be used in two directions.

**Forward dynamics** asks: if we apply these joint torques, what acceleration results? Starting from the manipulator equations, we solve for $\ddot{\mathbf{q}}$:

$$
\ddot{\mathbf{q}} = M(\mathbf{q})^{-1} \left[ \boldsymbol{\tau} - C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) \right].
$$

This is the direction used by simulators. A simulator receives torques or forces, computes accelerations and integrates forward in time to update velocity and position. It is the robot equivalent of simulating a ball rolling under gravity.

**Inverse dynamics** asks the opposite question: if we want a particular acceleration, what torques are required? In this case, the manipulator equations are used directly:

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}).
$$

This is the direction used by model-based controllers. If we have a desired trajectory, including desired positions, velocities and accelerations, inverse dynamics computes the torques needed to follow that trajectory.

Efficient algorithms matter here. A naive computation of the dynamics can be expensive for large robots, but the recursive Newton-Euler algorithm computes inverse dynamics in $O(n)$ time for an $n$-joint serial chain. This makes real-time control feasible.

> Luh, J. Y. S., Walker, M. W., and Paul, R. P. C. (1980). "On-line computational scheme for mechanical manipulators." *Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69-76.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

## Inverse dynamics control

In Chapter 3, we introduced feedback control as a way to correct error. A simple joint-space PID controller can certainly move a robot arm, but it has to fight the nonlinear coupled dynamics without explicitly knowing where those effects come from. The controller sees gravity, inertia and Coriolis effects only as errors to be corrected after they appear. This can make motion sluggish, imprecise or difficult to tune.

A better approach is to use the dynamics model directly. **Computed torque control**, also called **inverse dynamics control**, uses the manipulator equations to cancel the robot's dynamics:

$$
\boldsymbol{\tau} = M(\mathbf{q}) \mathbf{a}_d + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}),
$$

where $\mathbf{a}_d$ is the desired joint acceleration command. If the model is perfect, substituting this torque back into the manipulator equations gives

$$
\ddot{\mathbf{q}} = \mathbf{a}_d.
$$

The nonlinear, coupled robot arm has been transformed into a set of decoupled double integrators. In that transformed system, each joint behaves much more like the simple systems we controlled earlier in the unit.

In practice, we choose the desired acceleration using a PD tracking law:

$$
\mathbf{a}_d = \ddot{\mathbf{q}}_d + K_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p (\mathbf{q}_d - \mathbf{q}),
$$

where $\mathbf{q}_d$, $\dot{\mathbf{q}}_d$ and $\ddot{\mathbf{q}}_d$ are the desired joint position, velocity and acceleration. The feedback terms correct tracking error, while the model terms compensate for the expected dynamics.

This is an important pattern in robotics: use the model for what you can predict, and use feedback to correct what you cannot. Compared with pure PID, computed torque control is often cleaner because gravity, inertia and coupling are handled explicitly rather than being left for the feedback controller to discover through error.

The key assumption is that the dynamics model is accurate. If the model is wrong, the cancellation is incomplete and the controller must deal with residual nonlinear effects. This brings us to a central practical problem: how do we identify the model parameters of a real robot?

> Sciavicco, L. and Siciliano, B. (2000). *Modelling and Control of Robot Manipulators* (2nd ed.). Springer.

## System identification: dynamics as an estimation problem

Computed torque control requires accurate models of $M(\mathbf{q})$, $C(\mathbf{q}, \dot{\mathbf{q}})$ and $\mathbf{g}(\mathbf{q})$. These terms depend on physical parameters such as link masses, centres of mass and inertia tensors. For a real robot, the values in a CAD model are rarely exact. Manufacturing tolerances, cables, gearboxes, fasteners and end-effectors all shift the true values away from the nominal design.

**System identification** is the problem of estimating these dynamic parameters from measurements.

A useful property of robot dynamics is that the manipulator equations are **linear in the dynamic parameters**, even though they are nonlinear in the joint positions and velocities. We can rearrange the equations as

$$
\boldsymbol{\tau} = Y(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})\boldsymbol{\phi},
$$

where $Y$ is the **regressor**, a matrix of known functions of the measured joint motion, and $\boldsymbol{\phi}$ is a vector of unknown physical parameters. These parameters may include masses, inertia terms and centre-of-mass locations.

By moving the real robot, measuring joint positions, velocities, accelerations and torques, and stacking many observations together, we get a least-squares estimation problem. This connects directly to Chapter 4: we have a model, noisy measurements and unknown parameters, and we want the parameter values that best explain the data. If we also have a prior belief, for example from CAD, we can regularise the estimate toward physically plausible values.

The quality of the estimate depends strongly on the motion used to collect data. A trajectory that moves only one joint slowly will not reveal much about inertial coupling. A good identification trajectory must be **persistently exciting**, meaning it excites enough of the robot's dynamics to make the parameters observable. In practice, designing safe but informative excitation trajectories is a major part of system identification.

> Khosla, P. and Kanade, T. (1985). "Parameter identification of robot dynamics." In *Proceedings of the 24th IEEE Conference on Decision and Control*, pp. 1754-1760.

> Gautier, M. and Khalil, W. (1990). "Direct calculation of minimum set of inertial parameters of serial robots." *IEEE Transactions on Robotics and Automation*, 6(3), 368-373.

## Robot calibration: geometry as an estimation problem

System identification estimates dynamic parameters. There is a parallel problem for the geometric parameters introduced in Chapter 6. The DH parameters, URDF transforms and joint offsets used in a robot model are never exactly right.

The nominal geometry comes from design drawings or CAD models, but real manufacturing introduces small errors in link lengths, joint axes, joint offsets and mounting positions. These errors accumulate along the kinematic chain. A tiny angular or positional error near the base can produce millimetres of error at the end-effector, which can be unacceptable for precision tasks such as assembly, machining, insertion or surgery.

**Robot calibration** is the problem of estimating the true geometric parameters from measurements. A common approach is to move the robot through many joint configurations, measure the actual end-effector pose using an external tracking system or calibrated fixture, and then minimise the difference between the measured pose and the pose predicted by the kinematic model.

This is again an estimation problem. The kinematic chain provides the prediction model, the pose measurements provide data, and optimisation or Bayesian estimation gives parameter values that better match reality. The same conceptual pipeline appears repeatedly in robotics: write down a model, collect data, estimate the unknown parameters and then use the improved model for control or planning.

> Hollerbach, J., Khalil, W., and Gautier, M. (2016). "Model identification." In Siciliano, B. et al. (Eds.), *Springer Handbook of Robotics* (2nd ed.), pp. 113-138. Springer.

The connection across the unit is important. Robot modelling, from Chapter 2, gives the structure of the equations. State estimation, from Chapter 4, gives the tools for fitting those equations to reality. System identification and robot calibration are both examples of this pattern. The physics changes, but the estimation framework is the same.

## Contact modelling

Everything so far has assumed that the robot moves freely through space. Real robots, however, exist to interact with the world. They grasp objects, push surfaces, walk on floors, insert parts and collide with things. The moment a robot makes or breaks contact, the dynamics change fundamentally.

Contact introduces effects that do not appear in free-space dynamics. A surface can push back on the robot through a normal force. Friction can resist tangential sliding. Contact is also unilateral: a surface can push, but it cannot pull. These properties make contact dynamics difficult to model and simulate.

A common starting point is **rigid-body contact with Coulomb friction**. For a contact point, the normal force $f_n$ and the gap between surfaces $\delta$ must satisfy

$$
f_n \geq 0, \quad \delta \geq 0, \quad f_n \delta = 0.
$$

This is a **complementarity condition**. If the bodies are separated, then $\delta > 0$ and the normal force must be zero. If they are touching, then $\delta = 0$ and the normal force may be positive. The model forbids two impossible situations: a surface pulling on the robot, or a contact force appearing when the bodies are not touching.

Friction is commonly described using the **Coulomb friction cone**:

$$
\| \mathbf{f}_t \| \leq \mu f_n,
$$

where $\mathbf{f}_t$ is the tangential friction force and $\mu$ is the coefficient of friction. When the contact is sticking, the friction force lies inside the cone. When the contact is sliding, the force lies on the boundary and opposes motion.

Combining the manipulator equations with contact forces and these constraints typically leads to a **Linear Complementarity Problem (LCP)** or a **Quadratic Program (QP)** at each timestep. These are more expensive than free-space dynamics, but they allow simulators and planners to reason about contact in a principled way.

> Stewart, D. E. and Trinkle, J. C. (1996). "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and Coulomb friction." *International Journal for Numerical Methods in Engineering*, 39(15), 2673-2691.

> Posa, M., Cantu, C., and Tedrake, R. (2014). "A direct method for trajectory optimisation of rigid bodies through contact." *The International Journal of Robotics Research*, 33(1), 69-81.

## Contact in simulation

Physics engines handle contact in different ways, depending on the trade-off they make between physical accuracy, numerical stability and speed. Examples include MuJoCo, PyBullet, Gazebo, Isaac Sim and other robotics simulators.

One option is **rigid contact**, where non-penetration and complementarity are enforced as accurately as possible. This is physically meaningful, but numerically difficult. Perfectly rigid impacts create discontinuities and impulses, and the solver must handle abrupt changes in velocity.

A common engineering alternative is **soft contact**. Instead of enforcing non-penetration exactly, the simulator allows small penetrations and models the contact force using something like a spring-damper:

$$
f_n = k \max(0, -\delta) + b \max(0, -\dot{\delta}),
$$

where $k$ is contact stiffness and $b$ is contact damping. This approximation is smoother, more stable and often faster to simulate. It is not exact rigid-body physics, but it is useful engineering physics.

MuJoCo uses a soft-contact-style formulation designed for fast and stable simulation. This is one reason it is widely used in model-based control and reinforcement learning: it trades exact rigid-body semantics for robustness, differentiability and speed.

> Todorov, E., Erez, T., and Tassa, Y. (2012). "MuJoCo: A physics engine for model-based control." In *Proceedings of IROS*, pp. 5026-5033.

## The sim-to-real gap starts here

The gap between simulation and reality is not only about masses, inertias or link lengths. We have tools for identifying those. A major part of the sim-to-real gap comes from contact.

A controller or policy developed in simulation experiences the simulator's contact model. The real robot experiences the true physics of surfaces, materials, deformation, friction, compliance and impacts. Small differences in friction coefficient, contact stiffness, collision geometry or surface texture can produce large differences in behaviour. This is especially important for walking, grasping, pushing, insertion and dexterous manipulation.

One practical mitigation is **domain randomisation**. During training or controller tuning, we randomise contact parameters such as friction, stiffness, damping and object geometry. The aim is not to make the simulator perfectly accurate, but to force the controller to succeed across a family of possible worlds. A policy trained this way is less likely to depend on one fragile simulation detail and more likely to transfer to hardware.

Domain randomisation does not solve the sim-to-real problem completely, but it is a useful reminder of a broader idea: when the model is uncertain, we should train and test over that uncertainty rather than pretending the model is exact.

> Andrychowicz, M., Baker, B., Chociej, M., et al. (2020). "Learning dexterous in-hand manipulation." *The International Journal of Robotics Research*, 39(1), 3-20.

## Big picture

This chapter connects the geometry of robot arms to the physics required to move them. Kinematics tells us how joint positions map to poses. Dynamics tells us how forces and torques produce motion. Control uses these models to make the robot follow desired trajectories.

The manipulator equations are the central model:

$$
M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}.
$$

From them, we get forward dynamics for simulation, inverse dynamics for control, and system identification for fitting the model to real hardware. Once contact appears, the problem becomes harder again, because the world starts pushing back.

A useful way to summarise the progression is:

```text
Kinematics: where can the robot go?
Dynamics: what torques make it move?
Control: how do we choose those torques over time?
Calibration and identification: how do we make the model match reality?
Contact modelling: what happens when the world pushes back?
```

Robotics is powerful because these ideas connect. The same modelling and estimation principles keep reappearing, whether we are fitting a sensor model, calibrating a kinematic chain, estimating inertial parameters or randomising a simulator for learning.

## Key Papers

> Luh, J. Y. S., Walker, M. W., and Paul, R. P. C. (1980). "On-line computational scheme for mechanical manipulators." *Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69-76.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

> Khosla, P. and Kanade, T. (1985). "Parameter identification of robot dynamics." In *Proceedings of the 24th IEEE Conference on Decision and Control*, pp. 1754-1760.

> Gautier, M. and Khalil, W. (1990). "Direct calculation of minimum set of inertial parameters of serial robots." *IEEE Transactions on Robotics and Automation*, 6(3), 368-373.

> Hollerbach, J., Khalil, W., and Gautier, M. (2016). "Model identification." In Siciliano, B. et al. (Eds.), *Springer Handbook of Robotics* (2nd ed.), pp. 113-138. Springer.

> Stewart, D. E. and Trinkle, J. C. (1996). "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and Coulomb friction." *International Journal for Numerical Methods in Engineering*, 39(15), 2673-2691.

> Todorov, E., Erez, T., and Tassa, Y. (2012). "MuJoCo: A physics engine for model-based control." In *Proceedings of IROS*, pp. 5026-5033.

> Posa, M., Cantu, C., and Tedrake, R. (2014). "A direct method for trajectory optimisation of rigid bodies through contact." *The International Journal of Robotics Research*, 33(1), 69-81.

> Andrychowicz, M., Baker, B., Chociej, M., et al. (2020). "Learning dexterous in-hand manipulation." *The International Journal of Robotics Research*, 39(1), 3-20.

## Coming up next

We have now covered two broad ways to make robots behave: write down a model of the physics and use it for control, or design feedback controllers that correct errors directly. But what if we do not have a good model, or the world is too complex to model by hand?

Next, we look at perception and learning: how robots sense, interpret and learn from the world around them.

→ [Chapter 8: Perception and learning](../chapter-08-perception-and-learning/index.md)

# Week 6: Articulated robots and kinematics

Last week we looked at navigation: how a robot estimates where it is, builds a map and plans motion through the world. This week we step back and look at a different kind of robot entirely.

So far, most of our examples have been ground vehicles: robots whose main job is to move around in the world. Manipulators are different. A robot arm reaching for a cup, a gripper closing around an object, or a humanoid waving hello is not primarily described by its position on the floor. Instead, we describe it by the configuration of its links and joints.

To do this properly, we need to return to the fundamentals from [Week 1](../week-01-state-representations/) and [Week 2](../week-02-modelling/) and think more carefully about what state and configuration really mean.

## Articulated chains

A robot arm is an **articulated chain**: a sequence of rigid links connected by joints. Each joint contributes one or more degrees of freedom to the robot. The most common joint types are revolute joints, which rotate around an axis, and prismatic joints, which slide along an axis. A revolute joint behaves like an elbow or hinge, while a prismatic joint behaves more like a drawer sliding in and out.

The configuration of the arm is the set of all joint values:

$$
\mathbf{q} = (q_1, q_2, \dots, q_n).
$$

An arm with $n$ joints therefore lives in an $n$-dimensional **configuration space**:

$$
\mathcal{C} \subset \mathbb{R}^n.
$$

This is exactly the idea from [Week 2](../week-02-modelling/): the configuration is the minimal set of variables needed to describe the robot's pose. For a mobile robot, we often used $(x,y,\theta)$. For a robot arm, we usually use the joint vector $\mathbf{q}$.

The tip of the arm, called the **end-effector**, exists somewhere in **task space**. This is the space where we actually want things to happen: picking up a cup, pressing a button, turning a valve, inserting a peg, or waving at a person. The central question is therefore: how do joint values map to end-effector poses? This is the **kinematics problem**.

## DH parameters

To solve kinematics problems, we need a systematic way to describe the geometry of an articulated chain. In [Week 1](../week-01-state-representations/), we introduced homogeneous transformations:

$$
T = \begin{bmatrix} R & d \ 0 & 1 \end{bmatrix},
$$

and saw that these transformations can be chained together to describe a tree of coordinate frames. That was our first hint at robot kinematics.

The **Denavit-Hartenberg (DH) convention** is a classical way to assign coordinate frames to the joints of a robot arm. It describes the transformation between consecutive frames using four parameters.

| Parameter  | Meaning                                |
| ---------- | -------------------------------------- |
| $a_i$      | link length, measured along $x_i$      |
| $\alpha_i$ | link twist, measured around $x_i$      |
| $d_i$      | joint offset, measured along $z_{i-1}$ |
| $\theta_i$ | joint angle, measured around $z_{i-1}$ |

For a revolute joint, $\theta_i$ is the variable and $d_i$ is fixed. For a prismatic joint, $d_i$ is the variable and $\theta_i$ is fixed. The remaining parameters describe the fixed geometry of the robot.

The transformation from frame $i-1$ to frame $i$ is written as

$$
T_{i-1,i} = \text{Rot}(z, \theta_i) \cdot \text{Trans}(z, d_i) \cdot \text{Trans}(x, a_i) \cdot \text{Rot}(x, \alpha_i).
$$

This is just four elementary rigid-body transformations chained together. The power of the convention is that it gives a compact and repeatable way to write down the geometry of a serial robot. Once the DH table is known, the forward kinematics follows by multiplying the transformations together.

DH notation remains very useful for hand derivations, exams, simple robot arms and theoretical analysis. It forces us to think carefully about frames, axes, offsets and joint variables. However, it is not always the most convenient representation for modern robotics software, especially when a robot has meshes, collision geometry, sensors, transmissions, inertial properties or a complex frame tree.

> Denavit, J. and Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *Journal of Applied Mechanics*, 22(2), 215-221.

## URDF as a modern robot description format

In practical robotics systems, we often describe robots using a **URDF**, or Unified Robot Description Format. A URDF is an XML-based robot model used throughout the ROS ecosystem to describe a robot's links, joints, geometry and physical properties.

The conceptual idea is the same as the one we have already been using: a robot is a collection of rigid bodies connected by joints. Instead of writing a DH table, we write a structured description of the robot as a tree of links and joints.

A **link** represents a rigid body, such as an upper arm, forearm, gripper finger, wheel or camera body. A **joint** connects one link to another and describes the relative motion between them. The joint specifies its type, such as revolute, prismatic, fixed or continuous, as well as its axis, origin and limits.

A minimal URDF fragment looks like this:

```xml
<robot name="simple_arm">
  <link name="base_link"/>

  <link name="upper_arm"/>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>
</robot>
```

This says that the `upper_arm` link is attached to `base_link` by a revolute joint. The joint has a position and orientation relative to its parent frame, an axis of rotation and limits on its motion.

URDFs can also include visual geometry, collision geometry and inertial parameters. The **visual** model is what the robot looks like in a viewer such as RViz. The **collision** model is usually a simplified geometry used for collision checking and simulation. The **inertial** model gives mass, centre of mass and inertia, which are needed for dynamics and physics simulation.

This makes URDF more than just a kinematic notation. It is a practical bridge between geometry, simulation, planning and control. A URDF can be used to publish a robot's TF tree, visualise the robot, check collisions, run motion planning, connect sensors to frames and simulate dynamics.

Compared with DH parameters, URDF is usually easier to integrate into software systems. It does not require every transformation to be squeezed into the four-parameter DH convention. Instead, each joint can have an arbitrary origin transform and axis. This makes URDFs more natural for real robots, especially when models are exported from CAD or maintained as part of a ROS package.

There is one important limitation: standard URDF represents robots as **trees**. This works well for serial arms, mobile manipulators and many humanoids, but it cannot directly represent closed kinematic loops or parallel mechanisms without extra constraints or simulator-specific extensions. DH has a similar bias toward serial chains, so neither representation solves every modelling problem. The important distinction is that DH is primarily a compact mathematical convention, while URDF is a software-oriented robot description format.

In this unit, it is useful to understand both. DH teaches the geometry of serial-chain kinematics clearly. URDF shows how the same ideas are represented in modern robotics software.

## Forward kinematics

Once we have a transformation for each joint, we can chain them together exactly as we did with coordinate frames in Week 1. This gives the **forward kinematics**:

$$
T_{0n}(\mathbf{q}) = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{n-1,n}(q_n).
$$

This expression gives the pose of the end-effector as a function of the joint values $\mathbf{q}$.

Forward kinematics is the easy direction of the kinematics problem. Given the joint angles, we compute the end-effector pose. It is just matrix multiplication. There is no ambiguity, and for a given configuration there is always a unique end-effector pose.

Geometrically, the robot sweeps out a **workspace**: the set of all end-effector poses reachable by some joint configuration.

$$
\mathcal{W} = { T_{0n}(\mathbf{q}) \mid \mathbf{q} \in \mathcal{C} }.
$$

The workspace depends on the robot geometry, including link lengths, joint types and joint limits. It tells us what the robot can and cannot reach. Before designing a manipulation task, the first practical question is often simply whether the target lies inside the robot's workspace.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

## Inverse kinematics

Forward kinematics asks where the end-effector goes for a given joint configuration. The more useful problem is usually the reverse: given a desired end-effector pose, find joint values $\mathbf{q}$ that achieve it.

This is **inverse kinematics**, or IK.

IK is difficult because the mapping from joint space to task space is nonlinear and not generally invertible. For many robot arms, more than one configuration can reach the same end-effector pose. This is easy to see with your own arm: you can often keep your hand in roughly the same place while moving your elbow. When a robot has more degrees of freedom than are needed for the task, we call it **kinematically redundant**.

Sometimes there is no solution at all. The desired pose may lie outside the workspace, or it may require a joint angle that violates a joint limit. Even if the position is reachable, the desired orientation may not be.

For some robot geometries, especially six-degree-of-freedom arms with spherical wrists, it is possible to derive a closed-form **analytical IK** solution. Analytical IK is fast and exact, but it is specific to the geometry of the robot. If the robot changes, the derivation usually has to be done again.

A more general approach is **numerical IK**, where we treat inverse kinematics as an optimisation problem:

$$
\min_{\mathbf{q}} | T_{0n}(\mathbf{q}) - T_\text{desired} |.
$$

In practice, the error metric needs to account for both position and orientation, but the idea is simple: find a joint configuration whose forward kinematics is close to the desired pose.

The standard tool for numerical IK is the **Jacobian**. The Jacobian tells us how small changes in joint angles produce small changes in end-effector pose:

$$
\dot{\mathbf{x}} = J(\mathbf{q}) \dot{\mathbf{q}},
$$

where $\mathbf{x}$ is the end-effector pose and $J(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ is the Jacobian matrix.

We can invert this relationship approximately to find joint velocities that move the end-effector toward a target:

$$
\dot{\mathbf{q}} = J^\dagger(\mathbf{q}) \dot{\mathbf{x}}_\text{desired},
$$

where $J^\dagger$ is the **pseudoinverse** of the Jacobian. By repeatedly applying this update, the robot can move toward the desired end-effector pose. This approach is often called **Jacobian pseudoinverse IK** or **resolved-rate control**.

A major practical issue occurs at **singular configurations**. At a singularity, the Jacobian loses rank, meaning the robot loses the ability to move in some task-space direction. Near such configurations, the pseudoinverse can produce very large joint velocities for small desired end-effector motions. Real robots have joint limits, velocity limits and actuator limits, so this can cause unstable or erratic behaviour. If you have ever seen a robot arm suddenly move strangely near a particular pose, a singularity is a likely explanation.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

## Connecting back to configuration space

It is worth pausing to connect this back to the earlier parts of the unit.

In [Week 2](../week-02-modelling/), we introduced configuration space as an abstract idea. For a mobile robot, the configuration was often $(x,y,\theta)$, and we used that space to reason about what the robot could and could not do. For an articulated arm, the configuration space is the space of all joint values $\mathbf{q}$.

The forward kinematics maps configuration space into task space. The workspace is the image of that map: all the end-effector poses the robot can reach.

Configuration space also contains obstacles. Some joint configurations cause the robot to collide with itself. Others cause the robot to collide with the environment. Motion planning for arms therefore means finding a path through configuration space that reaches the target while avoiding collision. The same planning ideas from navigation still apply, but now the space is usually higher-dimensional and harder to visualise. Sampling-based planners such as RRT and RRT* are commonly used for this reason.

Kinematics is about the geometry of motion. At this stage, we are not asking what forces or torques are required. We are only asking where the robot is, where the end-effector can go and how joint motion changes the pose of the end-effector.

That is already a powerful foundation. If we know the robot's configuration, we can compute where its end-effector is. If we know the target pose, we can try to solve for a configuration that reaches it. If we know the obstacles, we can try to plan a collision-free path through configuration space.

But reaching the target is not just a geometric problem. A real robot has mass, inertia, friction, motors, gearboxes and torque limits. Next week we move from geometry to physics and ask what it actually takes to move the robot there.

## Key Papers

> Denavit, J. and Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *Journal of Applied Mechanics*, 22(2), 215-221.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

## Useful software reference

> ROS Wiki. "XML Robot Description Format (URDF)."

> ROS 2 Documentation. "URDF: Unified Robot Description Format."

## Coming up next

We know the geometry. Now we need the physics. Next, we look at dynamics: how forces, torques, mass and inertia determine how robots actually move.

→ [Week 7: Dynamics](../week-07-dynamics/)

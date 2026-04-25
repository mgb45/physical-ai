# Week 7: Articulated robots and kinematics

Last week we looked at [perception and learning](../week-08-perception-and-learning/). This week we take a step back and look at a different kind of robot entirely.

So far, we have been modelling **ground vehicles**: robots that move around in the world. But what about robots that **manipulate** things?

> a robot arm reaching for a cup, a gripper closing around an object, a humanoid waving hello

To describe these robots, we need to return to the fundamentals from [Week 1](../week-01-state-representations/) and [Week 2](../week-02-modelling/) and think harder about what state and configuration really mean.


## Articulated chains

A robot arm is an **articulated chain**: a sequence of rigid links connected by joints.

Each joint has one or more **degrees of freedom** (DOF). The two most common types are:

- a **revolute joint** — rotates around an axis (like an elbow)
- a **prismatic joint** — slides along an axis (like a drawer)

The **configuration** of the arm is just the set of all joint values:

$$
\mathbf{q} = (q_1, q_2, \dots, q_n)
$$

So an arm with $n$ joints lives in an $n$-dimensional **configuration space** $\mathcal{C} \subset \mathbb{R}^n$. This is exactly the idea from [Week 2](../week-02-modelling/): the configuration is the minimal description of the robot's state.

The tip of the arm — the **end-effector** — exists somewhere in **task space**. This is where we actually want to do things: pick up a cup, press a button, turn a valve.

So the key question becomes:

> how do joint angles map to end-effector poses?

This is the **kinematics problem**.


## DH parameters

We need a systematic way to describe the geometry of an articulated chain. Back in [Week 1](../week-01-state-representations/), we defined homogeneous transformations:

$$
T = \begin{bmatrix} R & d \\ 0 & 1 \end{bmatrix}
$$

and we said we could chain these together to build a TF tree describing the robot. That was our first hint at kinematics.

The **Denavit-Hartenberg (DH) convention** is a standard way to assign coordinate frames to joints and express the transformation between consecutive frames using just **four parameters**:

| Parameter | Meaning |
|-----------|---------|
| $a_i$ | link length (distance along $x_i$) |
| $\alpha_i$ | link twist (angle around $x_i$) |
| $d_i$ | joint offset (distance along $z_{i-1}$) |
| $\theta_i$ | joint angle (angle around $z_{i-1}$) |

For a revolute joint, $\theta_i$ is the variable. For a prismatic joint, $d_i$ is the variable. Everything else is fixed geometry of the robot.

The transformation from frame $i-1$ to frame $i$ is:

$$
T_{i-1,i} = \text{Rot}(z, \theta_i) \cdot \text{Trans}(z, d_i) \cdot \text{Trans}(x, a_i) \cdot \text{Rot}(x, \alpha_i)
$$

This is just four elementary rigid body moves, chained together. The key insight is that **any** robot link-joint pair can be described with these four numbers. It is a compact, unambiguous way to write down any articulated robot.

> Denavit, J. and Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *Journal of Applied Mechanics*, 22(2), 215–221.


## Forward kinematics

Now we have a transformation for each joint. We chain them together — exactly as we did in Week 1 — to get the **forward kinematics**:

$$
T_{0n}(\mathbf{q}) = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{n-1,n}(q_n)
$$

This gives us the pose (position and orientation) of the end-effector as a function of the joint angles $\mathbf{q}$.

Forward kinematics is the easy direction:

> given joint angles → compute end-effector pose

It is just matrix multiplication. No ambiguity. Always a unique answer.

We can think about this geometrically as sweeping out a **workspace** — the set of all end-effector poses reachable by some joint configuration:

$$
\mathcal{W} = \{ T_{0n}(\mathbf{q}) \mid \mathbf{q} \in \mathcal{C} \}
$$

The workspace depends on the robot geometry (link lengths, joint limits) and tells us what the robot can and cannot reach. Knowing your robot's workspace is the first thing you check before designing a task.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.


## Inverse kinematics

Forward kinematics is easy. The really hard — and useful — problem is the reverse:

> given a desired end-effector pose → find the joint angles $\mathbf{q}$

This is **inverse kinematics (IK)**.

Why is it hard?

### Multiple solutions

For most robot arms, there are **multiple configurations** that reach the same end-effector pose. Think about how many ways you can position your elbow while keeping your hand in the same place. This is called **kinematic redundancy**.

### No solution

Some poses are simply unreachable. The desired pose lies outside the workspace $\mathcal{W}$.

### Nonlinearity

The forward kinematics $T_{0n}(\mathbf{q})$ is a nonlinear function of $\mathbf{q}$. Inverting a nonlinear function is hard in general.

### Analytical IK

For simple robot geometries — especially 6-DOF arms with a spherical wrist — we can sometimes work out a **closed-form analytical solution**. This is fast and exact, but requires careful geometric reasoning, and is specific to the robot geometry. Change the robot, re-derive the solution.

### Numerical IK

The general approach is to treat IK as an **optimisation problem**:

$$
\min_{\mathbf{q}} \| T_{0n}(\mathbf{q}) - T_\text{desired} \|
$$

We can solve this iteratively. The standard tool is the **Jacobian**, which tells us how small changes in joint angles produce changes in end-effector pose:

$$
\dot{\mathbf{x}} = J(\mathbf{q}) \dot{\mathbf{q}}
$$

where $\mathbf{x}$ is the end-effector pose and $J(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ is the **Jacobian matrix**.

We can invert this relationship to find joint velocities that move the end-effector toward the target:

$$
\dot{\mathbf{q}} = J^\dagger(\mathbf{q}) \dot{\mathbf{x}}_\text{desired}
$$

where $J^\dagger$ is the **pseudoinverse** of the Jacobian. We iterate until the end-effector reaches the target. This is called **Jacobian pseudoinverse** or **resolved-rate** control.

A practical issue: when the Jacobian loses rank — at **singular configurations** — the pseudoinverse blows up. At a singularity, small desired motions in task space require arbitrarily large joint velocities. Robot arms have joint limits and actuator limits, so this causes real problems. You may have seen a robot arm behaving erratically near certain poses — that is probably a singularity.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.


## Connecting back to configuration space

It is worth pausing to appreciate what we have built.

In [Week 2](../week-02-modelling/) we talked about configuration space as an abstract concept. For a mobile robot, the configuration was $(x, y, \theta)$ and we reasoned about what the robot can and cannot do.

For an articulated arm, the configuration space is the space of all joint angles $\mathbf{q}$. The forward kinematics maps this to task space. The workspace is the image of this map.

But configuration space also has **obstacles**: joint configurations that cause collisions — links hitting each other, or the environment. Motion planning for arms requires navigating through configuration space to find paths that reach a target without collisions. The same ideas from navigation apply; they just live in a higher-dimensional space.

---

## Final thought

Kinematics is about the geometry of motion — no forces, no torques, just shapes and poses. It is the foundation for everything that comes next.

Once you understand kinematics:

> you know where the robot is  
> you know where the end-effector can go  
> you know how to get there  

But getting there requires more than geometry. Next week we ask: what does it actually take to **move** there?

---

## Key Papers

> Denavit, J. and Hartenberg, R. S. (1955). "A kinematic notation for lower-pair mechanisms based on matrices." *Journal of Applied Mechanics*, 22(2), 215–221.

> Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

> Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

> Featherstone, R. (2007). *Rigid Body Dynamics Algorithms*. Springer.

---

# Coming up next

We know the geometry. Now we need the physics.

→ [Week 8: Dynamics](../week-07-dynamics/)

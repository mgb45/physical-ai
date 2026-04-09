# Week 2 Modelling mobile robots

Last week we talked about **control**:

> how do we choose inputs $u$ to make the robot behave the way we want?

This week we ask a slightly more fundamental question:

> what actually happens when we apply those inputs?

In other words:

> how does our robot move?

This is the **modelling problem**.

## Configuration and task space

Let’s start with some definitions.

The **configuration** of a robot is the minimal set of variables that completely describe its state.

For a mobile robot, this is usually:

$$
\mathbf{x} = (x, y, \theta)
$$

- $(x, y)$: position in the world  
- $(\theta)$: orientation  

So this robot has **3 degrees of freedom**.

### A slightly strange example: a train

A train moves in 3D space.

But its configuration is just:

$$
q = \text{distance along the track}
$$

So it has **1 degree of freedom**, even though it exists in a 3D world.

Why?

Because it is constrained to move along the tracks.

### Configuration vs task space

This leads to an important distinction:

- **Configuration space**: what the robot *can do*  
- **Task space**: where the robot *exists*  

For the train:
- task space = 2D or 3D world  
- configuration space = 1D curve  

So the robot is moving on a **manifold** inside the task space.

### Fully actuated vs underactuated

Another important idea:

- **Fully actuated**: we can directly control all degrees of freedom  
- **Underactuated**: we cannot  

Example:

A hovercraft has:
- configuration: $$(x, y, \theta)$ → 3 DOF  
- actuators: 2  

So it is **underactuated**.

This is very common in robotics.

## Motion models

Now we get to the core idea.

A **motion model** describes how the state evolves:

$$
x_{k+1} = f(x_k, u_k)
$$

or in continuous time:

$$
\dot{x}(t) = f(x(t), u(t))
$$

This is just a more concrete version of what we wrote in Week 1.

### What goes into a motion model?

- state $x$: position, orientation, velocity  
- control $u$: motor commands, steering angle, etc  
- disturbances $\epsilon$: things we don’t control  

So really:

$$
x_{k+1} = f(x_k, u_k, \epsilon_k)
$$ 

## Bicycle models

A very common model for cars is the **bicycle model**.

We simplify the car to:

- one front wheel (steering)
- one rear wheel (driving)

### State and controls

State:

$$
(x, y, \theta)
$$

Controls:

- $v$$: forward velocity  
- $\delta$: steering angle  

---

### Key constraint

The robot **cannot move sideways**.

There is no way to directly generate motion in the lateral direction.

This is a huge deal.

> it means the system is constrained and underactuated

### Motion model

The motion looks like:

$$
x_{k+1} = x_k + v_k \cos(\theta_k) dt
$$

$$
y_{k+1} = y_k + v_k \sin(\theta_k) dt
$$

$$
\theta_{k+1} = \theta_k + \frac{v_k}{L} \tan(\delta_k) dt
$$ 

---

### Intuition

- velocity moves you forward in the direction you are facing  
- steering changes your heading  
- curved motion comes from combining both  

## Differential drive robots

Now let’s look at the robot you will actually use.

A **differential drive robot** has:

- two wheels  
- independently controlled  

### Configuration

Same as before:

$$
(x, y, \theta)
$$

### Controls

We often use:

- $v$: linear velocity  
- $\omega$: angular velocity  

So:

$$
u = (v, \omega)
$$

### Motion model

If the robot moves forward and rotates:

$$
\theta_{k+1} = \theta_k + \omega_k dt
$$

$$
x_{k+1} = x_k + v_k \cos(\theta_k) dt
$$

$$
y_{k+1} = y_k + v_k \sin(\theta_k) dt
$$

### Turning intuition

When turning:

- the robot follows a circular arc  
- it rotates around an **instantaneous centre of rotation (ICR)**  

The radius is:

$$
R = \frac{v}{\omega}
$$ 

So:

> straight line → \(\omega = 0\)  
> tight turn → large \(\omega\)  

### A slightly more exact model

We can write:

$$
\begin{bmatrix}
x_{k+1} \\
y_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
x_k \\
y_k
\end{bmatrix}
+
R
\begin{bmatrix}
-\sin(\theta_k) + \sin(\theta_{k+1}) \\
\cos(\theta_k) - \cos(\theta_{k+1})
\end{bmatrix}
$$ 

This just makes the circular motion explicit.

## A differential drive robot in a ball?

Now let’s ask a slightly weird question.

> what if we put our robot inside a ball?

Does the model change?

### The key idea

The **mechanism changes**, but the **state representation might not**.

We might still describe the robot by:

$$
(x, y, \theta)
$$

But:

- the control inputs change  
- the constraints change  
- the motion model \(f(x, u)\) changes  

### Moral of the story

> modelling is about abstraction

We choose:

- what state to represent  
- what controls to use  
- what physics to include  

Different choices → different models → different behaviour

# Simulators as models

So far, we have been writing down motion models like:

$$
x_{k+1} = f(x_k, u_k)
$$

and calling this a **robot model**.

But this is really just a modelling choice.

## It’s all just a model

There is no real distinction between:

- a robot model  
- a world model  
- a simulator  

They are all just:

> functions that describe how a system evolves over time

## What changes is what we include

So far, our state has been something like:

$$
x = (x, y, \theta)
$$

which only describes the robot.

That means our model is implicitly assuming:

> the world does not exist (or does not matter)

## Expanding the state

If we instead define a bigger state:

$$
x = (\text{robot}, \text{world})
$$

then the exact same idea becomes:

$$
x_{k+1} = f(x_k, u_k)
$$

But now:

- the robot moves  
- the world can change  
- interactions can happen  

Nothing new mathematically.

We just changed what we decided to model.

## A simulator is just a bigger model

A simulator is simply:

> a model with a richer state and more detailed dynamics

It might include:

- contact  
- friction  
- collisions  
- sensors  
- other agents  

But fundamentally, it is still just:

$$
x_{k+1} = f(x_k, u_k)
$$

---

## Why this perspective matters

Because it keeps things consistent.

Whether we are:

- writing down equations on paper  
- coding a simple kinematic model  
- using a physics engine  

we are always doing the same thing:

> choosing a state, and defining how it evolves

## The real design choice

The hard question is not:

> what is the “correct” model?

The real question is:

> what do we choose to include in the state?

Include too little:
- model is simple  
- but wrong  

Include too much:
- model is complex  
- but hard to use  

## Final thought

So far, we have used very simple models.

That was deliberate.

Because once you understand:

> everything is just $x_{k+1} = f(x_k, u_k)$

you can scale that idea to anything:

- a wheeled robot  
- a robot in a cluttered room  
- a multi-agent system  
- a full simulator  

Same idea.

Just a bigger $x$, and a more complicated $f$. All of robotics sits on top of modelling.

Unfortunately, f your model is wrong, your controller will be wrong. And your robot will let you know.

# Coming up next

So far we have assumed we know where the robot is.

Next:

> how do we estimate $x$ in the real world?

→ localisation and SLAM
# Week 2 Modelling mobile robots

So far we have thought about the best representation of our robot, and this week we move to modelling how this evolves over time, or in response to commands or actions we send the robot. To help we need to start with thinking about how robots can move, which is often more constrained than the representation we selected to describe their state.

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

Why? Because it is constrained to move along the tracks.

### Configuration vs task space

This leads to an important distinction:

- **Configuration space**: what the robot *can do*  
- **Task space**: where the robot *exists*  

For the train:
- task space = 2D or 3D world  
- configuration space = 1D curve  

It helps to think that the robot is moving on a *manifold* inside the task space.

### Fully actuated vs underactuated

Another important idea is understanding what we can control on our robot.

- **Fully actuated**: we can directly control all degrees of freedom  
- **Underactuated**: we cannot control all degrees of freedom directly, only a subset. 

For example:

A hovercraft has:
- configuration: $$(x, y, \theta)$ → 3 DOF  
- actuators: 2  

So it is **underactuated**. This is very common in robotics.

## Motion models

Now we get to the core idea.

A *motion or dynamics model** describes how the state evolves:

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

> Rajamani, R. (2011). *Vehicle Dynamics and Control* (2nd ed.). Springer.

### State and controls

State:

$$
(x, y, \theta)
$$

Controls:

- $v$$: forward velocity  
- $\delta$: steering angle  

The key constraint we consider is that the robot **cannot move sideways**. There is no way to directly generate motion in the lateral direction.

This means the system is constrained and underactuated

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

This is relatively intuitive, velocity moves you forward in the direction you are facing, steering changes your heading and curved motion comes from combining both.  

## Differential drive robots

Now let’s look at common wheeled robot model, also suitable for robots with smaller wheel bases and tighter turn radii.

A *differential drive robot* has two wheels that are independently controlled.  

> Siegwart, R., Nourbakhsh, I. R., and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

### Configuration

This is the same as before:

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

The intuition here is that when turning the robot follows a circular arc and it rotates around an **instantaneous centre of rotation (ICR)**.

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

# World models (simulators)

So far, we have been writing down motion models like:

$$
x_{k+1} = f(x_k, u_k)
$$

and calling this a **robot model**, because our state has been something like
$$
x = (x, y, \theta)
$$
which only describes the robot. That means our model is implicitly assuming the world does not exist (or does not matter).

## Expanding the state

If we instead define a bigger state:

$$
x = (\text{robot}, \text{world})
$$

then the exact same idea becomes:

$$
x_{k+1} = f(x_k, u_k)
$$

But now, the robot moves, the world can change and interactions can happen. There is nothing new mathematically, we just changed what we decided to model.

## A simulator is just a bigger model

A simulator is simply a model with a richer state and more detailed dynamics. It might include contact, friction, collisions, sensors and other agents, but fundamentally, it is still just:

$$
x_{k+1} = f(x_k, u_k)
$$

because [everything is functions and functions describe the world](https://youtu.be/PAZTIAfaNr8).

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

Unfortunately, if your model is wrong, your controller will be wrong. And your robot will let you know.

## Key Papers

> Rajamani, R. (2011). *Vehicle Dynamics and Control* (2nd ed.). Springer.

> Siegwart, R., Nourbakhsh, I. R., and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

> Thrun, S., Burgard, W., and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

# Coming up next

Next we look at how to control our robot state, subject to these dynamics.

→ Control
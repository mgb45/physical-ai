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

#### A slightly strange example: a train

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

#### What goes into a motion model?

- state $x$: position, orientation, velocity  
- control $u$: motor commands, steering angle, etc  
- disturbances $\epsilon$: things we don’t control  

So really:

$$
x_{k+1} = f(x_k, u_k, \epsilon_k)
$$ 

### Bicycle models

A very common model for cars is the **bicycle model**.

We simplify the car to:

- one front wheel (steering)
- one rear wheel (driving)

> Rajamani, R. (2011). *Vehicle Dynamics and Control* (2nd ed.). Springer.

#### State and controls

State:

$$
(x, y, \theta)
$$

Controls:

- $v$: forward velocity  
- $\delta$: steering angle  

The key constraint we consider is that the robot **cannot move sideways**. There is no way to directly generate motion in the lateral direction.

This means the system is constrained and underactuated

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

### Differential drive robots

Now let’s look at common wheeled robot model, also suitable for robots with smaller wheel bases and tighter turn radii.

A *differential drive robot* has two wheels that are independently controlled.  

> Siegwart, R., Nourbakhsh, I. R., and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

#### Configuration

This is the same as before:

$$
(x, y, \theta)
$$

#### Controls

We often use:

- $v$: linear velocity  
- $\omega$: angular velocity  

So:

$$
u = (v, \omega)
$$

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

For a slightly more exact model, we can write:

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

### A differential drive robot in a ball?

Now let’s ask a slightly weird question.

> what if we put our robot inside a ball?

Does the model change? The **mechanism changes**, but the **state representation might not**.

We might still describe the robot by:

$$
(x, y, \theta)
$$

But:

- the control inputs change  
- the constraints change  
- the motion model \(f(x, u)\) changes  

The moral of the story is that modelling is about abstraction

We choose:

- what state to represent  
- what controls to use  
- what physics to include  

Different choices → different models → different behaviour

## Observation models

So far, we have described models as:

$$
x_{k+1} = f(x_k, u_k)
$$

But modelling does not stop at motion. A complete model also has to account for **what the robot can observe about the world**.

Robots do not get state directly. They get **sensors**.

And sensors don't give truth — they give **measurements**, at a certain rate, with noise, delays, bias, distortions, missing data, and occasional lies.

A practitioner's view of a sensor is not "it returns numbers", but:

- **What physical quantity is being measured?**
- **What is the measurement model?**
- **What are the dominant failure modes?**
- **What is the timing model (rate, latency, synchronization)?**
- **How do I calibrate it (intrinsics/extrinsics, bias, scale)?**
- **How do I fuse it with other sensors?**


A very common abstraction is:

$$
z_t = h(x_t) + v_t
$$

- $x_t$ = latent "true" state (pose, velocity, map, object pose, etc.)
- $z_t$ = measurement at time $t$
- $h(\cdot)$ = measurement function (often nonlinear)
- $v_t$ = measurement noise (often approximated as Gaussian)

In practice, a more realistic view is:

$$
z_t = h(x_{t-t_l}) + b + v_t
$$

- $t_l$ = latency / time offset (sensors rarely align perfectly with the control loop)
- $b$ = bias / drift (IMUs, wheel odometry, magnetometers…)
- $v_t$ = stochastic noise

A big chunk of "robotics that works" is:
- estimating $x_t$,
- estimating $b$,
- and managing timing (time sync).

## Practical considerations around sensors

### Sampling, latency, and synchronization 

(why your robot looks haunted)

Sensors are discrete-time. Control is discrete-time. But they run at different rates.

- camera: e.g. 30 Hz (33 ms)
- lidar: e.g. 10 Hz (100 ms)
- IMU: e.g. 200-2100 Hz (1.0825 ms)
- wheel encoders: e.g. 50-200 Hz

If your sensor timestamps are wrong by 20-250 ms, that can be catastrophic at speed.

Practical checklist:
- Do I have **hardware timestamps** or am I using "message arrival time"?
- Are my sensors **time-synchronized** (PTP, NTP, shared clock, trigger line)?
- When I fuse, do I **propagate** states to measurement time?

### Calibration: intrinsics vs extrinsics

Two calibration types show up everywhere:

1) **Intrinsics**: parameters *inside* the sensor model  
   Example (camera): focal length, principal point, distortion coefficients.

2) **Extrinsics**: rigid transform between frames  
   Example: $T_{\text{base}\rightarrow \text{camera}}$.

This ties directly back to Week 1: homogeneous transforms and frame chaining.

If a sensor lives in frame $S$ and your robot base is frame $B$, you will write:

$$
p^S = T_{SB}\, p^B
$$

and you will spend a non-trivial portion of your life estimating $T_{SB}$.

### Noise, bias, drift 

(and why "Gaussian" is a convenient lie)

Common patterns:

- **white measurement noise**: $v_t \sim \mathcal{N}(0, R)$  
- **bias**: constant or slowly varying offset  
- **random walk drift**: $b_{t+1} = b_t + w_t$, $w_t \sim \mathcal{N}(0, Q)$  
- **outliers**: non-Gaussian, heavy-tailed errors (bad features, sun glare, wheel slip)

For robust systems:
- detect outliers (gating, RANSAC, Huber loss),
- downweight bad measurements,
- and always keep an eye on observability ("can I actually infer what I want from what I measure?").

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

#### Information: what does the sensor *actually constrain*?

A useful mental model: each sensor "observes" a subset of the state.

- wheel odometry: constrains *relative motion* on the ground (until slip)
- IMU: constrains angular velocity / acceleration (but drifts)
- GPS: constrains global position (but noisy / blocked indoors)
- camera: constrains geometry via features (but needs texture / light)
- lidar: constrains geometry via depth returns (but fails on glass/black surfaces)

Good fusion is about combining complementary constraints.

## Useful Sensors

### Vision

Vision is the most information-dense sensor we use, and also the most brittle.

A pinhole camera (simplified) maps a 3D point to pixels:

> Hartley, R. and Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.

$$
\tilde{u} \sim K \,[R\;|\;t] \, \tilde{P}
$$

- $\tilde{P} = [X, Y, Z, 1]^T$ homogeneous 3D point in some world frame
- $[R|t]$ camera pose (extrinsics)
- $K$ intrinsics matrix
- $\tilde{u} = [u, v, 1]^T$ pixel

**1) Feature-based geometry (classical CV)**  
- detect features (corners, blobs),
- match across frames,
- estimate motion / structure (PnP, essential matrix, bundle adjustment).

Used heavily in:
- visual odometry,
- visual-inertial odometry (VIO),
- SLAM.

**2) Dense perception (learning-heavy)**  
- semantic segmentation (pixels → class),
- depth estimation,
- object detection and tracking,
- pose estimation (6-DoF object pose).

Used heavily in:
- manipulation,
- human-robot interaction,
- scene understanding.

#### Failure modes you need to anticipate

- motion blur (fast motion + low shutter speed)
- rolling shutter artifacts
- low texture / repetitive texture
- lighting changes, glare, saturation
- dynamic scenes (people, moving objects)

Vision works best when you respect the physics (exposure, optics) *and* your algorithm assumptions.

---

### RGBD sensors

RGBD sensors give you:
- RGB image + depth map (or a point cloud).

But depth is not "free truth". Common depth modalities:
- structured light,
- time-of-flight,
- active stereo.

Practical concerns:
- depth noise increases with distance,
- failures on shiny / transparent / dark surfaces,
- missing depth (holes) near edges,
- indoor sunlight issues (for some active sensors).

From depth + intrinsics you can back-project pixels to 3D:

Given pixel $(u,v)$ with depth $d$:

$$
X = (u - c_x)\frac{d}{f_x}, \quad Y = (v - c_y)\frac{d}{f_y}, \quad Z = d
$$

Now you can do:
- plane fitting,
- grasp point selection,
- ICP registration,
- voxel maps.

---

### Tactile sensors

Touch is underrated because it's hard to instrument and interpret, but it's often the only reliable signal in contact-rich tasks.

Common forms:
- binary contact switches
- force/torque sensors (wrist F/T)
- pressure arrays ("tactile skins")
- joint torque sensing (via motor current + model)

A minimal model in manipulation:
- measure contact wrench $w = [f_x,f_y,f_z,\tau_x,\tau_y,\tau_z]^T$
- detect contact, slip, estimate friction / compliance

Practical uses:
- detect first contact and stop motion safely
- regulate force (impedance / admittance control)
- learn contact-rich skills (insertion, wiping, opening doors)

---

### Lidar

Lidar is geometry at scale: ranges + angles → point clouds.

A 2D lidar gives points in a plane; a 3D lidar gives a 3D point cloud.

Measurement model (simplified):
$$
z = r + v,\quad v \sim \mathcal{N}(0,\sigma^2(r))
$$

Practical issues:
- returns depend on surface reflectivity and angle
- "no return" is informative (range maxed out)
- spinning lidars are not instantaneous: the cloud is built over time (motion distortion)
- glass can be weird; rain/snow can be weird

---

### Positioning sensors

#### GPS / GNSS

GNSS gives global position outdoors, but:
- errors are correlated and environment-dependent,
- multipath near buildings is nasty,
- indoors it's mostly dead.

If you have RTK GNSS you can get cm-level accuracy in good conditions, but it becomes a system integration problem (base station, corrections, antenna placement).

#### Magnetometers

Magnetometers can give heading *sometimes*, but indoor distortions are common. Treat them carefully (calibrate hard/soft iron, detect anomalies).

---

### Odometric sensors

Wheel odometry is the "default" for mobile robots, but it is not ground truth.

Failure modes:
- wheel slip (sand, carpet, wet floors)
- incorrect wheel radius / wheelbase calibration
- encoder quantization / missed ticks
- contact loss (one wheel lifted)

This is why we fuse wheel odometry with IMU, lidar, or vision.

## World models (simulators)

So far, we have been writing down motion models like:

$$
x_{k+1} = f(x_k, u_k)
$$

and calling this a **robot model**, because our state has been something like
$$
x = (x, y, \theta)
$$
which only describes the robot. That means our model is implicitly assuming the world does not exist (or does not matter).

If we instead define a bigger state:

$$
x = (\text{robot}, \text{world})
$$

then the exact same idea becomes:

$$
x_{k+1} = f(x_k, u_k)
$$

But now, the robot moves, the world can change and interactions can happen. There is nothing new mathematically, we just changed what we decided to model.

A simulator is just a bigger model

A simulator is simply a model with a richer state and more detailed dynamics. It might include contact, friction, collisions, sensors and other agents, but fundamentally, it is still just:

$$
x_{k+1} = f(x_k, u_k)
$$

because [everything is functions and functions describe the world](https://youtu.be/PAZTIAfaNr8).

Final thought

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

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

> Hartley, R. and Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.

# Coming up next

Next we look at how to control our robot state, subject to these dynamics.

→ Control
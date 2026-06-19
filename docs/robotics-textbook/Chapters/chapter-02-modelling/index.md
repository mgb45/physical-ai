# Chapter 2: Modelling mobile robots

So far, we have thought about how to represent a robot. This chapter, we move from representation to modelling: how that representation evolves over time, and how it changes in response to commands or actions we send to the robot.

A useful starting point is to think carefully about how robots can move. This is often more constrained than the state representation we use to describe them.

## Configuration and task space

The **configuration** of a robot is the minimal set of variables needed to completely describe its state. For a mobile robot, this is usually written as

$$
\mathbf{x} = (x, y, \theta)
$$

where $(x,y)$ gives the robot's position in the world and $\theta$ gives its orientation. This means the robot has **3 degrees of freedom**.

### A slightly strange example: a train

A train exists in 3D space, but its configuration can often be described by a single variable:

$$
q = \text{distance along the track}.
$$

So even though the train exists in a 3D world, it has only **1 degree of freedom**. The reason is that it is constrained to move along the tracks.

This leads to an important distinction between **configuration space** and **task space**. Configuration space describes what the robot can actually do, while task space describes the world in which the robot exists. For the train, the task space is the 2D or 3D world, but the configuration space is a 1D curve through that world. A helpful way to think about this is that the robot moves on a *manifold* inside the task space.

### Fully actuated and underactuated systems

Another important modelling question is what we can directly control. A robot is **fully actuated** when we can directly control all of its degrees of freedom. It is **underactuated** when we can only directly control a subset of them.

For example, a hovercraft might have configuration $(x, y, \theta)$, giving it 3 degrees of freedom, but only 2 actuators. That makes it underactuated. This is very common in robotics: we often need to move a robot in ways that we cannot directly command.

## Motion models

A **motion model** describes how the robot state evolves over time. In discrete time, we write this as

$$
x_{k+1} = f(x_k, u_k),
$$

or, in continuous time, as

$$
\dot{x}(t) = f(x(t), u(t)).
$$

This is a more concrete version of the modelling idea introduced in Chapter 1.

A motion model usually includes the state $x$, such as position, orientation, and velocity; the control input $u$, such as motor commands or steering angle; and disturbances $\epsilon$, which represent effects we do not directly control. A more realistic discrete-time model is therefore

$$
x_{k+1} = f(x_k, u_k, \epsilon_k).
$$

## Bicycle models

A common model for car-like robots is the **bicycle model**. In this abstraction, the car is simplified to one front wheel for steering and one rear wheel for driving.

> Rajamani, R. (2011). *Vehicle Dynamics and Control* (2nd ed.). Springer.

The state of this simplified vehicle is

$$
(x, y, \theta),
$$

and the control inputs are the forward velocity $v$ and steering angle $\delta$.

A key constraint is that the robot cannot move sideways because there is no way to directly generate lateral motion. This means the system is constrained and underactuated.

The discrete-time bicycle model can be written as

$$
x_{k+1} = x_k + v_k \cos(\theta_k) dt,
$$

$$
y_{k+1} = y_k + v_k \sin(\theta_k) dt,
$$

$$
\theta_{k+1} = \theta_k + \frac{v_k}{L} \tan(\delta_k) dt.
$$

Intuitively, velocity moves the robot forward in the direction it is facing, while steering changes its heading. Curved motion comes from combining both effects.

## Differential drive robots

A **differential drive robot** is another common wheeled robot model. It is especially useful for robots with smaller wheel bases and tighter turning radii. A differential drive robot has two independently controlled wheels.

> Siegwart, R., Nourbakhsh, I. R., and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

The configuration is again

$$
(x, y, \theta),
$$

but the controls are often written as linear velocity $v$ and angular velocity $\omega$, so that

$$
u = (v, \omega).
$$

If the robot moves forward while rotating, its motion can be approximated by

$$
\theta_{k+1} = \theta_k + \omega_k dt,
$$

$$
x_{k+1} = x_k + v_k \cos(\theta_k) dt,
$$

$$
y_{k+1} = y_k + v_k \sin(\theta_k) dt.
$$

When the robot turns, it follows a circular arc around an **instantaneous centre of rotation (ICR)**. The turning radius is

$$
R = \frac{v}{\omega}.
$$

When $\omega = 0$, the robot moves in a straight line, so the turning radius is effectively infinite.

A more exact kinematic update over one time step is

$$
\begin{bmatrix}
x_{k+1} \\
y_{k+1}
\end{bmatrix}=
\begin{bmatrix}
x_k \\
y_k
\end{bmatrix}
+
R
\begin{bmatrix}
-\sin(\theta_k) + \sin(\theta_{k+1}) \\
\cos(\theta_k) - \cos(\theta_{k+1})
\end{bmatrix}.
$$

This expression makes the circular arc motion explicit. The vehicle is not translating independently in $x$ and $y$; instead, its position evolves according to a rigid-body rotation about an instantaneous centre of curvature.

To connect this with the rigid-body motion from Chapter 1, and to prepare for control in Chapter 3, we can write the dynamics in *state-space* form. Let

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix},
\qquad
\mathbf{u}_k =
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}.
$$

For a small time step $\Delta t$, the nonlinear unicycle model is

$$
\mathbf{x}_{k+1}
=
\mathbf{x}_k
+
\Delta t
\begin{bmatrix}
v_k \cos \theta_k \\
v_k \sin \theta_k \\
\omega_k
\end{bmatrix}.
$$

This is a discrete-time rigid-body motion model. Around a nominal trajectory $(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)$, we can linearise it as

$$
\delta \mathbf{x}_{k+1} =
A_k \delta \mathbf{x}_k
+
B_k \delta \mathbf{u}_k,
$$

where

$$
A_k =
\begin{bmatrix}
1 & 0 & -\Delta t \bar{v}_k \sin \bar{\theta}_k \\
0 & 1 & \Delta t \bar{v}_k \cos \bar{\theta}_k \\
0 & 0 & 1
\end{bmatrix},
$$

and

$$
B_k =
\begin{bmatrix}
\Delta t \cos \bar{\theta}_k & 0 \\
\Delta t \sin \bar{\theta}_k & 0 \\
0 & \Delta t
\end{bmatrix}.
$$

This gives us a locally linear model of the rigid-body dynamics. Next chapter, we will see that this form is useful for a common class of control problems, including the *linear quadratic regulator*.

### A differential drive robot in a ball?

Now consider a slightly weird question:

> What if we put our robot inside a ball?

Does the model change? The **mechanism** changes, but the **state representation** might not. We might still describe the robot by

$$
(x, y, \theta).
$$

However, the control inputs, constraints, and motion model $f(x,u)$ would change.

Fundamentally, robot modelling is about abstraction. We choose what state to represent, what controls to use, and what physics to include. Different choices lead to different models and different behaviour. Some models are easier to control; others help because they align closely with the information we can sense.

## Observation models

So far, we have described models as

$$
x_{k+1} = f(x_k, u_k).
$$

But modelling does not stop at motion. A complete model also has to account for **what the robot can observe about the world**. Robots do not get state directly. Instead, they get sensor measurements. Sensors do not give truth; they give measurements at a certain rate, with noise, delays, bias, distortions, missing data, and occasional lies.

A common abstraction of a measurement model is

$$
z_t = h(x_t) + v_t.
$$

Here, $x_t$ is the latent "true" state, such as pose, velocity, map, or object pose. The measurement at time $t$ is $z_t$, the measurement function is $h(\cdot)$, and the measurement noise is $v_t$, which is often approximated as Gaussian.

In practice, a more realistic model is

$$
z_t = h(x_{t-t_l}) + b + v_t,
$$

where $t_l$ represents latency or time offset, $b$ represents bias or drift, and $v_t$ represents stochastic noise.

A big part of making robotics work is estimating the state $x_t$, estimating biases such as $b$, and managing timing carefully through time synchronisation.

## Useful sensors

Perception is not a major focus of this unit, so this section is intended as a practical reference rather than a comprehensive treatment. The important point is that sensors are not interchangeable sources of truth. Each sensor measures something different, with different assumptions and different failure modes.

### Vision

Vision is one of the most information-dense sensors we use, but it is also one of the most brittle. We will not cover vision in detail here, but it is important to note that a camera produces colour images while also mapping 3D points to pixels.

> Hartley, R. and Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.

$$
\tilde{u} \sim K \,[R\;|\;t] \, \tilde{P}
$$

In this expression, $\tilde{P} = [X, Y, Z, 1]^T$ is a homogeneous 3D point in some world frame, $[R|t]$ describes the camera pose or extrinsics, $K$ is the intrinsics matrix, and $\tilde{u} = [u, v, 1]^T$ is the corresponding pixel.

We can calibrate a camera to find the intrinsics $K$. If we can detect image points that correspond to known points in the world, then we can estimate motion or structure. This idea is used heavily in visual odometry and some visual SLAM techniques.

Classical computer vision often uses **feature-based geometry**. In this approach, we detect image features such as corners or blobs, match them across frames, and then estimate motion or structure using methods such as PnP, the essential matrix, or bundle adjustment.

More generally, images can also be processed directly using deep learning. These **dense perception** methods are particularly useful in manipulation, human-robot interaction, and scene understanding. Examples include semantic segmentation, depth estimation, object detection and tracking, and 6-DoF object pose estimation.

Vision methods often fail in ways that matter for robotics. Motion blur, rolling shutter artefacts, low texture, repetitive texture, lighting changes, glare, saturation, and dynamic scenes can all cause problems. Vision works best when we respect both the physics of the sensor, such as exposure and optics, and the assumptions of the algorithm.

### RGBD sensors

RGBD sensors provide an RGB image together with a depth map or point cloud. Depth, however, is not free truth. Common depth modalities include structured light, time-of-flight, and active stereo.

In practice, depth noise usually increases with distance. Shiny, transparent, and dark surfaces can produce failures. Missing depth often appears near edges, and indoor sunlight can affect some active sensors.

Given a pixel $(u,v)$ with depth $d$, depth and camera intrinsics allow us to back-project the pixel into 3D:

$$
X = (u - c_x)\frac{d}{f_x}, \quad Y = (v - c_y)\frac{d}{f_y}, \quad Z = d.
$$

Once we have 3D points, we can do tasks such as plane fitting, grasp point selection, ICP registration, and voxel mapping.

### Tactile sensors

Touch is underrated because it is hard to instrument and interpret, but it is often the only reliable signal in contact-rich tasks. Common tactile sensing approaches include binary contact switches, wrist force/torque sensors, pressure arrays or tactile skins, and joint torque sensing through motor current plus a model.

A minimal manipulation model might measure the contact wrench

$$
w = [f_x,f_y,f_z,\tau_x,\tau_y,\tau_z]^T.
$$

From this signal, the robot can detect contact, identify slip, and estimate friction or compliance. Practical uses include detecting first contact and stopping safely, regulating force through impedance or admittance control, and learning contact-rich skills such as insertion, wiping, or opening doors.

### Lidar

Lidar is a distance and intensity sensor. Ranges and angles are combined to produce point clouds. A 2D lidar gives points in a plane, while a 3D lidar gives a 3D point cloud.

A simplified lidar measurement model is

$$
z = r + v,\quad v \sim \mathcal{N}(0,\sigma^2(r)).
$$

In practice, lidar returns depend on surface reflectivity and angle. A "no return" measurement can also be informative because it may indicate that the range has maxed out. Spinning lidars are not instantaneous: the cloud is built over time, so motion distortion can appear. Glass, rain, and snow can also produce unexpected behaviour.

### Positioning sensors

GNSS gives global position outdoors, but its errors are correlated and environment-dependent. Accuracy depends on satellite geometry, multipath near buildings can be severe, and indoors GNSS is usually not useful.

RTK GNSS can provide centimetre-level accuracy in good conditions, but it becomes a system integration problem involving a base station, corrections, antenna placement, or public reference services.

Magnetometers can sometimes provide heading, but indoor distortions are common. They need to be treated carefully through hard-iron and soft-iron calibration, anomaly detection, and awareness of environmental effects. For example, driving over reinforcing bars in buildings can disturb the measurement.

### Odometric sensors

Wheel odometry is often the default source of motion information for mobile robots. It usually comes from encoders that count wheel rotations. However, wheel odometry is not ground truth.

It can fail because of wheel slip on sand, carpet, or wet floors; incorrect wheel radius or wheelbase calibration; encoder quantisation; missed ticks; or contact loss when one wheel lifts off the ground. This is why wheel odometry is often fused with IMU, lidar, or vision.

## Practical considerations around sensors

### Sampling, latency, and synchronisation

Sensors are discrete-time systems, and control systems are also discrete-time systems. The difficulty is that they often run at different rates. A camera might run at 30 Hz, which corresponds to about 33 ms per frame. A lidar might run at 10 Hz, giving 100 ms between scans. An IMU might run anywhere from about 200 Hz to more than 2000 Hz, while wheel encoders might run at 50 to 200 Hz.

If sensor timestamps are wrong by even 20 to 250 ms, the result can be catastrophic at speed.

A practical question is whether the system uses **hardware timestamps** or merely records message arrival time. It is also important to know whether sensors are **time-synchronised**, for example through PTP, NTP, a shared clock, or a trigger line. When fusing measurements, we also need to propagate states to the actual measurement time rather than pretending every sensor measurement arrived at the same instant.

### Calibration: intrinsics and extrinsics

Two calibration types show up everywhere. **Intrinsics** are parameters inside the sensor model. For a camera, these include focal length, principal point, and distortion coefficients. **Extrinsics** describe the rigid transform between frames, such as the transform from the robot base to the camera, $T_{\text{base}\rightarrow \text{camera}}$.

This ties directly back to Chapter 1: homogeneous transforms and frame chaining. If a sensor lives in frame $S$ and the robot base is frame $B$, we might write

$$
p^S = T_{SB}\, p^B.
$$

A non-trivial portion of robotics work is spent estimating transforms like $T_{SB}$.

### Noise, bias, and drift

Gaussian noise is a convenient approximation, but it is not the whole story. Measurement systems can include white measurement noise,

$$
v_t \sim \mathcal{N}(0, R),
$$

biases that are constant or slowly varying, and random walk drift such as

$$
b_{t+1} = b_t + w_t,\quad w_t \sim \mathcal{N}(0, Q).
$$

They can also include outliers: non-Gaussian, heavy-tailed errors caused by bad visual features, sun glare, wheel slip, and other failures.

Robust systems detect outliers before acting on sensor information, for example through gating, RANSAC, or Huber losses. They downweight bad measurements and keep track of observability: can we actually infer what we want from what we measure?

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

## World models and simulators

So far, we have written motion models like

$$
x_{k+1} = f(x_k, u_k),
$$

and called them **robot models**, because the state has been something like

$$
x = (x, y, \theta),
$$

which only describes the robot. This implicitly assumes that the world either does not exist or does not matter.

If we instead define a larger state,

$$
x = (\text{robot}, \text{world}),
$$

then the same modelling idea becomes

$$
x_{k+1} = f(x_k, u_k).
$$

Now the robot can move, the world can change, and interactions can happen. There is nothing new mathematically; we have simply changed what we decided to model.

A simulator is just a bigger model. It has a richer state and more detailed dynamics. It might include contact, friction, collisions, sensors, and other agents, but fundamentally it is still

$$
x_{k+1} = f(x_k, u_k),
$$

because [everything is functions and functions describe the world](https://youtu.be/PAZTIAfaNr8). All of robotics sits on top of modelling. Unfortunately, if your model is wrong, your controller will be wrong, and your robot will let you know.

## Key papers

> Rajamani, R. (2011). *Vehicle Dynamics and Control* (2nd ed.). Springer.

> Siegwart, R., Nourbakhsh, I. R., and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

> Thrun, S., Burgard, W., and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

> Hartley, R. and Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.

# Coming up next

Next, we look at how to control our robot state subject to these dynamics.

→ [Chapter 3: Control](../chapter-03-control/index.md)

# Week 6 Robot perception and learning

Up to now we’ve mostly assumed we *know* the state (or can estimate it cleanly) and we’ve focused on modelling + control.

Reality check:

> Robots don’t get state. They get **sensors**.

And sensors don’t give truth — they give **measurements**, at a certain rate, with noise, delays, bias, distortions, missing data, and occasional lies.

This week is about:
1. how sensors actually behave (enough to be dangerous),
2. how we turn raw data into **useful state / features**,
3. and how we can learn policies from data (imitation learning / behaviour cloning / “robot learning”).

If you only remember one message:

> In robotics, *perception* is about building a usable representation of the world from measurements, and *learning* is about building a usable policy (or model) from data.

---

## Sensors

A practitioner’s view of a sensor is not “it returns numbers”, but:

- **What physical quantity is being measured?**
- **What is the measurement model?**
- **What are the dominant failure modes?**
- **What is the timing model (rate, latency, synchronization)?**
- **How do I calibrate it (intrinsics/extrinsics, bias, scale)?**
- **How do I fuse it with other sensors?**

### Measurement models (the one equation you will write forever)

A very common abstraction is:

$$
z_t = h(x_t) + v_t
$$

- $x_t$ = latent “true” state (pose, velocity, map, object pose, etc.)
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

A big chunk of “robotics that works” is:
- estimating $x_t$,
- estimating $b$,
- and managing timing (time sync).

### Sampling, latency, and synchronization (why your robot looks haunted)

Sensors are discrete-time. Control is discrete-time. But they run at different rates.

- camera: e.g. 30 Hz (33 ms)
- lidar: e.g. 10 Hz (100 ms)
- IMU: e.g. 200-2100 Hz (1.0825 ms)
- wheel encoders: e.g. 50-200 Hz

If your sensor timestamps are wrong by 20-250 ms, that can be catastrophic at speed.

Practical checklist:
- Do I have **hardware timestamps** or am I using “message arrival time”?
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

### Noise, bias, drift (and why “Gaussian” is a convenient lie)

Common patterns:

- **white measurement noise**: $v_t \sim \mathcal{N}(0, R)$  
- **bias**: constant or slowly varying offset  
- **random walk drift**: $b_{t+1} = b_t + w_t$, $w_t \sim \mathcal{N}(0, Q)$  
- **outliers**: non-Gaussian, heavy-tailed errors (bad features, sun glare, wheel slip)

For robust systems:
- detect outliers (gating, RANSAC, Huber loss),
- downweight bad measurements,
- and always keep an eye on observability (“can I actually infer what I want from what I measure?”).

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

### Information: what does the sensor *actually constrain*?

A useful mental model: each sensor “observes” a subset of the state.

- wheel odometry: constrains *relative motion* on the ground (until slip)
- IMU: constrains angular velocity / acceleration (but drifts)
- GPS: constrains global position (but noisy / blocked indoors)
- camera: constrains geometry via features (but needs texture / light)
- lidar: constrains geometry via depth returns (but fails on glass/black surfaces)

Good fusion is about combining complementary constraints.

---

## Vision

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

### Practitioner tasks in robot vision

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

### Failure modes you need to anticipate

- motion blur (fast motion + low shutter speed)
- rolling shutter artifacts
- low texture / repetitive texture
- lighting changes, glare, saturation
- dynamic scenes (people, moving objects)

Vision works best when you respect the physics (exposure, optics) *and* your algorithm assumptions.

---

## RGBD sensors

RGBD sensors give you:
- RGB image + depth map (or a point cloud).

But depth is not “free truth”. Common depth modalities:
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

## Tactile sensors

Touch is underrated because it’s hard to instrument and interpret, but it’s often the only reliable signal in contact-rich tasks.

Common forms:
- binary contact switches
- force/torque sensors (wrist F/T)
- pressure arrays (“tactile skins”)
- joint torque sensing (via motor current + model)

A minimal model in manipulation:
- measure contact wrench $w = [f_x,f_y,f_z,\tau_x,\tau_y,\tau_z]^T$
- detect contact, slip, estimate friction / compliance

Practical uses:
- detect first contact and stop motion safely
- regulate force (impedance / admittance control)
- learn contact-rich skills (insertion, wiping, opening doors)

---

## Lidar

Lidar is geometry at scale: ranges + angles → point clouds.

A 2D lidar gives points in a plane; a 3D lidar gives a 3D point cloud.

Measurement model (simplified):
$$
z = r + v,\quad v \sim \mathcal{N}(0,\sigma^2(r))
$$

Practical issues:
- returns depend on surface reflectivity and angle
- “no return” is informative (range maxed out)
- spinning lidars are not instantaneous: the cloud is built over time (motion distortion)
- glass can be weird; rain/snow can be weird

---

## Positioning sensors

### GPS / GNSS

GNSS gives global position outdoors, but:
- errors are correlated and environment-dependent,
- multipath near buildings is nasty,
- indoors it’s mostly dead.

If you have RTK GNSS you can get cm-level accuracy in good conditions, but it becomes a system integration problem (base station, corrections, antenna placement).

### Magnetometers

Magnetometers can give heading *sometimes*, but indoor distortions are common. Treat them carefully (calibrate hard/soft iron, detect anomalies).

---

## Odometric sensors

Wheel odometry is the “default” for mobile robots, but it is not ground truth.

Failure modes:
- wheel slip (sand, carpet, wet floors)
- incorrect wheel radius / wheelbase calibration
- encoder quantization / missed ticks
- contact loss (one wheel lifted)

This is why we fuse wheel odometry with IMU, lidar, or vision.

---

## Learning from demonstration

Now the “learning” part.

We previously framed robotics as optimal control:

$$
u(t) = \pi(x(t))
$$

In learning from demonstration (LfD), we assume we have data from an expert:
- states (or observations),
- actions,
- maybe rewards (often not),
- maybe multiple tasks.

Goal: learn a policy $\pi_\theta$ that imitates expert behaviour.

A key distinction:

- **state** $x$ (latent, clean, often unavailable)
- **observation** $o$ (what the robot actually sees: images, lidar, proprioception)

So a more honest policy is:

$$
a_t = \pi_\theta(o_t)
$$

This is robotics’ version of “learning from data”, with the complication that errors compound over time because the policy affects the next observation.

---

### Motion primitives and Dynamic movement primitives

A classic way to make learning easier is to not learn raw control directly, but to learn parameters of a structured movement.

A “motion primitive” is a reusable chunk of behaviour (reach, grasp, push).

**Dynamic Movement Primitives (DMPs)** represent trajectories as stable dynamical systems with a learnable forcing term.

A simplified 1D DMP form:
$$
\tau \dot{v} = \alpha_z(\beta_z(g - y) - v) + f(s)
$$

$$
\tau \dot{y} = v
$$
- $y$ = position
- $v$ = velocity
- $g$ = goal
- $\tau$ = time scaling
- $f(s)$ = nonlinear forcing function of phase $s$
- the $(g - y)$ term makes it converge (stability bias)

DMPs are popular because:
- you can fit them from demonstrations,
- you can change goal $g$ and speed $\tau$,
- and you get some robustness “for free”.

> Ijspeert, A. J., Nakanishi, J., Hoffmann, H., Pastor, P., and Schaal, S. (2013). "Dynamical Movement Primitives: Learning Attractor Models for Motor Behaviors." *Neural Computation*, 25(2), 328–373.

---

### Behaviour cloning with neural networks (crash course)

Behaviour cloning (BC) is supervised learning on demonstration data.

> Pomerleau, D. A. (1989). "ALVINN: An Autonomous Land Vehicle in a Neural Network." In *Advances in Neural Information Processing Systems (NeurIPS)*, vol. 1, pp. 305–313.

You collect a dataset:
$$
\mathcal{D} = \\{(o_i, a_i)\\}_{i=1}^N
$$

and fit a parametric policy $\pi_\theta$ by minimizing a loss:

#### Discrete actions (classification)

If actions are discrete (e.g., high-level decisions), use cross-entropy:

$$
\min_\theta \; \mathbb{E}_{(o,a)\sim \mathcal{D}} \big[-\log \pi_\theta(a \mid o)\big]
$$

#### Continuous actions (regression)

Robots often have continuous actions (joint velocities, torques).

The simplest BC uses MSE:

$$
\min_\theta \; \mathbb{E}_{(o,a)\sim \mathcal{D}} \big[\|\pi_\theta(o) - a\|^2\big]
$$

A more probabilistic and often better-behaved approach is to predict a distribution, e.g. Gaussian:

$$
\pi_\theta(a\mid o) = \mathcal{N}\big(\mu_\theta(o), \Sigma_\theta(o)\big)
$$

and maximize likelihood:

$$
\min_\theta \; \mathbb{E}_{(o,a)\sim \mathcal{D}} \Big[ \frac{1}{2}(a-\mu_\theta(o))^T\Sigma_\theta(o)^{-1}(a-\mu_\theta(o)) + \frac{1}{2}\log|\Sigma_\theta(o)| \Big]
$$

Why this matters:
- if the expert is stochastic / multi-modal, MSE averages actions (bad for control),
- predicting uncertainty gives you a confidence signal.

#### Multi-modal actions (mixture density / discretization)

If there are multiple valid actions (common in manipulation), one Gaussian is not enough.

Option A: Mixture of Gaussians
$$
\pi_\theta(a\mid o)=\sum_{k=1}^K w_k(o)\,\mathcal{N}\big(\mu_k(o),\Sigma_k(o)\big)
$$

Option B: Discretize actions into bins (works surprisingly well for some robot policies).

#### Sequence problems (RNN / Transformer policies)

In many tasks, the right action depends on history:

$$
a_t = \pi_\theta(o_{\{1:t\}})
$$

You can model this with:
- RNN/LSTM/GRU,
- Transformers over tokenized observations,
- or maintain a belief state $\hat{x}_t$ via a filter, then act on it.

A common practical approach:
- encode image observations with a CNN / ViT,
- fuse with proprioception,
- use a temporal model to output actions.

#### Diffusion policies (score matching view)

Mixture models require fixing the number of modes, and a single Gaussian collapses to averaging.
A more powerful alternative is to represent the action distribution *implicitly* via its **score function** — the gradient of the log-density with respect to the action — and then **integrate** that gradient to produce samples.

**The key object: the score**

Define the score of the (conditional) action distribution:

$$
s(a \mid o) = \nabla_a \log p(a \mid o)
$$

The score points in the direction of increasing log-probability: it tells you which way to nudge an action to make it more likely under the expert distribution.
A neural network $s_\theta(a, o)$ is trained to approximate this gradient field.

**Sampling = integrating the score (Langevin dynamics)**

Given $s_\theta$, you can draw a sample from $p(a \mid o)$ by starting from Gaussian noise and following the estimated gradient, injecting a small amount of noise at each step:

$$
a_{k+1} = a_k + \frac{\eta}{2}\, s_\theta(a_k, o) + \sqrt{\eta}\;\epsilon_k, \qquad \epsilon_k \sim \mathcal{N}(0, I)
$$

After $K$ steps (with $\eta$ small), $a_K$ is approximately distributed according to $p(a \mid o)$, regardless of how complex or multimodal that distribution is.
This is an Euler–Maruyama discretisation of the Langevin stochastic differential equation whose stationary distribution is exactly $p(a \mid o)$.

**Training: how do you learn the score without knowing $p$?**

You cannot evaluate $\nabla_a \log p$ directly, but **denoising score matching** lets you train without it.
For each demonstration action $a$ add scaled Gaussian noise:

$$
\tilde{a} = a + \sigma \epsilon, \qquad \epsilon \sim \mathcal{N}(0, I)
$$

The score of this noisy distribution is:

$$
\nabla_{\tilde{a}} \log p(\tilde{a}) = -\frac{\tilde{a} - a}{\sigma^2} = -\frac{\epsilon}{\sigma}
$$

So the training loss is simply:

$$
\mathcal{L}(\theta) = \mathbb{E}_{(o,a)\sim\mathcal{D},\;\epsilon\sim\mathcal{N}(0,I),\;\sigma}\left[\left\|s_\theta(\tilde{a}, \sigma, o) + \frac{\epsilon}{\sigma}\right\|^2\right]
$$

The network learns to estimate the direction (and magnitude) needed to move a noisy action back toward the true expert action, across many noise levels $\sigma$.

**Why this matters for robotics**

- **Arbitrary multi-modality** — the gradient field can have multiple basins of attraction; sampling will find them without specifying how many.
- **High-dimensional actions** — works for full-arm trajectories ("action chunks"), not just single timestep commands.
- **No mode collapse** — unlike a single Gaussian, the score field does not average over modes.

The cost: inference requires $K$ gradient-field integration steps (typically 10–100), which is slower than a single forward pass. In practice, accelerated samplers (DDIM, consistency models) reduce this to 1–10 steps.

> Chi, C., Feng, S., Du, Y., Xu, Z., Cousineau, E., Burchfiel, B., and Song, S. (2023). "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion." In *Robotics: Science and Systems (RSS)*.

#### Conditioning on goals / tasks

Most real tasks are goal-conditioned:

$$
a_t = \pi_\theta(o_t, g)
$$

where $g$ might be:
- a target pose,
- a language instruction,
- a desired end state image,
- a one-hot task ID.

This is the bridge from “one skill” to “many skills”.

---

### The core problem in behaviour cloning: covariate shift

BC trains on expert data distribution $o \sim d_{\pi_E}$, but deploys on the robot’s distribution $o \sim d_{\pi_\theta}$. 

If the learned policy makes a small mistake, it visits states the expert never visited in the dataset, and performance can collapse.

You can write the idea as:
- training minimizes error under $d_{\pi_E}$
- deployment accumulates error under $d_{\pi_\theta}$

This is why BC can look great in offline metrics but fail on the robot.

#### Mitigation strategies (what practitioners actually do)

1) **Get more data** in the places the policy fails  
   (not glamorous, often the best option)

2) **Noise injection / domain randomization**  
   during training, perturb observations or dynamics

3) **DAgger (Dataset Aggregation)**  
   Iterate:
   - run $\pi_\theta$ on the robot
   - have expert label actions for visited observations
   - add to dataset and retrain

Formally:
$$
\mathcal{D} \leftarrow \mathcal{D} \cup \\{(o_t, a^E_t)\\}_{t=1}^T
$$

DAgger aligns the training distribution with the learned policy distribution.

> Ross, S., Gordon, G., and Bagnell, D. (2011). "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning." In *Proceedings of AISTATS*, pp. 627–635.

4) **Use a stabilizing controller + learned residual**
   Instead of learning full control:

$$
a = a_{\text{base}}(x) + a_{\text{learned}}(o)
$$

   where $a_{\text{base}}$ is something like a PID / MPC / impedance controller.
   This massively improves safety and data efficiency.

5) **Behaviour Cloning + RL fine-tuning**
   Initialize with BC, then optimize a reward with RL.
   (This can fix distribution shift but adds complexity and safety concerns.)

---

### What does the network actually output?

In robotics, your policy output choice matters:

- **joint position commands** (easier, often stable if your low-level controller is good)
- **joint velocities** (common in mobile + manipulation)
- **torques** (more direct, more powerful, much easier to destabilize)
- **end-effector pose deltas** (often intuitive for manipulation)
- **gripper open/close** + continuous arm motion (hybrid action space)

A very common practical policy for manipulation is:
- predict $\Delta x, \Delta y, \Delta z, \Delta \phi, \Delta \theta, \Delta \psi$ (small pose increments)
- plus gripper command
- executed at 10–30 Hz with smoothing/clamping

---

### Perception + learning: what is the input?

We can roughly group policy inputs:

1) **State-based** (engineered):
   - pose estimate, object pose, velocities
   - then learn $\pi_\theta(x)$
   - easier learning, depends on good perception pipeline

2) **End-to-end** (raw):
   - images + proprioception
   - learn $\pi_\theta(o)$ directly
   - fewer hand-designed modules, more data hungry

A practical hybrid:
- learn a representation $f_\phi(o)$ (e.g. visual features)
- control uses $f_\phi(o)$ + proprioception
- optionally keep a classical estimator in the loop for safety

---

### Offline vs online evaluation (the trap)

Offline BC metrics (MSE, accuracy) do not always correlate with robot success.

Why?
- compounding errors,
- multi-modal action ambiguity,
- small timing differences matter.

Rule of thumb:
> If it doesn’t work in closed-loop rollouts, it doesn’t work.

Always evaluate policies by rolling them out (in sim first, then on hardware cautiously).

---

### Safety and deployment hygiene (non-negotiable)

Before running a learned policy on a real robot:
- saturate actions (limits on velocity/acceleration/torque)
- add an emergency stop
- define workspace constraints
- use a “supervisor” controller if possible
- start slow (low speed, low force, soft environment)

Robot learning failures are not just “bad predictions” — they’re potential hardware damage.

---

## Minimal implementation (behaviour cloning) — practical checklist

### 1) Decide the control interface first

Before you collect *any* data, decide what your policy outputs:
- Mobile base: $a = [v,\,\omega]$ (linear/angular velocity)
- Arm: $a = \Delta \mathbf{x}$ (end-effector pose delta) or joint velocities $\dot{\mathbf{q}}$
- Gripper: binary or continuous $a_g$

This choice determines:
- what to log,
- what loss makes sense,
- what safety limits you must enforce.

### 2) Define your observation vector (and keep it stable)

A common structure:

$$
o_t = \big[\;\text{proprio}_t,\; \text{extero}_t,\; \text{goal}_t\;\big]
$$

Examples:
- proprio: $\mathbf{q},\dot{\mathbf{q}}$, base velocity, IMU angular velocity
- extero: image, depth, lidar scan, object pose estimate
- goal: waypoint, desired end-effector pose, language embedding

**Rule:** the observation you train on must match deployment *exactly* (same scaling, same frame, same timestamp convention).

### 3) Log data with timestamps (this matters more than model choice)

Log *at minimum*:
- $t$ (timestamp, ideally hardware)
- observation $o_t$
- expert action $a^E_t$
- robot state estimate (if available)
- transforms $T$ (extrinsics / TF tree snapshots), or at least the static calibration used

If you can’t reproduce “what did the robot see at time $t$?”, debugging becomes impossible.

### 4) Dataset format (simple and robust)

Store transitions as sequences, not shuffled single steps:

$$
\mathcal{D} = \\{\tau_i\\}_{i=1}^M, \quad \tau_i = \\{(o_t, a_t)\\}_{t=1}^{T_i}
$$

Why sequence storage helps:
- you can train temporal models later,
- you can evaluate closed-loop more realistically,
- you can do train/val splits by trajectory (not by time-step leakage).

### 5) Normalize inputs and outputs

For vector observations and actions, compute dataset statistics:

$$
\mu_o,\sigma_o,\quad \mu_a,\sigma_a
$$

Train on normalized values:

$$
\hat{o} = (o - \mu_o) / (\sigma_o + \epsilon), \quad \hat{a} = (a - \mu_a) / (\sigma_a + \epsilon)
$$

This stabilizes training and makes learning rates less fragile.

For images:
- normalize per-channel (mean/std),
- consider resizing consistently,
- keep intrinsics consistent if you change resolution.

### 6) Choose a loss that matches your action type

**Continuous deterministic BC (baseline):**
$$
\mathcal{L}(\theta)=\mathbb{E}\big[\|\pi_\theta(o)-a\|^2\big]
$$

**Continuous stochastic BC (often better):** predict $(\mu_\theta(o), \Sigma_\theta(o))$ and use Gaussian NLL:
$$
\mathcal{L}(\theta) = \mathbb{E}\Big[ \frac{1}{2}(a-\mu)^T\Sigma^{-1}(a-\mu) + \frac{1}{2}\log|\Sigma| \Big]
$$

Practical note: many implementations predict diagonal $\Sigma$ (per-action variance) to keep it stable.

### 7) Evaluation: don’t trust offline loss alone

Do three evaluations:

1) **Offline** (sanity): MSE / NLL on held-out trajectories  
2) **Open-loop rollout** (optional): feed recorded $o_t$ and compare predicted actions  
3) **Closed-loop rollout** (required): run policy in sim/robot with safety clamps

Closed-loop success is the only metric that matters.

### 8) Deployment hygiene (minimum safety layer)

Even for a “minimal” implementation:
- clamp actions: $v,\omega$ limits or $\|\Delta \mathbf{x}\|$ limit
- low-pass filter actions (avoid jitter)
- stop on anomaly (NaNs, missing sensors, diverging pose)
- watchdog: if no new observation for $\Delta t$, command zero action

A typical safe executor:
$$
a_t = \text{clip}(\alpha \, a^r_t + (1-\alpha) \, a_{t-1})
$$

where $a^r_t$ is the raw action prediction.

---

## Sensor fusion cheat sheet (what fuses well, what fails, what to watch)

### The fusion goal

You want an estimate of some state (often pose + velocity):

$$
x_t = [\mathbf{p}_t,\;\mathbf{v}_t,\;\boldsymbol{\theta}_t,\;\mathbf{b}_t]
$$

where $\mathbf{b}_t$ includes sensor biases (IMU bias, gyro drift, etc.).

Nearly all practical fusion fits the pattern:

- **prediction** using a motion model (often IMU integration)
- **correction** using measurements (camera/lidar/GPS/wheels)

### Typical sensor roles

**IMU**
- Pros: high rate, observes fast motion
- Cons: drifts (bias integration)
- Use it for: short-term motion propagation (prediction)

**Wheel odometry**
- Pros: simple, good on flat ground with grip
- Cons: slip, scale errors
- Use it for: relative planar motion correction (especially indoors)

**Camera (VO/VIO)**
- Pros: rich information, good in textured environments
- Cons: lighting, blur, scale ambiguity (monocular), dynamic scenes
- Use it for: pose correction, map building, object pose

**Lidar**
- Pros: metric geometry, robust to lighting
- Cons: glass/black surfaces, motion distortion, lower rate
- Use it for: scan matching, mapping, stable pose correction

**GPS/GNSS**
- Pros: global reference outdoors
- Cons: multipath, outages, low rate
- Use it for: global position correction / drift reset outdoors

### Three failure modes to diagnose first (in order)

#### 1) Time synchronization bugs

Symptoms:
- “laggy” correction
- oscillations / weird overshoot
- consistent errors that change with speed

Fix:
- use sensor timestamps, not message receipt time
- verify camera trigger / IMU clock alignment
- log and plot time offsets

#### 2) Bad extrinsics (wrong transform between sensor and base)

Symptoms:
- pose estimate looks fine until you turn, then explodes
- map “smears” during rotation
- lidar/camera alignment looks off

Fix:
- re-check $T_{\text{base}\rightarrow \text{sensor}}$
- confirm frame conventions (right-hand vs left-hand, axis directions)
- validate by rotating in place and watching residuals

#### 3) Mis-modelled noise / unhandled outliers

Symptoms:
- estimator “snaps” to wrong features
- drift resets erratically
- performance depends heavily on environment

Fix:
- add gating / robust loss
- increase measurement covariance when conditions degrade
- detect and drop outliers (RANSAC, Huber)

### Practical tuning heuristics

- If you trust a sensor *too much*, it will dominate and break you when it fails.
- If you trust it *too little*, you drift forever.

When in doubt:
- start conservative (higher measurement noise),
- then gradually tighten.

### What to plot when debugging fusion

Plot over time:
- innovation / residual norms (measurement minus prediction)
- estimated biases ($b_t$)
- velocity and yaw rate consistency
- pose drift vs ground truth (if available)
- number of inliers / matched features / scan match score

---

### Vision language action models

A modern trend is to condition robot policies on language and images.

Conceptually:
$$
a_t = \pi_\theta(o_t, \text{instruction})
$$

This can be implemented by:
- a vision encoder (image → features),
- a language encoder (text → embedding),
- a fusion module (cross-attention),
- and an action head (predict actions).

Why this is powerful:
- language provides a compact task specification,
- the model can reuse representations across tasks.

But: it does not remove the need for
- calibration,
- timing correctness,
- and robust control execution.

---

## Summary: what you should be able to do after this week

1) Write down a basic sensor model $z = h(x) + v$ and list noise/bias/latency.
2) Explain why frames + extrinsics matter (and how they show up as transforms).
3) Understand what your sensor *actually constrains* about the world.
4) Implement behaviour cloning loss for continuous actions and explain what it optimizes.
5) Explain covariate shift and name at least two mitigations (DAgger, residual learning, noise injection).
6) Make a plan to deploy a learned policy safely.

---

## Key Papers

> Pomerleau, D. A. (1989). "ALVINN: An Autonomous Land Vehicle in a Neural Network." In *Advances in Neural Information Processing Systems (NeurIPS)*, vol. 1, pp. 305–313.

> Ijspeert, A. J., Nakanishi, J., Hoffmann, H., Pastor, P., and Schaal, S. (2013). "Dynamical Movement Primitives: Learning Attractor Models for Motor Behaviors." *Neural Computation*, 25(2), 328–373.

> Ross, S., Gordon, G., and Bagnell, D. (2011). "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning." In *Proceedings of AISTATS*, pp. 627–635.

> Fischler, M. A. and Bolles, R. C. (1981). "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

> Hartley, R. and Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.

---

# Coming up next

Perception and learning are powerful, but they don’t replace structure.

→ Next we make robots more complex: **articulated systems** and how to represent and control them.
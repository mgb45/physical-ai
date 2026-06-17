# Week 8: Robot perception and learning

In Week 2 we covered how sensors work and what information they provide. This week we build on that foundation.

Reality check:

> Robots don’t get state. They get **sensors**.

And sensors don’t give truth — they give **measurements**, at a certain rate, with noise, delays, bias, distortions, missing data, and occasional lies.

This week is about:
1. how we turn raw sensor data into **useful state / features**,
2. and how we can learn policies from data (imitation learning / behaviour cloning / “robot learning”).

If you only remember one message:

> In robotics, *perception* is about building a usable representation of the world from measurements, and *learning* is about building a usable policy (or model) from data.

---

## Deep learning for perception: from sensors to states

Before we talk about how to learn *policies*, we need to talk about how to turn raw sensor data into something useful.

In classical robotics, perception pipelines were hand-engineered: a human decided which features to extract, which thresholds to apply, and how to fuse sensor modalities. Deep learning changed this: we can now train end-to-end models that learn a mapping from pixels (or point clouds, or spectrograms) directly to the quantities we care about.

The canonical deep-learning perception pipeline looks like:

$$
\text{sensor data} \xrightarrow{f_\phi} \text{features} \xrightarrow{g_\psi} \text{state / label}
$$

where $f_\phi$ is a learned feature extractor (backbone) and $g_\psi$ is a task-specific head. What differs between tasks is *what* $g_\psi$ predicts.

---

### Task taxonomy: classification, regression, segmentation

The three most common perception tasks in robotics map onto three standard supervised-learning problems.

#### Classification

**What we want:** assign a discrete label from a fixed set.

**Robotics examples:**
- *Traffic light state* — predict \{red, yellow, green\}.
- *Object category* — predict which object class is present.
- *Scene context* — predict road type, terrain type, indoor/outdoor.

**Output:** a probability distribution over $C$ classes, produced by a softmax head:

$$
\hat{y} = \text{softmax}(W h + b), \qquad \hat{y} \in \mathbb{R}^C
$$

**Loss:** cross-entropy between prediction and one-hot ground-truth label $y$:

$$
\mathcal{L}_\text{cls} = -\sum_{c=1}^C y_c \log \hat{y}_c
$$

**Backbone choice:** ResNet, EfficientNet, or a Vision Transformer (ViT) pre-trained on ImageNet, fine-tuned on task data.

#### Regression

**What we want:** predict a continuous quantity.

**Robotics examples:**
- *Object pose estimation* — predict 6-DoF pose $(x, y, z, \phi, \theta, \psi)$ of an object in the camera frame. Used downstream in grasping.
- *Grasp point prediction* — predict the pixel location (and optionally depth, width, angle) of where to grasp an object.
- *Keypoint detection* — predict 2-D image coordinates of semantic landmarks on an object.

**Output:** a vector $\hat{y} \in \mathbb{R}^d$ produced by a fully-connected head after the backbone features.

**Loss (baseline):** mean squared error:

$$
\mathcal{L}_\text{reg} = \|\hat{y} - y\|^2
$$

For orientation, MSE on Euler angles is problematic (gimbal lock, discontinuities at $\pm\pi$). In practice, represent rotations as rotation matrices or unit quaternions and use a geodesic / angular loss, or predict the 6D continuous representation.

For grasp quality maps, the GG-CNN family predicts a per-pixel grasp-quality, angle, and width map in a single forward pass, making grasp synthesis fast enough for reactive control.

> Redmon, J., and Angelova, A. (2015). "Real-time grasp detection using convolutional neural networks." In *ICRA*.

> Morrison, D., Corke, P., and Leitner, J. (2020). "Learning robust, real-time, reactive robotic grasping." *The International Journal of Robotics Research*, 39(2–3), 183–201.

#### Segmentation

**What we want:** assign a label (or mask) to every pixel (or point in 3-D).

Three related sub-tasks:

| Sub-task | Output | Robotics use |
|---|---|---|
| Semantic segmentation | Per-pixel class label | Drivable surface, lane marking classification |
| Instance segmentation | Per-pixel class + instance ID | Individual object manipulation |
| Panoptic segmentation | Semantic + instance, unified | Full scene understanding |

**Drivability and lane classification** are typical semantic segmentation tasks:
- classes: `{road, lane-marking, sidewalk, vehicle, pedestrian, background}`
- the network predicts a class probability for every pixel
- downstream, a robot can reason about which pixels form a traversable corridor

**Loss:** standard cross-entropy summed over pixels:

$$
\mathcal{L}_\text{seg} = -\frac{1}{HW}\sum_{i=1}^{H}\sum_{j=1}^{W} \sum_{c=1}^C y_{ijc}\log \hat{y}_{ijc}
$$

For class-imbalanced scenes (road is 90 % of pixels), weighted or focal variants help.

---

### Common architectures

#### Masked R-CNN (instance segmentation from images)

Masked R-CNN extends Faster R-CNN with an additional mask prediction branch.

> He, K., Gkioxari, G., Dollár, P., and Girshick, R. (2017). "Mask R-CNN." In *ICCV*.

The pipeline has three stages:

1. **Backbone + FPN** — a deep CNN (e.g., ResNet-50) extracts multi-scale feature maps via a Feature Pyramid Network.
2. **Region Proposal Network (RPN)** — a sliding-window classifier proposes candidate bounding boxes ("regions of interest", RoIs) at multiple scales.
3. **Per-RoI heads** — for each proposed region:
   - a classification head predicts the object class,
   - a regression head refines the box coordinates,
   - a mask head predicts a binary foreground/background mask for every pixel inside the box.

The three losses are trained jointly:

$$
\mathcal{L} = \mathcal{L}_\text{cls} + \mathcal{L}_\text{box} + \mathcal{L}_\text{mask}
$$

In robotics, Masked R-CNN is used to:
- segment individual objects on a cluttered tabletop prior to grasping,
- track objects across frames,
- provide region proposals for 6-DoF pose estimators.

#### SAM — Segment Anything Model

SAM is a promptable segmentation foundation model trained on 1 billion+ masks.

> Kirillov, A., Mintun, E., Ravi, N., Mao, H., et al. (2023). "Segment Anything." In *ICCV*.

Architecture:
- **Image encoder** — a heavyweight Vision Transformer (ViT-H) runs once per image to produce a dense embedding.
- **Prompt encoder** — encodes sparse prompts (points, bounding boxes) or dense prompts (rough masks) into a prompt embedding.
- **Mask decoder** — a lightweight transformer decoder predicts one or more masks from the image embedding + prompt embedding, with associated confidence scores.

Key properties for robotics:
- *Zero-shot*: given a click or box prompt it can segment any object without task-specific fine-tuning.
- *Amortised cost*: the image encoder (expensive) runs once; the decoder (cheap) can be called interactively at low latency.
- *SAM 2* extends this to video, propagating masks temporally — useful for object tracking during manipulation.

A common robotics workflow: detect an object with a fast open-vocabulary detector (e.g., OWL-ViT or Grounding DINO, prompted with a text label), then pass the resulting bounding box as a prompt to SAM to get a clean instance mask.

#### PointNet (3-D point cloud processing)

Cameras give 2-D images; depth cameras and LiDAR give 3-D point clouds. A point cloud is a set of $N$ 3-D points $\{p_i\} \subset \mathbb{R}^3$ (possibly with additional features like colour or intensity). Unlike images, point clouds have no fixed spatial ordering.

PointNet is the canonical deep network for unordered 3-D point sets.

> Qi, C. R., Su, H., Mo, K., and Guibas, L. J. (2017). "PointNet: Deep learning on point sets for 3D classification and segmentation." In *CVPR*.

Key ideas:

1. **Permutation invariance** — apply a shared MLP independently to each point (a point-wise function), then aggregate with a symmetric function (max-pooling):

$$
f(\{p_1,\ldots,p_N\}) = g\!\left(\max_{i=1}^N h(p_i)\right)
$$

where $h$ is a point-wise MLP and $g$ is a classification/regression head. Because max-pooling is symmetric, the output does not depend on point ordering.

2. **T-Net (spatial transformer)** — a mini-network predicts a $3 \times 3$ (and optionally $64 \times 64$) alignment transform, applied to the point cloud before feature extraction. This provides some invariance to rigid-body transformations.

Outputs:
- **Global feature** → classification (object category from LiDAR scan).
- **Per-point features** (global feature concatenated with local features) → segmentation of each point.

**PointNet++ (Qi et al., 2017)** extends PointNet with hierarchical local grouping (ball-query neighborhoods), capturing local geometry that the original network misses.

In robotics:
- classify or detect objects in LiDAR sweeps (autonomous driving),
- estimate 6-DoF object pose from a depth image converted to a point cloud,
- segment a robot's own body points from environmental points.

#### Multimodal models

Real robots have multiple sensor streams: RGB cameras, depth cameras, LiDAR, IMU, force-torque sensors, proprioception. No single sensor is sufficient.

**Fusion strategies:**

| Strategy | Where | When to use |
|---|---|---|
| Early fusion | Concatenate raw inputs | Inputs are spatially aligned (e.g., RGB-D) |
| Late fusion | Combine high-level features or predictions | Modalities have very different scales / rates |
| Cross-attention fusion | Transformer cross-attention between modalities | Rich semantic alignment needed (e.g., vision + language) |

**Common multimodal pipelines in robotics:**

*Vision + language (VLMs applied to robotics):*
Large vision-language models (CLIP, BLIP-2, LLaVA) are pre-trained on image–text pairs at internet scale. The shared embedding space means you can query "find the red mug" and retrieve the matching image region. In manipulation, this enables open-vocabulary object detection: the robot receives a language goal and the VLM grounds it to a region in the image.

*RGB-D fusion for pose estimation:*
Encode the RGB image with a 2-D CNN (for texture/appearance) and the depth image or point cloud with PointNet (for geometry), then concatenate features before a pose regression head. The two streams are complementary: texture helps distinguish similar shapes; geometry helps in low-texture or adverse-lighting conditions.

*LiDAR + camera fusion for outdoor autonomy:*
Project LiDAR points onto the image plane (using camera intrinsics and extrinsics) to decorate each 3-D point with an RGB value. Alternatively, project image features back onto LiDAR points (e.g., PointPainting). Multi-modal networks like BEVFusion fuse everything in a common bird's-eye-view (BEV) representation.

> Liu, Z., Tang, H., Amini, A., Yang, X., Mao, H., Rus, D., and Han, S. (2023). "BEVFusion: Multi-task multi-sensor fusion with unified bird's-eye view representation." In *ICRA*.

---

### Putting it together: typical sensor-to-state pipelines

**Autonomous driving example:**

$$
\underbrace{\text{RGB camera}}_{\text{front/side}} \xrightarrow{\text{SegNet / DeepLab}} \underbrace{\text{drivable mask + lanes}}_{\text{semantic}}
$$

$$
\underbrace{\text{LiDAR}}_{\text{360°}} \xrightarrow{\text{PointPillars / CenterPoint}} \underbrace{\text{3-D bounding boxes}}_{\text{objects}}
$$

$$
\underbrace{\text{All sensors}} \xrightarrow{\text{fusion + tracking}} \underbrace{x_t = [\text{ego pose, lane position, object list}]}_{\text{state for planning}}
$$

**Tabletop manipulation example:**

$$
\underbrace{\text{RGB-D camera}} \xrightarrow{\text{Masked R-CNN / SAM}} \underbrace{\text{instance masks}}
$$

$$
\underbrace{\text{instance mask + depth}} \xrightarrow{\text{pose estimator (e.g. FoundPose, GDR-Net)}} \underbrace{T_{\text{obj}}^{\text{cam}} \in SE(3)}_{\text{6-DoF object pose}}
$$

$$
\underbrace{T_{\text{obj}}^{\text{cam}}} \xrightarrow{\text{grasp planner / GG-CNN}} \underbrace{(x_g, y_g, z_g, \theta_g)}_{\text{grasp point and angle}}
$$

In both cases, the output is a structured state that a planner or policy can act on — connecting deep learning perception back to the control and planning frameworks from earlier weeks.

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

1. Explain the three main deep-learning perception task types (classification, regression, segmentation) and give a robotics example of each.
2. Describe the architecture and typical use case of Masked R-CNN, SAM, and PointNet.
3. Sketch a sensor-to-state pipeline for a manipulation or driving scenario.
4. Implement behaviour cloning loss for continuous actions and explain what it optimizes.
5. Explain covariate shift and name at least two mitigations (DAgger, residual learning, noise injection).
6. Make a plan to deploy a learned policy safely.

---

## Key Papers

> Pomerleau, D. A. (1989). "ALVINN: An Autonomous Land Vehicle in a Neural Network." In *Advances in Neural Information Processing Systems (NeurIPS)*, vol. 1, pp. 305–313.

> Ijspeert, A. J., Nakanishi, J., Hoffmann, H., Pastor, P., and Schaal, S. (2013). "Dynamical Movement Primitives: Learning Attractor Models for Motor Behaviors." *Neural Computation*, 25(2), 328–373.

> Ross, S., Gordon, G., and Bagnell, D. (2011). "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning." In *Proceedings of AISTATS*, pp. 627–635.

> He, K., Gkioxari, G., Dollár, P., and Girshick, R. (2017). "Mask R-CNN." In *IEEE International Conference on Computer Vision (ICCV)*.

> Kirillov, A., Mintun, E., Ravi, N., Mao, H., et al. (2023). "Segment Anything." In *IEEE International Conference on Computer Vision (ICCV)*.

> Qi, C. R., Su, H., Mo, K., and Guibas, L. J. (2017). "PointNet: Deep learning on point sets for 3D classification and segmentation." In *IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*.

> Qi, C. R., Yi, L., Su, H., and Guibas, L. J. (2017). "PointNet++: Deep hierarchical feature learning on point sets in a metric space." In *Advances in Neural Information Processing Systems (NeurIPS)*.

> Morrison, D., Corke, P., and Leitner, J. (2020). "Learning robust, real-time, reactive robotic grasping." *The International Journal of Robotics Research*, 39(2–3), 183–201.

> Liu, Z., Tang, H., Amini, A., Yang, X., Mao, H., Rus, D., and Han, S. (2023). "BEVFusion: Multi-task multi-sensor fusion with unified bird's-eye view representation." In *IEEE International Conference on Robotics and Automation (ICRA)*.

---

# Coming up next

Perception and learning are powerful, but they don’t replace structure.

→ [Week 7: Kinematics](../week-06-kinematics/)
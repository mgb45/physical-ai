# Week 8: Robot perception and learning

So far, we have built most of the classical robotics stack. We have represented robot state, written down kinematic and dynamic models, designed controllers, estimated state with recursive Bayes filters, built maps, planned paths, and described the kinematics and dynamics of manipulators.

A common theme has been that we first write down a model, and then use that model to estimate, plan, or control. In system identification, we already saw the first step toward learning: the structure of the model was known, but some parameters were unknown, so we estimated them from data.

This week extends that idea.

In many robotics problems, the model is not fully known, the state is not directly available, and the controller is difficult to design by hand. A camera gives pixels, not object poses. A depth camera gives points, not a grasp strategy. A demonstration gives examples of behaviour, not the rule that generated them. A real robot behaves differently from its nominal model because of friction, backlash, compliance, cable forces, wear, payload changes and contact.

**Robot learning** is the use of data to learn some part of the robotics pipeline: a representation, a perception model, a dynamics model, a policy, a reward, or a correction to an existing model.

The aim is not to replace modelling. The aim is to use data where modelling is difficult, while still using structure wherever we have it.

A useful principle for the rest of the week is:

> model what you know, and learn what you cannot model.

## What does it mean to learn?

At its simplest, learning means fitting a function from examples.

Suppose we collect a dataset

$$
\mathcal{D} = \{(x_i, y_i)\}_{i=1}^N,
$$

where $x_i$ is an input and $y_i$ is the desired output. We choose a family of functions $f_\theta$, parameterised by $\theta$, and find parameters that make the predictions close to the data:

$$
\hat{y}_i = f_\theta(x_i).
$$

For regression, where $y_i$ is continuous, a familiar loss is squared error:

$$
\mathcal{L}(\theta) = \sum_{i=1}^{N} \|f_\theta(x_i) - y_i\|^2.
$$

This should look very familiar from linear regression and from system identification. In fact, system identification is a learning problem. If we write

$$
y = \Phi \theta + \epsilon,
$$

then least squares estimates the parameters $\theta$ from input-output data. The difference in modern robot learning usually only that the function class may be richer, the data may be higher-dimensional, and the quantity being learned may sit inside a closed-loop robot system.

For example, the input $x_i$ might be an image, a lidar scan, a joint state, or a short history of observations. The output $y_i$ might be an object class, a 6-DoF pose, the next state of the robot, or the action a human demonstrator would take.

The basic supervised-learning pattern is therefore:

$$
\text{input data} \longrightarrow \text{model} \longrightarrow \text{predicted output}.
$$

The robotics question is: **what should the input and output be?**

That design choice determines what kind of learning problem we are solving.

## Why is learning useful in robotics?

Learning is useful when an important mapping is hard to write down by hand but examples are available.

A few examples make this concrete.

A camera image contains information about objects, but writing an analytic equation from pixels to object identity is almost impossible. Instead, we collect labelled images and learn a perception model.

A robot arm has known rigid-body dynamics, but the exact friction, payload, cable forces or contact effects may be unknown. Instead of deriving every missing effect from first principles, we can learn a residual (additive) dynamics model from measured motion data.

A human demonstrator may know how to perform a task, such as inserting a plug or folding cloth, even if we cannot easily write down the cost function or controller. Instead of hand-designing the policy, we can learn from demonstrations.

This gives three important learning problems in robotics:

1. **Perception learning:** learn a mapping from raw observations to useful state or features.
2. **Dynamics learning:** learn how the system changes when actions are applied.
3. **Imitation learning:** learn a policy from demonstrations.

Reinforcement learning is another important case, where the robot learns from reward through trial and error. We will cover that in detail next week. This week we only need the high-level idea that reinforcement learning learns behaviour from interaction, whereas imitation learning learns behaviour from examples provided by an expert.

## Perception as learned state estimation

In Week 2, we described sensors using measurement models such as

$$
z_t = h(x_t) + v_t,
$$

where $x_t$ is the underlying state, $z_t$ is the measurement and $v_t$ is noise. In Week 4, we used recursive Bayes filtering to estimate state from measurements:

$$
p(x_t \mid z_{1:t}, u_{1:t}).
$$

This is already an inference problem: the robot does not directly know the state; it must infer it from observations.

For simple sensors, the measurement model $h$ may be known. A wheel encoder measures wheel rotation. An IMU measures acceleration and angular velocity. A range sensor measures distance along a ray.

For rich sensors, the measurement model is much harder. A camera image is an array of pixel intensities. The state we care about may be the pose of a mug, the drivable part of a road, the location of a person, or whether a drawer is open. These quantities are not directly measured. They are latent variables that must be inferred.

We can write this abstractly as

$$
\hat{x}_t = f_\theta(o_t),
$$

where $o_t$ is an observation, such as an image or point cloud, and $\hat{x}_t$ is an estimated state, feature or label. The function $f_\theta$ is learned from data.

This is why perception and state estimation are closely related. A classical estimator often uses a hand-written measurement model. A learned perception system uses data to learn part of that measurement model.

A modern robot stack often combines both. For example, a neural network might detect objects in an image, while a Kalman filter tracks their positions over time. A SLAM system might use learned visual features, but still use geometric optimisation for pose estimation. Learning does not remove the need for estimation; it often supplies better measurements to the estimator.

## Basic supervised learning problems

Most perception learning begins with supervised learning. We collect examples where the input and desired output are both known, then train a model to predict the output from the input.

### Classification

In classification, the output is a discrete label. For example, a robot might classify terrain as `floor`, `grass`, `gravel` or `stairs`, or classify a traffic light as `red`, `yellow` or `green`.

If there are $C$ possible classes, the model usually outputs a probability vector

$$
\hat{y} \in \mathbb{R}^C,
$$

where $\hat{y}_c$ is the predicted probability of class $c$. A common loss is cross-entropy:

$$
\mathcal{L}_{\text{cls}} = -\sum_{c=1}^{C} y_c \log \hat{y}_c,
$$

where $y$ is a one-hot vector representing the true class.

Classification is useful when the robot needs a symbolic or categorical decision, such as identifying an object class or recognising a terrain type.

### Regression

In regression, the output is continuous. For example, a robot might predict an object position, a joint torque correction, a grasp point, or the next state of the system.

The simplest regression loss is mean squared error:

$$
\mathcal{L}_{\text{reg}} = \|\hat{y} - y\|^2.
$$

Regression is directly connected to system identification. In both cases, we fit a function that maps inputs to continuous outputs. The difference is that the function may be nonlinear and the inputs may be high-dimensional.

For robotics, we need to be careful about what the continuous output represents. Predicting angles with squared error can be problematic because angles wrap around at $\pm\pi$. Predicting rotations also requires care: Euler angles can have singularities, so quaternions or rotation matrices are often better choices.

### Segmentation

In segmentation, the model assigns a label to every pixel in an image or every point in a point cloud. This is useful when the robot needs spatial structure rather than a single label.

For an image of height $H$ and width $W$, semantic segmentation predicts a class probability at every pixel:

$$
\hat{y}_{ij} \in \mathbb{R}^C.
$$

The loss is usually cross-entropy summed or averaged over pixels:

$$
\mathcal{L}_{\text{seg}} = -\frac{1}{HW}\sum_{i=1}^{H}\sum_{j=1}^{W}\sum_{c=1}^{C} y_{ijc}\log \hat{y}_{ijc}.
$$

Segmentation is important for robotics because actions are spatial. A mobile robot needs to know which parts of the image are drivable. A manipulation robot may need to separate one object from another in a cluttered scene.

## From raw sensors to useful state

A learned perception system is usually not the whole robot. It is one component that produces information for the rest of the stack.

For autonomous driving, a simplified perception pipeline might be:

$$
\text{camera image} \rightarrow \text{drivable area and lane markings},
$$

$$
\text{lidar scan} \rightarrow \text{3-D object detections},
$$

$$
\text{detections over time} \rightarrow \text{tracked objects and ego state}.
$$

The planner does not usually act directly on raw pixels. It acts on a structured representation: the road layout, object positions, object velocities and predicted trajectories.

For tabletop manipulation, a pipeline might be:

$$
\text{RGB-D image} \rightarrow \text{object mask},
$$

$$
\text{mask + depth} \rightarrow \text{object pose},
$$

$$
\text{object pose} \rightarrow \text{grasp or motion plan}.
$$

Again, the learned perception system converts raw observations into state-like quantities that connect back to planning and control.

This is a useful way to think about deep learning in robotics. A neural network is often a complicated measurement function. It turns messy observations into estimates that the rest of the robot can use.

## Dynamics learning

The first major robot learning problem is dynamics learning.

In earlier weeks, we wrote robot models such as

$$
x_{t+1} = f(x_t, u_t)
$$

or, in continuous time,

$$
\dot{x} = f(x,u).
$$

In dynamics learning, we collect transition data

$$
\mathcal{D} = \{(x_t, u_t, x_{t+1})\}_{t=1}^{N}
$$

and fit a model

$$
\hat{x}_{t+1} = f_\theta(x_t, u_t).
$$

This is supervised learning: the input is $(x_t,u_t)$ and the target output is $x_{t+1}$.

A basic loss is

$$
\mathcal{L}(\theta) = \sum_{t=1}^{N} \|f_\theta(x_t,u_t) - x_{t+1}\|^2.
$$

This directly generalises system identification. In classical system identification, we may know the model form and estimate a small number of parameters. In learned dynamics, we may use a more flexible function approximator when the model form is uncertain or too complicated.

### Learning a residual model

In robotics, we often do know a lot about the dynamics. We know the rigid-body equations, the kinematics, the actuator limits and the approximate mass properties. It would be wasteful to ignore that knowledge.

A common approach is to learn only the part the model gets wrong:

$$
x_{t+1} = f_{\text{known}}(x_t,u_t) + f_\theta(x_t,u_t).
$$

Here $f_{\text{known}}$ might be a kinematic model, a rigid-body simulator or a nominal dynamics model. The learned term $f_\theta$ is a residual correction.

This is a very important pattern in robot learning. It says: use physics for the part we understand, and use data for the part we do not.

Residual models can compensate for friction, delays, soft contacts, backlash, unmodelled payloads or systematic simulator errors. They are usually more data-efficient than learning the full dynamics from scratch because the learned model only needs to explain the error.

### Probabilistic dynamics models

A deterministic dynamics model predicts one next state:

$$
\hat{x}_{t+1} = f_\theta(x_t,u_t).
$$

But real robots are uncertain. Sensors are noisy, contact is unpredictable and the same action may not always produce exactly the same result. We can instead learn a probabilistic model:

$$
p_\theta(x_{t+1} \mid x_t,u_t).
$$

For example, the model might output a Gaussian distribution:

$$
p_\theta(x_{t+1} \mid x_t,u_t) = \mathcal{N}\left(\mu_\theta(x_t,u_t), \Sigma_\theta(x_t,u_t)\right).
$$

The mean predicts the most likely next state, while the covariance represents uncertainty. This connects directly back to recursive Bayes filtering: if the dynamics are uncertain, we should propagate uncertainty through the model rather than only predicting a single trajectory.

Probabilistic dynamics are especially useful for planning. A controller should behave differently when a model is confident than when it is uncertain.

## Learning for control

Once we have a learned dynamics model, we can use it inside a controller or planner.

Recall the finite-horizon optimal control problem:

$$
u^*_{t:t+T} = \arg\min_{u_{t:t+T}} \mathbb{E}\left[\sum_{k=t}^{t+T} J(x_k,u_k)\right].
$$

Previously, the dynamics inside this optimisation came from a hand-written model. With dynamics learning, the rollout can instead use

$$
\hat{x}_{k+1} = f_\theta(\hat{x}_k,u_k).
$$

This gives **model-based learning for control**. We learn a model from data, then use optimal control or planning to choose actions.

This is closely related to the methods students have already seen. LQR and iLQR require a model of how state changes with action. If the model is not known exactly, we can estimate it. We can use ILQR with a learned non-linear dynamics model. Alternatively, in the simplest case, we can assume a linear model and do local linear system identification:

$$
x_{t+1} \approx A x_t + B u_t.
$$

Then LQR can be applied to the learned local model. More generally, we can learn nonlinear dynamics and use model predictive control to repeatedly plan over a short horizon.

This is one of the cleanest bridges from system identification to robot learning.

## Imitation learning

The second major robot learning problem this week is imitation learning.

In imitation learning, we do not start with a reward function. Instead, we start with demonstrations. An expert shows the robot what to do, and the robot learns to imitate the expert.

A demonstration dataset contains trajectories:

$$
\mathcal{D} = \{\tau_i\}_{i=1}^{M},
$$

where each trajectory is a sequence

$$
\tau_i = \{(o_t, a_t^E)\}_{t=1}^{T_i}.
$$

Here $o_t$ is the observation available to the robot, and $a_t^E$ is the action taken by the expert.

The goal is to learn a policy

$$
a_t = \pi_\theta(o_t)
$$

that produces actions similar to the expert.

This is again supervised learning. The input is the observation $o_t$, and the target output is the expert action $a_t^E$.

### Behaviour cloning

The simplest imitation-learning method is **behaviour cloning**. We fit the policy directly to the demonstrations.

For continuous actions, such as joint velocities or end-effector pose increments, a basic objective is

$$
\min_\theta \sum_{(o,a^E)\in \mathcal{D}} \|\pi_\theta(o) - a^E\|^2.
$$

For discrete actions, such as choosing between a small set of high-level behaviours, we can use cross-entropy:

$$
\min_\theta \sum_{(o,a^E)\in \mathcal{D}} -\log \pi_\theta(a^E \mid o).
$$

Behaviour cloning is attractive because it is simple. It reduces robot learning to supervised learning.

However, there is an important difference between ordinary supervised learning and robot control. In ordinary supervised learning, a prediction error usually does not affect the next input. In robotics, it does. If the policy makes a small mistake, the robot moves to a slightly different state. Then it observes a situation that may not have appeared in the demonstration data. This can lead to more errors, which lead to even less familiar states.

This is called **covariate shift** or **distribution shift**.

Training data comes from the expert distribution:

$$
o_t \sim d_{\pi_E},
$$

but deployment data comes from the learned policy distribution:

$$
o_t \sim d_{\pi_\theta}.
$$

If these distributions differ, low supervised-learning loss does not guarantee good closed-loop performance.

### DAgger

One way to reduce distribution shift is **DAgger**, or Dataset Aggregation. The idea is to let the learned policy visit its own states, then ask the expert what should have been done there.

The procedure is:

1. Train an initial policy from demonstrations.
2. Run the policy to collect observations it actually visits.
3. Ask the expert to label those observations with correct actions.
4. Add the new data to the dataset.
5. Retrain the policy.

Mathematically, the dataset is updated as

$$
\mathcal{D} \leftarrow \mathcal{D} \cup \{(o_t, a_t^E)\}_{t=1}^{T}.
$$

DAgger makes the training distribution closer to the deployment distribution. This is often much more important than choosing a more complicated neural network.

## Choosing the action representation

A learned policy does not simply output “the robot moves”. We must choose the action representation.

For a mobile robot, the action might be

$$
a_t = (v_t, \omega_t),
$$

where $v_t$ is linear velocity and $\omega_t$ is angular velocity.

For a manipulator, the action might be joint positions, joint velocities, joint torques, end-effector pose increments, or gripper commands.

This choice matters because it determines the difficulty and safety of the learning problem. Predicting torques gives the policy direct control over the actuators, but it is easy to destabilise the robot. Predicting end-effector pose increments is often easier and safer, because a lower-level controller can handle the detailed joint control.

A common manipulation policy outputs

$$
a_t = (\Delta x, \Delta y, \Delta z, \Delta \phi, \Delta \theta, \Delta \psi, g),
$$

where the first six terms describe a small end-effector pose change and $g$ is a gripper command. A classical controller then tracks the desired motion.

This is another example of using structure. The learned model chooses a high-level action, while the existing controller handles stabilisation and actuator-level control.

## Learning policies as probability distributions

A deterministic behaviour cloning policy predicts a single action:

$$
a_t = \pi_\theta(o_t).
$$

This can be too restrictive. Many robotics tasks are multi-modal: there may be several valid actions from the same observation. For example, a robot might grasp a cup from the left or from the right. If we train with mean squared error, the policy may average these two actions and choose a bad action in the middle.

A better approach is to model a distribution over actions:

$$
\pi_\theta(a \mid o).
$$

For continuous actions, a simple choice is a Gaussian:

$$
\pi_\theta(a \mid o) = \mathcal{N}(\mu_\theta(o), \Sigma_\theta(o)).
$$

Training by maximum likelihood gives the loss

$$
\mathcal{L}(\theta)
=
\frac{1}{2}(a-\mu_\theta(o))^T\Sigma_\theta(o)^{-1}(a-\mu_\theta(o))
+
\frac{1}{2}\log|\Sigma_\theta(o)|.
$$

The mean represents the predicted action, and the covariance represents uncertainty or variability. This is a probabilistic version of behaviour cloning.

If the action distribution is strongly multi-modal, one Gaussian may still be insufficient. Mixtures of Gaussians, discretised action bins and diffusion policies are all ways to represent more complex action distributions. The detailed methods are less important here than the principle: when many actions are valid, the policy should not be forced to average them into one action.

## Choosing a model class

Once we know the input, output and dataset, we still have to choose the kind of model we are going to fit. This is a modelling decision, not just an implementation detail. Different model classes make different assumptions about the task, the data and the structure of the behaviour we are trying to learn.

A useful way to ask the question is:

> what structure do I already believe the solution should have?

If the behaviour is a smooth reaching motion, a motion primitive may be enough. If the action distribution is multi-modal, a probabilistic neural policy may be more appropriate. If the task depends on long histories, language instructions or many subtasks, a sequence model may be useful. The model class should match the structure of the problem.

### Neural network policies

The most common choice in modern robot learning is a neural network. At this level, a neural network is just a flexible nonlinear function approximator:

$$
\text{input} \rightarrow \text{many learned intermediate features} \rightarrow \text{output}.
$$

For imitation learning, this often means a policy of the form

$$
a_t = \pi_\theta(o_t),
$$

where $o_t$ might include images, proprioception, force readings and a goal, and $a_t$ might be a velocity command, pose increment or gripper command.

A simple neural policy predicts one action. This is easy to train, but it can fail when there are multiple good actions for the same observation. For example, there may be several valid grasps for the same object. A deterministic policy trained with squared error may average these actions and produce a grasp that is not actually valid.

One solution is a **mixture density network**. Instead of predicting one Gaussian action distribution, the model predicts a mixture:

$$
\pi_\theta(a \mid o) = \sum_{k=1}^{K} w_k(o)\,\mathcal{N}(a; \mu_k(o), \Sigma_k(o)).
$$

Each component can represent a different mode of behaviour. In a grasping task, one component might represent grasping from the left, another from the right and another from above. The model does not have to average these possibilities into a single action.

A more recent alternative is a **diffusion policy**. A diffusion policy represents the action distribution by learning how to gradually denoise an initially random action or action sequence. Rather than outputting one action in a single forward pass, it starts with noise and iteratively refines it into an action that looks like one from the demonstration data.

Conceptually:

$$
\text{noise} \rightarrow \text{denoising model conditioned on observation} \rightarrow \text{action sequence}.
$$

This is useful when the policy needs to represent complex, high-dimensional and multi-modal action distributions. For robotics, diffusion policies are often used to predict short **chunks** of future actions rather than only the next immediate action. This can make behaviour smoother and more temporally coherent.

The tradeoff is computational cost. A deterministic neural policy may require one forward pass. A diffusion policy usually requires multiple denoising steps, although faster samplers can reduce this cost.

### Motion primitives

Neural networks are flexible, but flexibility is not always what we need. Many robot behaviours have strong geometric and dynamical structure. Reaching, wiping, inserting, opening and handing over objects are not arbitrary input-output mappings; they are structured motions.

A **motion primitive** is a reusable movement pattern. Instead of learning a policy that maps every observation directly to every action, we learn or select parameters of a movement primitive. The primitive then generates the detailed trajectory.

A classic example is the **Dynamic Movement Primitive** (DMP). A DMP represents a motion as a stable dynamical system plus a learned forcing term:

$$
\tau \dot{v} = \alpha_z(\beta_z(g-y)-v) + f(s),
$$

$$
\tau \dot{y} = v.
$$

Here $y$ is position, $v$ is velocity, $g$ is the goal, $\tau$ controls timing and $f(s)$ shapes the trajectory as a function of phase $s$. The important point is that the system has a built-in tendency to converge to the goal. This is an inductive bias: rather than learning arbitrary motion, we learn motion inside a stable movement structure.

DMPs are useful when we have demonstrations of a skill and want to reproduce it with changes in goal position or speed. For example, a reaching motion can be demonstrated once and then adapted to a new target by changing $g$.

A **probabilistic movement primitive** extends this idea by representing a distribution over trajectories rather than a single trajectory. This is useful when demonstrations vary, or when the robot needs to reason about uncertainty in the motion. Instead of saying "this is the trajectory", a probabilistic primitive says "these are the likely trajectories".

Motion primitives are less general than large neural policies, but they are often easier to interpret, easier to constrain and more data-efficient. They are a good choice when the task is a structured motion rather than open-ended decision making.

### Sequence models, Decision Transformers and vision-language-action models

Some tasks cannot be solved well by looking only at the current observation. The robot may need to remember what it has already done, interpret a task instruction, or choose actions based on long-term progress.

For these cases, we can model robot behaviour as a sequence:

$$
(o_1,a_1,o_2,a_2,\ldots,o_t) \rightarrow a_t.
$$

A **transformer** is a neural network architecture designed for sequence modelling. It uses attention to decide which parts of the previous sequence are relevant for predicting the next output. In robotics, this lets a policy condition on histories of observations, actions, goals and sometimes rewards.

A **Decision Transformer** treats control as a sequence modelling problem. Instead of learning a value function or explicitly solving an optimal control problem, it predicts the next action conditioned on previous states, previous actions and a desired return. In simplified form:

$$
a_t = \pi_\theta(a_t \mid s_{1:t}, a_{1:t-1}, R_{1:t}),
$$

where $R$ represents a desired or remaining return. This is most naturally an offline learning method: it learns from a dataset of trajectories and then generates actions that resemble high-return behaviour.

A **vision-language-action model** goes one step further by conditioning actions on visual observations and language instructions:

$$
a_t = \pi_\theta(a_t \mid \text{images}, \text{language instruction}, \text{robot state}).
$$

The appeal is that language provides a flexible task specification. Instead of training a separate policy for every task, the instruction can say what the robot should do: "pick up the red block", "put the spoon in the drawer", or "move the object next to the cup". Modern VLA systems connect robot action learning to the large-scale pretraining used in vision-language models.

These models are powerful, but they are not magic. They usually need large datasets, careful action representations, strong safety layers and a reliable low-level controller. For this unit, the important point is not that transformers replace robotics structure. The important point is that they provide a model class for learning from large, diverse sequences of robot experience.

### How to choose

There is no universally best model class. The choice should follow the structure of the problem.

| Model class | Best suited for | Main advantage | Main limitation |
|---|---|---|---|
| Deterministic neural policy | Simple continuous control from observations | Simple and fast | Averages multi-modal actions |
| Mixture density policy | Multiple valid actions | Represents several modes | Must choose number of components |
| Diffusion policy | Complex multi-modal action sequences | Flexible action distributions | Slower inference |
| DMP | Smooth goal-directed motion | Stable and data-efficient | Limited to structured movements |
| Probabilistic movement primitive | Variable demonstrations and uncertain trajectories | Distribution over motions | Less suited to open-ended decisions |
| Decision Transformer | Offline trajectory datasets | Learns from long sequences | Needs good trajectory data |
| VLA model | Language-conditioned robot tasks | Flexible task specification | Data- and compute-hungry |

The practical guideline is simple: start with the simplest model that expresses the structure you need. Use a motion primitive when the task is naturally a movement. Use a probabilistic policy when there are multiple valid actions. Use a sequence or VLA model when history, task context or language is central. Use classical controllers wherever possible to stabilise the low-level behaviour.


## Representations and latent variables

So far, we have written policies as functions of observations:

$$
a_t = \pi_\theta(o_t).
$$

But raw observations can be very large. An image may contain hundreds of thousands of pixels. Most of those pixels are irrelevant to the current action.

A common strategy is to first learn or compute a lower-dimensional representation:

$$
z_t = f_\phi(o_t),
$$

and then act using

$$
a_t = \pi_\theta(z_t).
$$

The variable $z_t$ is a **latent representation**. It is not directly measured; it is inferred from observations. It might encode object positions, contact state, task progress, visual features or other information useful for control.

This connects back to state estimation. In recursive Bayes filtering, we maintain a belief over hidden state. In learned representation models, a neural network may learn a compact latent state from data.

For sequential tasks, the current observation may not be enough. A single image may not reveal velocity, intent or whether an object was already touched. The policy may need history:

$$
a_t = \pi_\theta(o_{1:t}).
$$

This can be handled by explicitly estimating state, or by using a temporal model such as a recurrent neural network or transformer. Conceptually, both are trying to solve the same problem: summarise past observations into a useful internal state.

### Inductive biases

An **inductive bias** is an assumption that helps a learning system generalise beyond its training data.

This definition is important. Learning from finite data is impossible without some bias. If many functions fit the data equally well, the learner needs a reason to prefer one over another. Linear regression has an inductive bias toward linear functions. A convolutional neural network has an inductive bias toward local spatial patterns. A Kalman filter has an inductive bias toward a particular probabilistic state-space model.

Robotics is full of useful inductive biases:

- rigid-body geometry,
- kinematic constraints,
- dynamics models,
- locality in images,
- temporal smoothness,
- conservation laws,
- stable controller structures,
- collision constraints,
- task structure,
- symmetries and coordinate frames.

These biases are not weaknesses. They are what make learning practical with limited robot data.

A useful taxonomy is based on where the structure enters the learning system.

#### Bias in the model

We may build known physics into the model and learn only unknown parameters or residuals:

$$
x_{t+1} = f_{\text{physics}}(x_t,u_t;\theta) + f_{\text{residual}}(x_t,u_t).
$$

This is the most direct extension of system identification.

#### Bias in the representation

We may choose state variables that make the problem easier. For example, using object poses instead of raw pixels can make a manipulation policy much easier to learn. The perception system still has to estimate those poses, but the controller receives a structured input.

#### Bias in the controller

We may wrap learning inside a stabilising controller:

$$
u_t = u_{\text{base}}(x_t) + u_\theta(o_t),
$$

where $u_{\text{base}}$ is a PD, impedance, LQR, iLQR or MPC controller, and $u_\theta$ is a learned correction. This can improve safety and data efficiency.

#### Bias in the data

The dataset itself encodes assumptions. Demonstrations show which states matter. Domain randomisation shows which variations the policy should ignore. Carefully designed excitation trajectories reveal useful dynamics for system identification.

The main message is that practical robot learning is not “just throw a neural network at it”. It is about choosing the right structure and learning the right missing piece.


## Practical workflow for robot learning

A practical robot learning project should start with the interface, not the neural network.

* First, decide what the policy or model should take as input. Will it use raw images, object poses, proprioception, force readings, goal poses or language instructions? The input should contain the information needed to solve the task, but unnecessary complexity makes learning harder.

* Second, decide what the model should output. For perception, this might be a class, mask or pose. For dynamics learning, it might be the next state or state difference. For imitation learning, it might be a velocity command, pose increment or gripper action.

* Third, collect data with timestamps. Robotics data is temporal. Observations, actions and states must be aligned in time. A small timestamp error can make a good model look bad or train a controller that behaves poorly.

* Fourth, choose a loss that matches the output. Use cross-entropy for discrete labels, squared error or likelihood losses for continuous outputs, and sequence losses when history matters.

* Fifth, evaluate in closed loop. Offline loss is useful for debugging, but robot behaviour must be tested by rolling out the policy or controller. A policy with low mean squared error can still fail if small errors compound over time.

* Finally, add safety constraints before deployment. Actions should be clipped, velocities and accelerations limited, workspaces bounded, sensors monitored and emergency stops available. A learned policy is still a controller connected to real hardware.


## Big picture

This week is the bridge between classical robotics and robot learning.

In classical robotics, we often start with known models:

$$
z_t = h(x_t) + v_t,
$$

$$
x_{t+1} = f(x_t,u_t),
$$

$$
u_t = \pi(x_t).
$$

In robot learning, one or more of these functions may be learned from data:

$$
\hat{x}_t = h_\theta(o_t),
$$

$$
\hat{x}_{t+1} = f_\theta(x_t,u_t),
$$

$$
u_t = \pi_\theta(o_t).
$$

The core idea is simple: collect input-output examples, choose a model class, define a loss, fit the parameters and evaluate the result. The robotics difficulty is that learned models operate inside feedback loops, under uncertainty, with safety constraints and limited data.

## Key papers and references

> Pomerleau, D. A. (1989). "ALVINN: An Autonomous Land Vehicle in a Neural Network." In *Advances in Neural Information Processing Systems (NeurIPS)*, vol. 1, pp. 305-313.

> Ross, S., Gordon, G., and Bagnell, D. (2011). "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning." In *Proceedings of AISTATS*, pp. 627-635.

> Ijspeert, A. J., Nakanishi, J., Hoffmann, H., Pastor, P., and Schaal, S. (2013). "Dynamical Movement Primitives: Learning Attractor Models for Motor Behaviors." *Neural Computation*, 25(2), 328-373.

> He, K., Gkioxari, G., Dollar, P., and Girshick, R. (2017). "Mask R-CNN." In *IEEE International Conference on Computer Vision (ICCV)*.

> Qi, C. R., Su, H., Mo, K., and Guibas, L. J. (2017). "PointNet: Deep learning on point sets for 3D classification and segmentation." In *IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*.

> Morrison, D., Corke, P., and Leitner, J. (2020). "Learning robust, real-time, reactive robotic grasping." *The International Journal of Robotics Research*, 39(2-3), 183-201.

> Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.

## Coming up next

This week introduced learning as an extension of modelling, estimation and control. We saw how data can be used to learn perception models, dynamics models and imitation policies.

Next week we look at what happens when the robot does not have demonstrations and instead learns by interacting with the world through rewards.

→ [Week 9: Reinforcement Learning](../week-09-rl/)

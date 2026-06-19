# Week 9: Reinforcement learning

So far in the unit, we have mostly assumed that we can write down a useful model of the robot and its environment. We used that model to estimate state, plan trajectories and compute control actions. Reinforcement learning asks a different question: what if we do not have a usable model, or the model is a complex simulator that is difficult to analyse directly? What if the task objective is hard to express as a classical controller, but easy to score after the fact? What if we want the robot to improve through experience?

This is the setting for **reinforcement learning** (RL). In RL, an agent interacts with an environment over time. At each step, it observes something about the world, chooses an action, receives a reward and updates its behaviour so that future rewards become larger. In robotics, the agent is usually the robot or the robot controller. The environment includes the robot body, the task, the objects, the terrain, the sensors and sometimes a simulator.

The core idea is simple:

> an agent learns a policy for choosing actions that maximise long-term reward.

This is closely related to the optimal control problems we have already seen. The language is different, but the structure is familiar: there is a state, an action, a dynamics model or transition process, and an objective over time.

## Markov decision processes

The standard mathematical model for reinforcement learning is a **Markov decision process**, or **MDP**. An MDP formalises sequential decision making under uncertainty when the agent can observe the full state of the system.

An MDP is usually written as the tuple

$$
\mathcal{M} = (\mathcal{S}, \mathcal{A}, P, R, \gamma),
$$

where $\mathcal{S}$ is the state space, $\mathcal{A}$ is the action space, $P(s' \mid s,a)$ is the transition probability, $R(s,a,s')$ is the reward function and $\gamma \in [0,1)$ is the discount factor.

The **state** $s_t \in \mathcal{S}$ is assumed to contain all the information needed to predict the future, given the action. This is the **Markov property**:

$$
P(s_{t+1} \mid s_t, a_t, s_{t-1}, a_{t-1}, \dots) = P(s_{t+1} \mid s_t, a_t).
$$

In words, once we know the current state and action, the earlier history does not provide any extra information about the next state. This assumption is very strong, but it is what makes the problem mathematically tractable.

A robot arm reaching task might define the state as the joint positions, joint velocities and object pose. The action might be a vector of joint torques or desired joint velocities. The transition model describes how the robot and object evolve after the action is applied. The reward might be positive when the gripper reaches the object, and negative for collisions, large torques or excessive time.

A **policy** is the agent's decision rule. It maps states to actions, either deterministically,

$$
a = \pi(s),
$$

or stochastically,

$$
\pi(a \mid s) = P(a_t = a \mid s_t = s).
$$

The objective is to find a policy that maximises expected return. The return from time $t$ is

$$
G_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \cdots
= \sum_{k=0}^{\infty} \gamma^k r_{t+k}.
$$

The discount factor $\gamma$ determines how much the agent values future rewards. A small $\gamma$ makes the agent short-sighted. A value close to one makes it plan further ahead.

## Partially observable Markov decision processes

Many robotics problems are not fully observable. A mobile robot does not know its exact pose; it estimates it from odometry, lidar, cameras or GPS. A manipulation robot may not know the exact pose, friction or mass of the object it is holding. A human-robot interaction system may not know the human's intent, attention or workload.

When the agent cannot directly observe the full state, the better model is a **partially observable Markov decision process**, or **POMDP**. A POMDP extends an MDP by adding observations and an observation model:

$$
\mathcal{P} = (\mathcal{S}, \mathcal{A}, P, R, \Omega, O, \gamma),
$$

where $\Omega$ is the observation space and

$$
O(o \mid s',a)
$$

is the probability of receiving observation $o$ after taking action $a$ and arriving in state $s'$.

In a POMDP, the agent does not condition its policy directly on the true state, because the true state is hidden. Instead, it must act using its observation history,

$$
h_t = (o_0, a_0, o_1, a_1, \dots, o_t),
$$

or using a **belief state**. The belief state is a probability distribution over possible true states:

$$
b_t(s) = P(s_t = s \mid o_{1:t}, a_{0:t-1}).
$$

This connects directly to state estimation. The Kalman filter, EKF, particle filter and SLAM methods from earlier weeks can all be viewed as ways of maintaining a belief over hidden state. In a POMDP, the policy is then written as

$$
\pi(a \mid b),
$$

rather than $\pi(a \mid s)$.

This is especially important in robotics because most real robot learning problems are at least partially observable. The robot almost never gets the true state directly. It receives sensor measurements, which are noisy, delayed, biased and incomplete. In practice, deep RL systems often deal with partial observability by stacking recent observations, using recurrent neural networks or feeding the policy an estimated state from a separate estimator.

The distinction is useful:

| Model | What the agent knows | Policy form | Robotics example |
|---|---|---|---|
| MDP | Full state $s_t$ | $\pi(a \mid s)$ | Simulated arm with perfect joint and object state |
| POMDP | Observations or belief | $\pi(a \mid h)$ or $\pi(a \mid b)$ | Mobile robot using noisy sensors to navigate |

MDPs are easier to solve and are the starting point for most RL theory. POMDPs are usually more realistic for robotics.

## Value functions and the Bellman equation

The central object in reinforcement learning is the **value function**. The value function tells us how good it is to be in a particular state, assuming we follow a particular policy.

The **state-value function** for policy $\pi$ is

$$
V^\pi(s) = \mathbb{E}_\pi \left[ G_t \mid s_t = s \right].
$$

This is the expected return starting in state $s$ and following $\pi$ thereafter.

The key observation is that value has a recursive structure. The value of a state is the immediate reward plus the discounted value of the next state:

$$
V^\pi(s) = \sum_a \pi(a \mid s) \sum_{s'} P(s' \mid s,a)
\left[ R(s,a,s') + \gamma V^\pi(s') \right].
$$

This is the **Bellman equation**. It says that long-horizon decision making can be decomposed into a one-step reward plus the value of what comes next.

> Bellman, R. (1957). *Dynamic Programming*. Princeton University Press.

We can also define the **action-value function**:

$$
Q^\pi(s,a) = \mathbb{E}_\pi \left[ G_t \mid s_t = s, a_t = a \right].
$$

This asks a slightly different question: if we are in state $s$, take action $a$, and then follow policy $\pi$, how much return should we expect?

The relationship between $V$ and $Q$ is

$$
V^\pi(s) = \sum_a \pi(a \mid s) Q^\pi(s,a).
$$

The optimal value function is

$$
V^*(s) = \max_\pi V^\pi(s),
$$

and it satisfies the **Bellman optimality equation**:

$$
V^*(s) = \max_a \sum_{s'} P(s' \mid s,a)
\left[ R(s,a,s') + \gamma V^*(s') \right].
$$

If we know $V^*$, we can recover an optimal policy by choosing the action that maximises the right-hand side. Much of RL is about estimating $V^*$, $Q^*$ or the policy directly, often without knowing the transition model.

## Temporal-difference learning

The Bellman equation assumes we can reason over transition probabilities. In practice, we often do not know $P(s' \mid s,a)$. Instead, we learn from experience.

**Temporal-difference learning** updates a value estimate from real transitions. After observing a transition $(s_t, r_t, s_{t+1})$, we update

$$
V(s_t) \leftarrow V(s_t) + \alpha
\left[ r_t + \gamma V(s_{t+1}) - V(s_t) \right],
$$

where $\alpha$ is the learning rate.

The term in brackets is the **TD error**:

$$
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t).
$$

The TD error measures surprise. If $\delta_t$ is positive, the outcome was better than expected and the value of $s_t$ is increased. If it is negative, the outcome was worse than expected and the value is decreased.

The important feature of TD learning is **bootstrapping**: we update one estimate using another estimate. We do not wait until the end of the whole trajectory. This makes learning far more practical for long-horizon problems.

> Sutton, R. S. (1988). "Learning to Predict by the Methods of Temporal Differences." *Machine Learning*, 3(1), 9-44.

## Dynamic programming: value iteration and policy iteration

If the model is known and the state and action spaces are finite, we can solve the MDP using dynamic programming.

**Value iteration** repeatedly applies the Bellman optimality update:

$$
V_{k+1}(s) = \max_a \sum_{s'} P(s' \mid s,a)
\left[ R(s,a,s') + \gamma V_k(s') \right].
$$

Starting from any initial value function, this process converges to $V^*$ under standard assumptions. Once the values have converged, the optimal policy is

$$
\pi^*(s) = \arg\max_a \sum_{s'} P(s' \mid s,a)
\left[ R(s,a,s') + \gamma V^*(s') \right].
$$

Value iteration is conceptually clean, but it requires a known model and a discrete state-action space. This limits its direct use on real robots, where states and actions are often continuous and high-dimensional.

**Policy iteration** takes a different approach. It alternates between policy evaluation and policy improvement. In the evaluation step, we compute the value function $V^\pi$ for the current policy. In the improvement step, we update the policy greedily with respect to that value function:

$$
\pi'(s) = \arg\max_a \sum_{s'} P(s' \mid s,a)
\left[ R(s,a,s') + \gamma V^\pi(s') \right].
$$

Each improvement step produces a policy that is at least as good as the previous one. Policy iteration often needs fewer outer iterations than value iteration, but the evaluation step can be expensive. Practical variants often use approximate or truncated evaluation.

Both methods are important because they show the structure of optimal decision making. Even when we cannot use them directly, many modern RL algorithms are built from the same Bellman recursion.

## Q-learning

Q-learning removes the need for a transition model. It directly learns the optimal action-value function $Q^*(s,a)$ from sampled experience.

After observing a transition $(s_t, a_t, r_t, s_{t+1})$, Q-learning performs the update

$$
Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha
\left[ r_t + \gamma \max_{a'} Q(s_{t+1}, a') - Q(s_t, a_t) \right].
$$

This is a temporal-difference update for action values. The target uses the best estimated next action, regardless of the action the current behaviour policy actually takes. For this reason, Q-learning is called **off-policy**: it can learn the optimal policy while following a different, exploratory policy.

A common exploration strategy is **$\epsilon$-greedy**. With probability $\epsilon$, the agent chooses a random action. With probability $1-\epsilon$, it chooses the action with the highest current $Q$ value. Early in training, $\epsilon$ is usually large to encourage exploration. Later, it is reduced so that the agent exploits what it has learned.

Under suitable conditions, tabular Q-learning converges to $Q^*$ for finite MDPs. The optimal policy is then

$$
\pi^*(s) = \arg\max_a Q^*(s,a).
$$

> Watkins, C. J. C. H. and Dayan, P. (1992). "Q-learning." *Machine Learning*, 8(3-4), 279-292.

Q-learning is powerful because it does not need an explicit model. Its limitation is that the tabular version requires us to store a value for every state-action pair, which is impossible for most robotics problems.

## Deep reinforcement learning

Real robots have large or continuous state spaces. A robot may observe images, lidar scans, joint angles, velocities, contact forces and object poses. It is not possible to enumerate every state. Deep RL replaces tables with function approximators, usually neural networks.

### Deep Q-networks

A **Deep Q-Network** approximates the action-value function with a neural network:

$$
Q(s,a) \approx Q(s,a;\theta),
$$

where $\theta$ are learnable parameters.

The network is trained by minimising a squared TD error:

$$
\mathcal{L}(\theta) = \mathbb{E}
\left[
\left(
 r + \gamma \max_{a'} Q(s', a'; \theta^-) - Q(s,a;\theta)
\right)^2
\right].
$$

Here $\theta^-$ denotes a **target network**, a delayed copy of the Q-network used to stabilise learning. DQN also uses **experience replay**, where transitions are stored in a buffer and sampled randomly in mini-batches. This reduces temporal correlation in the training data and makes neural network optimisation more stable.

DQN was an important step in deep RL because it showed that a neural network could learn useful control policies from high-dimensional inputs such as pixels. However, standard DQN is mainly suited to discrete action spaces, which limits its direct applicability to many robot control problems.

> Mnih, V., Kavukcuoglu, K., Silver, D., et al. (2015). "Human-level control through deep reinforcement learning." *Nature*, 518(7540), 529-533.

### Actor-critic methods

For continuous control, robotics often uses **actor-critic** methods. These methods maintain two learned objects. The **actor** is the policy, usually written as

$$
\pi(a \mid s;\phi),
$$

and the **critic** estimates either a value function $V(s;\theta)$ or an action-value function $Q(s,a;\theta)$.

The critic evaluates how good the actor's actions are. The actor is then updated to choose actions that the critic rates more highly. This separation is useful because continuous actions cannot be handled by a simple max over a finite action table.

Modern actor-critic algorithms such as A3C, TD3 and SAC are widely used for simulated locomotion, manipulation and continuous control.

> Mnih, V., Badia, A. P., Mirza, M., et al. (2016). "Asynchronous Methods for Deep Reinforcement Learning." In *Proceedings of ICML*, pp. 1928-1937.

> Fujimoto, S., van Hoof, H., and Meger, D. (2018). "Addressing Function Approximation Error in Actor-Critic Methods." In *Proceedings of ICML*, pp. 1587-1596.

> Haarnoja, T., Zhou, A., Abbeel, P., and Levine, S. (2018). "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor." In *Proceedings of ICML*, pp. 1861-1870.

### Proximal policy optimisation

**Proximal Policy Optimisation**, or **PPO**, is one of the most widely used policy-gradient methods in deep RL. It is popular because it is relatively simple, stable and effective across a broad range of tasks.

The problem PPO addresses is that policy updates can be destructive. If we change the policy too aggressively, performance can collapse and recovery may be difficult. PPO therefore tries to improve the policy while keeping the new policy close to the old one.

Define the probability ratio

$$
r_t(\phi) = \frac{\pi(a_t \mid s_t;\phi)}{\pi(a_t \mid s_t;\phi_\text{old})}.
$$

PPO optimises the clipped surrogate objective

$$
\mathcal{L}^{\text{CLIP}}(\phi) = \mathbb{E}_t
\left[
\min\left(
 r_t(\phi) \hat{A}_t,
 \text{clip}(r_t(\phi), 1-\epsilon, 1+\epsilon) \hat{A}_t
\right)
\right],
$$

where $\hat{A}_t$ is an estimate of the **advantage**:

$$
\hat{A}_t = Q(s_t,a_t) - V(s_t).
$$

The advantage measures whether an action was better or worse than expected in that state. The clipping term prevents the new policy from moving too far away from the old policy in one update. This acts like a simple trust region.

PPO is often used as a default starting point for robotics RL, especially in simulation. It has been used for locomotion, manipulation and large-scale policy training. It is not always the most sample-efficient algorithm, but it is robust enough to be useful in many settings.

> Schulman, J., Wolski, F., Dhariwal, P., Radford, A., and Klimov, O. (2017). "Proximal Policy Optimization Algorithms." arXiv preprint arXiv:1707.06347.

## RL in robotics: promise and pain

The appeal of reinforcement learning in robotics is clear. Given a simulator and a reward function, a robot can in principle learn behaviour without a hand-designed controller, without an analytic dynamics model and without a carefully engineered planner.

This has led to impressive results in games, legged and humanoid locomotion, dexterous manipulation and some real-world robot systems. However, RL is not a magic replacement for modelling and control. It introduces its own practical difficulties.

**Reward engineering** is often the first problem. The algorithm optimises the reward that is specified, not the task we intended. If the reward has loopholes, the agent may exploit them. A robot trained to move quickly may learn unsafe motions unless safety is explicitly encoded. A robot trained to reach an object may learn to knock it over if the reward only cares about end-effector distance.

**Sample efficiency** is a second major issue. RL often requires millions or billions of environment interactions. On real hardware, this is slow, expensive and can damage the robot. This is why most robotics RL is trained in simulation first.

**Sim-to-real transfer** is then the next obstacle. Simulators differ from the real world in friction, contact, delays, compliance, sensor noise and unmodelled dynamics. A policy that works beautifully in simulation can fail immediately on real hardware. Domain randomisation addresses this by training across a distribution of simulated worlds, rather than one fixed simulator setting. The goal is to learn a policy that is robust to modelling error.

**Long-horizon credit assignment** is also difficult. If a task fails after hundreds of steps, it is hard to know which earlier action caused the failure. Sparse rewards make this worse because the agent receives little feedback about intermediate progress.

**Stability and reproducibility** remain persistent problems. Deep RL can be sensitive to hyperparameters, random seeds, network architecture and implementation details. Two implementations of the same algorithm may behave differently.

For these reasons, RL is best viewed as one tool in the robotics toolbox. It is especially useful when the task is hard to model or hand-design, but easy to simulate and score. It is less attractive when safety constraints are strict, data are expensive or a reliable model-based controller already exists.

## Big picture

Reinforcement learning reframes control as learning from interaction. Instead of deriving a controller directly from a known model, we define a reward and allow the agent to improve through trial and error.

The MDP provides the clean theoretical model: the agent observes the state, acts, receives reward and transitions to a new state. The POMDP is the more realistic robotics model: the true state is hidden, sensors are imperfect and the robot must act under uncertainty.

Bellman equations explain the recursive structure of optimal decision making. Temporal-difference learning shows how to learn values from experience. Q-learning learns action values without a model. Deep RL scales these ideas to high-dimensional observations and continuous control, using neural networks and actor-critic methods.

The central robotics lesson is that RL does not remove the need for good engineering. A successful robot learning system still needs sensible state representations, safe exploration, good simulation, careful reward design, robust estimation and attention to transfer from simulation to reality.

## Key Papers

> Bellman, R. (1957). *Dynamic Programming*. Princeton University Press.

> Sutton, R. S. (1988). "Learning to Predict by the Methods of Temporal Differences." *Machine Learning*, 3(1), 9-44.

> Watkins, C. J. C. H. and Dayan, P. (1992). "Q-learning." *Machine Learning*, 8(3-4), 279-292.

> Mnih, V., Kavukcuoglu, K., Silver, D., et al. (2015). "Human-level control through deep reinforcement learning." *Nature*, 518(7540), 529-533.

> Mnih, V., Badia, A. P., Mirza, M., et al. (2016). "Asynchronous Methods for Deep Reinforcement Learning." In *Proceedings of ICML*, pp. 1928-1937.

> Schulman, J., Wolski, F., Dhariwal, P., Radford, A., and Klimov, O. (2017). "Proximal Policy Optimization Algorithms." arXiv preprint arXiv:1707.06347.

> Fujimoto, S., van Hoof, H., and Meger, D. (2018). "Addressing Function Approximation Error in Actor-Critic Methods." In *Proceedings of ICML*, pp. 1587-1596.

> Haarnoja, T., Zhou, A., Abbeel, P., and Levine, S. (2018). "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor." In *Proceedings of ICML*, pp. 1861-1870.

> Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.

## Coming up next

We have now covered the major pillars of robot intelligence: representation, modelling, control, estimation, perception and learning.

Next, we ask how all of these ideas change when a human is in the loop.

→ [Week 10: Human Robot Interaction](../week-10-hri/)

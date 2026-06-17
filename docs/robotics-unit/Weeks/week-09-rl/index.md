# Week 9: Reinforcement learning

So far we have assumed we have a model of our robot, and used it to compute good controls. But what if we don't have that model? What if the cost function is too complex to write down? What if we just want the robot to *figure it out*?

This is the setting for **reinforcement learning (RL)**.

Recall from the overview that we can think about our problem in terms of a reward instead of a cost:

> an agent takes actions in an environment, receives rewards, and tries to maximise the total reward over time.

More formally, we define:

- **state** $s$: what the agent observes
- **action** $a$: what the agent does
- **reward** $r(s, a)$: signal from the environment
- **policy** $\pi(a|s)$: the agent's strategy

At each timestep the agent is in state $s_t$, takes action $a_t$, transitions to state $s_{t+1}$, and receives reward $r_t$. The goal is to find a policy $\pi$ that maximises the **expected return**:

$$
G_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \dots = \sum_{k=0}^{\infty} \gamma^k r_{t+k}
$$

The **discount factor** $\gamma \in [0, 1)$ controls how much the agent cares about the future. A small $\gamma$ means focus on immediate rewards. A $\gamma$ close to 1 means plan for the long term.

This is just the optimal control problem from week 1, written in different language.

## Bellman equation

The central idea in RL is the **value function**.

The **state value function** $V^\pi(s)$ tells us: if I follow policy $\pi$ from state $s$, how much reward do I expect in total?

$$
V^\pi(s) = \mathbb{E}_\pi \left[ G_t \mid s_t = s \right]
$$

The clever observation is that this has a **recursive structure**. The value of a state is the immediate reward plus the discounted value of the next state:

$$
V^\pi(s) = \sum_a \pi(a|s) \sum_{s'} p(s'|s,a) \left[ r(s,a,s') + \gamma V^\pi(s') \right]
$$

This is the **Bellman equation**. It says:

> the value now = reward now + discounted value later

> Bellman, R. (1957). *Dynamic Programming*. Princeton University Press.

We can also define the **action-value function** $Q^\pi(s, a)$, which asks: what is the expected return if I am in state $s$, take action $a$, and then follow policy $\pi$ afterwards?

$$
Q^\pi(s,a) = \mathbb{E}_\pi \left[ G_t \mid s_t = s, a_t = a \right]
$$

The relationship between $V$ and $Q$ is simple:

$$
V^\pi(s) = \sum_a \pi(a|s) Q^\pi(s,a)
$$

The **optimal value function** $V^*(s) = \max_\pi V^\pi(s)$ satisfies the **Bellman optimality equation**:

$$
V^*(s) = \max_a \sum_{s'} p(s'|s,a) \left[ r(s,a,s') + \gamma V^*(s') \right]
$$

If we knew $V^*$, we could recover the optimal policy greedily: always pick the action that maximises the right-hand side. The entire challenge of RL is computing these quantities without full knowledge of the environment.

## Temporal difference learning

Here is the first practical problem.

To compute $V^\pi(s)$ exactly, we would need to know all transitions $p(s'|s,a)$. In practice, we often don't. Instead we can **learn from experience**.

**Temporal difference (TD) learning** updates our estimate of $V$ using real transitions, without waiting to observe the full return.

The core idea: after observing a transition $(s_t, r_t, s_{t+1})$, we update:

$$
V(s_t) \leftarrow V(s_t) + \alpha \left[ r_t + \gamma V(s_{t+1}) - V(s_t) \right]
$$

The term in brackets is called the **TD error**:

$$
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

It measures how surprised we are. If $\delta_t > 0$, we got more reward than expected, so we increase our estimate of $V(s_t)$. If $\delta_t < 0$, we are disappointed, and decrease it.

The key point is **bootstrapping**: we update an estimate using another estimate. We don't wait to see the full trajectory — we update after every single step.

This is what makes TD learning fast and practical.

> Sutton, R. S. (1988). "Learning to Predict by the Methods of Temporal Differences." *Machine Learning*, 3(1), 9–44.

## Value iteration

Now suppose we *do* have a model (we know transitions and rewards). Can we solve for $V^*$ directly?

Yes — using **value iteration**.

The algorithm is simple. Start with an arbitrary estimate $V_0$, and repeatedly apply the Bellman optimality equation:

$$
V_{k+1}(s) = \max_a \sum_{s'} p(s'|s,a) \left[ r(s,a,s') + \gamma V_k(s') \right]
$$

We keep applying this update until $V$ stops changing:

$$
\| V_{k+1} - V_k \|_\infty < \epsilon
$$

It can be shown that this **converges** to $V^*$ for any starting point. Once we have $V^*$, the optimal policy is:

$$
\pi^*(s) = \arg\max_a \sum_{s'} p(s'|s,a) \left[ r(s,a,s') + \gamma V^*(s') \right]
$$

Value iteration is clean and works, but requires:

- a finite, discrete state and action space
- knowing the model $p(s'|s,a)$

For most real robots, neither is true.

## Policy iteration

**Policy iteration** is an alternative dynamic programming method. Instead of iterating on values directly, we alternate between two steps:

1. **Policy evaluation**: given the current policy $\pi$, compute $V^\pi$ by solving the Bellman equations (or iterating until convergence)

2. **Policy improvement**: update the policy greedily with respect to $V^\pi$:

$$
\pi'(s) = \arg\max_a \sum_{s'} p(s'|s,a) \left[ r(s,a,s') + \gamma V^\pi(s') \right]
$$

Repeat until the policy stops changing.

It can be shown that each improvement step is guaranteed to produce a policy at least as good as the current one, and the process converges to $\pi^*$.

Policy iteration often converges in fewer iterations than value iteration, but each evaluation step can be expensive. In practice, **truncated policy iteration** cuts this short and is often faster overall.

Both value iteration and policy iteration are examples of **dynamic programming**: they break the global optimisation problem into smaller subproblems and solve them recursively. They require a model and work over discrete spaces, which limits their direct applicability to real robots.

## Q-learning

Now we drop the model entirely.

**Q-learning** is a **model-free**, **off-policy** algorithm that directly learns the optimal action-value function $Q^*(s,a)$ from experience.

The update rule is:

$$
Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \left[ r_t + \gamma \max_{a'} Q(s_{t+1}, a') - Q(s_t, a_t) \right]
$$

This looks just like TD learning, but now we are updating $Q$ instead of $V$, and we take the max over next actions instead of following the current policy.

**Off-policy** means we can learn about the optimal policy while following a different (e.g. exploratory) policy. This is useful because we need to explore the environment to gather good data, but we also want to learn about what would happen if we behaved optimally.

A common exploration strategy is **$\epsilon$-greedy**: with probability $\epsilon$ take a random action, otherwise take the greedy action $\arg\max_a Q(s,a)$. Over time, we decay $\epsilon$ as the estimates improve.

Under mild conditions, Q-learning is guaranteed to converge to $Q^*$ for finite state and action spaces.

> Watkins, C. J. C. H. and Dayan, P. (1992). "Q-learning." *Machine Learning*, 8(3–4), 279–292.

The recovered optimal policy is simply:

$$
\pi^*(s) = \arg\max_a Q^*(s,a)
$$

No model needed. No planning needed. Just experience.

## Deep RL

Q-learning works when we can store $Q(s,a)$ in a table. But real robots have high-dimensional, continuous state spaces — camera images, joint angles, laser scans. We can't enumerate every state.

The solution: **approximate $Q(s,a)$ with a neural network**.

$$
Q(s,a) \approx Q(s,a;\theta)
$$

where $\theta$ are learnable parameters. This is **Deep Q-Networks (DQN)**, introduced by DeepMind in 2013.

> Mnih, V., Kavukcuoglu, K., Silver, D., et al. (2015). "Human-level control through deep reinforcement learning." *Nature*, 518(7540), 529–533.

The training objective is to minimise the **TD loss**:

$$
\mathcal{L}(\theta) = \mathbb{E} \left[ \left( r + \gamma \max_{a'} Q(s', a'; \theta^-) - Q(s, a; \theta) \right)^2 \right]
$$

Two key tricks make this stable:

- **Experience replay**: store past transitions $(s, a, r, s')$ in a buffer and sample mini-batches randomly. This breaks temporal correlations.
- **Target network**: a separate copy $\theta^-$ that is updated slowly. This prevents chasing a moving target.

DQN can handle high-dimensional inputs like raw pixels, but is limited to discrete action spaces.

For **continuous actions** we use **actor-critic** methods instead. These maintain two networks:

- **Actor** $\pi(a|s;\phi)$: the policy — maps states to actions
- **Critic** $V(s;\theta)$ or $Q(s,a;\theta)$: evaluates how good actions are

The critic provides a learning signal for the actor. The actor tries to improve based on that signal. Together they can handle continuous action spaces like joint velocities or wheel speeds.

This family of algorithms — **A3C**, **SAC**, **TD3** — forms the backbone of modern deep RL for robotics.

> Mnih, V., Badia, A. P., Mirza, M., et al. (2016). "Asynchronous Methods for Deep Reinforcement Learning." In *Proceedings of ICML*, pp. 1928–1937.

> Haarnoja, T., Zhou, A., Abbeel, P., and Levine, S. (2018). "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor." In *Proceedings of ICML*, pp. 1861–1870.

> Fujimoto, S., van Hoof, H., and Meger, D. (2018). "Addressing Function Approximation Error in Actor-Critic Methods." In *Proceedings of ICML*, pp. 1587–1596.

## PPO

**Proximal Policy Optimisation (PPO)** is currently one of the most widely used deep RL algorithms, especially for robotics and continuous control.

The core problem it solves: if we update our policy too aggressively, we can accidentally make it much worse, and it's hard to recover. We want to take the **largest update we can while staying close to the current policy**.

PPO achieves this by clipping the policy update. Define the **probability ratio**:

$$
r_t(\phi) = \frac{\pi(a_t|s_t;\phi)}{\pi(a_t|s_t;\phi_\text{old})}
$$

The clipped surrogate objective is:

$$
\mathcal{L}^\text{CLIP}(\phi) = \mathbb{E}_t \left[ \min\left( r_t(\phi) \hat{A}_t, \; \text{clip}(r_t(\phi), 1-\epsilon, 1+\epsilon) \hat{A}_t \right) \right]
$$

where $\hat{A}_t$ is the **advantage**: how much better was this action compared to what we expected?

$$
\hat{A}_t = Q(s_t, a_t) - V(s_t)
$$

The clip prevents the ratio from straying too far from 1, which enforces a **trust region** around the current policy without needing to solve a constrained optimisation problem.

Why is PPO popular?

- Simple to implement
- Relatively stable to train
- Works well across a wide range of tasks
- Scales to large networks and parallel environments

PPO is used to train locomotion policies for legged robots, manipulation policies for robot arms, and was used to train the OpenAI Dota and robotics hand policies. If you want a default starting point for a robotics RL problem, PPO is a sensible choice.

> Schulman, J., Wolski, F., Dhariwal, P., Radford, A., and Klimov, O. (2017). "Proximal Policy Optimization Algorithms." arXiv preprint arXiv:1707.06347.

## The promise and the pain.

So what is reinforcement learning actually good for?

The promise is compelling:

> given a simulator and a reward function, a robot can learn to solve complex tasks — with no hand-designed controller, no model, no demonstrations.

And there are genuine success stories. RL has produced impressive locomotion policies for legged robots, dexterous manipulation in simulation, and game-playing agents that exceed human performance. We have seen policies learned entirely in simulation deployed on real hardware.

But there is a long list of practical difficulties.

**Reward engineering is hard.** The algorithm will optimise whatever you specify, exactly. If your reward is slightly wrong, you will get surprising and often unhelpful behaviour. The robot will find every loophole you did not anticipate. Getting a reward function right for a real task is much harder than it sounds.

**Sample inefficiency.** Learning from scratch requires millions of environment interactions. In the real world, each interaction takes time and causes wear. Even in simulation, this can take days of compute. Model-based RL and offline RL try to address this, but it remains a significant bottleneck.

**Sim-to-real transfer.** We usually train in simulation because it is cheap and safe. But the real world is different from the simulator in subtle ways — surface friction, sensor noise, actuation delays, unmodelled dynamics. Policies trained in simulation often fail when deployed on the real robot. **Domain randomisation** (training across many simulated conditions) and **domain adaptation** (aligning simulation and reality) are active research areas.

**Credit assignment over long horizons.** If a robot fails after 500 steps, which action was the mistake? Assigning blame over long sequences is hard, and rewards that arrive late are difficult to learn from.

**Stability and reproducibility.** Deep RL is notoriously sensitive to hyperparameters, random seeds, and implementation details. A policy that works in one environment may completely fail in a slightly different one.

None of this means RL is not useful. It means RL is a powerful but sharp tool. Used carefully — with a good simulator, a well-designed reward, and realistic transfer conditions — it can solve problems that classical control cannot.

> The gap between "it works in simulation" and "it works on the robot" is the central challenge of applying RL to real systems.

The rest of the field is slowly closing that gap.

---

## Key Papers

> Bellman, R. (1957). *Dynamic Programming*. Princeton University Press.

> Sutton, R. S. (1988). "Learning to Predict by the Methods of Temporal Differences." *Machine Learning*, 3(1), 9–44.

> Watkins, C. J. C. H. and Dayan, P. (1992). "Q-learning." *Machine Learning*, 8(3–4), 279–292.

> Mnih, V., Kavukcuoglu, K., Silver, D., et al. (2015). "Human-level control through deep reinforcement learning." *Nature*, 518(7540), 529–533.

> Schulman, J., Wolski, F., Dhariwal, P., Radford, A., and Klimov, O. (2017). "Proximal Policy Optimization Algorithms." arXiv preprint arXiv:1707.06347.

> Haarnoja, T., Zhou, A., Abbeel, P., and Levine, S. (2018). "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor." In *Proceedings of ICML*, pp. 1861–1870.

> Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.

---

# Coming up next

We have now covered the major pillars of robot intelligence: representation, modelling, control, estimation, perception, and learning.

Next, we ask: how do all of these ideas change when a human is in the loop?

→ [Week 10: Human Robot Interaction](../week-10-hri/)


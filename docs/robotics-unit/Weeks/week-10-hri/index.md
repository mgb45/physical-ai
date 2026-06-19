# Week 10: Human-robot interaction

So far, we have mostly designed robots that act alone. We have defined their state, modelled their motion, estimated their pose, planned paths, controlled their dynamics and reasoned about perception and learning. This week we put a human in the loop.

That changes the problem substantially. The robot is no longer solving an optimisation problem in isolation. It is sharing space, goals, attention and responsibility with a person. Humans are uncertain, context-dependent and socially sensitive. They form opinions about robots. They trust them, fear them, get frustrated by them and anthropomorphise them. A robot that ignores these human factors can fail in practice even if its controller is mathematically sound.

**Human-robot interaction (HRI)** is the study of how to design robots that work effectively, safely and acceptably with people. It sits at the intersection of robotics, cognitive science, psychology, design, ethics and human factors engineering.

The central question for this week is:

> how should a robot behave when its environment contains people who also perceive, decide and act?

---

## Modelling humans

The first step is the same one we have taken throughout the unit: we need a model. However, modelling a human is harder than modelling a robot. Robots are designed systems. Their sensors, actuators, state variables and control laws are known, at least approximately. People are biological, cognitive and social agents. Their behaviour is shaped by physics, intent, attention, emotion, culture and context.

A useful way to manage this complexity is to model humans at multiple levels of abstraction.

At the **physical level**, a human is a body with pose, velocity, joints, mass and inertia. In a shared workspace, we may care about where the person's limbs are, how fast they are moving and whether their motion could intersect the robot's motion. The kinematics and dynamics ideas from earlier weeks still apply, although the human body is more difficult to observe and model than a robot arm.

At the **cognitive level**, a human is a goal-directed agent. People form beliefs about the world, have objectives and choose actions to achieve those objectives. One classical way to describe this is the **BDI framework**, in which an agent is modelled in terms of beliefs, desires and intentions. This gives us a way to reason about a person not merely as a moving obstacle, but as an agent trying to accomplish something.

> Rao, A. S. and Georgeff, M. P. (1991). "Modeling Rational Agents within a BDI-Architecture." In *Proceedings of the 2nd International Conference on Principles of Knowledge Representation and Reasoning*, pp. 473-484.

The key insight is that if the robot can infer what a person intends, it can predict their future behaviour and respond cooperatively.

At the **social level**, humans follow norms. People have expectations about personal space, turn-taking, eye contact, interruption and responsibility. A robot can be physically safe and still socially uncomfortable. For example, a mobile robot that stops too close to someone may not collide with them, but it may still feel invasive.

The concept of **proxemics**, introduced by Hall, describes social zones around a person. The exact distances vary with culture and context, but the broad idea is useful for robot design. The intimate zone is reserved for close contact, the personal zone is used for familiar social interaction, the social zone is appropriate for strangers and routine interaction, and the public zone is used for addressing groups.

> Hall, E. T. (1966). *The Hidden Dimension*. Doubleday.

For HRI, the important point is that the human is not just another object in the environment. The human has a body, goals, attention, preferences and social expectations. A compact state description might therefore look like

$$
x_\text{human} = (\text{pose}, \text{velocity}, \text{intent}, \text{attention}, \text{workload}, \text{social context}).
$$

This state is only partially observable. The robot can measure some variables directly, such as pose or gaze direction, but must infer others, such as intent, workload or trust.

---

## Human motion prediction

Human motion prediction is a central subproblem in HRI. If a robot shares physical space with people, it must predict where they may move next.

The simplest model treats a person as a point mass with constant velocity:

$$
x_{t+1} = x_t + v_t \Delta t.
$$

This can work over short horizons, but it fails when people slow down, turn, stop, avoid obstacles or interact with others. It is useful as a baseline, not as a complete human model.

A more expressive approach is the **Social Force Model**, which treats pedestrians as if they are influenced by attractive and repulsive forces. A person is attracted toward their goal and repelled by obstacles and other people. This gives a simple continuous model of crowd behaviour without explicitly representing high-level intent.

> Helbing, D. and Molnár, P. (1995). "Social Force Model for Pedestrian Dynamics." *Physical Review E*, 51(5), 4282-4286.

For richer models, we can treat the human as a decision-making agent with a policy, just like our robots. In that view, a person chooses actions to optimise some internal reward function. If we can infer that reward function from demonstrations, we can predict future behaviour. This leads to **inverse reinforcement learning (IRL)**, where the goal is to infer what objective would make the observed behaviour appear rational.

> Abbeel, P. and Ng, A. Y. (2004). "Apprenticeship Learning via Inverse Reinforcement Learning." In *Proceedings of ICML*, pp. 1-8.

Modern HRI systems often use learning-based trajectory predictors that take recent observations of human motion and output a distribution over future trajectories:

$$
p(x_{t+1:t+H}^\text{human} \mid x_{t-L:t}^\text{human}, z_{t-L:t}).
$$

The probabilistic form matters. The robot should not plan around a single predicted human trajectory as if it were certain. It should reason about a distribution of possible futures and choose actions that remain safe under uncertainty.

---

## Safety in human-robot interaction

When robots operate near people, safety is not optional. A robot arm moving at high speed can cause serious injury. Even a small mobile robot bumping into a person can be unacceptable in a public or clinical setting.

Safety in HRI has both standards-based and control-based aspects. International standards define baseline requirements for industrial and collaborative robots. Important standards include **ISO 10218**, which covers safety requirements for industrial robots and robot systems, and **ISO/TS 15066**, which provides guidance for collaborative robot operation.

These standards describe several modes of collaboration. In a **safety-rated monitored stop**, the robot stops when a human enters the workspace. In **speed and separation monitoring**, the robot adjusts its speed based on the distance to the human. In **power and force limiting**, the robot is allowed to continue moving near people, but its forces and torques are constrained to reduce the risk of injury.

Power and force limiting requires the robot to detect contact and respond safely. Contact may be detected using joint torque sensing, external force-torque sensors, tactile sensors or model-based residuals. Once contact is detected, the robot must not rigidly fight the human. This motivates **compliance control**.

A compliant robot behaves like a spring-damper system rather than a perfectly rigid position controller. A simple Cartesian impedance controller can be written as

$$
F = K(x_d - x) + D(\dot{x}_d - \dot{x}),
$$

where $K$ is stiffness, $D$ is damping, $x_d$ is the desired pose and $x$ is the measured pose. Lower stiffness allows the robot to yield under external forces. This makes physical interaction safer and often more useful, especially in tasks involving contact, handover or uncertain object locations.

Collision avoidance is the complementary strategy. Instead of responding safely after contact, the robot predicts and avoids contact before it occurs. In planning terms, the human becomes a dynamic obstacle with uncertain future motion. Rather than imposing a single deterministic constraint, the robot should reason about risk:

$$
\Pr(\text{collision over horizon } H) \leq \epsilon.
$$

This is a chance constraint. It says that the probability of collision over the planning horizon should remain below an acceptable threshold $\epsilon$. In practice, estimating this probability is difficult because human motion predictions are uncertain and the robot's own perception may be noisy.

**Shared control** sits between full autonomy and direct teleoperation. The human provides high-level intent or continuous input, while the robot modifies the action to improve safety or performance. For example, a powered wheelchair might follow the user's joystick command while subtly steering away from walls. The challenge is to assist without making the person feel overridden.

---

## Legibility, predictability and transparency

Safety is necessary, but it is not sufficient. A robot can be physically safe and still be frustrating, confusing or uncomfortable to work with. Good HRI also requires the human to understand what the robot is doing.

**Predictability** means that a person can anticipate the robot's future behaviour based on past experience. A predictable robot behaves consistently. Similar situations produce similar actions, which helps the human form a reliable mental model.

**Legibility** is different. A legible robot actively communicates its intention through its behaviour. The robot's motion is designed so that an observer can infer the goal as early as possible. Dragan, Lee and Srinivasa formalised this distinction: predictable motion matches what the observer expects the robot to do, while legible motion helps the observer infer what the robot is trying to do.

> Dragan, A. D., Lee, K. C. T., and Srinivasa, S. S. (2013). "Legibility and Predictability of Robot Motion." In *Proceedings of the ACM/IEEE International Conference on Human-Robot Interaction*, pp. 301-308.

These two objectives can conflict. Suppose a robot is reaching for one of two cups. The most efficient trajectory may move straight between them and only become disambiguated near the end. A more legible trajectory may curve slightly toward the intended cup early, allowing the human to infer the goal sooner.

**Transparency** concerns the robot's internal state and decision-making process. The human may need to know why the robot stopped, why it selected one route rather than another, whether it is waiting for permission or whether it has failed. Transparency can be communicated through motion, lights, sound, speech, displays or interface design.

Together, predictability, legibility and transparency ask:

> can the human understand what the robot is doing, why it is doing it and what it is likely to do next?

If the answer is no, trust and coordination degrade.

---

## Trust, acceptance and ethics

HRI is also a social and ethical problem. People decide whether to rely on a robot based on their assessment of its competence, reliability and intent. This is usually described as **trust**.

Trust must be calibrated. **Undertrust** occurs when a person refuses to use a robot that would help them. **Overtrust** occurs when a person relies on a robot beyond its actual capability. Both are dangerous. A good HRI system should support appropriate trust: the person should understand what the robot can and cannot do.

Technology acceptance models help explain whether people adopt a system. The **Technology Acceptance Model (TAM)** identifies perceived usefulness and perceived ease of use as major factors in adoption.

> Davis, F. D. (1989). "Perceived Usefulness, Perceived Ease of Use, and User Acceptance of Information Technology." *MIS Quarterly*, 13(3), 319-340.

For robots, perceived safety and social acceptability are also central. A robot may be technically useful but still rejected if it feels unsafe, intrusive or unpleasant to interact with.

The **Uncanny Valley** is a related idea in social and humanoid robotics. As robots become more human-like, people's responses often become more positive, but only up to a point. A robot that is almost human, but not quite, can produce discomfort.

> Mori, M., MacDorman, K. F., and Kageki, N. (2012). "The Uncanny Valley [From the Field]." *IEEE Robotics and Automation Magazine*, 19(2), 98-100.

Ethics adds another layer of responsibility. Robots that share human environments collect data, make decisions and influence people's behaviour. Important ethical issues include bias, consent, autonomy, accountability and privacy. A robot trained on data from a narrow population may perform poorly for other groups. A care robot that records video in a home may collect sensitive information. An autonomous system that makes a harmful decision raises questions about who is responsible: the user, the operator, the manufacturer or the designer.

These are not optional philosophical add-ons. They are design requirements.

---

## Human-robot teaming as a multi-agent system

Many real deployments involve humans and robots working as a team rather than simply coexisting in the same space. Examples include warehouse fulfilment, search and rescue, surgical assistance, field robotics, assistive care and shared manufacturing workcells.

A useful way to formalise this is as a **multi-agent system**. Let there be $N$ agents:

$$
\mathcal{A} = \{1,2,\dots,N\},
$$

where some agents are humans and some are robots. We can write

$$
\mathcal{A} = \mathcal{H} \cup \mathcal{R},
$$

with $\mathcal{H}$ the set of human agents and $\mathcal{R}$ the set of robot agents.

Each agent $i$ has a local state $x_i$, receives observations $z_i$, chooses actions $u_i$ and may communicate messages $m_i$ to other agents. The joint team state is

$$
X_t = (x_{1,t}, x_{2,t}, \dots, x_{N,t}),
$$

and the joint action is

$$
U_t = (u_{1,t}, u_{2,t}, \dots, u_{N,t}).
$$

The system evolves according to a joint transition model:

$$
X_{t+1} \sim p(X_{t+1} \mid X_t, U_t),
$$

and each agent receives only partial information:

$$
z_{i,t} \sim p(z_{i,t} \mid X_t).
$$

The team has some shared objective, such as completing a task efficiently and safely:

$$
J = \mathbb{E}\left[\sum_{t=0}^{T} r(X_t, U_t)\right],
$$

where the reward $r$ may encode task progress, safety, human workload, comfort, time, energy use and communication cost.

The ideal centralised solution would choose the full joint policy

$$
\pi(U_t \mid X_t),
$$

but this is rarely realistic. Humans and robots do not share perfect state information, and a robot cannot directly command the human's actions. Instead, each agent has its own policy based on its own observations, memory and communication:

$$
u_{i,t} \sim \pi_i(u_{i,t} \mid z_{i,0:t}, m_{i,0:t}).
$$

For a human-robot team, the robot's practical problem is therefore not simply "choose the best action". It is:

> choose robot actions that help the team objective, while accounting for the human's goals, beliefs, workload, likely actions and need for understandable behaviour.

This leads to several coupled subproblems.

**Task allocation** asks who should do what. Some tasks are better suited to robots because they require precision, repeatability, strength or operation in hazardous environments. Others are better suited to humans because they require judgement, dexterity, social reasoning or contextual interpretation. We can represent a task set as

$$
\mathcal{T} = \{\tau_1, \tau_2, \dots, \tau_M\},
$$

and define an assignment function

$$
a: \mathcal{T} \rightarrow \mathcal{A}.
$$

The allocation should minimise some team cost:

$$
a^* = \arg\min_a \; C(a; X_t),
$$

where the cost can include completion time, risk, human workload, robot battery use, required skill and dependency constraints between tasks.

**Coordination** asks how agents align their actions over time. Even if the task allocation is correct, the team can fail if agents act at the wrong time, block each other or make incompatible assumptions. Coordination requires each agent to maintain a model of what the others are doing and what they are likely to do next.

**Communication** reduces uncertainty between agents. Communication can be explicit, such as speech, text, gesture or interface messages, or implicit, such as motion that signals intent. Communication is useful but not free: too much communication can increase workload, interrupt concentration or slow the task.

**Situation awareness** is the shared understanding of the task state. Each team member needs to know what has been done, what remains, what others are doing and what constraints matter. Breakdowns in situation awareness are a common cause of failures in human-machine teams.

**Handover** is a particularly important teaming problem. When a robot physically hands an object to a person, both agents must coordinate position, timing, grip force, release timing and intent. A handover is not just a motion planning problem; it is a coupled multi-agent interaction with perception, prediction, communication and safety constraints.

The multi-agent view is useful because it makes the hidden structure of HRI explicit. The human is not merely an obstacle. The human is another agent in the system. The robot must reason about the joint state, the shared task and the fact that each agent has only partial information.

---

## Affective and social robotics

Some robots are designed not only to work near people, but to interact socially with them. Examples include healthcare companions, educational robots, customer-service robots and therapeutic robots.

**Affective computing** studies systems that can recognise, interpret or simulate affective states such as emotion, stress, engagement or frustration. In robotics, this often involves two related problems. The first is emotion or affect recognition: inferring a person's state from facial expression, body language, speech, gaze, posture or physiological signals. The second is expressive behaviour: generating robot actions that communicate appropriate social signals.

A social robot may use gaze, head motion, facial expression, posture, speech rhythm, gesture or screen-based expressions. Even simple behaviours can matter. Looking toward a speaker can signal attention. Nodding can signal acknowledgement. Turning toward an object can signal intent. Pointing can disambiguate a reference.

Non-verbal communication is therefore a major part of HRI. A robot that can interpret and produce gaze, gesture and body orientation can often interact more naturally than one that relies on speech alone.

The difficulty is that social norms vary across cultures, ages, contexts and individuals. A behaviour that feels friendly in one context may feel intrusive in another. Social robotics therefore requires adaptation and careful evaluation, not just clever behaviour generation.

---

## Personalisation

No two people interact with a robot in exactly the same way. They differ in physical ability, prior experience, cultural background, task expertise, trust, communication preference and current emotional state. A one-size-fits-all interaction policy is therefore unlikely to work well.

**Personalisation** means adapting robot behaviour to the individual user. This can happen over different timescales.

Short-term adaptation occurs within an interaction. If the person appears confused, the robot may slow down, provide more explanation or switch communication modality. If the person appears overloaded, the robot may reduce interruptions or take over more routine subtasks.

Long-term adaptation occurs across repeated interactions. The robot may learn that a particular user prefers concise instructions, slower motion, more autonomy, less speech, or a specific handover pose. This creates a user model:

$$
\theta_\text{user} = (\text{preferences}, \text{capabilities}, \text{experience}, \text{trust}, \text{communication style}).
$$

The robot then conditions its behaviour on this model:

$$
u_t \sim \pi_R(u_t \mid x_t, z_t, \theta_\text{user}).
$$

Personalisation can improve usability and acceptance, but it also raises privacy concerns. A robot that learns a user's routines, abilities and emotional patterns may hold sensitive information. The design of personalisation systems must therefore consider consent, data minimisation, storage, access and deletion from the beginning.

---

## Evaluation methods

HRI cannot be evaluated only by running a simulation and measuring tracking error. The central question is whether the interaction works for the human. This requires measuring task performance, experience, workload, trust, safety and social acceptability.

The standard method is a **user study**. Participants interact with the robot under controlled conditions while researchers measure outcomes of interest. These outcomes may include task completion time, error rate, intervention rate, subjective workload, perceived safety, trust, usability and behavioural responses.

Subjective measures are often collected using validated questionnaires. The **NASA Task Load Index (NASA-TLX)** measures perceived workload across dimensions such as mental demand, physical demand, temporal demand, effort, performance and frustration.

> Hart, S. G. and Staveland, L. E. (1988). "Development of NASA-TLX (Task Load Index): Results of Empirical and Theoretical Research." In *Advances in Psychology*, vol. 52, pp. 139-183.

Other common instruments include the **Godspeed questionnaire**, which measures perceptions such as anthropomorphism, animacy, likeability, perceived intelligence and perceived safety, and the **System Usability Scale (SUS)**, which provides a compact measure of usability.

Behavioural measures can also be informative. These include how close a person stands to the robot, where they look, whether they hesitate, how they recover from errors and whether they take over control. Physiological measures such as heart rate, skin conductance and eye tracking can provide additional evidence about stress, arousal or attention.

A useful early-stage technique is the **Wizard of Oz (WoZ)** study. In a WoZ study, a hidden human operator controls some or all of the robot's behaviour, giving participants the impression that the robot is autonomous. This allows researchers to test interaction concepts before building the complete autonomous system.

A persistent challenge is **ecological validity**. A short laboratory study may not capture how people behave around robots in real deployments over days, weeks or months. Longitudinal field studies are harder to run, but they often reveal issues that do not appear in controlled lab settings.

Because HRI studies involve people, they must comply with ethical review requirements. Participants need informed consent, privacy protections, the right to withdraw and clear information about what data is collected and how it will be used.

---

## Big picture

Human-robot interaction extends the robotics stack by adding people as active participants in the system. A human is not merely an obstacle to avoid or a command source to obey. A human is a sensing, deciding, communicating and adapting agent.

This means HRI combines several ideas from the rest of the unit:

- state estimation becomes human state and intent inference;
- planning becomes planning under human motion uncertainty;
- control becomes safe and compliant interaction;
- learning becomes personalisation and adaptation;
- evaluation becomes user-centred measurement.

The multi-agent view makes the main challenge clear. In a human-robot team, the robot must choose actions that make sense not only for its own dynamics and objective, but for the joint system of humans, robots, tasks, communication and shared context.

A technically correct robot is not necessarily a good teammate. A good teammate is safe, useful, understandable, adaptable and respectful of human autonomy.

---

## Key papers and references

> Helbing, D. and Molnár, P. (1995). "Social Force Model for Pedestrian Dynamics." *Physical Review E*, 51(5), 4282-4286.

> Abbeel, P. and Ng, A. Y. (2004). "Apprenticeship Learning via Inverse Reinforcement Learning." In *Proceedings of ICML*, pp. 1-8.

> Dragan, A. D., Lee, K. C. T., and Srinivasa, S. S. (2013). "Legibility and Predictability of Robot Motion." In *Proceedings of HRI*, pp. 301-308.

> Davis, F. D. (1989). "Perceived Usefulness, Perceived Ease of Use, and User Acceptance of Information Technology." *MIS Quarterly*, 13(3), 319-340.

> Mori, M., MacDorman, K. F., and Kageki, N. (2012). "The Uncanny Valley [From the Field]." *IEEE Robotics and Automation Magazine*, 19(2), 98-100.

> Hart, S. G. and Staveland, L. E. (1988). "Development of NASA-TLX (Task Load Index): Results of Empirical and Theoretical Research." In *Advances in Psychology*, vol. 52, pp. 139-183.

> Rao, A. S. and Georgeff, M. P. (1991). "Modeling Rational Agents within a BDI-Architecture." In *Proceedings of KR*, pp. 473-484.

> Hall, E. T. (1966). *The Hidden Dimension*. Doubleday.

---

# Coming up next

We have now covered the full picture: how to represent robots, model their motion, estimate their state, plan their actions, control their dynamics, learn from data and interact with people.

The field moves quickly, but the core structure remains the same. Robotics is about building systems that perceive, decide and act under uncertainty. Human-robot interaction adds one final and essential constraint: those systems must also work with and for people.

Good luck.

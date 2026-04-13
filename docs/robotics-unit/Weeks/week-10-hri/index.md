# Week 10 Human Robot Interaction

So far, we have designed robots that operate alone. Now we put a human in the loop.

This changes everything. The robot is no longer just solving an optimisation problem in isolation — it is doing so while sharing space, goals, and attention with a person. Humans are unpredictable, context-dependent, and they form opinions about robots. They trust them, fear them, get frustrated by them, and anthropomorphise them. A robot that ignores all of this will fail in practice, even if its control policy is mathematically perfect.

**Human robot interaction (HRI)** is the study of how to design robots that work effectively alongside people. It sits at the intersection of robotics, cognitive science, psychology, and ethics.

## Modelling humans

The first step is the same one we always take: we need a model. What kind of model do we need for a human?

This turns out to be much harder than modelling a robot. Robots follow equations. People are messier.

### Physical, cognitive, intent and social behaviour

We can think about humans at several levels of abstraction.

At the **physical level**, a human is a system with dynamics, just like our robot. They have a body with joints, mass, and inertia. They can move in predictable ways given the laws of physics, and we can apply the same kind of kinematics and dynamics we developed in weeks 7 and 8.

At the **cognitive level**, humans are goal-directed agents. They form *beliefs* about the world, have *desires* or objectives, and take *actions* to achieve them. This **BDI (Beliefs, Desires, Intentions)** framework gives us a mental model of the human as a rational agent

> Rao, A. S. and Georgeff, M. P. (1991). "Modeling Rational Agents within a BDI-Architecture." In *Proceedings of the 2nd International Conference on Principles of Knowledge Representation and Reasoning*, pp. 473–484. — much like our robot. The key insight is that if we can infer what a person *intends*, we can predict their future behaviour and respond cooperatively.

At the **social level**, humans follow norms. There are cultural conventions about personal space, turn-taking, and gaze. Robots that violate these norms feel uncomfortable even if they are physically safe. The *proxemics* framework, originally from sociology, classifies space around a person into zones:

> Hall, E. T. (1966). *The Hidden Dimension*. Doubleday.

- **Intimate zone** (~0–0.5 m): reserved for close contact
- **Personal zone** (~0.5–1.2 m): comfortable for social interaction
- **Social zone** (~1.2–3.5 m): appropriate for strangers
- **Public zone** (>3.5 m): addressing a group

A robot that enters the intimate zone uninvited will cause discomfort even if no physical contact occurs. Modelling social behaviour means being aware of these norms and incorporating them into the robot's decision making.

Putting this together, we can describe a human as an agent with:

$$
x_\text{human} = (\text{pose}, \text{velocity}, \text{intent}, \text{attention}, \text{social context})
$$

Each of these layers requires different sensing and inference machinery.

### How do they move?

Modelling human motion is a key subproblem, especially for robots that share physical space.

The simplest approach is to treat a pedestrian as a **point mass** and apply a *constant velocity* model. This is fine for short prediction horizons but fails as soon as people slow down, turn, or stop to interact.

A more expressive model is the **Social Force Model**, which treats each person as subject to attractive forces (towards their goal) and repulsive forces (away from obstacles and other people). This captures crowd behaviour well without needing to reason about intent explicitly.

> Helbing, D. and Molnár, P. (1995). "Social Force Model for Pedestrian Dynamics." *Physical Review E*, 51(5), 4282–4286.

For richer intent modelling, we can treat human motion as the result of a **policy** — just like our robot. Given a goal and a belief about the world, what trajectory would a rational agent choose? This leads to **inverse reinforcement learning** approaches that infer the reward function driving human behaviour from observed demonstrations.

> Abbeel, P. and Ng, A. Y. (2004). "Apprenticeship Learning via Inverse Reinforcement Learning." In *Proceedings of ICML*, pp. 1–8. Once we have that reward function, we can use it to predict future motion.

In practice, deep learning has become the dominant tool for short-horizon motion prediction from sensor data, with models that take a sequence of observed positions and output a distribution over future trajectories.

> The better our model of human motion, the earlier we can anticipate what a person is going to do — and the safer and more natural the robot's response.

## Safety

When a robot operates near people, safety is not optional. A robot arm that collides with a human at high speed can cause serious injury. Even a mobile robot bumping into someone at low speed is unacceptable.

### Standards, shared control, physical safety (collision avoidance and compliance control)

**International standards** define the baseline requirements. The key standards in industrial robotics are:

- **ISO 10218** (Parts 1 & 2): safety requirements for industrial robots and robot systems
- **ISO/TS 15066**: technical specification for *collaborative robot* operation, where humans and robots share a workspace

These standards define four modes of human-robot collaboration, ranging from **safety-rated monitored stop** (the robot freezes when a human enters the workspace) through to **power and force limiting** (the robot can continue to move but caps the forces it can exert).

**Power and force limiting** is perhaps the most practically useful mode. It requires two things. First, the robot must be able to *detect* contact. This is typically done via torque sensors at the joints or via an external force-torque sensor. Second, the robot must respond appropriately. This is where **compliance control** comes in.

A compliant robot behaves like a spring-damper system rather than a rigid position controller. When a force is applied, the robot yields. A simple Cartesian impedance controller defines:

$$
F = K(x_d - x) + D(\dot{x}_d - \dot{x})
$$

where $K$ and $D$ are stiffness and damping matrices, $x_d$ is the desired position, and $x$ is the actual position. When an unexpected contact force is detected, the robot effectively absorbs it rather than fighting it. This is why modern collaborative robots feel "soft" to touch — they are not weaker, they are actively compliant.

**Collision avoidance** is the complementary approach: predict and avoid contact before it happens. This requires a real-time model of the human's current position and predicted trajectory. The robot's motion planner then treats the human as a dynamic obstacle. The key challenge is that the human model is uncertain and the future trajectory is probabilistic, so we need to plan in terms of *risk* rather than hard constraints.

**Shared control** is a middle ground between full autonomy and direct teleoperation. The robot contributes some autonomous behaviour (eg. obstacle avoidance, goal-directed assistance) while the human retains control over high-level intent. A wheelchair navigation assistant, for example, might smoothly redirect the wheelchair to avoid a wall while still broadly following the user's steering input. The challenge is finding the right division of authority so that the human feels in control without being overridden.

## Legibility, predictability and Transparency

Safety is a necessary condition. But it is not sufficient. Even a perfectly safe robot can be deeply frustrating to work with if its behaviour is opaque.

**Predictability** is about whether a person can anticipate what the robot is going to do next, based on experience. A predictable robot is consistent: the same situation leads to the same behaviour. This builds mental models and reduces cognitive load.

**Legibility** goes further. A legible robot actively *communicates* its intentions through its motion, before it completes the action. Dragan et al. formalised this: a *predictable* trajectory minimises expected deviation from the robot's past behaviour; a *legible* trajectory is one that allows an observer to infer the robot's goal as quickly as possible.

> Dragan, A. D., Lee, K. C. T., and Srinivasa, S. S. (2013). "Legibility and Predictability of Robot Motion." In *Proceedings of the ACM/IEEE International Conference on Human-Robot Interaction (HRI)*, pp. 301–308. These are not the same thing. A robot fetching a cup might take a slightly indirect path that clearly signals "I am going to the cup" rather than the shortest-path trajectory that is ambiguous until the final moment.

**Transparency** is about making the robot's internal state and decision-making process understandable to the human. Why did the robot stop? Why did it choose this path rather than that one? If a person cannot answer these questions, trust degrades. Transparency is difficult because robot policies are often high-dimensional and non-interpretable. Research on **explainable AI** (XAI) is partly motivated by this need.

Together, legibility, predictability, and transparency address the question:

> Does the person know what the robot is doing, why it is doing it, and what it will do next?

If the answer is no, the interaction will feel unnatural, unsafe, or both.

## Trust acceptance and ethics

Human-robot interaction is also a social and ethical problem.

**Trust** is a central concept. People decide whether to delegate a task to a robot based on their assessment of its competence, reliability, and intent. Appropriate trust calibration matters: both undertrust (the person ignores a helpful robot) and overtrust (the person defers to a robot that is actually making mistakes) are dangerous. Trust is known to degrade quickly after failures and rebuild slowly — which means one bad experience can undo many good ones.

Models of **technology acceptance** help us understand adoption. The **Technology Acceptance Model (TAM)** identifies two primary drivers: *perceived usefulness* (does the robot help me?) and *perceived ease of use* (is it easy to interact with?).

> Davis, F. D. (1989). "Perceived Usefulness, Perceived Ease of Use, and User Acceptance of Information Technology." *MIS Quarterly*, 13(3), 319–340. In HRI, a third factor is often added: *perceived safety*. People will not use a robot they consider risky, even if it is objectively safe.

The **Uncanny Valley** is a well-known phenomenon in social and humanoid robotics.

> Mori, M., MacDorman, K. F., and Kageki, N. (2012). "The Uncanny Valley [From the Field]." *IEEE Robotics and Automation Magazine*, 19(2), 98–100. As robots become more human-like in appearance, human reaction to them becomes increasingly positive — up to a point. When the robot is *almost* but not quite human, it provokes a strong sense of unease or revulsion. The valley describes this dip before the curve recovers for a fully realistic human appearance. The practical implication: robots that are clearly mechanical or clearly human tend to be received more warmly than those that fall in between.

**Ethics** adds a layer of responsibility. When a robot shares space with people, it makes decisions that affect them. Key ethical considerations include:

- **Bias**: if a robot is trained on data from a narrow demographic, it may perform poorly for others. A gesture recognition system that fails for users who are older, or from a different cultural background, is both unjust and unsafe.
- **Consent and autonomy**: people should understand what a robot can do and retain meaningful control, especially in care or medical settings.
- **Accountability**: when something goes wrong, who is responsible — the user, the operator, the manufacturer, or the designer?
- **Privacy**: robots with cameras and microphones that operate in homes or workplaces collect sensitive data. How is it stored, who can access it, and can it be used against the people it was collected from?

These are not hypothetical concerns. They are design requirements.

## Human robot teams

Many real applications place robots and humans in a *team*, where they divide work and coordinate to achieve a shared goal. Think of a warehouse with both human workers and autonomous mobile robots, or a surgical team where a robot assists a surgeon.

The fundamental problem in human-robot teaming is **task allocation**: who does what, and when? This involves:

- **Role assignment**: which tasks are best suited to the robot (speed, precision, hazardous environments) and which to the human (dexterity, judgement, communication)?
- **Situation awareness**: each team member needs to maintain an accurate model of what the others are doing, what has been done, and what remains. Breakdowns in situation awareness are a leading cause of failure in human-machine teams.
- **Communication**: how does the robot signal its current state and intentions? How does the human convey high-level goals or corrections? Speech, gesture, GUI interfaces, and motion are all channels.
- **Handover**: physically handing objects between humans and robots — and recovering gracefully when the timing is off — is a rich research problem in its own right.

Effective teaming requires the robot to maintain a model of the human's current task, attention, and workload, and to adapt its behaviour accordingly. A robot that aggressively interrupts a human who is in the middle of a high-concentration task is counterproductive, even if its own subtask is urgent.

## Affective and social robotics

Some robots are designed not just to work alongside humans but to *interact socially* with them — healthcare companions, educational robots, customer service assistants. These robots need to navigate the full complexity of human social and emotional signals.

**Affective computing** is the study of systems that can recognise, interpret, and simulate human affect (emotion). In robotics, this means:

- **Emotion recognition**: inferring emotional state from facial expressions, body language, tone of voice, or physiological signals such as heart rate.
- **Expressive behaviour**: generating robot behaviour that conveys appropriate emotional signals — a friendly tone, an attentive gaze, a reassuring gesture.

Human social robots typically have a face, eyes, or some display that can convey emotional states. Research shows that even very simple social signals — a robot that *looks* at you when you speak to it, or tilts its head when confused — dramatically improve people's perception of the interaction.

**Non-verbal communication** is a rich channel that HRI researchers have studied extensively. Gaze direction signals attention and intent. Nodding signals acknowledgement. Pointing disambiguates references. A robot that can both interpret and produce these signals can interact far more naturally than one that relies on speech alone.

The challenge is that social norms vary enormously across cultures, ages, and individuals. A behaviour that is warm and welcoming in one context may be invasive or inappropriate in another.

## Personalisation

No two people interact with a robot in the same way. They differ in physical capability, cultural background, prior experience with technology, current emotional state, task expertise, and personal preference.

A one-size-fits-all interaction policy will work poorly for everyone. **Personalisation** is the process of adapting the robot's behaviour to the individual.

This can happen at multiple timescales:

- **Short-term adaptation**: adjusting communication style or pace in real time based on immediate cues (the person looks confused, so the robot slows down and adds more explanation).
- **Long-term learning**: building a persistent user model that accumulates preferences over multiple sessions (this user prefers concise instructions; that user prefers to be shown rather than told).

The robot must infer user needs, which are often not stated explicitly. A person who takes a long time on a step may be struggling or may simply be careful. The robot needs to distinguish these cases before deciding whether to intervene.

Personalisation also raises the privacy concerns noted above. A robot that knows a great deal about a user — their routines, capabilities, emotional patterns — holds sensitive information. The design of personalisation systems must account for this from the outset.

## Evaluation methods

Unlike most of the technical content in this unit, HRI cannot be evaluated by running a simulation and measuring tracking error. The primary question in HRI is whether the interaction *works for the human*, which requires measuring human experience.

This means **user studies**. A user study involves recruiting participants, asking them to interact with the robot under controlled conditions, and measuring outcomes of interest. These might be:

- **Task performance**: did the team complete the task? How long did it take? How many errors were made?
- **Subjective experience**: how did the person feel? Validated questionnaires capture this systematically. Common instruments include:
  - **NASA Task Load Index (NASA-TLX)**: measures perceived workload across six dimensions (mental demand, physical demand, temporal demand, performance, effort, frustration)

> Hart, S. G. and Staveland, L. E. (1988). "Development of NASA-TLX (Task Load Index): Results of Empirical and Theoretical Research." In *Advances in Psychology*, vol. 52, pp. 139–183.
  - **Godspeed questionnaire**: measures anthropomorphism, animacy, likeability, perceived intelligence, and perceived safety
  - **System Usability Scale (SUS)**: a quick measure of overall usability
- **Behavioural measures**: proxemics (how close does the person stand?), gaze patterns, hesitation, recovery from errors.
- **Physiological measures**: heart rate, skin conductance, or eye tracking as objective proxies for arousal, stress, or attention.

**Wizard of Oz (WoZ)** studies are a useful technique early in the design process, before a full autonomous system exists. A human operator (the "wizard", hidden from the participant) remotely controls the robot's responses, giving the illusion of full autonomy. This lets researchers study human reactions to a notional robot behaviour without building the underlying system first.

A persistent challenge is **ecological validity**: lab studies with recruited participants may not capture how people behave in real deployments over extended periods. Longitudinal field studies are harder to run but produce more realistic data.

Finally, because people are the subject of these studies, HRI research must comply with ethical review requirements — informed consent, data privacy, the right to withdraw, and careful attention to any populations that may be vulnerable.

---

## Key Papers

> Helbing, D. and Molnár, P. (1995). "Social Force Model for Pedestrian Dynamics." *Physical Review E*, 51(5), 4282–4286.

> Abbeel, P. and Ng, A. Y. (2004). "Apprenticeship Learning via Inverse Reinforcement Learning." In *Proceedings of ICML*, pp. 1–8.

> Dragan, A. D., Lee, K. C. T., and Srinivasa, S. S. (2013). "Legibility and Predictability of Robot Motion." In *Proceedings of HRI*, pp. 301–308.

> Davis, F. D. (1989). "Perceived Usefulness, Perceived Ease of Use, and User Acceptance of Information Technology." *MIS Quarterly*, 13(3), 319–340.

> Mori, M., MacDorman, K. F., and Kageki, N. (2012). "The Uncanny Valley [From the Field]." *IEEE Robotics and Automation Magazine*, 19(2), 98–100.

> Hart, S. G. and Staveland, L. E. (1988). "Development of NASA-TLX (Task Load Index): Results of Empirical and Theoretical Research." In *Advances in Psychology*, vol. 52, pp. 139–183.

> Rao, A. S. and Georgeff, M. P. (1991). "Modeling Rational Agents within a BDI-Architecture." In *Proceedings of KR*, pp. 473–484.

> Hall, E. T. (1966). *The Hidden Dimension*. Doubleday.

---

# Coming up next

We have now covered the full picture — from how to represent and control a robot, to how it should behave in the company of people.

These tools give you a foundation for tackling real robotics problems. The field moves fast, the challenges are genuine, and the impact of getting it right is significant.

Good luck.


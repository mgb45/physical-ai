# Preface

Welcome to intelligent robotics. I am going to attempt to cram everything I know into 10 chapters.

But first some acknowledgements. These notes were informed by teaching materials and discussions developed over many years by colleagues and students including Dana Kulić, Juxi Leitner, Pamela Carreno-Medrano, Tan Chee Pin, Subramanian Ramamoorthy, Todor Davchev, Miguel Jaques, Daniel Angelov, Yordan Hristov, Artūras Straižys, Juyan Zhang, Haoyang Jiang, Boxuan Zhang, Bryce Ferenczi and many others. Any errors, ommissions or questionable editorial decisions are more own. Figures are AI generated placeholders.

Ok. Buckle up!

## All problems are optimal control problems

Let's start with some definitions. First, **ANY** robot task basically amounts to the following.

We seek to find a control input $u(t)$ and corresponding state trajectory $x(t)$ that minimize the cost

$$
\min_{u(\cdot)} \; J = \Phi(x(T)) + \int_{0}^{T} L(x(t), u(t), t)\, dt
$$

subject to the dynamical system

$$
\dot{x}(t) = f(x(t), u(t), t), \quad t \in [0, T].
$$

over some time *horizon* T. For convenience, and because we usually program and control our robots using computers, we often also rewrite that differential equation as a discrete *difference* equation, and turn that integral into a sum. 

The *dynamics* describes how the world or our robot changes in response to our actions (or because of other peoples actions or physics or something).

The cost $J$ defines our robot objective (eg. make coffee), the *state* $x$ is the minimal set of information we decide to use to represent our robot (or the environment, or the robot and the environment) and the *control* $u$ are the set of commands we can send to our robot. These may be positions, velocities, angles, motor current or voltages depending on the task and *actuators* our robot has available. This is also known as an *optimal control problem*.

Typically when we think about lower level continuous commands (eg. positions and velocities) we talk about controls, but sometimes if we are thinking a layer of abstraction higher we may refer to these as actions $a$. If we're a glass-half-full kind of person, we can also choose to think in terms of a *reward* instead of a cost, and we want to maximise some *return* over a time period. When we formulate the problem this way, we often talk about a *reinforcement learning* problem.

We may also talk about this as a decision making problem, in which case our controls or actions $a$ are decision variables. Sometimes, we may separate into two or more levels of abstraction (*a hierarchy*) and talk about *planning*: the set of high level actions we need to take to solve a task, and *control*: the low level actions we need to take to reach a *subgoal*, *setpoint* or *post condition*. We also need *perception* to figure out what is happening in the world, which relies on *sensors*. These sensors are often unreliable or *uncertain* so we may need to think in terms of *partial observability* or *probabalistic beliefs* when we are *inferring* the state of the world or our robot. We often describe robotics as the study of *decision making under uncertainty*, which as a general problem also covers everything else you could ever work on.

This leads to robot control paradigms like *sense*, *plan*, *act* or what we refer to as a *perception, planning, control* loop.

Regardless of how we formulate the problem, our goal as roboticists is to eventually come up with a *policy*, a set of rules or functions or code that decides what commands to send to our robot. We typically denote the policy as

$$
u(t) = \pi(x(t)).
$$

Note: $\pi \neq 3.14...$, it is just some function. If our policy needs to work across all forms of uncertainty, we may call it *robust*. Alternatively we may need to have an *adaptive* policy that can cope with a dynamic and changing world.  

This formulation is general (not just robots), so applies to any *autonomous system* or someone or thing with *agency*, which we sometimes call an *agent*. There may be multiple agents in the world, a *multi-agent* setting.

Ok. So this is a way for you to solve all problems, unit  is over. Yay. We can all go home.

## Why is robotics hard

Sorry. We know how to formulate the problem, but the real world is messy. 

1. What is the cost function to make coffee? How do we come up with good cost functions?
2. What should the state of our robot be? Its position, velocity, orientation? The position, velocity and orientation of everything in the world? What is the right *representation* for a given task? What is the position and orientation of a banana? How do you peel a banana?  We are going to deal with some big existential questions this semester.
3. How can I extract information from the world? What sensors should I use? How to combine uncertain information? 
4. How do we come up with the dynamics?
5. How do we solve for policies?
6. How do we model humans in the world?

More generally, what is *intelligence*?

We won't answer this question, but we will walk through a range of partial answers this semester. 

## How we answer these questions (partially)

1. Representing Robot Pose: We start with representations - for common robot forms, how do we describe their state in the world? 
2. Robot Modelling: We will look at some standard dynamics and *kinematic* models of robots and how we model these, starting with ground vehicles.
3. Robot control: We will look at specific controller designs and ways to find policies for a range of standard robots. 
4. SLAM part 1 (Localisation): We will look at a common state estimation problem, where is our robot in the world?  
5. SLAM part 2 (Planning and Navigation): We will continue to look at state estimation, what is the state of the world (or *map*) and how should we move through it. 
6. Articulated robots (kinematics): We will make our robots a bit more compicated and think about the state and representing robots with arms or multiple joints. 
7. Articulated robots (dynamics): We will think about the dynamics of these more complicated robot systems.
8. 6. Perception and learning: We will briefly look at cameras as a sensor, but also look at ways to *learn* policies from *demonstrations*.
9. Reinforcement learning: We will think about more general ways of solving the optimal control problem, when we don't have a lot of information.

We often think about robotics as both a science and system. We use science and theory to model or understand one component of the general problem, but our choices then lead to implications in other aspects of the system because everything is tightly coupled. Good roboticists manage to straddle the detail in each little subsystem, but also the big picture, how does everything work together?

As part of this unit, we will run a rolling project. We have deliberately simplified the robot - just a wheeled platform with very limited sensing and unusual dynamics. We will use this to work through all of the modelling and control choices above, as well as the practical difficulties of making an autnomous integrated sytem work in the real world. But, don't forget - these principles apply everywhere and to everything - we use the exact same approaches on our most complex humanoid robots in our research. 

## How to cite

If you find these notes useful, please cite them as:

```bibtex
@book{robots,
  title        = "Intelligent Robotics",
  howpublished = "Online textbook draft",
  author       = "Burke, Michael",
  year         = 2026,
}
```

## Let's dive in

[Chapter 1: Robot States →](./Chapters/chapter-01-state-representations/index.md)

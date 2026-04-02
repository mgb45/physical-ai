# Week 2 Robot control

Last week we talked about **how to represent our robot**.

This week we ask:

> ok… now how do we actually make it move?

More specifically:

> how do we choose the control inputs \(u\) so that the robot does what we want?

---

## Open loop control

Let’s start with the simplest idea.

We compute a sequence of controls ahead of time:

$$
u_0, u_1, u_2, \dots
$$

and then just execute them.

No checking. No correction. Just vibes.

This is called **open loop control**.

---

### Why might this work?

If:
- our model is perfect  
- the world is perfectly predictable  
- nothing disturbs the system  

then we can plan once and execute.

In fact, this is basically what a **planner** gives us: a trajectory and corresponding controls.

---

### Why does this fail?

Because the real world exists.

- wheels slip  
- sensors are noisy  
- models are wrong  
- humans do unpredictable things  

So what actually happens is:

> small errors → become big errors → robot ends up somewhere random

---

## Feedback control loops

So instead of blindly executing commands, we do something smarter.

We measure what actually happened, and correct for it.

This gives us a **feedback loop**:

- desired state \(x_d\)
- actual state \(x\)
- compute error
- apply control

You can think of it as:

> look → compare → correct → repeat

---

### Control problem formulation

At each time step:

- we have a desired state \(x_d\)
- we measure the current state \(x\)
- we apply a control \(u\)

The robot itself is often called the **plant**, and the controller is the thing choosing \(u\)

So the job of control is:

> choose \(u_k\) so that \(x_k \rightarrow x_d\)

---

## PD control

The most common starting point is **Proportional-Derivative (PD) control**.

We define an error:

$$
e(t) = x_d(t) - x(t)
$$

and then choose control as:

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$

---

### Intuition

- **Proportional term**:  
  push toward the goal  
  bigger error → bigger correction  

- **Derivative term**:  
  damp motion  
  prevents overshooting  

So PD is basically:

> go toward the goal, but don’t be too aggressive

---

### Example: driving to a goal

For a simple robot:

- state: position \((x, y)\), orientation \(\theta\)
- control: velocity \(v\), angular velocity \(\omega\)

We can:

1. compute the angle to the goal  
2. compute the distance to the goal  
3. apply proportional control:

$$
\omega_k = K_{p\theta} (\theta_g - \theta_k)
$$

$$
v_k = K_{p} \sqrt{(x_g - x_k)^2 + (y_g - y_k)^2}
$$

This is surprisingly effective :contentReference[oaicite:1]{index=1}.

---

### Why we like PD

- simple  
- fast  
- works surprisingly well  

---

### Why we don’t stop here

- no notion of optimality  
- requires tuning gains  
- struggles with complex systems  
- does not explicitly handle constraints  

---

## LQR (Linear Quadratic Regulator)

Now we step up a level.

Instead of saying:

> push toward the goal

we say:

> choose controls that **minimise a cost function**

---

### The setup

We assume a linear system:

$$
x_{k+1} = A x_k + B u_k
$$

and define a cost:

$$
J = \sum_{k=0}^{\infty} \left[(x_k - x_g)^T Q (x_k - x_g) + u_k^T R u_k \right]
$$

---

### What does this mean?

We are penalising:

- state error → \(Q\)  
- control effort → \(R\)

So we are balancing:

> get to the goal vs don’t use ridiculous control inputs

---

### The result

The optimal controller has the form:

$$
u_k = -K(x_k - x_g)
$$

So it still *looks* like feedback.

But now:

> the gains \(K\) come from solving an optimisation problem :contentReference[oaicite:2]{index=2}

---

### Big idea

PD control:
> pick gains → hope for good behaviour  

LQR:
> define objective → derive optimal gains  

---

## Iterative LQR (iLQR)

LQR is great… but only works for **linear systems**.

Robots are almost never linear.

So what do we do?

We cheat (in a principled way).

---

### The idea

For a nonlinear system:

$$
x_{k+1} = f(x_k, u_k)
$$

we:

1. start with a guess for controls  
2. simulate the trajectory  
3. locally approximate the system as linear  
4. solve LQR  
5. update controls  
6. repeat  

This is **iterative LQR (iLQR)** 

---

### Intuition

> repeatedly solve easier problems to approximate a hard one

---

### Limitations

- depends on initial guess  
- can get stuck in local minima  
- typically computed offline  

---

## MPC (Model Predictive Control)

Now we take one more step.

Instead of solving once and committing, we:

> solve → act → re-solve → act → repeat

---

### The idea

At each timestep:

1. plan over a short horizon  
2. compute optimal controls  
3. execute only the first control  
4. shift horizon forward  
5. repeat  

This is **Model Predictive Control (MPC)** 
---

### Why this is powerful

- naturally handles disturbances  
- adapts to new information  
- can incorporate constraints  

---

### Tradeoff

- more computationally expensive  
- requires solving optimisation repeatedly  

---

## Big picture

Let’s zoom out.

All of these methods are trying to answer the same question:

> how do we choose \(u\) so that the robot behaves the way we want?

We can think of them as a spectrum:

- **Open loop** → no feedback  
- **PD control** → simple feedback  
- **LQR** → optimal feedback (linear systems)  
- **iLQR** → approximate optimal control (nonlinear systems)  
- **MPC** → receding horizon, adaptive optimal control  

---

## Final thought

Control is where everything comes together:

- state representation  
- dynamics  
- objectives  

If Week 1 was:

> what is the state?

Week 2 is:

> what should I do given that state?

---

# Coming up next

We have been assuming we *know* the dynamics.

Next:

> how do we model how robots move?

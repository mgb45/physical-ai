# Week 1: State Representations

Let’s start with the first big question:

> **How should I represent my robot?**

This turns out to be one of the most important design decisions you’ll make.

---

## The simplest possible robot

The obvious starting point is to treat the robot as a **point in space**.

$$
\mathbf{p} =
\begin{bmatrix}
p_x \\
p_y \\
p_z
\end{bmatrix}
$$

This gives us a position in 3D.

But there’s a catch:

> These numbers only make sense **relative to some coordinate frame**.

So before we go any further, we need to talk about…

---

## Reference frames

A **reference frame** is just a coordinate system we use to describe things.

- The same point can have *different coordinates* depending on the frame
- We usually define:
  - a **world frame** (fixed)
  - a **robot frame** (attached to the robot)

So instead of saying “the robot is at (1, 2, 0)”, we should really say:

> “the robot is at (1, 2, 0) **in frame 0**”

This becomes important very quickly.

---

## Position **and** Orientation

A point only gives us position.

But robots aren’t just points — they have **orientation**.

Think:
- Which way is the robot facing?
- Where is its camera pointing?

So a robot’s state is really:

$$
x = (\text{position}, \text{orientation})
$$

---

## Rotations in 2D

Let’s start simple.

In 2D, orientation can be described by a single angle:

$$
\theta
$$

But a more powerful way is using a **rotation matrix**:

$$
R(\theta) =
\begin{bmatrix}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{bmatrix}
$$

What does this do?

> It tells us how to convert coordinates between frames.

For example:

$$
\mathbf{p}^0 = R_{01} \mathbf{p}^1
$$

Meaning:
- take a point expressed in frame 1
- convert it into frame 0 

---

## Rotations in 3D

In 3D, things get more interesting.

Instead of a single angle, we use a **3×3 rotation matrix**:

$$
R \in \mathbb{R}^{3 \times 3}
$$

Each column tells us:

> where the axes of one frame are expressed in another frame

We often build rotations from **elementary rotations**:

- Rotation about x-axis:
$$
R_x(\theta)
$$

- Rotation about y-axis:
$$
R_y(\theta)
$$

- Rotation about z-axis:
$$
R_z(\theta)
$$

For example:

$$
R_z(\theta) =
\begin{bmatrix}
\cos \theta & -\sin \theta & 0 \\
\sin \theta & \cos \theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$ 
:contentReference[oaicite:2]{index=2}

---

## Rotations don’t commute

This is a big deal:

> **Order matters.**

Rotating around x then z is NOT the same as:
rotating around z then x.

So we have to be very explicit about:
- the order of operations
- the frame we’re rotating in

---

## Translations

So far we’ve only rotated things.

What about moving them?

A translation is simple:

$$
\mathbf{p}^0 = \mathbf{p}^1 + \mathbf{d}
$$

Just shift by some vector.

---

## Rigid body motion

Now we combine both:

- rotation
- translation

This gives us **rigid body motion**:

$$
\mathbf{p}^0 = R_{01} \mathbf{p}^1 + \mathbf{d}
$$ 

This is the fundamental building block of robotics.

---

## Homogeneous transformations

To make life easier, we combine everything into one matrix:

$$
T =
\begin{bmatrix}
R & d \\
0 & 1
\end{bmatrix}
$$

Then we can write:

$$
\begin{bmatrix}
\mathbf{p}^0 \\
1
\end{bmatrix}
=
T
\begin{bmatrix}
\mathbf{p}^1 \\
1
\end{bmatrix}
$$

Why do this?

> Because now we can **chain transformations easily**:

$$
T_{0n} = T_{01} T_{12} \dots T_{n-1,n}
$$ 

---

## Big picture (important)

Let’s zoom out.

Everything we’ve done so far is about:

> **How do I describe where my robot is, and how it’s oriented?**

This matters because:
- control depends on state
- planning depends on geometry
- perception depends on frames

If you get representation wrong → everything else becomes painful.

---

# Representing the Environment

Ok, we’ve described the robot.

Now…

> **What does the world look like?**

---

## 2D metric maps

The simplest case:

- represent the world as a **2D plane**
- store exact geometry

Examples:
- floor plans
- lidar maps

---

## 3D metric maps

Now extend to 3D:

- point clouds
- meshes
- volumetric maps

More expressive, but:
- more compute
- more memory

---

## Occupancy grids

A very common representation.

We divide space into cells:

- each cell stores probability of being occupied

So instead of:
> “there is a wall here”

we say:
> “there is a 0.8 probability of a wall here”

This handles **uncertainty nicely**.

---

## Topological maps

Sometimes we don’t care about geometry.

Instead:

- nodes = places
- edges = connections

Like a graph:
- rooms
- corridors
- intersections

Great for:
- high-level planning

---

## Final thought

There is no “best” representation.

Only:

> **the right representation for the task**

And this is a recurring theme in robotics. The representations above quickly become limited in terms of tasks. In more complex cases, it may be better to *learn* representations instead.

---

# Coming up next

Now that we can represent state…

→ How do we actually **control** a robot?

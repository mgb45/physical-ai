# Week 5 Navigation and mapping

So far, we have:

- defined state
- built models of how the robot moves
- designed controllers to choose actions

But we have been quietly assuming something pretty unrealistic:

> we know where the robot is

This week we remove that assumption.

---

## SLAM

Let's start with the core problem:

> how do I know where I am if I don't know where anything is?

This is the **SLAM problem**:

> **Simultaneous Localisation and Mapping**

We want to estimate:

- the robot state
- the map of the environment

at the same time.

---

### Why is this hard?

Because:

- to build a map, I need to know where I am
- to know where I am, I need a map

So we get a circular dependency.

---

### The key idea

We solve both problems **jointly**.

As the robot moves:

- it updates its belief about its position
- it updates its belief about the map
- each helps improve the other

---

## Joint state estimation

Instead of estimating just the robot state:

$$
x_r = (x, y, \theta)
$$

we estimate a **joint state** that stacks the robot pose and all $N$ landmark positions:

$$
\mathbf{x} = \begin{pmatrix} x_r \\ m_1 \\ m_2 \\ \vdots \\ m_N \end{pmatrix} \in \mathbb{R}^{3 + 2N}
$$

where each landmark $m_i = (m_{i,x},\, m_{i,y})^\top$ is a 2D position in the world frame.

We maintain a Gaussian belief over this joint state:

$$
p(\mathbf{x}_k \mid z_{1:k}, u_{1:k}) = \mathcal{N}(\boldsymbol{\mu}_k,\, \boldsymbol{\Sigma}_k)
$$

The covariance $\boldsymbol{\Sigma}_k$ has a block structure:

$$
\boldsymbol{\Sigma}_k = \begin{pmatrix} \Sigma_{rr} & \Sigma_{rm_1} & \cdots & \Sigma_{rm_N} \\ \Sigma_{m_1 r} & \Sigma_{m_1 m_1} & \cdots & \Sigma_{m_1 m_N} \\ \vdots & & \ddots & \vdots \\ \Sigma_{m_N r} & \cdots & & \Sigma_{m_N m_N} \end{pmatrix}
$$

The off-diagonal blocks $\Sigma_{r m_i}$ capture the **correlations between robot pose and landmark positions**. This is the key insight: once the robot returns to a known landmark, uncertainty in the entire map is reduced.

---

## EKF SLAM

One concrete implementation uses the **Extended Kalman Filter (EKF)** to maintain this Gaussian belief.

---

### Predict step

The robot moves according to a nonlinear motion model (e.g. differential drive):

$$
x_{k+1} = f(x_{r,k},\, u_k) + \epsilon_k, \qquad \epsilon_k \sim \mathcal{N}(0, Q_k)
$$

The landmarks are stationary, so the full state prediction is:

$$
\overline{\boldsymbol{\mu}}_k = \begin{pmatrix} f(\mu_{r,k-1},\, u_k) \\ \mu_{m_1} \\ \vdots \\ \mu_{m_N} \end{pmatrix}
$$

We linearise the motion model with the Jacobian:

$$
F_k = \left.\frac{\partial f}{\partial x_r}\right|_{\mu_{r,k-1},\, u_k}
$$

and embed it into the full-state Jacobian:

$$
\mathbf{F}_k = \begin{pmatrix} F_k & 0 \\ 0 & I_{2N} \end{pmatrix}
$$

The predicted covariance is then:

$$
\overline{\boldsymbol{\Sigma}}_k = \mathbf{F}_k\, \boldsymbol{\Sigma}_{k-1}\, \mathbf{F}_k^\top + \mathbf{Q}_k
$$

where $\mathbf{Q}_k$ has $Q_k$ in the robot block and zeros elsewhere (landmarks do not move).

---

### Measurement model

Suppose the robot observes landmark $i$ with a range-bearing sensor. The expected observation is:

$$
\hat{z}_k^i = h^i(\overline{\boldsymbol{\mu}}_k) = \begin{pmatrix} \sqrt{(\mu_{m_i,x} - \bar{\mu}_x)^2 + (\mu_{m_i,y} - \bar{\mu}_y)^2} \\ \text{atan2}(\mu_{m_i,y} - \bar{\mu}_y,\; \mu_{m_i,x} - \bar{\mu}_x) - \bar{\mu}_\theta \end{pmatrix}
$$

The Jacobian $H_k^i$ maps the full state to the observation space. It is sparse: only the columns corresponding to the robot pose and landmark $i$ are non-zero:

$$
H_k^i = \left.\frac{\partial h^i}{\partial \mathbf{x}}\right|_{\overline{\boldsymbol{\mu}}_k}
$$

---

### Update step

For each observed landmark $i$:

$$
y_k^i = z_k^i - \hat{z}_k^i \qquad \text{(innovation)}
$$

$$
S_k^i = H_k^i\, \overline{\boldsymbol{\Sigma}}_k\, (H_k^i)^\top + R_k \qquad \text{(innovation covariance)}
$$

$$
K_k^i = \overline{\boldsymbol{\Sigma}}_k\, (H_k^i)^\top\, (S_k^i)^{-1} \qquad \text{(Kalman gain)}
$$

$$
\boldsymbol{\mu}_k = \overline{\boldsymbol{\mu}}_k + K_k^i\, y_k^i
$$

$$
\boldsymbol{\Sigma}_k = (I - K_k^i\, H_k^i)\, \overline{\boldsymbol{\Sigma}}_k
$$

Because the gain $K_k^i$ has non-zero entries for all landmarks (through the off-diagonal covariance blocks), **observing one landmark updates the estimated positions of all other landmarks** — this is the loop-closure effect.

---

### Initialising new landmarks

When a new landmark $j$ is seen for the first time, we initialise its position by inverting the observation model:

$$
\mu_{m_j} = \begin{pmatrix} \bar{\mu}_x + r\cos(\phi + \bar{\mu}_\theta) \\ \bar{\mu}_y + r\sin(\phi + \bar{\mu}_\theta) \end{pmatrix}
$$

where $(r, \phi)$ is the observed range and bearing. The landmark's initial uncertainty is computed via the Jacobian of this inverse model, propagating both sensor noise and the current robot pose uncertainty.

---

### Data association

Before updating, we must decide **which observed feature corresponds to which landmark** in the map. This is the *data association* problem. A common approach is nearest-neighbour matching using the Mahalanobis distance:

$$
d_M^2 = (z - \hat{z})^\top S^{-1} (z - \hat{z})
$$

A new landmark is initialised only when no existing landmark is within a threshold. Incorrect associations can corrupt the map, making data association one of the most challenging aspects of SLAM in practice.

---

### Key intuition

> uncertainty is shared between robot and map

When the robot re-observes a known landmark, the error in that observation feeds back through the correlated covariance to reduce uncertainty everywhere. This is **loop closure**: returning to a previously visited place dramatically improves the map.

> Smith, R., Self, M., and Cheeseman, P. (1986). "On the representation and estimation of spatial uncertainty." *International Journal of Robotics Research*, 5(4), 56–68.

> Durrant-Whyte, H. and Bailey, T. (2006). "Simultaneous Localisation and Mapping (SLAM): Part I." *IEEE Robotics and Automation Magazine*, 13(2), 99–110.

---

## Occupancy grid SLAM

Landmarks are nice.

But what if the world does not have nice discrete features?

---

### Alternative representation

We represent the world as a regular grid of cells. Each cell $c$ stores the **log-odds** of occupancy:

$$
l_c = \log \frac{p(c = \text{occ})}{1 - p(c = \text{occ})}
$$

Starting from a prior $l_0 = 0$ (i.e. $p = 0.5$), we update each cell that a sensor ray passes through or terminates at:

$$
l_c \leftarrow l_c + \log \frac{p(c = \text{occ} \mid z)}{1 - p(c = \text{occ} \mid z)} - l_0
$$

In practice this means:

- cells at the end of a ray (sensor hit) get a positive increment $l_\text{occ}$
- cells along the free part of a ray get a negative decrement $l_\text{free}$

Recovering the probability is always possible:

$$
p(c = \text{occ}) = 1 - \frac{1}{1 + e^{l_c}}
$$

The log-odds representation is numerically convenient: sequential updates are additions, and the values are symmetric around zero.

---

### What changes?

- the map has $W \times H$ cells, each updated independently
- we still need a robot pose estimate to know which cells each ray hits (ray-casting via Bresenham's line algorithm)
- because the map is so large, joint EKF-style estimation is infeasible

---

### Tradeoff

- flexible representation, works directly with lidar or depth cameras
- no need to extract discrete landmarks

but:

- computationally expensive for large environments
- requires accurate localisation to avoid blurry maps

---

## Rao-Blackwellisation and FastSLAM

At this point the joint state becomes enormous, especially with occupancy grids.

---

### The idea

We exploit the **conditional independence** structure of the SLAM problem. Given a full robot trajectory $x_{0:k}$, the map cells (or landmarks) become conditionally independent of each other:

$$
p(\mathbf{x}_{0:k}, m \mid z_{1:k}, u_{1:k}) = p(m \mid \mathbf{x}_{0:k}, z_{1:k})\; p(\mathbf{x}_{0:k} \mid z_{1:k}, u_{1:k})
$$

This factorisation is called **Rao-Blackwellisation**. Instead of estimating everything jointly, we:

1. **sample** robot trajectories with a particle filter
2. **maintain a separate map** for each particle, conditioned on its trajectory

---

### FastSLAM algorithm

Each particle $i$ stores:

- a robot trajectory hypothesis $x_{0:k}^{(i)}$
- a set of $N$ landmark EKFs $\{\mu_{m_j}^{(i)}, \Sigma_{m_j}^{(i)}\}_{j=1}^N$ (one per landmark, per particle)

**Predict:** sample a new pose from the motion model:

$$
x_k^{(i)} \sim p(x_k \mid x_{k-1}^{(i)}, u_k)
$$

**Update:** for each observed landmark $j$, compute the particle weight using the likelihood under that particle's landmark EKF:

$$
w_k^{(i)} \propto \mathcal{N}\!\left(z_k^j;\; h(x_k^{(i)}, \mu_{m_j}^{(i)}),\; H_k^{(i)}\Sigma_{m_j}^{(i)}(H_k^{(i)})^\top + R\right)
$$

Then update the relevant EKF for landmark $j$ in particle $i$.

**Resample** particles by weight.

FastSLAM scales as $O(M \log N)$ in the number of particles $M$ and landmarks $N$, compared to $O(N^2)$ for EKF SLAM — a significant practical improvement.

> Montemerlo, M., Thrun, S., Koller, D., and Wegbreit, B. (2002). "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem." In *Proceedings of the AAAI National Conference on Artificial Intelligence*, pp. 593–598.

---

## Global vs Local Planning

Now suppose we have a map. We can ask:

> how do we get from A to B?

Before we look at specific algorithms it is important to distinguish two levels of planning that operate simultaneously in most autonomous robots.

---

### Global planning

A **global planner** computes a path from the robot's current pose to a distant goal, using a complete (or nearly complete) map of the environment:

- operates over the full map
- runs relatively infrequently (once per goal, or when the map changes significantly)
- produces a **reference path** or **waypoint sequence**
- does not need to react to transient obstacles not in the map
- examples: A*, D*, RRT, PRM

---

### Local planning

A **local planner** is responsible for executing motion in the short term, around the robot's immediate vicinity:

- operates over a small local window (e.g. a 5 m radius around the robot)
- runs at high frequency (10–50 Hz)
- must react to **dynamic obstacles** not in the global map (people, other vehicles)
- tries to follow the global plan while avoiding immediate hazards
- examples: Dynamic Window Approach, tentacle-based navigation, trajectory rollout

---

### How they interact

The global planner provides a **desired heading or waypoint**, and the local planner finds a feasible motion toward that waypoint within the robot's kinematic constraints and the current local sensor field. If the local planner cannot make progress (e.g. a deadlock), the global planner must re-plan.

---

## Bug algorithms

Let's start with the simplest local reactive strategy.

Assume:

- we know where the goal is
- we can sense obstacles locally
- we do not have a map

---

### Bug 0

Move toward the goal. When a boundary is hit, follow it (e.g. always keep the obstacle on the left) until the direction to the goal is clear again, then resume heading for the goal.

**Algorithm:**

```
while not at goal:
    if path to goal is clear:
        move toward goal
    else:
        follow obstacle boundary
        if direction to goal is free:
            leave boundary and move toward goal
```

Bug 0 is not guaranteed to terminate in all environments — it can loop.

---

### Bug 1

Fully circumnavigate each obstacle encountered. Record the point on the boundary that is closest to the goal, and exit there.

**Algorithm:**

```
while not at goal:
    move toward goal
    if obstacle hit at hit point H:
        circumnavigate entire obstacle
        record the point L closest to the goal
        return to L
        leave toward goal
```

**Guarantee:** Bug 1 terminates if a path exists. The path length is at most:

$$
d(q_{\text{start}}, q_{\text{goal}}) + \frac{1}{2}\sum_{i} P_i
$$

where $P_i$ is the perimeter of obstacle $i$.

---

### Bug 2

Instead of circumnavigating the full boundary, follow it only until the **goal line** (the straight line from start to goal) is re-crossed at a point closer to the goal than the hit point.

**Algorithm:**

```
Compute the M-line: the straight line from start to goal
while not at goal:
    move along M-line toward goal
    if obstacle hit at H:
        follow obstacle boundary
        if M-line crossed at point L with d(L, goal) < d(H, goal):
            leave boundary, resume along M-line
```

**Guarantee:** terminates with path length bounded by:

$$
d(q_{\text{start}}, q_{\text{goal}}) + \frac{1}{2}\sum_{i} n_i P_i
$$

where $n_i$ is the number of times the M-line intersects obstacle $i$. Bug 2 is generally shorter than Bug 1 in practice.

These are simple and surprisingly effective in the right conditions.

> Lumelsky, V. J. and Stepanov, A. A. (1987). "Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape." *Algorithmica*, 2(1–4), 403–430.

---

### Limitations

- inefficient and path length can grow with obstacle perimeter
- no global optimality guarantee
- does not handle non-simply-connected environments well
- purely reactive: no benefit from accumulated map knowledge

---

## A* and D*

Now assume we **do have a map**.

We can turn planning into a graph search problem.

---

### Graph construction

We discretise the environment into a grid or a graph $G = (V, E)$:

- each node $n \in V$ represents a configuration (e.g. a grid cell)
- each edge $(n, n') \in E$ has a cost $c(n, n')$ (e.g. Euclidean distance between cell centres)

We seek a minimum-cost path from start $s$ to goal $g$.

---

### A* algorithm

A* maintains two quantities for each node $n$:

- $g(n)$: the **cost** of the cheapest path found so far from start to $n$
- $h(n)$: a **heuristic** estimate of the cost from $n$ to the goal

The priority function is:

$$
f(n) = g(n) + h(n)
$$

Nodes are expanded in order of increasing $f(n)$.

**Algorithm:**

```
open_set = {start}
g(start) = 0
f(start) = h(start)

while open_set is not empty:
    n = node in open_set with smallest f(n)
    if n == goal: return path

    remove n from open_set
    for each neighbour n' of n:
        tentative_g = g(n) + c(n, n')
        if tentative_g < g(n'):
            g(n') = tentative_g
            f(n') = g(n') + h(n')
            parent(n') = n
            add n' to open_set if not already there
```

---

### Admissibility and optimality

A heuristic $h(n)$ is **admissible** if it never overestimates the true cost to the goal:

$$
h(n) \leq h^*(n) \qquad \forall n
$$

where $h^*(n)$ is the true cheapest cost from $n$ to the goal. A common choice for grid-based planning is the Euclidean distance:

$$
h(n) = \|n - g\|_2
$$

or, when only 4-connected moves are allowed, the Manhattan distance:

$$
h(n) = |n_x - g_x| + |n_y - g_y|
$$

**Theorem (Hart et al., 1968):** if $h$ is admissible, A* returns an optimal path.

A* is also **complete**: if a path exists, it will be found.

> Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.

---

### D*

D* (Dynamic A*) extends A* to handle **changing maps**. Instead of searching from start to goal, it searches from goal to start, maintaining back-pointers. When an edge cost changes (e.g. a new obstacle is detected), only the affected portion of the plan is repaired rather than replanning from scratch.

> Stentz, A. (1994). "Optimal and Efficient Path Planning for Partially-Known Environments." In *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3310–3317.

---

## Probabilistic Roadmaps (PRM)

Instead of a grid, we can represent free space as a sparse graph by random sampling.

---

### Algorithm

**Build phase:**

```
while roadmap is not dense enough:
    q = sample random configuration
    if q is collision-free:
        add q to V
        for each q' in V near q:
            if path q→q' is collision-free:
                add edge (q, q') to E
```

**Query phase:**

```
connect q_start and q_goal to the roadmap
find shortest path using A* or Dijkstra
```

> Kavraki, L. E., Švestka, P., Latombe, J. C., and Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580.

---

### Strengths

- effective for high-dimensional configuration spaces (e.g. robot arms)
- roadmap can be reused for multiple queries in the same environment

---

### Weaknesses

- struggles with narrow passages (low probability of sampling a valid connection)
- assumes a static environment (no online re-planning)

---

## RRT

Instead of building a full roadmap, we grow a **single tree** from the start state toward the goal.

---

### Algorithm

```
T = {x_start}

for i = 1 to N:
    x_rand = sample random state (with probability p_goal, use x_goal)
    x_near = nearest node in T to x_rand
    x_new  = Steer(x_near, x_rand, step_size)

    if x_new is collision-free:
        add x_new to T, with parent x_near
        if ||x_new - x_goal|| < goal_threshold:
            return path from x_start to x_new
```

The **Steer** function moves from $x_\text{near}$ toward $x_\text{rand}$ by at most `step_size`:

$$
x_\text{new} = x_\text{near} + \Delta \cdot \frac{x_\text{rand} - x_\text{near}}{\|x_\text{rand} - x_\text{near}\|}
$$

> LaValle, S. M. (1998). "Rapidly-Exploring Random Trees: A New Tool for Path Planning." Technical Report TR 98-11, Iowa State University.

---

### Key property

The probability of sampling in any region of the configuration space is proportional to its volume (Voronoi bias), so the tree automatically expands toward unexplored space:

$$
P(\text{node } n \text{ extended}) \propto \text{Vol}(\text{Voronoi region of } n)
$$

---

### RRT*

The original RRT is not asymptotically optimal — the first solution found may be suboptimal and does not improve. **RRT*** adds two modifications:

1. **Near-set rewiring:** when adding $x_\text{new}$, check all nearby nodes to find the cheapest path to $x_\text{new}$:

$$
x_\text{parent} = \arg\min_{x' \in \text{Near}(x_\text{new})} \left[c(x_\text{start} \to x') + c(x' \to x_\text{new})\right]
$$

2. **Rewire the tree:** for each node $x'$ near $x_\text{new}$, check whether routing through $x_\text{new}$ is cheaper:

$$
\text{if } c(x_\text{start} \to x_\text{new}) + c(x_\text{new} \to x') < c(x_\text{start} \to x'):\quad \text{reparent } x' \text{ to } x_\text{new}
$$

RRT* is **asymptotically optimal**: as the number of samples $N \to \infty$, the path cost converges to the optimal.

> Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *International Journal of Robotics Research*, 30(7), 846–894.

---

### When to use RRT

- systems with complex kinematics or dynamics
- continuous, high-dimensional configuration spaces
- single-query planning (one start, one goal)

---

## Local planning methods

Global planners produce a reference path, but they cannot react quickly to dynamic obstacles or deal with the robot's kinematic constraints at low level. We need a **local planner**.

---

### Dynamic Window Approach (DWA)

The DWA searches directly in **velocity space** $(v, \omega)$ for a safe, goal-directed motion command.

**Step 1 — Dynamic window.** Given the current velocity $(v_c, \omega_c)$ and the robot's acceleration limits $(\dot{v}_\text{max}, \dot{\omega}_\text{max})$, the reachable velocities within one time step $\tau$ form a window:

$$
V_d = \{(v,\omega) \mid v \in [v_c - \dot{v}_\text{max}\tau,\; v_c + \dot{v}_\text{max}\tau],\quad \omega \in [\omega_c - \dot{\omega}_\text{max}\tau,\; \omega_c + \dot{\omega}_\text{max}\tau]\}
$$

We intersect this with the **admissible velocities** $V_a$ — those that allow the robot to stop before hitting an obstacle:

$$
V_a = \{(v,\omega) \mid v \leq \sqrt{2\, d(v,\omega)\, \dot{v}_\text{max}}\}
$$

where $d(v, \omega)$ is the distance to the nearest obstacle along the circular arc $(v, \omega)$ would trace.

**Step 2 — Score candidate velocities.** For each $(v, \omega)$ in $V_d \cap V_a$, simulate the arc for a short horizon and compute:

$$
G(v, \omega) = \sigma\!\left[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{clearance}(v,\omega) + \gamma \cdot v\right]
$$

where:

- $\text{heading}(v,\omega)$ measures alignment with the goal direction after the arc
- $\text{clearance}(v,\omega)$ is the distance to the nearest obstacle on the arc
- $v$ rewards higher speed
- $\alpha, \beta, \gamma$ are tunable weights
- $\sigma(\cdot)$ is a smoothing normalisation

**Step 3 — Select and execute:**

$$
(v^*, \omega^*) = \arg\max_{(v,\omega) \in V_d \cap V_a} G(v, \omega)
$$

DWA runs at sensor rate (typically 20–50 Hz) and produces smooth, kinematically feasible commands directly.

> Fox, D., Burgard, W., and Thrun, S. (1997). "The Dynamic Window Approach to Collision Avoidance." *IEEE Robotics and Automation Magazine*, 4(1), 23–33.

---

### Trajectory rollout

A close relative of DWA is **trajectory rollout**, which simulates each candidate velocity $(v, \omega)$ not just for one arc, but for a longer trajectory of $T$ steps:

$$
\mathbf{x}_{t+1}^{(v,\omega)} = f(\mathbf{x}_t^{(v,\omega)},\, v,\, \omega)
$$

and scores the full rollout:

$$
J(v,\omega) = \sum_{t=0}^{T} c(\mathbf{x}_t^{(v,\omega)})
$$

where $c(\cdot)$ can encode clearance, deviation from the reference path, and speed. This is an early form of **model predictive control** applied to local navigation.

---

### Driving with tentacles

An alternative to velocity-space search is **tentacle-based navigation**, in which a fixed set of candidate arcs (tentacles) is pre-computed offline.

Each tentacle $k$ is defined by a constant curvature $\kappa_k$:

$$
\kappa_k = \frac{\omega_k}{v}, \qquad k = 1, \dots, K
$$

giving $K$ pre-computed arc paths that fan out in front of the robot (typically $K = 7$–$15$, symmetric about the forward direction).

**At runtime:**

1. **Cast** each tentacle against the local occupancy grid. A tentacle is **blocked** if any cell it passes through is occupied.
2. **Score** each free tentacle:

$$
s_k = w_g \cdot \text{goal\_alignment}(\kappa_k) + w_c \cdot \text{clearance}(\kappa_k) + w_v \cdot v_k
$$

3. **Select** the highest-scoring free tentacle and apply the corresponding $(v, \omega)$ command.

Tentacle methods are extremely fast at runtime because the geometry is precomputed — only occupancy lookups and a linear scoring pass are needed. They are widely used in high-speed autonomous driving where compute is limited and reaction must be near-instantaneous.

> Urmson, C. et al. (2008). "Autonomous driving in urban environments: Boss and the Urban Challenge." *Journal of Field Robotics*, 25(8), 425–466.

---

### Comparison: local planning methods

| Method | Velocity space | Pre-computed | Handles kinematics | Reactive speed |
|---|---|---|---|---|
| Bug algorithms | No | No | No | Slow |
| DWA | Yes (continuous) | No | Yes | Fast |
| Trajectory rollout | Yes (discrete) | No | Yes | Moderate |
| Tentacles | Yes (discrete) | Yes (arcs) | Yes | Very fast |

---

## Big picture

Let's zoom out.

This week connects everything:

- modelling → how the robot moves
- estimation → where the robot is
- mapping → what the world looks like
- planning → what the robot should do (globally and locally)

The full navigation stack looks like:

```
Sensor data
    ↓
SLAM (localise + build map)
    ↓
Global planner (A*, RRT, …) → reference path
    ↓
Local planner (DWA, tentacles, …) → velocity commands
    ↓
Controller → wheel torques
```

Each layer operates at a different timescale and level of abstraction.

---

## Final thought

Robotics is not just:

> choosing actions

It is:

> choosing actions **under uncertainty, in a world we do not fully know, that changes while we act**

That is what makes it hard.

And also what makes it interesting.

---

## Key Papers

> Smith, R., Self, M., and Cheeseman, P. (1986). "On the representation and estimation of spatial uncertainty." *International Journal of Robotics Research*, 5(4), 56–68.

> Durrant-Whyte, H. and Bailey, T. (2006). "Simultaneous Localisation and Mapping (SLAM): Part I." *IEEE Robotics and Automation Magazine*, 13(2), 99–110.

> Montemerlo, M., Thrun, S., Koller, D., and Wegbreit, B. (2002). "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem." In *Proceedings of AAAI*, pp. 593–598.

> Lumelsky, V. J. and Stepanov, A. A. (1987). "Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape." *Algorithmica*, 2(1–4), 403–430.

> Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.

> Stentz, A. (1994). "Optimal and Efficient Path Planning for Partially-Known Environments." In *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3310–3317.

> Kavraki, L. E., Švestka, P., Latombe, J. C., and Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580.

> LaValle, S. M. (1998). "Rapidly-Exploring Random Trees: A New Tool for Path Planning." Technical Report TR 98-11, Iowa State University.

> Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *International Journal of Robotics Research*, 30(7), 846–894.

> Fox, D., Burgard, W., and Thrun, S. (1997). "The Dynamic Window Approach to Collision Avoidance." *IEEE Robotics and Automation Magazine*, 4(1), 23–33.

> Urmson, C. et al. (2008). "Autonomous driving in urban environments: Boss and the Urban Challenge." *Journal of Field Robotics*, 25(8), 425–466.

# Coming up next

We now have:

- models
- estimators
- planners (global and local)

Next:

> how do we *learn* these instead of hand-designing them?

→ reinforcement learning

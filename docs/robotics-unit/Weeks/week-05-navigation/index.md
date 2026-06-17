# Week 5 Navigation and mapping

So far, we have defined state, built models of how a robot moves and designed controllers that choose actions. Throughout that discussion, however, we have been quietly relying on a very strong assumption: we know where the robot is and what the world looks like. This week we remove that assumption.

## SLAM

Let’s start with the core problem:

> how do I know where I am if I don’t know where anything is?

This is the **SLAM problem**, or **simultaneous localisation and mapping**. In SLAM, the robot must estimate its own state and the map of the environment at the same time.

The difficulty is that these two estimates depend on each other. To build a map, the robot needs to know where it is. To know where it is, the robot needs a map. This creates a circular dependency.

The key idea in SLAM is to solve both problems **jointly**. As the robot moves, it updates its belief about its position and its belief about the map. Each estimate supports and improves the other.

---

## Joint state estimation

Lets illustrate this for a robot keeping track of its state using a topological map of N landmarks. Instead of estimating only the robot state,

$$
x_r = (x, y, \theta),
$$

we estimate a **joint state** that stacks the robot pose together with all $N$ landmark positions:

$$
\mathbf{x} = \begin{pmatrix} x_r \\ m_1 \\ m_2 \\ \vdots \\ m_N \end{pmatrix} \in \mathbb{R}^{3 + 2N}.
$$

Each landmark $m_i = (m_{i,x},\, m_{i,y})^\top$ is a 2D position in the world frame.

We maintain a Gaussian belief over this joint state:

$$
p(\mathbf{x}_k \mid z_{1:k}, u_{1:k}) = \mathcal{N}(\boldsymbol{\mu}_k,\, \boldsymbol{\Sigma}_k).
$$

The covariance matrix $\boldsymbol{\Sigma}_k$ has a block structure:

$$
\boldsymbol{\Sigma}_k = \begin{pmatrix} \Sigma_{rr} & \Sigma_{rm_1} & \cdots & \Sigma_{rm_N} \\ \Sigma_{m_1 r} & \Sigma_{m_1 m_1} & \cdots & \Sigma_{m_1 m_N} \\ \vdots & & \ddots & \vdots \\ \Sigma_{m_N r} & \cdots & & \Sigma_{m_N m_N} \end{pmatrix}.
$$

The off-diagonal blocks, such as $\Sigma_{r m_i}$, capture the **correlations between robot pose and landmark positions**. This is the key insight. Once the robot returns to a known landmark, uncertainty in the entire map can be reduced because the robot pose and landmark estimates are statistically linked.

---

## EKF SLAM

One concrete implementation of SLAM uses the **Extended Kalman Filter (EKF)** to maintain the Gaussian belief over robot pose and landmarks.

### Predict step

During the prediction step, the robot moves according to a nonlinear motion model, such as a differential-drive model:

$$
x_{k+1} = f(x_{r,k},\, u_k) + \epsilon_k, \qquad \epsilon_k \sim \mathcal{N}(0, Q_k).
$$

The landmarks are assumed to be stationary. This means the robot part of the state changes, while the landmark estimates remain the same:

$$
\overline{\boldsymbol{\mu}}_k = \begin{pmatrix} f(\mu_{r,k-1},\, u_k) \\ \mu_{m_1} \\ \vdots \\ \mu_{m_N} \end{pmatrix}.
$$

Because the motion model is nonlinear, we linearise it using the Jacobian

$$
F_k = \left.\frac{\partial f}{\partial x_r}\right|_{\mu_{r,k-1},\, u_k}.
$$

This robot-state Jacobian is then embedded into the full-state Jacobian:

$$
\mathbf{F}_k = \begin{pmatrix} F_k & 0 \\ 0 & I_{2N} \end{pmatrix}.
$$

The predicted covariance becomes

$$
\overline{\boldsymbol{\Sigma}}_k = \mathbf{F}_k\, \boldsymbol{\Sigma}_{k-1}\, \mathbf{F}_k^\top + \mathbf{Q}_k,
$$

where $\mathbf{Q}_k$ contains $Q_k$ in the robot block and zeros elsewhere, since landmarks do not move.

### Measurement model

Now suppose the robot observes landmark $i$ using a range-bearing sensor. The expected observation is

$$
\hat{z}_k^i = h^i(\overline{\boldsymbol{\mu}}_k) = \begin{pmatrix} \sqrt{(\mu_{m_i,x} - \bar{\mu}_x)^2 + (\mu_{m_i,y} - \bar{\mu}_y)^2} \\ \text{atan2}(\mu_{m_i,y} - \bar{\mu}_y,\; \mu_{m_i,x} - \bar{\mu}_x) - \bar{\mu}_\theta \end{pmatrix}.
$$

The Jacobian $H_k^i$ maps the full state to the observation space:

$$
H_k^i = \left.\frac{\partial h^i}{\partial \mathbf{x}}\right|_{\overline{\boldsymbol{\mu}}_k}.
$$

This matrix is sparse. Only the columns corresponding to the robot pose and landmark $i$ are non-zero, because a single landmark observation directly depends only on the robot pose and that landmark.

### Update step

For each observed landmark $i$, we first compute the innovation, which is the difference between the actual observation and the expected observation:

$$
y_k^i = z_k^i - \hat{z}_k^i.
$$

The innovation covariance is

$$
S_k^i = H_k^i\, \overline{\boldsymbol{\Sigma}}_k\, (H_k^i)^\top + R_k,
$$

and the Kalman gain is

$$
K_k^i = \overline{\boldsymbol{\Sigma}}_k\, (H_k^i)^\top\, (S_k^i)^{-1}.
$$

The mean and covariance are then updated as

$$
\boldsymbol{\mu}_k = \overline{\boldsymbol{\mu}}_k + K_k^i\, y_k^i
$$

and

$$
\boldsymbol{\Sigma}_k = (I - K_k^i\, H_k^i)\, \overline{\boldsymbol{\Sigma}}_k.
$$

Although the observation is of only one landmark, the gain $K_k^i$ can have non-zero entries for all landmarks because of the off-diagonal covariance blocks. This means that **observing one landmark can update the estimated positions of other landmarks**.

### Initialising new landmarks

When a new landmark $j$ is seen for the first time, we initialise its position by inverting the observation model:

$$
\mu_{m_j} = \begin{pmatrix} \bar{\mu}_x + r\cos(\phi + \bar{\mu}_\theta) \\ \bar{\mu}_y + r\sin(\phi + \bar{\mu}_\theta) \end{pmatrix},
$$

where $(r, \phi)$ is the observed range and bearing. The landmark’s initial uncertainty is computed using the Jacobian of this inverse model, propagating both the sensor noise and the current robot pose uncertainty.

### Data association

Before applying a measurement update, the robot must decide **which observed feature corresponds to which landmark** in the map. This is the *data association* problem.

A common approach is nearest-neighbour matching using the Mahalanobis distance:

$$
d_M^2 = (z - \hat{z})^\top S^{-1} (z - \hat{z}).
$$

A new landmark is initialised only when no existing landmark lies within a chosen threshold. Incorrect associations can corrupt the map, which makes data association one of the most challenging parts of practical SLAM.

### Key intuition

The key intuition is that uncertainty is shared between the robot and the map. When the robot re-observes a known landmark, the error in that observation flows back through the correlated covariance structure and can reduce uncertainty throughout the map. This is **loop closure**: returning to a previously visited place can dramatically improve the map.

> Smith, R., Self, M., and Cheeseman, P. (1986). "On the representation and estimation of spatial uncertainty." *International Journal of Robotics Research*, 5(4), 56–68.

> Durrant-Whyte, H. and Bailey, T. (2006). "Simultaneous Localisation and Mapping (SLAM): Part I." *IEEE Robotics and Automation Magazine*, 13(2), 99–110.

---

## Occupancy grid SLAM

Landmarks are useful when the world contains nice discrete features. But many environments do not naturally break into clean landmark observations. In those cases, we often use an occupancy grid.

An occupancy grid represents the world as a regular grid of cells. Each cell $c$ stores the **log-odds** of occupancy:

$$
l_c = \log \frac{p(c = \text{occ})}{1 - p(c = \text{occ})}.
$$

We usually start from a prior $l_0 = 0$, which corresponds to $p = 0.5$. Each time a sensor ray passes through or terminates at a cell, we update that cell:

$$
l_c \leftarrow l_c + \log \frac{p(c = \text{occ} \mid z)}{1 - p(c = \text{occ} \mid z)} - l_0.
$$

In practice, cells at the end of a sensor ray receive a positive occupancy increment $l_\text{occ}$, while cells along the free part of the ray receive a negative free-space decrement $l_\text{free}$.

We can always convert the log-odds value back into a probability:

$$
p(c = \text{occ}) = 1 - \frac{1}{1 + e^{l_c}}.
$$

The log-odds representation is convenient because sequential updates become additions, and the values are symmetric around zero.

The main change from landmark SLAM is the size of the map. Instead of a small set of landmark positions, the map may contain $W \times H$ cells, each updated independently. We still need a robot pose estimate to know which cells each ray passes through, often using ray-casting methods such as Bresenham’s line algorithm. However, because the map is so large, joint EKF-style estimation quickly becomes infeasible.

Occupancy grids are flexible and work directly with lidar or depth cameras without requiring discrete landmark extraction. The tradeoff is that they can be computationally expensive in large environments and they require accurate localisation; otherwise, the resulting map becomes blurry or inconsistent.

---

## Rao-Blackwellisation and FastSLAM

As the joint state grows, especially with occupancy grids, estimating everything together becomes expensive. FastSLAM addresses this by exploiting the **conditional independence** structure of the SLAM problem.

Given a full robot trajectory $x_{0:k}$, the map cells or landmarks become conditionally independent of each other:

$$
p(\mathbf{x}_{0:k}, m \mid z_{1:k}, u_{1:k}) = p(m \mid \mathbf{x}_{0:k}, z_{1:k})\; p(\mathbf{x}_{0:k} \mid z_{1:k}, u_{1:k}).
$$

This factorisation is called **Rao-Blackwellisation**. Instead of estimating everything jointly, we sample possible robot trajectories with a particle filter, and for each particle we maintain a separate map conditioned on that trajectory.

### FastSLAM algorithm

Each particle $i$ stores a robot trajectory hypothesis $x_{0:k}^{(i)}$ and a set of landmark EKFs,

$$
\{\mu_{m_j}^{(i)}, \Sigma_{m_j}^{(i)}\}_{j=1}^N,
$$

with one small EKF per landmark, per particle.

During prediction, a new pose is sampled from the motion model:

$$
x_k^{(i)} \sim p(x_k \mid x_{k-1}^{(i)}, u_k).
$$

During the update step, each observed landmark $j$ is used to compute the particle weight under that particle’s landmark estimate:

$$
w_k^{(i)} \propto \mathcal{N}\!\left(z_k^j;\; h(x_k^{(i)}, \mu_{m_j}^{(i)}),\; H_k^{(i)}\Sigma_{m_j}^{(i)}(H_k^{(i)})^\top + R\right).
$$

The relevant EKF for landmark $j$ is then updated inside particle $i$, and particles are resampled according to their weights.

FastSLAM scales as $O(M \log N)$ in the number of particles $M$ and landmarks $N$, compared with $O(N^2)$ for EKF SLAM. This is a significant practical improvement.

> Montemerlo, M., Thrun, S., Koller, D., and Wegbreit, B. (2002). "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem." In *Proceedings of the AAAI National Conference on Artificial Intelligence*, pp. 593–598.

---

## Global vs Local Planning

Once we have a map, we can ask a new question:

> how do we get from A to B?

Before looking at specific algorithms, it is useful to distinguish two levels of planning that operate together in most autonomous robots.

A **global planner** computes a path from the robot’s current pose to a distant goal using a complete, or nearly complete, map of the environment. It operates over the full map and usually runs relatively infrequently, such as once per goal or when the map changes significantly. Its output is normally a reference path or waypoint sequence. Global planners include methods such as A*, D*, RRT and PRM.

A **local planner**, in contrast, is responsible for short-term motion around the robot’s immediate surroundings. It operates over a small local window, often only a few metres around the robot, and runs at high frequency. It must react to dynamic obstacles such as people, other vehicles, or objects that were not in the global map. Local planners include the Dynamic Window Approach, tentacle-based navigation and trajectory rollout.

In practice, these two planners interact continuously. The global planner provides a desired heading, path or waypoint, while the local planner chooses a feasible motion toward that reference under the robot’s kinematic constraints and current sensor observations. If the local planner cannot make progress, for example because of a deadlock or newly discovered obstacle, the global planner must re-plan.

---

## Bug algorithms

Bug algorithms are among the simplest local reactive strategies. They assume that the robot knows where the goal is, can sense obstacles locally and does not have a map.

### Bug 0

Bug 0 moves directly toward the goal whenever possible. When the robot hits an obstacle boundary, it follows that boundary, for example by always keeping the obstacle on its left, until the direction to the goal is clear again. It then leaves the boundary and resumes heading for the goal.

```text
while not at goal:
    if path to goal is clear:
        move toward goal
    else:
        follow obstacle boundary
        if direction to goal is free:
            leave boundary and move toward goal
```

Bug 0 is simple, but it is not guaranteed to terminate in all environments. It can get trapped in loops.

### Bug 1

Bug 1 is more cautious. When it encounters an obstacle, it fully circumnavigates the obstacle boundary, records the boundary point closest to the goal and then leaves the obstacle from that point.

```text
while not at goal:
    move toward goal
    if obstacle hit at hit point H:
        circumnavigate entire obstacle
        record the point L closest to the goal
        return to L
        leave toward goal
```

Bug 1 is guaranteed to terminate if a path exists. Its path length is at most

$$
d(q_{\text{start}}, q_{\text{goal}}) + \frac{1}{2}\sum_{i} P_i,
$$

where $P_i$ is the perimeter of obstacle $i$.

### Bug 2

Bug 2 avoids full circumnavigation when possible. It uses the **M-line**, the straight line from start to goal. When the robot hits an obstacle, it follows the obstacle boundary only until the M-line is crossed again at a point closer to the goal than the original hit point.

```text
Compute the M-line: the straight line from start to goal
while not at goal:
    move along M-line toward goal
    if obstacle hit at H:
        follow obstacle boundary
        if M-line crossed at point L with d(L, goal) < d(H, goal):
            leave boundary, resume along M-line
```

Bug 2 is also guaranteed to terminate if a path exists. Its path length is bounded by

$$
d(q_{\text{start}}, q_{\text{goal}}) + \frac{1}{2}\sum_{i} n_i P_i,
$$

where $n_i$ is the number of times the M-line intersects obstacle $i$. In practice, Bug 2 is generally shorter than Bug 1.

These algorithms are simple and surprisingly effective in the right conditions.

> Lumelsky, V. J. and Stepanov, A. A. (1987). "Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape." *Algorithmica*, 2(1–4), 403–430.

Their limitations are also important. They can be inefficient, their path length can grow with obstacle perimeter, they provide no global optimality guarantee and they do not benefit from accumulated map knowledge. They are purely reactive methods.

---

## A* and D*

Now suppose we **do have a map**. Planning can then be formulated as a graph search problem.

We discretise the environment into a grid or graph $G = (V, E)$. Each node $n \in V$ represents a configuration, such as a grid cell, and each edge $(n, n') \in E$ has a cost $c(n, n')$, such as the Euclidean distance between cell centres. The goal is to find a minimum-cost path from a start node $s$ to a goal node $g$.

### A* algorithm

A* maintains two quantities for each node $n$. The first is $g(n)$, the cost of the cheapest path found so far from the start to $n$. The second is $h(n)$, a heuristic estimate of the remaining cost from $n$ to the goal.

The priority function is

$$
f(n) = g(n) + h(n).
$$

Nodes are expanded in order of increasing $f(n)$.

```text
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

A heuristic $h(n)$ is **admissible** if it never overestimates the true cost to the goal:

$$
h(n) \leq h^*(n) \qquad \forall n,
$$

where $h^*(n)$ is the true cheapest cost from $n$ to the goal. For grid-based planning, a common choice is Euclidean distance,

$$
h(n) = \|n - g\|_2,
$$

or Manhattan distance when only 4-connected moves are allowed:

$$
h(n) = |n_x - g_x| + |n_y - g_y|.
$$

**Theorem (Hart et al., 1968):** if $h$ is admissible, A* returns an optimal path. A* is also complete, meaning that if a path exists, it will be found.

> Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.

### D*

D*, or Dynamic A*, extends A* to handle **changing maps**. Instead of repeatedly replanning from scratch whenever the map changes, D* repairs only the affected portion of the plan. It searches from the goal back toward the start while maintaining back-pointers. When an edge cost changes, such as when a new obstacle is detected, only the relevant parts of the search tree need to be updated.

> Stentz, A. (1994). "Optimal and Efficient Path Planning for Partially-Known Environments." In *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3310–3317.

---

## Probabilistic Roadmaps (PRM)

Instead of planning on a dense grid, a Probabilistic Roadmap represents free space as a sparse graph built by random sampling.

During the build phase, random configurations are sampled. Collision-free configurations are added as graph nodes, and nearby nodes are connected if the path between them is also collision-free.

```text
while roadmap is not dense enough:
    q = sample random configuration
    if q is collision-free:
        add q to V
        for each q' in V near q:
            if path q -> q' is collision-free:
                add edge (q, q') to E
```

During the query phase, the start and goal configurations are connected to the roadmap, and a graph search method such as A* or Dijkstra’s algorithm is used to find a path.

```text
connect q_start and q_goal to the roadmap
find shortest path using A* or Dijkstra
```

> Kavraki, L. E., Švestka, P., Latombe, J. C., and Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces." *IEEE Transactions on Robotics and Automation*, 12(4), 566–580.

PRMs are effective for high-dimensional configuration spaces, such as robot arms, and the roadmap can be reused for multiple queries in the same environment. Their main weakness is that they struggle with narrow passages, because the probability of sampling useful connections in a narrow region can be low. They also assume a mostly static environment and are not naturally suited to online replanning.

---

## RRT

Rapidly-exploring Random Trees, or RRTs, take a different approach. Instead of building a full roadmap, an RRT grows a **single tree** from the start state toward unexplored regions of the space and, eventually, toward the goal.

```text
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
x_\text{new} = x_\text{near} + \Delta \cdot \frac{x_\text{rand} - x_\text{near}}{\|x_\text{rand} - x_\text{near}\|}.
$$

> LaValle, S. M. (1998). "Rapidly-Exploring Random Trees: A New Tool for Path Planning." Technical Report TR 98-11, Iowa State University.

The key property of RRT is its Voronoi bias. The probability of extending a node is proportional to the volume of its Voronoi region:

$$
P(\text{node } n \text{ extended}) \propto \text{Vol}(\text{Voronoi region of } n).
$$

This means the tree naturally expands toward unexplored regions.

### RRT*

The original RRT is not asymptotically optimal. The first solution found may be poor, and continuing to sample does not necessarily improve it. **RRT*** adds two ideas: near-set rewiring and tree rewiring.

When adding $x_\text{new}$, RRT* checks nearby nodes and chooses the parent that gives the cheapest path to $x_\text{new}$:

$$
x_\text{parent} = \arg\min_{x' \in \text{Near}(x_\text{new})} \left[c(x_\text{start} \to x') + c(x' \to x_\text{new})\right].
$$

Then, for each nearby node $x'$, it checks whether routing through $x_\text{new}$ would be cheaper:

$$
\text{if } c(x_\text{start} \to x_\text{new}) + c(x_\text{new} \to x') < c(x_\text{start} \to x'):\quad \text{reparent } x' \text{ to } x_\text{new}.
$$

RRT* is **asymptotically optimal**: as the number of samples $N \to \infty$, the path cost converges to the optimal cost.

> Karaman, S. and Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning." *International Journal of Robotics Research*, 30(7), 846–894.

RRT is especially useful for systems with complex kinematics or dynamics, continuous high-dimensional configuration spaces and single-query planning problems where there is one start and one goal.

---

## Local planning methods

Global planners produce a reference path, but they cannot react quickly to dynamic obstacles or directly handle low-level kinematic constraints. For that we need a **local planner**.

### Dynamic Window Approach (DWA)

The Dynamic Window Approach searches directly in **velocity space**, using candidate commands $(v, \omega)$, to find a safe and goal-directed motion.

The first step is to compute the dynamic window. Given the current velocity $(v_c, \omega_c)$ and the robot’s acceleration limits $(\dot{v}_\text{max}, \dot{\omega}_\text{max})$, the velocities reachable within one time step $\tau$ are

$$
V_d = \{(v,\omega) \mid v \in [v_c - \dot{v}_\text{max}\tau,\; v_c + \dot{v}_\text{max}\tau],\quad \omega \in [\omega_c - \dot{\omega}_\text{max}\tau,\; \omega_c + \dot{\omega}_\text{max}\tau]\}.
$$

This set is intersected with the **admissible velocities** $V_a$, which are the velocities that allow the robot to stop before hitting an obstacle:

$$
V_a = \{(v,\omega) \mid v \leq \sqrt{2\, d(v,\omega)\, \dot{v}_\text{max}}\}.
$$

Here, $d(v, \omega)$ is the distance to the nearest obstacle along the circular arc traced by the command $(v, \omega)$.

The second step is to score each candidate velocity in $V_d \cap V_a$. Each candidate arc is simulated over a short horizon and evaluated using

$$
G(v, \omega) = \sigma\!\left[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{clearance}(v,\omega) + \gamma \cdot v\right].
$$

The heading term measures alignment with the goal direction after the arc, the clearance term measures the distance to the nearest obstacle along the arc and the speed term rewards faster motion. The weights $\alpha$, $\beta$ and $\gamma$ tune the relative importance of these objectives, while $\sigma(\cdot)$ is a smoothing normalisation.

The final command is the highest-scoring admissible velocity:

$$
(v^*, \omega^*) = \arg\max_{(v,\omega) \in V_d \cap V_a} G(v, \omega).
$$

DWA runs at sensor rate, typically around 20–50 Hz, and directly produces smooth, kinematically feasible commands.

> Fox, D., Burgard, W., and Thrun, S. (1997). "The Dynamic Window Approach to Collision Avoidance." *IEEE Robotics and Automation Magazine*, 4(1), 23–33.

### Trajectory rollout

Trajectory rollout is closely related to DWA. Instead of scoring only a short arc, it simulates each candidate velocity $(v, \omega)$ over a longer trajectory of $T$ steps:

$$
\mathbf{x}_{t+1}^{(v,\omega)} = f(\mathbf{x}_t^{(v,\omega)},\, v,\, \omega).
$$

The full rollout is scored using a cost function such as

$$
J(v,\omega) = \sum_{t=0}^{T} c(\mathbf{x}_t^{(v,\omega)}),
$$

where $c(\cdot)$ can encode clearance, deviation from the reference path and speed. This can be viewed as an early form of **model predictive control** applied to local navigation.

### Driving with tentacles

Tentacle-based navigation uses a fixed set of candidate arcs, or tentacles, that are precomputed offline. Each tentacle $k$ is defined by a constant curvature

$$
\kappa_k = \frac{\omega_k}{v}, \qquad k = 1, \dots, K.
$$

This gives $K$ precomputed arc paths that fan out in front of the robot, often with $K = 7$–$15$ and symmetry about the forward direction.

At runtime, each tentacle is checked against the local occupancy grid. A tentacle is blocked if any cell along it is occupied. The free tentacles are then scored using

$$
s_k = w_g \cdot \text{goal\_alignment}(\kappa_k) + w_c \cdot \text{clearance}(\kappa_k) + w_v \cdot v_k.
$$

The robot selects the highest-scoring free tentacle and applies the corresponding $(v, \omega)$ command.

Tentacle methods are extremely fast at runtime because the geometry is precomputed. The online computation is mostly occupancy lookup and a linear scoring pass. This makes them useful in high-speed autonomous driving, where compute can be limited and reaction time must be very short.

> Urmson, C. et al. (2008). "Autonomous driving in urban environments: Boss and the Urban Challenge." *Journal of Field Robotics*, 25(8), 425–466.

### Comparing local planning methods

Bug algorithms are simple and reactive, but they do not operate in velocity space and do not explicitly handle robot kinematics. DWA searches continuous velocity space, handles kinematics and reacts quickly. Trajectory rollout also handles kinematics but simulates longer candidate trajectories, making it somewhat more computationally expensive. Tentacle methods use precomputed arcs, which makes them extremely fast, but less flexible than continuous optimisation over velocity space.

| Method | Velocity space | Pre-computed | Handles kinematics | Reactive speed |
|---|---|---|---|---|
| Bug algorithms | No | No | No | Slow |
| DWA | Yes (continuous) | No | Yes | Fast |
| Trajectory rollout | Yes (discrete) | No | Yes | Moderate |
| Tentacles | Yes (discrete) | Yes (arcs) | Yes | Very fast |

---

## Big picture

This week connects the major components of robot navigation. Modelling tells us how the robot moves. Estimation tells us where the robot is. Mapping tells us what the world looks like. Planning tells us what the robot should do, both globally and locally.

The full navigation stack can be viewed as

```text
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

Robotics is not just choosing actions. It is choosing actions **under uncertainty, in a world we do not fully know, that changes while we act**. That is what makes it hard, and also what makes it interesting.


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

We now have models, estimators and planners, both global and local. Next, we look at perception and learning: how robots sense and interpret the world around them.

→ [Week 6: Perception and Learning](../week-08-perception-and-learning/)

# Week 4 Localisation and state estimation

## Uncertainty and beliefs

So far, we have spoken about how to represent our robot and our world and model the dynamics of how things change when we interact with it, but we haven't discussed how we obtain this information. In practice we can never know the *ground truth* state of our robot and the world, and always measure or sense it subject to some error. There may also be completely un-modelled aspects on stochastic effects in the world, which also introduce uncertainty. We need methods to cope with this uncertainty - and the best language to think about uncertainty is probability. 


Typically, we are taught to reason about probabilities in terms of frequencies - if I roll a dice 10 times, what are the chances I roll a 6? This *frequentist* perspective doesn't really match the robotics problem we face, so instead we tend to think about probability as a belief, given what I have sensed in the world, I believe my state is x with some probability. 

In robotics, we can rarely trust our sensors. Instead, we may think about the *likelihood* of making some observation $z_t$, $p(z_t|x_t)$ in a given state. As an example, think about a temperature sensor that measures temperature with zero-mean gaussian noise of standard deviation 5 degrees. If the true temperature were 25 degrees, and our sensor measured 25.05 degrees, the likelihood of making this observation would be $\mathcal{N}(z_t|x_t,\sigma) = \mathcal{N}(25.05|25,5)$, quite probable. Here $\mathcal{N}$ is shortand for a Gaussian distribution over $z_t$ with mean $x_t$ and standard deviation $\sigma$. However, if the true temperature were 0 degrees, it is much less likely our temperature sensor would return a measurement of 25.05 degrees. 

But we may have other sources of informaton. What if for example we knew that our temperature sensor is located in Antartica? When we incorporate this $prior$, that 0 degree temperature starts to seem a lot more probable.

## Bayes Rule

Bayes rule is a law of probability (you can easily derive this using conditional probability laws) that gives us a principled way of combining prior information with likelihoods. We write

$$ p(x_t|z_t) = \frac{p(z_t|x_t)p(x_t)}{p(z_t)} $$

Our *posterior* belief, the probability that I am in state $x_t$, given the observation $z_t$ is equal to the likelihood of seeing observation $z_t$ if I were in state $x_t$, multipled by the prior belief that I was in state $x_t$, normalised by the *marginal likelihood* of ever seeing the observation $z_t$.

We call 

$$ p(z_t) = \int p(z_t|x_t) p(x_t) d x_t $$

a marginal likelihood because it marginalises out all the possible ways we could get an observation $z_t$, by averaging (takes an expectation) over all the states $x_t$ in proportion to the probabability of these occuring.

## Recursive state estimation

Bayes rule gave us a way of expressing our belief that our robot was in a state given a single observation, but in practice we are interested in the conditional probability distibution: $p(x_t|z_{1:t})$ - given observations or measurements $z$ taken at times steps $1$ to $t$, what is the probability that my robot (or world) is in the state $x_t$. We refer to this as a state estimation or tracking problem.

Using Bayes rule, we can express this as

$$p(x_t|z_{1:t}) = \frac{p(z_{1:t}|x_t) p(x_t)}{[p(z_{1:t})]} = \frac{p(z_{t}|x_t,z_{1:t-1}) p(z_{1:t-1}|x_t)|p(x_t)}{[p(z_t|z_{1:t-1})p(z_{1:t-1})]} \quad \text {factorising and conditioning the joint distribution}$$ 

$$ p(x_t|z_{1:t}) = \frac{p(z_{t}|x_t) p(x_t|z_{1:t-1})}{p(z_t|z_{1:t-1})} \quad \text{Simplifiying the likelihood and applying another Bayes rule}$$

Which can in turn be broken down into two steps:

$$ p(x_t|z_{1:t-1}) = \int p(x_t|x_{t-1}) p(x_{t-1}|z_{1:t-1}) d x_{t-1} \quad \text{Predict} $$

$$ p(x_t|z_{1:t}) = \frac{p(z_{t}|x_t) p(x_t|z_{1:t-1})}{p(z_t|z_{1:t-1})} \quad \text{Update} $$

The recursion above is very powerful, it provides a sequential way of updating our belief in a robot state as new information comes in. Looking at the equations above, we can see what we need to make this happen. We start with a prior belief in our robot state $p(x_0)$.

Last week, when we introduced robot models we saw that these could be expressed as $x_t = f(x_{t-1},u_{t},\epsilon)$, with $\epsilon$  our uncertainty or disturbances affecting our model. Another way to describe this is that our dynamics are a probabalistic transtion $p(x_t|x_{t-1},u_{t-1})$, given a state and action at time $t-1$, what is the probability that my robot is in the state $x_t$. For simplicity, we will drop the $u$ for now. The equations above use this to make a prediction about where our robot could be after taking some action.

We then get a measurement or observation as multiply the likelihood of obtaining this in a given state by the predicted probability of being in that state to get a posterior belief. This becomes the prior for the next recursion. As we repeat this, we accumulate information and refine our beliefs over time.

Let's make this more concrete with an example. My robot starts somewhere near the position (0,0). We use the dynamics to make a prediction about all the places it could possibily end up for a given movement command. We then make a measurement, and reweight this by the chances of getting that measurement for each of these places. We now use this as the new belief for our robot state, and repeat.

This is a general rule that applies regardless of dynamics, sensors or representations. Lets look at some common representations.

## Discrete Bayes filters

## Kalman filters

### Extended Kalman filters

## Particle filters



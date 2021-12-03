More-Thuente Line Search {#more-thuente-line-search-design}
========================

We implement a More-Thuente Line Search method as one of the line search methods. It implements and CRPT interface of the `LineSearch` class from the `line_search.hpp`.

The implementation closely follows the paper ["Line Search Algorithms with Guaranteed Sufficient Decrease"][1] by Jorge J. More and David J. Thuente. We have a short summary of that paper in one of the sections below.

# Target use cases
Line search algorithms are commonly used to aid optimization problems such as Newton's method.

More-Thuente line search is a more robust version of a line search than one with a fixed step. It guarantees picking a step that yields a sufficient decrease in the objective function, while at the same time aiming to have the resulting value of the objective function close to the optimum.

We aim to use this method as an aid to the Newton's method used in the NDT implementation for localization.

# Assumptions
We assume here, that one of the following two cases holds:
- We search for a *minimum* of the function \f$f\f$ starting at some point \f$x_0\f$. Then \f$\phi^\prime(0) = f^\prime(x_0) < 0\f$.
- We search for a *maximum* of the function \f$f\f$ starting at some point \f$x_0\f$. Then \f$\phi^\prime(0) = f^\prime(x_0) > 0\f$.

The implementation will implicitly assume the type of the problem (minimization vs maximization) based on the sign of the derivative \f$f^\prime(x_0)\f$.

Additionally, the step \f$\alpha\f$ as well as its bounds must be non-negative.

# Short paper recap
## Definitions
It is assumed that the objective function \f$\phi: \mathbb{R} \rightarrow \mathbb{R}\f$ defined on \f$[0, \infty]\f$ is smooth and continuously differentiable with \f$\phi^\prime(0) < 0\f$ we search for such a step \f$\alpha > 0\f$ that that so-called *Strong Wolfe Conditions* hold (Equations 1.1 and 1.2 in the paper):
\f[
    \phi(\alpha) \le \phi(0) + \mu \phi^\prime(0) \alpha \\
    \mid \phi^\prime(\alpha) \mid \le \eta \mid \phi^\prime(0) \mid
\f]

Usually, \f$\mu\f$ is a small value below \f$1/2\f$ and \f$\eta\f$ is a value close to \f$1\f$. Note that \f$\mu \le \eta\f$.

In our case, with an optimization function \f$f(x)\f$, starting point \f$x_0\f$, direction of optimization \f$d\f$ and step length \f$\alpha\f$, we define the function \f$\phi\f$ as follows (Equation 1.3 in the paper):
\f[
    \phi(\alpha) \equiv f(x_0 + \alpha p), \hspace{5mm} \alpha \geq 0
\f]

During the procedure we make use of an auxiliary function \f$\psi(\alpha)\f$ defined as follows (just before Equation. 2.1 in the paper):
\f[
  \psi(\alpha) \equiv \phi(\alpha) - \mu \phi^\prime(0) \alpha
\f]

## Iterative search for step length

The algorithm can be summarized as follows (follows the *Search Algorithm* in Section 2 of the paper).

\note This algorithm uses function \f$\psi\f$ in steps 2. and 3. until the following holds:
\f[
    \psi(\alpha_t) \le 0, \hspace{10mm} \phi^\prime(\alpha_t) > 0
\f]
After this statement becomes true the algorithm above starts using function \f$\phi\f$ in the steps 2. and 3. instead.

For a given step \f$\alpha_t\f$ and an interval of values \f$[\alpha_l, \alpha_u]\f$:
1. Check if the Strong Wolfe Conditions hold for \f$\alpha_t\f$. If they do - terminate the procedure with \f$\alpha_t\f$ as the result.
2. Generate next step length from the interval \f$[\alpha_l, \alpha_u]\f$ and the current step using either function \f$\psi\f$ or \f$\phi\f$. This part is shown in the Section 4 "TRIAL VALUE SELECTION" in the paper.
3. Update the interval \f$[\alpha_l, \alpha_u]\f$ using either function \f$\psi\f$ or \f$\phi\f$ and the current step \f$\alpha_t\f$. This part is covered in two algorithms: *Updating Algorithm* (right after theorem 2.1 in the paper) when the \f$\psi\f$ function is still used and *Modified Updating Algorithm* (shown after theorem 3.2 in the paper) used after we switch to using function \f$\phi\f$. These algorithms differ solely in the function used within them.

[1]: https://www.ii.uib.no/~lennart/drgrad/More1994.pdf

# Calibration

Let $d$ be the distance between left and right anchor. $l$ the length of the left rope, $r$ the length of the right rope.

Let $(0,0)$ be at the left top at the left anchor.

The length of $l$ and $r$ at point $p_n$ with coordinates $(x_n, y_n)$ can be calculated as

$$
l_n = \sqrt{x_n^2 + y^2}
$$ (1)
$$
r_n = \sqrt{(d-x_n)^2 + y_n^2}
$$ (2)

To calibrate the plotter we hang up a piece of paper with four points $p_{0..3}$ with $p_0$ = $(0, 0)$ and $p_n = p_0 + (o_{x_n}, o_{y_n})$ with $n\in\{1,2,3\}$.

We move the plotter to point $p_0$ and start the calibration process. We move to each of the three points $p_{1..3}$ and measure the change in length of $l$ and $r$ between $p_0$ and $p_n$ as $o_{l_n}$ and $o_{l_r}$ respectively.

First we want to determine the length of the left rope $l_0$ at point $p_0$ and the coordinates of $p_0$.

We introduce offsets to (1) and get

$$
(l_0+o_{l_n}) = \sqrt{(x_0+o_{x_n})^2 + (y_0+o_{y_n})^2}
$$

This can be solved for $y_0^2$ and applied to (1) to get

$$
\begin{pmatrix}
2o_{l_n} & -2o_{x_n} & -2o_{y_n}
\end{pmatrix}
\begin{pmatrix}
l_0\\
x_0\\
y_0
\end{pmatrix}
= -o_{l_n}^2 + o_{x_n}^2 + o_{y_n}^2
$$

which can be solved as system of linear equations with three offsets.

Similarly we determine $d$ and $r_0$. We introduce offsets to (2) and get

$$
(r_0+o_{r_n}) = \sqrt{(d-(x_0+0_{x_n}))^2 + (y_0+0_{y_n})^2}
$$

Solving again for $y_0^2$ and applying to (2) we get

$$
\begin{pmatrix}
-2o_{x_n} & -2o_{r_n}
\end{pmatrix}
\begin{pmatrix}
d\\
r_0
\end{pmatrix}
= x_0^2 + o_{r_n}^2 - (x_0 + o_{x_n})^2-2y_0 o_{y_n} - o_{y_n}^2
$$

which can be solved as system of linear equations using any two of the three offsets.
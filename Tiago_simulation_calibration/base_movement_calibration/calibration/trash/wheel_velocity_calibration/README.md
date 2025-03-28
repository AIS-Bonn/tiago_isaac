
The calibration in this folder and its explanation are only necessary to understand for implementing the Quadratic Programing Solver Holonomic Controller instead of the more straightforward Mecanum wheeled robot controller that is implemented by a simple matrix multiplication.
<b>So unless you know what this means, don't bother reading this.</b>


In this folder, we have the calibrations that match the target wheel rotation velocities.
The result is that we tuned 2 constants that are gains for linear velocity and angular velocity, which are then set in the action graph in Isaac Sim.
We can tune these constants just by math alone (no need for any data).
(Question: Why does the Holonomic Controller of Isaac not do that? We could try to help out the nvidia source code here.)

INSIGHT:
- The gains to be tuned are just multiplicative factors with the velocity command (see also the holnomic contrller source code).
The gains can be tuned as follows:

Linear Gain:
- set gain $c$ to 1
- choose some x command, e.g x=1. let x=1, y=0, z=0 run
- now output of the holonomic controller is a wheel velocity $\omega$.
- we want that $x = \omega \cdot r$ where $r=0.0762$ is the wheel radius
- thus, we need to set the gain $c'$ s.t. $$\cdot x = c' \cdot \omega \cdot r$$
- meaning $c' = \frac{x}{\omega \cdot r}$


For our Tiago robot, we get c = 2.82842659952 for the linear gain because:
for $x=1$, we got $\omega= 4.6398091371$ from the controller back, thus $c' = \frac{1}{4.6398091371 \cdot 0.0762} = 2.82842659952$

Rotational Gain:
For a pure rotational command $z$, one can derive the following relationship:
$$z = \frac{r \cdot \omega}{(L + W)} $$
where $\omega = - \omega_1 = omega_2 = -omega_3 = omega_4$
and $omega_i$ are the wheel rotational velocities.
$r$ is the wheel radius and $L$ and $W$ are the distances from the center of the robot to the wheels along the x and y axis.
So, we need to tune c' such that $$z = c' \frac{r \cdot \omega}{(L + W)} $$
We have for Tiago $L = 0.244$ and $W = 0.22317$

z=1 with initial c=1 yielded $\omega = 222.74559825689673$

and thus we need to set $c'=\frac{z(L + W)}{r \cdot \omega} = \frac{1(0.244 + 0.22317)}{0.0762 \cdot 222.74559825689673} = 0.02752395532$
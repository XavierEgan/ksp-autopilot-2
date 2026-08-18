# Starting point
The starting point for this project was a massive refactor of my previous autopilot. It simply uses a bunch of pids lined up to control the flight of the plane

There are also other issues with it such as oscilates when accelerating to supersonic that I hope I can potentially fix later with a MPC

# New approach
I will make a Total Energy Control System (TECS) that will control the energy of the plane instead of controlling speed and pitch seperatally.

## Math
I will be getting the math from [this paper](research/20080007401.pdf)

Note I have to make some assumptions since some of the math doesnt make any sense as written due to apparent damage to the document. For example the document says "the logitudanal acceleration ... $V_c$" however V is not typically acceleration, V means velocity. We can also see later that in equal (1) they do use a dot above V, meaning either they are using jerk in that equation, or a dot is missing. $\dot{V}$ being jerk would result in equation (2) adding $1/s$ to radians, meaning the dot must be missing.

We start out with \
$\gamma_c$ = vertical flight path angle command (rad)\
$\dot{V}_c$ = logitudinal acceleration command (m/s^2)

Then we calculate errors\
$\gamma_\epsilon = \gamma_c - \gamma$ = vertical flight path angle error (rad)\
$\dot{V}_\epsilon = \dot{V}_c - \dot{V}$ = logitudinal acceleration error (m/s^2)

We can then calculate the specific total energy rate error\
$\dot{E}_{s\epsilon} = \gamma_\epsilon + \dot{V}_\epsilon / g$


and the energy rate distribution error\
$\dot{D}_\epsilon = -\gamma_\epsilon + \dot{V}_\epsilon / g$

relative to the aircrafts vertical flight path and speed targets.\
Where $g$ is acceleration due to gravity

We then drive $\dot{E}_{s\epsilon}$ to zero by commanding thrust, and drive $\dot{D}_\epsilon$ to zero by commanding elevator position. This therefore gives us our desired vertical flight path and speed targets.
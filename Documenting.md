# Starting point
The starting point for this project is a pretty much greenfield rewrite of my current autopilot.

The biggest improvemetnts i want to make is define the control cascade in advance and use math more to get the inner pids to already sorta do the right thing, then the outer pids only have to deal with a small error. This means we are much less reiliant on well tuned pids

I would also like to use a TECS instead of controlling speed and pitch seperately

I'm also gonna make eveything a stream since its quite a bit faster

## Math / Controls
### Definitions
$p$ = body roll rate (angular velocity around roll axis)\
$q$ = body pitch rate (angular velocity around pitch axis)\
$r$ = body yaw rate (angular velocity around yaw axis)

$\phi$ = bank angle\
$\beta$ = sideslip angle (positive = right side of aircraft hits air)

$\psi$ = heading\
$\dot\psi$ = heading rate

$R$ = turn radii\
$n$ = load factor ($n = L / mg$)

$\hat x_b$ = body unit vector (along nose)\
$\hat y_b$ = body unit vector (along wing)\
$\hat z_b$ = body unit vector (along rudder)\
$\hat z_n$ = down (towards the center of kerbin)\
$\omega$ = angular velocity vector ($\omega = p\hat x_b + q\hat y_b + r\hat z_n)


### Inner controls
#### Pitch 

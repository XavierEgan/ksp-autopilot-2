# Starting point
The starting point for this project is a pretty much greenfield rewrite of my current autopilot.

The biggest improvemetnts i want to make is define the control cascade in advance and use math more to get the inner pids to already sorta do the right thing, then the outer pids only have to deal with a small error. This means we are much less reiliant on well tuned pids

I would also like to use a TECS instead of controlling speed and pitch seperately

I'm also gonna make eveything a stream since its quite a bit faster


Recreation of the discrete "grid world" empowerment experiments, in continuous space.

Reuses (albeit slightly altered) the menu systems, loggers and video recorder.

The empowerment values are currently approximated by the amount of sheep within 50 units of a given dog, scaling with distance within this sub-range, a decision still needs to be made on either:
    a) if an actual empowerment value needs to be calculated and used.
    b) if the current approximation is sufficient.
    c) if a better approximation can be made.

TODO:
- (Maybe) try alternative empowerment approximations, or create a proper empowerment calculation. 

- KNOWN BUGS:
    - [MITIGATED BUT NOT FIXED] Occasionally sheep "teleport", moving a large distance in one frame/tick. Two sheep are pushed into eachother, causing a spike in sheep-to-sheep resulsion forces. Could cap forces? Reducing the repulsion coefficient has mitigated this issue, but it does mean that the sheep huddle much closer together, making the overall herding task significantly easier.
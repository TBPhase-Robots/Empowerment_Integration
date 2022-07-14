Recreation of the discrete "grid world" empowerment experiments, in continuous space.

Reuses (albeit slightly altered) the menu systems, loggers and video recorder.

The empowerment values are currently approximated by the amount of sheep within 50 units of a given dog, scaling with distance within this sub-range, a decision still needs to be made on either:
    a) if an actual empowerment value needs to be calculated and used.
    b) if the current approximation is sufficient.
    c) if a better approximation can be made.

TODO:
- Try alternative empowerment approximations, or create a proper empowerment calculation. 
- Test if the post processing notebooks work with the altered SimLog. (I think it should be fine??)
- Test and potentially alter the effect of the "sheep_grazing_chance" config variable. Sheep may need to make larger moves when grazing, but want to avoid their movements from making too large jumps in one frame. Right now, discrete values [0.05, 0.2, 0.5]  in the various config_exp_N files map to continuous values [0.3, 0.6, 0.9]. However, their movements don't massively affect the difficulty of the herding task.

- KNOWN BUGS:
    - Occasionally, sheep "teleport", moving a large distance in one frame/tick. Two sheep are pushed into eachother, causing a spike in sheep-to-sheep resulsion forces. Could cap forces? Reducing the repulsion coefficient has mitigated this issue, but it does mean that the sheep huddle much closer together, making the overall herding task significantly easier.
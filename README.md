# Reconstruction #

Bunch of codes to automatically generate assets for Unreal Engine 4 from custom 3D scanner, for simulation of Earthquake. This project is for my master's thesis, and therefore very messy. Also, complete computer vision is
obviously non-trivial problem, so most part of the codebase is not structured well to avoid too early optimization.

### Testing ###
Generally, recognition code is not testable in normal way.
Also, our dataset is lacking; we must generalize beyond dataset on hand without evaluating with the larger, real-world data.

Don't try to structure non-testable, ill-defined code well.

Rather, focus on well-defined components and build clean interface around it so messing with hard problem become less-prone to bugs from below.



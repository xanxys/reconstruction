# Reconstruction #

Bunch of codes to automatically generate assets for Unreal Engine 4 from custom 3D scanner, for simulation of Earthquake. This project is for my master's thesis, and therefore very messy. Also, complete computer vision is
obviously non-trivial problem, so most part of the codebase is not structured well to avoid too early optimization.

### Testing ###
Generally, recognition code is not testable in normal way.
Also, our dataset is lacking; we must generalize beyond dataset on hand without evaluating with the larger, real-world data.

Don't try to structure non-testable, ill-defined code well.

Rather, focus on well-defined components and build clean interface around it so messing with hard problem become less-prone to bugs from below.


### Running ###
reconstruction uses docker to build and run.

1. `git clone`
2. `cd reconstrction`
3. `sudo docker build` . (take note on final snapshot id)
4. `./run_local.sh` (you need to rewrite snapshot id with what you got in step 3)
5. You're now inside bash in container.
6. `cd /root/local` (mapped to `./` in host)
7. `scons` (if you modified source code)
8. `./recon --convert ../data/scan-*`

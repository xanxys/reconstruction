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
3. `sudo docker build ./docker` . (take note on final snapshot id)
4. `./run_local.sh` (you need to rewrite snapshot id with what you got in step 3)
5. You're now inside bash in container.
6. `cd /root/local` (mapped to `./` in host)
7. `scons -j 4` (if you modified source code. you can change 4 to any number of CPU cores for parallel compilation)
8. `time build/recon --convert ../data/scan-20140827-*`
(You can supply `--debug` to see useful stuff)

Now move that scan- directory to somewhere accesible from UE4.

1. Launch ExExperiment UE4 project
2. Drag&Drop all files into Game/Auto (Contents Browser), optionally click through "degenerate tangent base" errors
3. Click Import Earthquake
4. You should have all actors placed nicely in the scene


```
sudo docker run -ti -v (pwd):/root/local -v (pwd)/../capturer:/root/data 1c6bbe46e13e bash
```

### Related Repositories ###
* [reconstruction](https://bitbucket.org/xanxys/reconstruction): this one
* [capturer](https://bitbucket.org/xanxys/capturer)
* [thesis-master](https://bitbucket.org/xanxys/thesis-master)

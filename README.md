# Reconstruction #

Bunch of codes to automatically generate assets for Unreal Engine 4 from custom 3D scanner, for simulation of Earthquake. This project is for my master's thesis, and therefore very messy. Also, complete computer vision is
obviously non-trivial problem, so most part of the codebase is not structured well to avoid too early optimization.

### Basic Dataflow ###

#### `recon`: Scene Recognizer ####
Generates 3D scene asset from scans. The generated asset
will *not* contain any information derived from Earthquake. But,
the asset *do* contain raw collision sounds. Written mostly in C++,
with occasional sub-process calls to python code (under `extpy`).

#### `pack_eq`: Earhquake Experiment Creator ####
Takes the 3D scene asset, Earthquake waveform,
and experimental parameters (such as player location and Earthquake scaling) and generates experiment package readable by LoaderPlugin in UE4.
This code is written in Python.

#### `LoaderPlugin`: Unpack Experiment Onto Scene ####
Places static objects, and writes some files to be read at runtime.
Also loads UE4 assets for runtime instantiation. Written in C++,
and runs in UE4 editor as a plugin.

#### `EqExperiment`: Execute Experiment ####
Creates experiment-ready VR content for Oculus Rift DK2.
This is not pure code, but a UE4 map + bunch of custom C++ GameMode and
Actors to be placed on the map. Basic function of the map is to run
the Earthquake experiment, but it also exposes various functionality
for other demonstrations, and controls for experimenters etc.


### Testing ###
Generally, recognition code is not testable in normal way.
Also, our dataset is lacking; we must generalize beyond dataset on hand without evaluating with the larger, real-world data.

Don't try to structure non-testable, ill-defined code well.

Rather, focus on well-defined components and build clean interface around it so messing with hard problem become less-prone to bugs from below.


### Running ###
reconstruction uses docker to build and run.

1. `git clone`
2. `cd reconstrction`
3. `sudo docker build -t xanxys/recon ./docker`: this will create an image named `xanxys/recon`
4. `./run_local.sh`
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

Create new container from the image and run bash in it:
```
sudo docker run -ti -v (pwd):/root/local -v (pwd)/../capturer:/root/data xanxys/recon bash
```

Note that everytime you run this command (or `./run_local.sh`), a new
container will be created, thus using up disk space.

 You should remove old containers occasionally. (Don't try to re-use
 containers, because that will ruin reproducibility of using clean
 containers).


#### Sound Tools ####
Example command sequence:

1. `cd sound`
2. `./simulate_collision --sound-assets ./raw_freesound/ --simulate collisions.wav`

#### EqExperiment Runtime ####
##### **WARNING** #####
To avoid code complication, we store information that is passed from
LoaderPlugin to EqExperiment runtime in **fixed location on the disk (`C:\VR14\runtime.json` **,
so **you cannot run packaged exe on its own**. And you also can't have
differently configured scene assets / experiment packages on the same
machine at the same time.

I think current fucking method is enough for relative stable spec
of subject experiments.

If you're bored, try to refactor implementation by using
[DataTables](https://www.unrealengine.com/blog/driving-gameplay-with-data-from-excel).

You **need to manually create** `C:\VR14` to avoid any User Access Control
problems. (symptom: LoaderPlugin not writing the file)

### Related Repositories ###
* [reconstruction](https://bitbucket.org/xanxys/reconstruction): this one
* [capturer](https://bitbucket.org/xanxys/capturer)
* [thesis-master](https://bitbucket.org/xanxys/thesis-master)

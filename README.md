# TUM-CPS-HZ-Planning

A minimal setup guide to add the InformedHZ planner to OMPL, integrate it with PDT

## OMPL Installation

Use the official OMPL repository to install/build OMPL. You’ll add the InformedHZ planner into this tree in later steps.

https://github.com/ompl/ompl/tree/main

## PDT Installation

PDT provides benchmarking and visualization tooling for OMPL planners. You’ll copy planner factory and config files here so PDT recognizes InformedHZ.

https://github.com/robotic-esp/pdt


## CGAL Installation

CGAL is required for geometric decomposition used by the planner. Install the development package:

```bash
sudo apt-get install libcgal-dev
```

---

## Copy the files to your ompl and pdt paths:

Place the planner’s headers/sources into OMPL, and the supporting configs/factory hooks into PDT. Adjust ~/ompl and ~/pdt to your local paths.

/ompl
```bash
cp InformedHZ.h ~/ompl/src/ompl/geometric/planners/rrt/
cp InformedHZ.cpp ~/ompl/src/ompl/geometric/planners/rrt/src/

cp informedhz ~/ompl/src/ompl/geometric/planners/rrt/

cp CMakeLists.txt ~/ompl/src/ompl/
```

/pdt
```bash
cp hz_smoketest.json ~/pdt/parameters/demo/
cp hz_visualization.json ~/pdt/parameters/demo/hz_visualization.json

cp planner_factory.cpp ~/pdt/src/factories/src/
cp default_informedhz.json ~/pdt/parameters/defaults/planners/
cp get_best_cost.cpp ~/pdt/src/utilities/src/
cp planner_type.h ~/pdt/src/common/include/pdt/common/
```

## Run Benchmarks with PDT

- Build PDT 

```(/path/to/pdt/build)```, then run the provided smoketest to validate the integration:

```bash
cd /path/to/pdt/build
./bin/benchmark -c ../parameters/demo/hz_smoketest.json
```

- Visualization

Launch the visualization to inspect problem setups and planner behavior interactively:

````bash
./bin/visualization -c ../parameters/demo/hz_visualization.json
````

Rebuild if made changes:

* in ~/ompl do 

```bash
make
sudo make install
```


* in /pdt do:

```bash
make
```
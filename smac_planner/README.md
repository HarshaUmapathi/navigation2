# Smac Planner

The SmacPlanner is a plugin for the Nav2 Planner server. It includes currently 2 distinct plugins:
- `SmacPlanner`: a highly optimized fully reconfigurable Hybrid-A* implementation supporting Dubin and Reeds-Shepp models
- `SmacPlanner2D`: a highly optimized fully reconfigurable grid-based A* implementation supporting Moore and Von Neumann models.

It also introduces the following basic building blocks:
- `CostmapDownsampler`: A library to take in a costmap object and downsample it to another resolution.
- `AStar`: A generic and highly optimized A* template library used by the planning plugins to search. Template implementations are provided for grid-A* and SE2 Hybrid-A* planning. Additional template for 3D planning also could be made available.
- `CollisionChecker`: Collision check based on a robot's radius or footprint.
- `Smoother`: A Conjugate-gradient (CG) smoother with several optional cost function implementations for use. This is a cost-aware smoother unlike b-splines or bezier curves.

We have users reporting using this on:
- Delivery robots
- Industrial robots

## Introduction

The `smac_planner` package contains an optimized templated A* search algorithm used to create multiple A\*-based planners for multiple types of robot platforms. It was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/). We support differential-drive and omni-directional drive robots using the `SmacPlanner2D` planner which implements a cost-aware A\* planner. We support cars, car-like, and ackermann vehicles using the `SmacPlanner` plugin which implements a Hybrid-A\* planner. This plugin is also useful for curvature constrained planning, like when planning robot at high speeds to make sure they don't flip over or otherwise skid out of control.

The `SmacPlanner` fully-implements the Hybrid-A* planner as proposed in [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf), including hybrid searching, CG smoothing, analytic expansions and hueristic functions.

## Features 

We further improve on the Hybrid-A\* work in the following ways:
- Remove need for upsampling by searching with 10x smaller motion primitives (same as their upsampling ratio).
- Multi-resolution search allowing planning to occur at a coarser resolution for wider spaces (O(N^2) faster).
- Cost-aware penalty functions in search resulting in far smoother plans (further reducing requirement to smooth).
- Additional cost functions in the CG smoother including path deflection.
- Approximated smoother Voronoi field using costmap inflation function.
- Fixed 3 mathematical issues in the original paper resulting in higher quality smoothing.
- Faster planning than original paper by highly optimizing the template A\* algorithm.
- Automatically adjusted search motion model sizes by motion model, costmap resolution, and bin sizing.
- Closest path on approach within tolerance if exact path cannot be found or in invalid space.
- Multi-model hybrid searching including Dubin and Reeds-Shepp models. More models may be trivially added.
- Time monitoring of planning to dynamically scale the maximum CG smoothing time based on remaining planning duration requested. 
- High unit and integration test coverage.
- Uses modern C++14 language features and individual components are easily reusable.

All of these features (multi-resolution, models, smoother, etc) are also available in the 2D `SmacPlanner2D` plugin.

The 2D A\* implementation also does not have any of the weird artifacts introduced by the gradient wavefront-based 2D A\* implementation in the NavFn Planner. While this 2D A\* planner is slightly slower, I believe it's well worth the increased quality in paths. Though the `SmacPlanner2D` is grid-based, any reasonable local trajectory planner - including those supported by Navigation2 - will not have any issue with grid-based plans.

TODO(stevemacenski) Run-time metrics, show gif(s) or images

## Design

The basic design centralizes a templated A\* implementation that handles the search of a graph of nodes. The implementation is templated by the nodes, `NodeT`, which contain the methods needed to compute the hueristics, travel costs, and search neighborhoods. The outcome of this design is then a standard A\* implementation that can be used to traverse any type of graph as long as a node template can be created for it.

We provide by default a 2D node template (`Node2D`) which does 2D grid-search with either 4 or 8-connected neighborhoods, but the smoother can be used to smooth it out. We also provide a SE2 node template (`NodeSE2`) which does SE2 (X, Y, theta) search and collision checking on Dubin or Reeds-Shepp motion models. Additional templates could be easily made and included for 3D grid search and non-grid base searching like routing.

In the ROS2 facing plugin, we take in the global goals and pre-process the data to feed into the templated A\* used. This includes processing any requests to downsample the costmap to another resolution to speed up search and smoothing the resulting A\* path. For the SE2 and `SmacPlanner` plugins, the path is promised to be kinematically feasible due to the kinematically valid models used in branching search. The 2D A\* is also promised to be feasible for differential and omni-directional robots.

We isolated the A\*, costmap downsampler, smoother, and Node template objects from ROS2 to allow them to be easily testable independently of ROS or the planner. The only place ROS is used is in the planner plugins themselves. 

## Parameters

See inline description of parameters in the `SmacPlanner`. This includes comments as specific parameters apply to `SmacPlanner2D` and `SmacPlanner` in SE2 place.

```
planner_server:
  ros__parameters:
    planner_plugin_types: ["smac_planner/SmacPlanner"]
    planner_plugin_ids: ["GridBased"]
    use_sim_time: True

    GridBased:
      tolerance: 0.5                    # tolerance for planning if unable to reach exact pose, in meters
      downsample_costmap: false         # whether or not to downsample the map
      downsampling_factor: 1            # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      allow_unknown: false              # allow traveling in unknown space
      max_iterations: -1                # maximum total iterations to search for before failing
      max_on_approach_iterations: 1000  # maximum number of iterations to attempt to reach goal once in tolerance
      max_planning_time_ms: 2000.0      # max time in ms for planner to plan and smooth. Will scale maximum smoothing times based on remaining time after planning.
      smooth_path: true                 # Whether to smooth searched path
      motion_model_for_search: "DUBIN"  # 2D Moore, Von Neumann; SE2 Dubin, Redds-Shepp
      angle_quantization_bins: 72       # For SE2 node: Number of angle bins for search, must be 1 for 2D node (no angle search)
      minimum_turning_radius: 0.20      # For SE2 node & smoother: minimum turning radius in m of path / vehicle
      reverse_penalty: 2.1              # For Reeds-Shepp model: penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.20              # For SE2 node: penalty to apply if motion is changing directions, must be >= 0
      non_straight_penalty: 1.10        # For SE2 node: penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 1.3                 # For SE2 node: penalty to apply to higher cost zones, must be >= 0

      smoother:
        smoother:
          w_curve: 30.0                 # weight to minimize curvature of path
          w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
          w_smooth: 30000.0             # weight to maximize smoothness of path
          w_cost: 0.025                 # weight to steer robot away from collision and cost
          cost_scaling_factor: 10.0     # this should match the inflation layer's parameter

        optimizer:
          max_time: 0.10                # maximum compute time for smoother
          max_iterations: 500           # max iterations of smoother
          debug_optimizer: false        # print debug info
          gradient_tol: 1.0e-10         # gradient change termination tolerance
          fn_tol: 1.0e-20               # function change termination tolerance
          param_tol: 1.0e-15            # parameter block change termination tolerance
          advanced:                     # I do not recommend users mess with this unless they're doing production tuning
            min_line_search_step_size: 1.0e-20
            max_num_line_search_step_size_iterations: 50
            line_search_sufficient_function_decrease: 1.0e-20
            max_num_line_search_direction_restarts: 10
            max_line_search_step_expansion: 50
```

## Topics

| Topic           | Type              |
|-----------------|-------------------|
| unsmoothed_path | nav_msgs/Path     |


## Install

```
sudo apt-get install ros-<ros2-distro>-smac-planner
```

## Etc (Important Side Notes)

### Potential Fields

//  - Do low potential field in all areas -- this should be the new defacto-default (really should have been already but ppl ignore it). Footprint + inflation important  // NOLINT

### 2D Search and Smoothing

- recommend A* 2D no smooth, any controller handles the blockiness fine and then faster. Its nice for humans but robot doesnt need it because its still cost aware.

### Costmap Resolutions

- recommend users set costmap resolution for global at the hybrid-A* rate with 0 costmap downsampling For the case that hybrid-A* is the only planner algorithm in use, that way there's no downsampling CPU use and overall alot lower CPU / memory usage at lower rates. But use if you have other planners that want a higher ersolution or in testing. Lowering th eresulution will make the planner very much faster. 

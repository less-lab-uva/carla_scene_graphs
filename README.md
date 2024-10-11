# CARLA Scene Graphs

![Video of short driving scenario in CARLA with corresponding scene graphs extracted using our tool](./imgs/stacked.gif)

This repo contains code to generate Scene Graphs for the CARLA simulator.
We have detailed guides in the [docs folder](./docs) explaining the scene graph generator (SGG) as well as instructions and demos on how to utilize the framework.

* [Quickstart guide](./docs/How-to-instantiate-it.md)
* [Main components and functions](./docs/Main-components-and-functions.md)
* [Scene Graph Abstractions](./docs/Abstractions.md)
* [Example with CARLA Leaderboard 2.0](./docs/Example-with-CARLA-Leaderboard-2.0.md)

# Installing CARLA
This Python module interfaces with the CARLA API to generate scene graphs by querying the state of the simulator.
This has been tested with several versions of CARLA including `0.9.10` and `0.9.14`. 
Note that the CARLA API is not always backward compatible - if you identify errors working on new versions of CARLA,
the interface may have changed.

Instructions for installing CARLA version `0.9.14` and the relevant dependencies can be found in [this README](./install_carla.md).

# Quick start
We offer two quick start guides:

* For a minimal working example running in a plain CARLA environment that you can extend, see [this guide](./docs/How-to-instantiate-it.md).
* To generate scene graphs for the scenarios described in the [CARLA leaderboard 2.0](https://leaderboard.carla.org/get_started/), see [this guide](./docs/Example-with-CARLA-Leaderboard-2.0.md).

## Publications using this plugin
This plugin has been successfully deployed in several techniques. Please let us know if you use the tool!
* ICSE'24 [link](https://dl.acm.org/doi/abs/10.1145/3597503.3639178) S<sup>3</sup>C: Spatial Semantic Scene Coverage for Autonomous Vehicles
* ICRA'24 [link](https://ieeexplore.ieee.org/abstract/document/10610973/) Specifying and Monitoring Safe Driving Properties with Scene Graphs

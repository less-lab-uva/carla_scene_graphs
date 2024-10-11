# CARLA Scene Graphs

![Video of short driving scenario in CARLA with corresponding scene graphs extracted using our tool](./imgs/stacked.gif)

This repo contains code to generate Scene Graphs for the CARLA simulator.
We have detailed guides in the [docs folder](./docs) explaining the scene graph generator (SGG) as well as instructions and demos on how to utilize the framework.

* [Quickstart guide](./docs/How-to-instantiate-it.md)
* [Main components and functions](./docs/Main-components-and-functions.md)
* [Scene Graph Abstractions](./docs/Abstractions.md)
* [Example with CARLA Leaderboard 2.0](./docs/Example-with-CARLA-Leaderboard-2.0.md)

# CARLA Entities
The SGG contains default mappings that describe the vehicles available in CARLA's [vehicle catalog](https://carla.readthedocs.io/en/latest/catalogue_vehicles/). 
The SGG will create nodes with the following labels based on the CARLA entity type.
These mappings are maintained in [actors.py](./carla_sgg/actors.py) and described below. Client applications can change this mapping for their use case.

| SGG Label    | CARLA Types                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `car`        | vehicle.audi.a2, vehicle.audi.etron, vehicle.audi.tt, vehicle.bmw.grandtourer, vehicle.chevrolet.impala, vehicle.citroen.c3, vehicle.dodge.charger_2020, vehicle.dodge.charger_police, vehicle.dodge.charger_police_2020, vehicle.ford.crown, vehicle.ford.mustang, vehicle.jeep.wrangler_rubicon, vehicle.lincoln.mkz_2017, vehicle.lincoln.mkz_2020, vehicle.mercedes.coupe, vehicle.mercedes.coupe_2020, vehicle.micro.microlino, vehicle.mini.cooper_s, vehicle.mini.cooper_s_2021, vehicle.nissan.micra, vehicle.nissan.patrol, vehicle.nissan.patrol_2021, vehicle.seat.leon, vehicle.tesla.model3, vehicle.toyota.prius, vehicle.bmw.isetta, vehicle.dodge_charger.police, vehicle.mustang.mustang, vehicle.lincoln.mkz2017, vehicle.mercedes-benz.coupe, vehicle.mini.cooperst |
| `truck`      | vehicle.carlamotors.carlacola, vehicle.carlamotors.european_hgv, vehicle.carlamotors.firetruck, vehicle.tesla.cybertruck                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `van`        | vehicle.ford.ambulance, vehicle.mercedes.sprinter, vehicle.volkswagen.t2, vehicle.volkswagen.t2_2021                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `bus`        | vehicle.mitsubishi.fusorosa                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| `motorcycle` | vehicle.harley-davidson.low_rider, vehicle.kawasaki.ninja, vehicle.vespa.zx125, vehicle.yamaha.yzf                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `bicycle`    | vehicle.bh.crossbike, vehicle.diamondback.century, vehicle.gazelle.omafiets                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Additionally, CARLA maintains a separate "special type" for different vehicles, representing e.g. emergency vehicles:

| SGG Label       | CARLA Types                                                                                                                                          |
|-----------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| `emergency`     | vehicle.dodge.charger_police, vehicle.dodge.charger_police_2020, vehicle.dodge_charger.police, vehicle.carlamotors.firetruck, vehicle.ford.ambulance |
| `taxi`          | vehicle.ford.crown                                                                                                                                   |
| `electric`      | vehicle.micro.microlino, vehicle.tesla.model3, vehicle.toyota.prius, vehicle.tesla.cybertruck                                                        |


The mapping also handles traffic signals:

| SGG Label       | CARLA Types             |
|-----------------|-------------------------|
| `traffic_light` | traffic.traffic_light   |
| `stop_sign`     | traffic.stop            |

# Relationships
The abstractions generated by this tool capture many rich semantic and spatial relationships. 
See this [document](./docs/Abstractions.md#roadscene2vec-style-abstractions) for more details on the [semantic](./docs/Abstractions.md#semantic-relationships), [distance](./docs/Abstractions.md#distance-relationships), and [angular](./docs/Abstractions.md#angular-relationships) captured. 

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

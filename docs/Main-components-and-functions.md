# Components
The SGG is broken into several components:

## Main components
* The Scene Graph Generator (SGG) [carla_sgg/sgg.py](../carla_sgg/sgg.py)
  * Generates the "base SG" - the most semantically rich scene graph. See [methods](#the-scene-graph-generator-sgg) below.
* The Scene Graph Abstractor (SGA) [carla_sgg/sgg_abstractor.py](../carla_sgg/sgg_abstractor.py)
  * Functions for _abstracting_ the base SG into forms more applicable to the given task. See the [Abstractions](../Abstractions) wiki page for more information about the different abstractions available.

## Helper components
* [utils.py](../carla_sgg/utils.py)
  * Helper functions for working with different objects
* [viz.py](../carla_sgg/viz.py)
  * Functions to plot/visualize scene graphs
* [actors.py](../carla_sgg/actors.py)
  * Mappings between different CARLA actor types and their attributes within the SG

# Methods
## The Scene Graph Generator (SGG)
The SGG lives in [carla_sgg/sgg.py](../carla_sgg/sgg.py) and declares a type `SGG` that must be instantiated for the given simulation episode. Then, at each time step, the SGG can be invoked to generate a new scene graph based on the current state of the simulation. See [this guide](../How-to-instantiate-it/) for more information about setting up a minimum viable example.

### The Constructor
The SGG takes in a reference to the CARLA client object being used for the simulation as well as the ID of the _ego vehicle_.
The ego vehicle, also called the hero vehicle in some documentation, is the vehicle under examination. 
All scene graphs contain an ego vehicle at their core, and implicitly everything is defined relative to the ego vehicle.
Below is a minimum viable usage of the constructor.
```python
    # setup the CARLA client
    client = carla.Client('localhost', 2000)
    # This is longer than typical - the client timeout must wait for the SGG indexing to complete
    client.set_timeout(120.0)

    # load the environment
    world = client.load_world('Town10_Opt')  # set town string to load a different map

    # it is recommended to run the simulation in synchronous mode when generating scene graphs
    # to ensure alignment between the scene graph and other sensor data
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    world = client.reload_world(False)

    # create the ego vehicle
    blueprint_library = world.get_blueprint_library()
    ego_bp = blueprint_library.find('vehicle.tesla.model3')  # change the blueprint id for a different vehicle type
    spawn_points = world.get_map().get_spawn_points()
    ego = world.spawn_actor(ego_bp, spawn_points[0])

    # Setup the SGG - this call will take several seconds while the SGG indexes the world and performs initial bookkeeping
    sgg = SGG(client, ego.id)
```

### Generating the SG
The method `generate_graph_for_frame` generates the SG based on the current simulation state.
This method takes in two parameters: the world snapshot and the ego vehicle control commands.
The control commands are optional and are used to enrich the SG based on the _commanded action_ for the ego vehicle to contrast with the actual action taken by the vehicle; this is useful for several types of analysis.
This can either be a dictionary as shown below, or an instance of [carla.VehicleControl](https://carla.readthedocs.io/en/0.9.14/python_api/#carlavehiclecontrol).
```python
        for frame_number in range(100):
            # get the simulation state world snapshot
            frame = world.tick()
            # can either be a dictionary as below, or carla.VehicleControl
            ego_control = {
                'throttle': 5.0,
                'steer': -1.0
            }
            sg = sgg.generate_graph_for_frame(frame, ego_control)
```
## The Scene Graph Abstractor
The scene graph abstractor takes in the base scene graph (`sg`) produced by the `generate_graph_for_frame` [function](#generating-the-sg) and constructs a higher-order abstraction of the graph data. See the [Abstractions](../Abstractions) page for more information about each of these abstractions.

### The most-used abstraction: RoadScene2Vec + Junctions
The most-used abstraction is inspired by the [RoadScene2Vec](https://github.com/AICPS/roadscene2vec) (RSV) framework. 
The original RSV work focused on generating SGs from arbitrary real-world image data, which limited its ability to reason about road structure.
We extend the RSV framework to include richer road information including junctions, roads, and lanes.

In addition to the SG, the method takes in a set of proximity thresholds, directional thresholds, proximity relations, and directional relations. These do not have to be specified; if omitted the [default values](#defaults) will be used instead. These parameters are specified in the same manner as the original [RSV work](https://github.com/AICPS/roadscene2vec/blob/main/roadscene2vec/config/scenegraph_extraction_config.yaml).
```python
process_to_junction_rsv(sg, proximity_thresholds=None, directional_thresholds=None, proximity_relations=None, directional_relations=None)
```

#### Defaults:
```python
DEFAULT_PROXIMITY_THRESHOLDS = [['safety_hazard', 2], ['near_coll',4],['super_near',7],['very_near',10],['near',16],['visible',25]]
DEFAULT_DIRECTIONAL_THRESHOLDS = [['inDFrontOf',[[45,90],[90,135]]], ['inSFrontOf',[[0,45],[135,180]]], ['atDRearOf',[[225,270],[270,315]]], ['atSRearOf',[[180,225],[315,360]]]]
DEFAULT_PROXIMITY_RELATIONS = [['ego', 'person', 25], ['ego', 'bicycle', 25], ['ego', 'car', 25], ['ego', 'motorcycle', 25], ['ego', 'airplane', 25], ['ego', 'bus', 25], ['ego', 'train', 25], ['ego', 'truck', 25], ['ego', 'boat', 25], ['ego', 'traffic light', 25], ['ego', 'fire hydrant', 25], ['ego', 'street sign', 25], ['ego', 'stop sign', 25], ['ego', 'parking meter', 25], ['ego', 'bench', 25]]
DEFAULT_DIRECTIONAL_RELATIONS = [['ego', 'person', 25], ['ego', 'bicycle', 25], ['ego', 'car', 25], ['ego', 'motorcycle', 25], ['ego', 'airplane', 25], ['ego', 'bus', 25], ['ego', 'train', 25], ['ego', 'truck', 25], ['ego', 'boat', 25], ['ego', 'traffic light', 25], ['ego', 'fire hydrant', 25], ['ego', 'street sign', 25], ['ego', 'stop sign', 25], ['ego', 'parking meter', 25], ['ego', 'bench', 25]]
```

### Several default helper abstractions
* `entities(sg)`
  * Abstracts to lanes and other entities without including any relationships
* `semgraph(sg)`
  * Abstracts to entities without including any relationships
* `semgraphrel(sg)`
  * Abstracts to entities (excluding lanes) including their default relationships 
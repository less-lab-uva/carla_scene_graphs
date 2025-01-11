# Instantiating the SGG
This page outlines the minimal working example to generate a scene graph.
Check [here](./Main-components-and-functions.md) for a more detailed description of the API, and [here](./Abstractions.md) for detailed information about the available abstractions.

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

# Generating the Scene Graph
This method takes in two parameters: the world snapshot and the ego vehicle control commands. The control commands can either be a dictionary as shown below, or an instance of [carla.VehicleControl](https://carla.readthedocs.io/en/0.9.14/python_api/#carlavehiclecontrol).
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

# Full Example
See [mwe.py](../mwe.py) for a full minimal working example.
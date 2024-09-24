from timeit import default_timer as timer 
from collections import defaultdict
from itertools import combinations
import os
import time
from typing import Union
import carla
import networkx as nx
import numpy as np
from tqdm import tqdm
import copy
from shapely.geometry.polygon import Polygon
import uuid
import matplotlib
import matplotlib.pyplot as plt
import pickle
from pathlib import Path
from carla_sgg.sgg_abstractor import Curvature, process_to_abstract, process_to_rsv

import sys

# Add missing methods that are not present in older versions of CARLA
def length(vector3d):
    return np.linalg.norm([vector3d.x, vector3d.y, vector3d.z])

def distance(vector3d1, vector3d2):
    return np.linalg.norm([vector3d1.x - vector3d2.x, vector3d1.y - vector3d2.y, vector3d1.z - vector3d2.z])

def make_unit_vector(vector3d):
    return vector3d / vector3d.length()

test_vector = carla.Vector3D
if not hasattr(test_vector, 'length'):
    carla.Vector3D.length = length
if not hasattr(test_vector, 'distance'):
    carla.Vector3D.distance = distance
if not hasattr(test_vector, 'make_unit_vector'):
    carla.Vector3D.make_unit_vector = make_unit_vector


from carla_sgg.viz import draw_graph
# handling the largest maps requires a higher recursion limit
sys.setrecursionlimit(2000)

from carla_sgg.utils import *

class IgnoreWaypointPickler(pickle.Pickler):
    def reducer_override(self, obj):
        """Custom reducer for MyClass."""
        if getattr(obj, "__name__", None) == "Waypoint":
            return None
        else:
            # For any other object, fallback to usual reduction
            return NotImplemented
    def persistent_id(self, obj):
        if str(type(obj)) == "<class 'carla.libcarla.Waypoint'>":
            return 'please_ignore_me'
        else:
            return None  # default behavior

class SGG:
    def __new__(cls, *args, **kwargs):
        if (len(args) > 0 or len(kwargs) > 0) and hasattr(cls, 'sgg_singleton'):
            del cls.sgg_singleton
        
        if not hasattr(cls, 'sgg_singleton'):
            cls.sgg_singleton = super(SGG, cls).__new__(cls)
            cls.sgg_singleton.__initialization(*args, **kwargs)
        return cls.sgg_singleton

    def __initialization(self, carla_client, ego_id=None, segment_length: float=1.0,
                  angular_precision: "list[float]"=[1, 2.5, 5, 7.5, 10],
                    closeness_inflation: float = 0.25, init: bool=True, frame_buffer: int=10,
                    check_occlusion: bool=False):
        self.timestep = 0
        self._carla_client = carla_client
        self._segment_length = segment_length
        self._angular_precision = angular_precision
        self._closeness_inflation = closeness_inflation
        self._ego_id = ego_id
        self._init = False
        self._base_graph = None
        self._world = None
        self._recent_frames = LRUDict(frame_buffer)
        self._base_abstraction_buffer = LRUDict(frame_buffer)
        self._check_occlusion = check_occlusion
        if init:
            self.init_to_world()


    def _check_init(self):
        if self._init:
            raise ValueError("Cannot initialize or alter parameters after initialization. You must use a separate SGG object per map and configuration")

    @property
    def ego_id(self):
        return self._ego_id

    @ego_id.setter
    def ego_id(self, val):
        self._ego_id = val

    @property
    def segment_length(self):
        return self._segment_length

    @segment_length.setter
    def segment_length(self, val: float):
        self._check_init()
        self._segment_length = val

    @property
    def angular_precision(self):
        return self._angular_precision

    @angular_precision.setter
    def angular_precision(self, val: "list[float]"):
        self._check_init()
        self._angular_precision = val

    @property
    def closeness_inflation(self):
        return self._closeness_inflation

    @closeness_inflation.setter
    def closeness_inflation(self, val: float):
        self._check_init()
        self._closeness_inflation = val

    def get_waypoint_bbox(self, waypoint):
        forward = get_vert_from_location(waypoint.transform.get_forward_vector().make_unit_vector())
        forward *= self._segment_length  # need to not divide by 2 to ensure no gaps
        right = get_vert_from_location(waypoint.transform.get_right_vector().make_unit_vector())
        right *= waypoint.lane_width / 2
        start = get_vert_from_location(waypoint.transform.location)
        lower_right = start + right - forward
        upper_right = lower_right + forward + forward
        upper_left = upper_right - right - right
        lower_left = upper_left - forward - forward
        points = [(lower_right[0], lower_right[1]),
                (upper_right[0], upper_right[1]),
                (upper_left[0], upper_left[1]),
                (lower_left[0], lower_left[1])]
        return points

    def init_to_world(self, parse_junction_boundaries=False):
        print('Performing init, this may take a moment...')
        self._check_init()
        self._init = True
        world = self._carla_client.get_world()
        self._world = world
        carla_map = world.get_map()
        waypoint_precision = self._segment_length
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        try:
            # handle v0.9.11
            from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
            grp = GlobalRoutePlanner(GlobalRoutePlannerDAO(carla_map, waypoint_precision))
            grp.setup()
        except:
            # handle v0.9.14
            grp = GlobalRoutePlanner(carla_map, waypoint_precision)
        carla_graph = grp._graph
        new_graph = nx.DiGraph()
        new_graph.graph['bbox'] = {}
        map = world.get_map()
        new_graph.graph['waypoint_precision'] = waypoint_precision
        new_graph.graph['angular_precision'] = self._angular_precision
        seg_id_map = {}
        waypoint_to_segment = {}
        waypoints_from_segment_id = {}
        nodes_from_position = {}
        # make copy of the topology because we will re-use this data structure
        segments = copy.copy(grp._topology)
        # the segments contain the minimal graph where all decision points are, i.e. junctions/lane change locations
        # note that if a segment has a lane change available, that implies that the lane change is available for the rest of the lane
        # Step 1: increase the precision of the road graph by adding in nodes between all of the junctions to represent the road itself
        for id, seg in enumerate(segments):
            seg['id'] = id
            seg_id_map[id] = seg
            waypoints_from_segment_id[id] = []
            path = seg['path']
            path.insert(0, seg['entry'])
            path.append(seg['exit'])
            # first pass to add the nodes
            nodes = []
            for waypoint_id, waypoint in enumerate(path):
                waypoint_to_segment[waypoint] = id
                vert = get_waypoint_location(waypoint)
                node_id = str(waypoint.id)
                count = 1
                while node_id in new_graph.nodes:
                    node_id = str(waypoint.id) + f"_{count}"
                    count += 1
                waypoints_from_segment_id[id].append(node_id)
                hashloc = hashable_location(vert, waypoint_precision)
                if hashloc not in nodes_from_position:
                    nodes_from_position[hashloc] = []
                nodes_from_position[hashloc].append(node_id)
                waypoint_bbox = self.get_waypoint_bbox(waypoint)
                right_lane_marking = waypoint.right_lane_marking
                left_lane_marking = waypoint.left_lane_marking
                new_graph.add_node(node_id, vertex=vert, parent_segment=id, waypoint=waypoint.id, waypoint_obj=waypoint,
                                   waypoint_road_id=waypoint.road_id, waypoint_section_id=waypoint.section_id, waypoint_s=waypoint.s,
                                   waypoint_lane_change=waypoint.lane_change, waypoint_lane_id=waypoint.lane_id, waypoint_bbox_polygon=Polygon(waypoint_bbox),
                                   sem_class=str(waypoint.lane_type), is_junction=waypoint.is_junction, waypoint_junction_id=waypoint.junction_id,
                                   forward=get_vert_from_location(waypoint.transform.get_forward_vector()),
                                   right=get_vert_from_location(waypoint.transform.get_right_vector()),
                                   up=get_vert_from_location(waypoint.transform.get_up_vector()),
                                   vertex_point=vertex_to_point(vert),
                                   right_lane_marking_color=str(right_lane_marking.color),
                                   right_lane_marking_lane_change=str(right_lane_marking.lane_change),
                                   right_lane_marking_type=str(right_lane_marking.type),
                                   right_lane_marking_width=right_lane_marking.width,
                                   left_lane_marking_color=str(left_lane_marking.color),
                                   left_lane_marking_lane_change=str(left_lane_marking.lane_change),
                                   left_lane_marking_type=str(left_lane_marking.type),
                                   left_lane_marking_width=left_lane_marking.width
                                   )
                new_graph.graph['bbox'][node_id] = waypoint_bbox
                nodes.append(node_id)
            # second pass to add the edges
            for node_id1, node_id2 in zip(nodes, nodes[1:]):
                dist = get_node_distance(new_graph, node_id1, node_id2)
                new_graph.add_edge(node_id1, node_id2,
                                distance=dist,
                                lane_change=False)
        # Step 2: copy over all edges with special handling for those corresponding to lane changes
        # Lane change edges are denoted by having length 0
        # Non-lane change edges are copied over by just connecting the entry and exit waypoints for the segments
        original_edges = carla_graph.edges(data=True)
        edges_to_check = [(start, end, data) for (start, end, data) in original_edges if data['length'] > 0]
        print(f'Reduced from checking {len(original_edges)} to {len(edges_to_check)}')
        # for start, end, data in carla_graph.edges(data=True):
        for start, end, data in tqdm(edges_to_check):
            entry_waypoint = data['entry_waypoint']
            exit_waypoint = data['exit_waypoint']
            path = data['path']
            length = data['length']
            entry_node = nodes_from_position[hashable_location(get_waypoint_location(entry_waypoint), waypoint_precision)][0]
            try:
                exit_hash_loc = hashable_location(get_waypoint_location(exit_waypoint), waypoint_precision)
                if exit_hash_loc not in nodes_from_position:
                    prev = exit_waypoint.previous(waypoint_precision)
                    if len(prev) == 1 and prev[0].road_id == exit_waypoint.road_id and prev[0].lane_id == exit_waypoint.lane_id:
                        exit_hash_loc = hashable_location(get_waypoint_location(prev[0]), waypoint_precision)
                if exit_hash_loc not in nodes_from_position:
                    next = exit_waypoint.next(waypoint_precision)
                    if len(next) == 1 and next[0].road_id == exit_waypoint.road_id and next[0].lane_id == exit_waypoint.lane_id:
                        exit_hash_loc = hashable_location(get_waypoint_location(next[0]), waypoint_precision)
                exit_node = nodes_from_position[exit_hash_loc][0]
            except Exception as e:
                if end >= 0 and length == 0:
                    raise RuntimeError()
            entry_segment_id = new_graph.nodes[entry_node]['parent_segment']
            entry_nodes = waypoints_from_segment_id[entry_segment_id]
            exit_segment_id = new_graph.nodes[exit_node]['parent_segment']
            exit_nodes = waypoints_from_segment_id[exit_segment_id]
            if length == 0:
                # this is a lane change maneuver edge
                # we will handle this by connecting every node in the lane to its adjacent node in the other lane
                # here "adjacent" means the nearest node that is not more than 90 degrees off the direction of travel:
                #  a3  b3
                #  a2  b2
                #  a1  b1
                # here, a2 will be connected to b2 and vice versa. However, if there is a curve such that b2 is actually sligtly behind a2:
                #  a3   
                #   a2  b3
                #        b2
                #    a1  b1
                # here, b2 could be connected to a2, but a2 would be connected to b3
                # NOTE: this only handles one direction at a time, e.g. the a to b edges. The b to a edges have their own corresponding edge in the outer graph
                exit_index = -1
                for entry_index, node_id1 in enumerate(entry_nodes):
                    # iterate over the entry waypoints and find the corresponding exit waypoint
                    found = False
                    current_vert = new_graph.nodes[node_id1]['vertex']
                    while not found and exit_index < len(exit_nodes) - 1:
                        exit_index += 1
                        potential_exit_waypoint_id = exit_nodes[exit_index]
                        potential_exit_vert = new_graph.nodes[potential_exit_waypoint_id]['vertex']
                        # check angle
                        if entry_index == len(entry_nodes) - 1:
                            # if we are at the end of the segment, we need to calc angle using previous instead of next
                            # Note: for small segment sizes, it may be impractical to actually perform such a lane chane,
                            # but since the segment size is set as a parameter, this 
                            previous_entry_waypoint_id = entry_nodes[entry_index - 1]
                            prev_vert = new_graph.nodes[previous_entry_waypoint_id]['vertex']
                            vec_to_previous = current_vert - prev_vert
                            next_vert = current_vert + -1 * vec_to_previous
                        else:
                            next_entry_waypoint_id = entry_nodes[entry_index + 1]
                            next_vert = new_graph.nodes[next_entry_waypoint_id]['vertex']
                        forward_vector = next_vert - current_vert
                        side_vector = potential_exit_vert - current_vert
                        cosine = vector_cosine(forward_vector, side_vector)
                        if cosine >= 0:  # should be 0, but could use -1e-5 allow for a small deviation bc of rounding.
                            found = True
                        else:
                            pass
                    if found:
                        node_id2 = exit_nodes[exit_index]
                        new_graph.add_edge(node_id1, node_id2,
                                distance=get_node_distance(new_graph, node_id1, node_id2),
                                lane_change=True)
            else:
                # this is not a lane change maneuver edge
                pass
        # Step 3: Merge duplicates. Some waypoints overlap bc the start of one road segment _is_ the end of the other
        for hashloc, nodes in nodes_from_position.items():
            if len(nodes) > 1:
                for extra_node in nodes[1:]:
                    nx.contracted_nodes(new_graph, nodes[0], extra_node, self_loops=False, copy=False)
        # once all nodes and edges are established, determine the angles
        node_colors = {}
        closeness_map = {}
        print(f'Processing {len(new_graph.nodes)} new nodes')
        for node in tqdm(new_graph.nodes):
            vert = new_graph.nodes[node]['vertex']
            close_loc = hashable_location(vert, waypoint_precision, inflation=self._closeness_inflation)
            if close_loc not in closeness_map:
                closeness_map[close_loc] = []
            closeness_map[close_loc].append(node)
            out_nodes = [v for u, v, d in new_graph.edges(node, data=True) if not d['lane_change']]
            in_nodes = [u for u, v, d in new_graph.in_edges(node, data=True) if not d['lane_change']]
            if len(out_nodes) != 1 or len(in_nodes) != 1:
                angle = np.nan
                angle_index = np.nan
            else:
                avg_out = get_average_vertex(new_graph, out_nodes)
                avg_in = get_average_vertex(new_graph, in_nodes)
                forward_vector = avg_out - vert
                back_vector = avg_in - vert
                angle = np.nan_to_num(180 - np.rad2deg(np.arccos(vector_cosine(forward_vector, back_vector))), 0)
                left_turn = not is_left(avg_in, avg_out, vert)  # the not is because we are calculating if our vert is to the left of the others
                sign = -1 if left_turn else 1  # use sign to represent direction
                angle *= sign
                # convert to precision
                angle_index = 0
                while angle_index < len(self._angular_precision) and abs(angle) > self._angular_precision[angle_index]:
                    angle_index += 1
                angle_index *= sign
            new_graph.nodes[node]['angle'] = angle
            new_graph.nodes[node]['curve'] = angle_index
            node_colors[node] = (angle_index)
        new_graph.graph['closeness_map'] = closeness_map
        found_ids = {}
        def get_id(road_id):
            if road_id not in found_ids:
                found_ids[road_id] = len(found_ids)
            return found_ids[road_id]
        node_colors = {k: f'C{get_id(new_graph.nodes[k]["waypoint_road_id"]) % 10}' for k in new_graph.nodes}
        new_graph.graph['node_colors'] = node_colors
        new_graph.graph['node_size'] = {node: 20 for node in new_graph.nodes}

        # Step 3.5 - add the ends of lanes before junctions
        junctions = {}
        junction_nodes = defaultdict(list)
        junction_nodes_to_check = [node for node in new_graph.nodes if new_graph.nodes[node]['is_junction']]
        if parse_junction_boundaries:
            print(f'Parsing {len(junction_nodes_to_check)} junctions in two passes:')
            for node in tqdm(junction_nodes_to_check):
                junction = new_graph.nodes[node]['waypoint_obj'].get_junction()
                junctions[junction.id] = junction
                junction_nodes[junction.id].append(node)
            for junction_id, junction in tqdm(junctions.items()):
                junction_bbox = junction.bounding_box
                queue = junction_nodes[junction_id]
                checked_nodes = set()
                boundaries = defaultdict(list)
                print(f'Searching {len(junction_nodes[junction_id])} original nodes')
                pbar = tqdm(total=None)
                queue_count = 0
                while len(queue) > 0:
                    queue_count += 1
                    cur_node = queue.pop()
                    if "waypoint" not in new_graph.nodes[cur_node] or new_graph.nodes[cur_node]['waypoint'] in checked_nodes:
                        continue
                    checked_nodes.add(new_graph.nodes[cur_node]['waypoint'])
                    out_nodes = [v for u, v, d in new_graph.edges(cur_node, data=True) if new_graph.nodes[v]['sem_class'] == 'Driving' and not d['lane_change']]
                    in_nodes = [u for u, v, d in new_graph.in_edges(cur_node, data=True) if new_graph.nodes[u]['sem_class'] == 'Driving' and not d['lane_change']]
                    for other_node in [*out_nodes, *in_nodes]:
                        if not junction_bbox.contains(new_graph.nodes[other_node]['waypoint_obj'].transform.location, carla.Transform()):
                            boundaries[other_node].append(cur_node)
                        else:
                            queue.append(other_node)
                    pbar.update(1)
                pbar.close()
                print(f'Finished after parsing {queue_count} junction nodes total')
                print(f'Parsing {len(boundaries)} boundary nodes')
                for outside_node, inside_nodes in tqdm(boundaries.items()):
                    pts = [new_graph.nodes[outside_node]['vertex']]
                    pts.extend([new_graph.nodes[inside_node]['vertex'] for inside_node in inside_nodes])
                    pts = np.array(pts)
                    avg_vert = np.mean(pts, axis=0)
                    actor_name = f'junction_boundary_{junction_id}_{outside_node}'
                    sem_class = 'junction_boundary'
                    outside_data = new_graph.nodes[outside_node]
                    new_bbox = copy.copy(new_graph.graph['bbox'][outside_node])
                    offset = (avg_vert - new_graph.nodes[outside_node]['vertex'])[0:2]
                    new_bbox += offset
                    self.__add_to_graph(new_graph, avg_vert, outside_data['forward'], outside_data['right'], outside_data['up'],
                                        new_bbox, actor_name=actor_name, sem_class=sem_class)
                    for other_node in [outside_node, *inside_nodes]:
                        new_graph.add_edge(actor_name, other_node, distance=np.nan, dist_relationship='isIn')
                    new_graph.graph['node_colors'][actor_name] = (1, 0.5, 0, 1)
                    new_graph.graph['node_size'][actor_name] = 50
                    new_graph.graph['bbox'][actor_name] = new_bbox
            

        static_id = 1
        # Step 4: add static vehicles
        try:
            # in newer versions of CARLA we can tell entities apart
            static_vehicle_bbs = [
                carla.CityObjectLabel.Bicycle,
                carla.CityObjectLabel.Bus,
                carla.CityObjectLabel.Car,
                carla.CityObjectLabel.Motorcycle,
                carla.CityObjectLabel.Truck,
            ]
        except:
            # in older versions of carla we only have the vehicle class
            static_vehicle_bbs = [
                carla.libcarla.CityObjectLabel.Vehicles,
            ]
        print(f'Parsing static background objects')
        for type_index, bb_type in enumerate(tqdm(static_vehicle_bbs)):
            try:
                bounding_boxes = [env_obj.bounding_box for env_obj in world.get_environment_objects(bb_type)]
            except AttributeError:
                # handle older versions of carla
                # without the get_environment_objects method, we need to check for existing actors manually
                bounding_boxes_to_check = world.get_level_bbs(bb_type)
                actors_to_check = [actor for actor in world.get_actors() if 'spectator' not in actor.type_id]
                bounding_boxes = []
                # we don't want to include the bounding boxes of actors, only background vehicles
                for bb in bounding_boxes_to_check:
                    try:
                        for actor in actors_to_check:
                            actor_transform = actor.get_transform()
                            points_to_check = []
                            try:
                                points_to_check.extend(actor.bounding_box.get_world_vertices(actor_transform))
                            except AttributeError:
                                pass  # in older verions of carla, some actors don't have bounding boxes
                            points_to_check.append(actor.get_location())
                            for pt in points_to_check:  
                                # the level bb is already in the world frame, don't translate
                                if bb.contains(pt, carla.Transform()):
                                    raise StopIteration
                    except StopIteration:
                        continue
                    bounding_boxes.append(bb)
            for index, bb in enumerate(bounding_boxes):
                sem_class = f'{bb_type}'
                actor_name = f'{sem_class}_{index}'
                vertex = get_vert_from_location(bb.location)
                forward, right, up = get_rotation_pointers(bb.rotation)
                # for stationary objects the bbox is already in the world frame
                bbox = carla_bb_points_to_matplotlib(bb.get_local_vertices())
                self.__add_to_graph(new_graph, vertex, forward, right, up, bbox, actor_name=actor_name, sem_class=sem_class, entity_id=-static_id, threedbbox=numpy3dbb(bb.get_local_vertices()))
                new_graph.graph['node_colors'][actor_name] = (1,0,1,1)
                new_graph.graph['node_size'][actor_name] = 75
                new_graph.graph['bbox'][actor_name] = bbox
                static_id += 1
        # Step 5: other static objects
        # This list contains all of the other types of objects that CARLA knows about.
        # The default configuration is to only include static vehicles which are handled in step 4 above.
        # Traffic signs are also handled separately.
        # This helper code is available if, e.g., fences, railings, etc. were desired in the SG.
        bb_types = [
                        # carla.CityObjectLabel.Bridge,
                        # carla.CityObjectLabel.Buildings,
                        # carla.CityObjectLabel.Dynamic,
                        # carla.CityObjectLabel.Fences,
                        # carla.CityObjectLabel.Ground,
                        # carla.CityObjectLabel.GuardRail,
                        # # carla.CityObjectLabel.NONE,
                        # # carla.CityObjectLabel.Other,
                        # carla.CityObjectLabel.Pedestrians,
                        # carla.CityObjectLabel.Poles,
                        # carla.CityObjectLabel.RailTrack,
                        # carla.CityObjectLabel.Rider,
                        # carla.CityObjectLabel.RoadLines,
                        # carla.CityObjectLabel.Roads,
                        # # carla.CityObjectLabel.Sidewalks,
                        # carla.CityObjectLabel.Sky,
                        # carla.CityObjectLabel.Static,
                        # carla.CityObjectLabel.Terrain,
                        # carla.CityObjectLabel.TrafficLight,
                        # carla.CityObjectLabel.TrafficSigns,
                        # carla.CityObjectLabel.Train,
                        # carla.CityObjectLabel.Vegetation,
                        # carla.CityObjectLabel.Walls,
                        # carla.CityObjectLabel.Water
                    ]
        for type_index, bb_type in enumerate(bb_types):
            bounding_boxes = world.get_level_bbs(bb_type)
            for num, bb in enumerate(bounding_boxes):
                node_name = f'bb_{type_index}_{num}'
                # copy in bounding box
                if not 'bbox' in new_graph.graph:
                    new_graph.graph['bbox'] = {}
                new_graph.graph['bbox'][node_name] = carla_bb_to_matplotlib(bb.get_local_vertices())
                vertex = get_vert_from_location(bb.location)
                new_graph.add_node(node_name, vertex=vertex, sem_class=str(bb_type), vertex_point=vertex_to_point(vertex))
                new_graph.graph['node_colors'][node_name] = matplotlib.cm.tab20b(type_index)
                new_graph.graph['node_size'][node_name] = 50
        # precalc numpy position array
        new_graph.graph['numpy_position_array'] = np.array([new_graph.nodes[node]['vertex'][0:2] for node in new_graph.nodes])
        self._base_graph = new_graph

    def should_consider_actor(self, ego, ego_render_bbox, world_snapshot, actor):
        try:
            if type(actor) == carla.libcarla.TrafficLight:
                # for traffic lights check if any of the constituent traffic lights are in the render box
                return any([ego_render_bbox.intersects(Polygon(carla_bb_points_to_matplotlib(bbox.get_local_vertices()))) \
                        for bbox in actor.get_light_boxes()])
        except (AttributeError, RuntimeError) as e:  # runtime error is encountered on large maps when the traffic light is too far away
            # in older versions of CARLA, we can't get the separate traffic light boxes
            # in that case, revert to checking location
            return ego_render_bbox.contains(vertex_to_point(get_vert_from_location(actor.get_location())))
        # for other actors, check to see the actor itself is in the render bbox
        try:
            actor_transform = get_actor_transform(world_snapshot, actor)
            bbox = get_actor_bb_matplotlib(actor, actor_transform)
            bbox_poly = Polygon(bbox)
            return ego_render_bbox.intersects(bbox_poly)
        except AttributeError:
            # in older versions of CARLA, not all actors have bounding_boxes
            return ego_render_bbox.contains(vertex_to_point(get_vert_from_location(actor.get_location())))


    def generate_graph_for_frame(self, frame_num, ego_control: "Union[dict[str, any], carla.VehicleControl]"=None, render_dist: float=50, perform_init: bool=False, exclude_ids: list=None):
        if not self._init:
            if perform_init:
                self.init_to_world()
            else:
                raise ValueError("Cannot generate graph without running initialization. It is recommented to manually call initialization before starting the scenario.")
        if exclude_ids is None:
            exclude_ids = []
        if self.ego_id not in exclude_ids:
            exclude_ids.append(self.ego_id)
        world_snapshot = self._world.get_snapshot()
        ego = self._world.get_actor(self._ego_id)
        ego_transform = get_actor_transform(world_snapshot, ego)
        ego_render_bbox, max_dist_from_ego = self.__get_ego_render_bbox(ego_transform, render_dist)
        ego_point = get_vert_from_location(ego_transform.location)[0:2]
        norms = np.linalg.norm(self._base_graph.graph['numpy_position_array'] - ego_point, axis=1)
        nodes_near_ego = [node for node, norm in zip(self._base_graph.nodes, norms) if norm <= max_dist_from_ego]
        graph_nodes = get_nodes_in_polygon(self._base_graph, nodes_near_ego, ego_render_bbox)
        graph_nodes = filter_by_valid_gradient(self._base_graph, graph_nodes, get_vert_from_location(ego_transform.location), ['Driving'])
        sub_graph = nx.induced_subgraph(self._base_graph, graph_nodes)
        sub_graph = sub_graph.copy()
        subgraph_driving_nodes = filter_semantics(sub_graph, sub_graph.nodes, 'Driving')
        for key in ['node_colors', 'node_size', 'bbox']:
            if key not in sub_graph.graph:
                sub_graph.graph[key] = {}
        self.__add_actor_to_graph(world_snapshot, sub_graph, ego, actor_name='ego', sem_class='ego', filtered_nodes=subgraph_driving_nodes)
        sub_graph.nodes['ego']['carla_affected_by_traffic_light'] = ego.is_at_traffic_light()
        copy_carla_control(ego.get_control(), sub_graph.nodes['ego'])
        copy_carla_actor_state(sub_graph.nodes['ego'], ego)
        # copy in supplied ego control
        if ego_control is not None:
            if isinstance(ego_control, carla.VehicleControl):
                # convert from VehicleControl to dict
                ego_control = copy_carla_control(ego_control)
            for key, value in ego_control.items():
                sub_graph.nodes['ego'][f'ego_control_{key}'] = value
        sub_graph.graph['node_colors']['ego'] = (0,0,0,0.5)
        sub_graph.graph['node_size']['ego'] = 100
        sub_graph.graph['bbox']['ego'] = get_actor_bb_matplotlib(ego, ego_transform)
        actors = [actor for actor in self._world.get_actors() if ego_render_bbox.contains(vertex_to_point(get_vert_from_location(actor.get_location())))\
                    and actor.id not in exclude_ids and not 'spectator' in actor.type_id]
        actors = list({
            actor.id: actor for actor in self._world.get_actors() if actor.id not in exclude_ids and not 'spectator' in actor.type_id and \
                not 'sensor' in actor.type_id and self.should_consider_actor(ego, ego_render_bbox, world_snapshot, actor)
        }.values())
        for index, actor in enumerate(actors):
            sem_class = str(actor.type_id)
            if sem_class == 'traffic.traffic_light':
                traffic_light_nodes = []
                try:
                    for index, bbox in enumerate(actor.get_light_boxes()):
                        pts_to_check = bbox.get_local_vertices()
                        pts_to_check.append(bbox.location)
                        visible_from_ego = self.is_visible_from_ego(ego, actor, pts_to_check, bbox, carla.Transform())
                        if not visible_from_ego:
                            print('skipping for occlusion', f'{sem_class}_{actor.id}_{index}')
                            # if we hit something in the middle, then disregard
                            continue
                        actor_name = f'{sem_class}_{actor.id}_{index}'
                        actor_bbox = carla_bb_points_to_matplotlib(bbox.get_local_vertices())
                        forward = bbox.location + actor.get_transform().get_forward_vector().make_unit_vector()
                        forward = [forward.x, forward.y, forward.z]
                        right = bbox.location + actor.get_transform().get_right_vector().make_unit_vector()
                        right = [right.x, right.y, right.z]
                        up = bbox.location + actor.get_transform().get_up_vector().make_unit_vector()
                        up = [up.x, up.y, up.z]
                        actor_name = self.__add_to_graph(sub_graph, get_vert_from_location(bbox.location),
                                                          get_vert_from_location(forward),
                                                           get_vert_from_location(right),
                                                            get_vert_from_location(up),
                                                             actor_bbox, actor_name, sem_class, filtered_nodes=subgraph_driving_nodes)
                        traffic_light_nodes.append(actor_name)
                        sub_graph.nodes[actor_name]['carla_id'] = actor.id
                        sub_graph.nodes[actor_name]['carla_type_id'] = actor.type_id
                        sub_graph.nodes[actor_name]['carla_parent_id'] = actor.parent.id if actor.parent is not None else None
                        sub_graph.nodes[actor_name]['carla_semantic_tags'] = str(actor.semantic_tags)
                        sub_graph.nodes[actor_name]['carla_actor_state'] = str(actor.actor_state)
                        sub_graph.nodes[actor_name]['carla_attr'] = str(actor.attributes)
                except AttributeError:
                    # in older versions of CARLA, we cannot get the list of individual lights
                    actor_name = f'{sem_class}_{actor.id}'
                    actor_bbox = None
                    actor_name = self.__add_to_graph(sub_graph, get_vert_from_location(actor.get_location()),
                                                      get_vert_from_location(actor.get_transform().get_forward_vector()),
                                                        get_vert_from_location(actor.get_transform().get_right_vector()), 
                                                          get_vert_from_location(actor.get_transform().get_up_vector()),
                                                            actor_bbox, actor_name, sem_class, filtered_nodes=subgraph_driving_nodes,
                                                            threedbbox=parse3dbb(actor, get_actor_transform(world_snapshot, actor)))
                    sub_graph.nodes[actor_name]['carla_id'] = actor.id
                    sub_graph.nodes[actor_name]['carla_type_id'] = actor.type_id
                    traffic_light_nodes.append(actor_name)
                # handle edges for what the light controls
                try:
                    affected_waypoints = set(actor.get_affected_lane_waypoints())
                    nodes = [node for node in subgraph_driving_nodes if \
                                any([get_distance(sub_graph, node, get_vert_from_location(waypoint.transform.location), plane_projection=False) < self.segment_length * 0.5\
                                        for waypoint in affected_waypoints])]
                except AttributeError:
                    # in older versions of CARLA, we cannot directly get the list of affected waypoints
                    bbox = Polygon(carla_bb_points_to_matplotlib(actor.trigger_volume.get_world_vertices(actor.get_transform())))
                    nodes = [node for node in subgraph_driving_nodes if get_bbox_intersects_poly(sub_graph, bbox, node)]
                for actor_name in traffic_light_nodes:
                    for node in nodes:
                        sub_graph.add_edge(actor_name, node, distance=np.nan, dist_relationship='controlsTrafficOf')
                    sub_graph.nodes[actor_name]['light_state'] = str(actor.state)
                    sub_graph.graph['bbox'][actor_name] = actor_bbox
                    sub_graph.graph['node_colors'][actor_name] = (1,0.5,0,1)
                    sub_graph.graph['node_size'][actor_name] = 50
            else:
                actor_name = f'{sem_class}_{actor.id}'
                pts_to_check = []
                can_bbox = True
                try:
                    pts_to_check.extend(actor.bounding_box.get_world_vertices(actor.get_transform()))
                except AttributeError:
                    # in older versions of carla, not all actors have bboxes
                    can_bbox = False
                pts_to_check.append(actor.get_location())
                if can_bbox:
                    visible_from_ego = self.is_visible_from_ego(ego, actor, pts_to_check, actor.bounding_box, actor.get_transform())
                else:
                    visible_from_ego = self.is_visible_from_ego(ego, actor, pts_to_check, None, None)
                if not visible_from_ego:
                    # if we hit something in the middle, then disregard
                    continue
                if can_bbox:
                    actor_name = self.__add_actor_to_graph(world_snapshot, sub_graph, actor, actor_name=actor_name, sem_class=sem_class, filtered_nodes=subgraph_driving_nodes)
                    actor_transform = get_actor_transform(world_snapshot, actor)
                    sub_graph.graph['bbox'][actor_name] = get_actor_bb_matplotlib(actor, actor_transform)
                else:
                    actor_name = self.__add_to_graph(sub_graph, get_vert_from_location(actor.get_location()),
                                                      get_vert_from_location(actor.get_transform().get_forward_vector()),
                                                        get_vert_from_location(actor.get_transform().get_right_vector()), 
                                                        get_vert_from_location(actor.get_transform().get_up_vector()),
                                                         None, actor_name, sem_class, filtered_nodes=subgraph_driving_nodes)
                    sub_graph.graph['bbox'][actor_name] = None
                if 'walker' in actor_name:
                    sub_graph.graph['node_colors'][actor_name] = (0,1,1,1)
                else:
                    sub_graph.graph['node_colors'][actor_name] = (1,0,1,1)
                if sem_class == 'traffic.stop':
                    bbox = Polygon(carla_bb_points_to_matplotlib(actor.trigger_volume.get_world_vertices(actor.get_transform())))
                    nodes = [node for node in subgraph_driving_nodes if get_bbox_intersects_poly(sub_graph, bbox, node)]
                    for node in nodes:
                        sub_graph.add_edge(actor_name, node, distance=np.nan, dist_relationship='controlsTrafficOf')
                sub_graph.graph['node_size'][actor_name] = 75
                sub_graph.nodes[actor_name]['carla_id'] = actor.id
                sub_graph.nodes[actor_name]['carla_type_id'] = actor.type_id
                if isinstance(actor, carla.Vehicle):
                    light_state = actor.get_light_state()
                    # save the raw integer value for interoperability
                    sub_graph.nodes[actor_name]['vehicle_light_state'] = light_state
                    # add separate attributes for each light type for easy access
                    for light_type_name, light_type_value in zip(carla.VehicleLightState.names, carla.VehicleLightState.values):
                        sub_graph.nodes[actor_name][f'light_{light_type_name}'] = (light_state & light_type_value) > 0
        landmarks = {}
        waypoint_map = defaultdict(list)
        for node in sub_graph.nodes:
            data = sub_graph.nodes[node]
            if data['sem_class'] == 'Driving':
                landmark_list = data['waypoint_obj'].get_landmarks(render_dist)
                landmarks.update({landmark.id: landmark for landmark in landmark_list})
                for landmark in landmark_list:
                    waypoint_map[landmark.id].append(node)
        landmarks = [landmark for landmark in landmarks.values() if ego_render_bbox.contains(vertex_to_point(get_vert_from_location(landmark.transform.location)))]
        for index, landmark in enumerate(landmarks):
            sem_class = f'landmark_{landmark.country}_{landmark.type}_{landmark.sub_type}'
            actor_name = f'{sem_class}_{landmark.id}'
            self.__add_landmark_to_graph(world_snapshot, sub_graph, landmark, actor_name=actor_name, sem_class=sem_class)
            sub_graph.nodes[actor_name]['landmark_country'] = landmark.country
            sub_graph.nodes[actor_name]['landmark_type'] = landmark.type
            sub_graph.nodes[actor_name]['landmark_sub_type'] = landmark.sub_type
            sub_graph.nodes[actor_name]['landmark_value'] = landmark.value
            sub_graph.nodes[actor_name]['landmark_unit'] = landmark.unit
            sub_graph.nodes[actor_name]['landmark_text'] = landmark.text
            sub_graph.nodes[actor_name]['landmark_orientation'] = landmark.orientation
            sub_graph.nodes[actor_name]['landmark_height'] = landmark.height
            sub_graph.nodes[actor_name]['landmark_width'] = landmark.width
            sub_graph.nodes[actor_name]['landmark_pitch'] = landmark.pitch
            sub_graph.nodes[actor_name]['landmark_roll'] = landmark.roll
            sub_graph.nodes[actor_name]['landmark_z_offset'] = landmark.z_offset
            sub_graph.nodes[actor_name]['landmark_h_offset'] = landmark.h_offset
            sub_graph.nodes[actor_name]['landmark_id'] = landmark.id
            sub_graph.nodes[actor_name]['landmark_name'] = landmark.name
            sub_graph.nodes[actor_name]['landmark_lane_validities'] = str(landmark.get_lane_validities())
            sub_graph.nodes[actor_name]['landmark_is_dynamic'] = landmark.is_dynamic
            sub_graph.graph['node_colors'][actor_name] = (0,0,0,1)
            sub_graph.graph['node_size'][actor_name] = 500
            for waypoint_node in waypoint_map[landmark.id]:
                sub_graph.add_edge(actor_name, waypoint_node, distance=np.nan, dist_relationship='mayAffect')
        if 'closeness_map' in sub_graph.graph:
            del sub_graph.graph['closeness_map']
        if 'numpy_position_array' in sub_graph.graph:
            del sub_graph.graph['numpy_position_array']  # this is only used in calculating the subgraph, no need to store
        if 'bbox_polygon' in sub_graph.graph:
            del sub_graph.graph['bbox_polygon']
        self._recent_frames[frame_num] = sub_graph
        # delete all extra references for other nodes
        # need to check node_colors, node_size, and bbox
        for data_type in ['node_colors', 'node_size', 'bbox']:
            # recreate the dict filtering to only the nodes we care about
            sub_graph.graph[data_type] = {node: sub_graph.graph[data_type][node] for node in sub_graph.nodes if node in sub_graph.graph[data_type]}
        return sub_graph

    def is_visible_from_ego(self, ego, actor, pts_to_check, actor_bbox, actor_bbox_transform):
        if not self._check_occlusion:
            return True
        one_visible = False
        for loc in pts_to_check:
            hit_points = self._world.cast_ray(ego.get_location(), loc)
            # filter the points to remove those inside the actor
            if actor_bbox is not None:
                hit_points = [pt for pt in hit_points if not actor_bbox.contains(pt.location, actor_bbox_transform)]
            if len(hit_points) > 0 and hit_points[-1].label in actor.semantic_tags:
                # remove the last point because that is the object itself
                del hit_points[-1]
            # filter the points to remove self-collisions with ego and those that are NONE
            hit_points = [pt for pt in hit_points if not ego.bounding_box.contains(pt.location, ego.get_transform()) and \
                          carla.libcarla.CityObjectLabel.values[pt.label] != carla.libcarla.CityObjectLabel.NONE]
            if len(hit_points) == 0:
                one_visible = True
        return one_visible
    
    def get_base_abstraction(self, frame_num=None, graph=None):
        if frame_num in self._base_abstraction_buffer:
            return self._base_abstraction_buffer[frame_num]
        if frame_num in self._recent_frames:
            graph = self._recent_frames[frame_num]
        if graph is None:
            raise ValueError("Graph not contained in buffer and not provided.")
        self._base_abstraction_buffer[frame_num] = process_to_abstract(graph)
        return self._base_abstraction_buffer[frame_num]
    
    def get_all_abstractions(self, frame_num=None, graph=None, proximity_thresholds=None, directional_thresholds=None, proximity_relations=None, directional_relations=None):
        if frame_num in self._recent_frames:
            graph = self._recent_frames[frame_num]
        if graph is None:
            raise ValueError("Graph not contained in buffer and not provided.")
        base_abstraction = self.get_base_abstraction(frame_num)
        for filter in [False, True]:
            for curvature in Curvature:
                process_to_rsv(graph, proximity_thresholds, directional_thresholds, proximity_relations, directional_relations, filter, curvature, base_abstraction)

    def save(self, sg, sg_path):
        sg_path = Path(sg_path)
        if not sg_path.exists():
            sg_path.mkdir(parents=True, exist_ok=True)
        # Save pkl file
        with open(sg_path / f"{self.timestep}.pkl",'wb') as f:
            IgnoreWaypointPickler(f).dump(sg)
        self.timestep += 1

    def __add_to_graph(self, graph, vertex, forward, right, up, bbox, actor_name=None, sem_class=None, lane_types=None, filtered_nodes=None, entity_id=None, threedbbox=None):
        if lane_types is None:
            lane_types = ['Driving', 'Sidewalk']
        if filtered_nodes is None:
            filtered_nodes = filter_semantics(graph, graph.nodes, lane_types)
        if bbox is not None:
            filtered_nodes = get_intersecting_nodes(graph, filtered_nodes, Polygon(bbox))
        else:
            filtered_nodes = get_containing_nodes(graph, filtered_nodes, vertex_to_point(vertex))
        if actor_name is None:
            actor_name = f'{sem_class}_{uuid.uuid4()}'
        graph.add_node(actor_name, vertex=vertex, sem_class=sem_class, forward=forward, right=right, up=up, vertex_point=vertex_to_point(vertex), entity_id=entity_id, threedbbox=threedbbox)
        for node in filtered_nodes:
            dist = np.linalg.norm(graph.nodes[node]['vertex'] - vertex)
            graph.add_edge(actor_name, node, distance=dist, dist_relationship='isIn')
        return actor_name
    
    def __get_ego_render_bbox(self, transform, front_dist, side_dist=None, back_dist=0):
        if side_dist is None:
            side_dist = front_dist / 2
        # small buffer to ensure we get the node ego is on
        map_precision = self._segment_length * 2
        forward = get_vert_from_location(transform.get_forward_vector().make_unit_vector())
        forward *= (front_dist + map_precision)
        back = -(back_dist + map_precision) * get_vert_from_location(transform.get_forward_vector().make_unit_vector())
        right = get_vert_from_location(transform.get_right_vector().make_unit_vector())
        right *= side_dist
        start = get_vert_from_location(transform.location)
        lower_right = start + right + back
        upper_right = lower_right + forward
        upper_left = upper_right - right - right
        lower_left = upper_left - forward
        points = [(lower_right[0], lower_right[1]),
                (upper_right[0], upper_right[1]),
                (upper_left[0], upper_left[1]),
                (lower_left[0], lower_left[1])]
        ego_render_bbox = Polygon(points)
        max_dist = max([np.linalg.norm(start[0:2] - pt[0:2])] for pt in [lower_right, lower_left, upper_right, upper_left])
        return ego_render_bbox, max_dist
    

    def __add_actor_to_graph(self, world_snapshot, graph, actor, actor_name=None, sem_class=None, lane_types=None, filtered_nodes=None):
        actor_transform = get_actor_transform(world_snapshot, actor)
        bbox = get_actor_bb_matplotlib(actor, actor_transform)
        if bbox is None:
            return
        vertex = get_vert_from_location(actor_transform.location)
        forward, right, up = get_rotation_pointers(actor_transform.rotation)
        actor_name = self.__add_to_graph(graph, vertex, forward, right, up, bbox,
                                         actor_name, sem_class, lane_types, filtered_nodes,
                                         entity_id=actor.id, threedbbox=parse3dbb(actor, actor_transform))
        return actor_name
    
    def __add_landmark_to_graph(self, world_snapshot, graph, landmark, actor_name=None, sem_class=None, lane_types=None, filtered_nodes=None):
        landmark_transform = landmark.transform
        if landmark_transform is None:
            return
        vertex = get_vert_from_location(landmark_transform.location)
        forward, right, up = get_rotation_pointers(landmark_transform.rotation)
        actor_name = self.__add_to_graph(graph, vertex, forward, right, up, None, actor_name, sem_class, lane_types, filtered_nodes)
        return actor_name
    

def copy_carla_control(control, attr_dict=None):
    if attr_dict is None:
        attr_dict = {}
    attr_dict['carla_throttle'] = control.throttle
    attr_dict['carla_steer'] = control.steer
    attr_dict['carla_brake'] = control.brake
    attr_dict['carla_hand_brake'] = control.hand_brake
    attr_dict['carla_reverse'] = control.reverse
    attr_dict['carla_manual_gear_shift'] = control.manual_gear_shift
    attr_dict['carla_gear'] = control.gear
    return attr_dict


def copy_carla_actor_state(attr_dict, actor):
    copy_vector(attr_dict, 'carla_acc', actor.get_acceleration())
    copy_vector(attr_dict, 'carla_ang_vel', actor.get_angular_velocity())
    vel = actor.get_velocity()
    copy_vector(attr_dict, 'carla_vel', vel)
    attr_dict['carla_speed'] = vel.length()

def copy_vector(attr_dict, prefix, vector):
    attr_dict[f'{prefix}_x'] = vector.x
    attr_dict[f'{prefix}_y'] = vector.y
    attr_dict[f'{prefix}_z'] = vector.z
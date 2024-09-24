from collections import defaultdict
import copy
from enum import Enum
import itertools
import math
import networkx as nx
import numpy as np

from carla_sgg.actors import get_base_type, get_special_type
from carla_sgg.utils import get_curvature_from_node_name, get_node_distance, valid_gradient


class Curvature(Enum):
    NONE=''
    SINGLE='single'
    RELATIONAL_DISTANCES='reldists'


class Node:
    def __init__(self, name, base_class=None, attr=None):
        self.name = name
        self.base_class = base_class
        self.attr = {} if attr is None else attr

    def __repr__(self):
        return str(self.name)  # name should always be a string anyway, but just to be safe


class EgoMultipleLaneException(Exception):
    pass

class EgoNotInLaneException(Exception):
    pass

class MultipleLaneChoicesException(Exception):
    pass


def process_to_abstract(graph):
    actor_counter = defaultdict(int)
    new_graph = nx.MultiDiGraph()
    new_graph.graph['node_locs'] = {}
    node_map = {}
    driving_nodes = [node for node in graph.nodes if graph.nodes[node]['sem_class']  == 'Driving' and 
                     valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    sidewalk_nodes = [node for node in graph.nodes if graph.nodes[node]['sem_class']  == 'Sidewalk' and 
                     valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    # add all of the nodes from the road first
    for node in driving_nodes:
        is_junction = graph.nodes[node]['is_junction']
        if is_junction:
            in_edges = [u for u, _ in graph.in_edges(node) if u in driving_nodes]
            out_edges = [v for _, v in graph.out_edges(node) if v in driving_nodes]
            in_degree = len(in_edges)
            out_degree = len(out_edges)
            base_class = f'Driving^{in_degree}:{out_degree}'
        else:
            curve = graph.nodes[node]['curve']
            base_class = f'Driving@{curve}'
        name = f'{base_class}_{actor_counter[base_class]}'
        actor_counter[base_class] += 1
        new_node = Node(name=name, base_class=base_class)
        node_map[node] = new_node
        new_graph.add_node(new_node)
    # then add all of the sidewalks
    for node in sidewalk_nodes:
        new_node = Node(name='Sidewalk')
        node_map[node] = new_node
        new_graph.add_node(new_node)
    # then add all of the connections between roads and connections between sidewalks
    ground_nodes = driving_nodes + sidewalk_nodes
    for node in ground_nodes:
        for u, v, a in graph.out_edges(node, data=True):
            if v not in ground_nodes:
                if valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[v]['vertex']):
                    raise AssertionError("Found invalid edge when parsing the road graph for abstract")
                else:
                    continue  # the original graph generated added an edge that matches a change in the Z plane, skip
            label= 'can_lane_change_to' if a['lane_change'] else 'travels_to'
            source = node_map[u]
            target = node_map[v]
            new_graph.add_edge(source, target, label=label)
    # handle non-ego objects
    actors = [node for node in graph.nodes if graph.nodes[node]['sem_class']  != 'Driving' and
                    graph.nodes[node]['sem_class']  != 'Sidewalk' and
                    valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    for actor in actors:
        edges = [edge for edge in graph.out_edges(actor, data=True)]
        sem_class = graph.nodes[actor]['sem_class']
        if len(edges) == 0:
            if sem_class == 'ego':
                raise EgoNotInLaneException
            continue
        # create the actor node
        if sem_class != 'ego':
            base_class = get_base_type(sem_class)
            if base_class is None:
                continue
            name = f'{base_class}_{actor_counter[base_class]}'
            actor_counter[base_class] += 1
        else:
            name = 'ego'
        new_node = Node(name=name, attr=graph.nodes[actor])
        node_map[actor] = new_node
        # add the edges into the graph
        for u, v, a in edges:
            if v not in node_map:  # the other end of the edge was removed by the Z filter
                continue
            target = node_map[v]
            label = a['dist_relationship']
            new_graph.add_edge(new_node, target, label=label)
    for old_node, new_node in node_map.items():
        new_graph.graph['node_locs'][new_node] = graph.nodes[old_node]['vertex']
    return new_graph

DEFAULT_PROXIMITY_THRESHOLDS = [['safety_hazard', 2], ['near_coll',4],['super_near',7],['very_near',10],['near',16],['visible',25]]
DEFAULT_DIRECTIONAL_THRESHOLDS = [['inDFrontOf',[[45,90],[90,135]]], ['inSFrontOf',[[0,45],[135,180]]], ['atDRearOf',[[225,270],[270,315]]], ['atSRearOf',[[180,225],[315,360]]]]
DEFAULT_PROXIMITY_RELATIONS = [['ego', 'person', 25], ['ego', 'bicycle', 25], ['ego', 'car', 25], ['ego', 'motorcycle', 25], ['ego', 'airplane', 25], ['ego', 'bus', 25], ['ego', 'train', 25], ['ego', 'truck', 25], ['ego', 'boat', 25], ['ego', 'traffic light', 25], ['ego', 'fire hydrant', 25], ['ego', 'street sign', 25], ['ego', 'stop sign', 25], ['ego', 'parking meter', 25], ['ego', 'bench', 25]]
DEFAULT_DIRECTIONAL_RELATIONS = [['ego', 'person', 25], ['ego', 'bicycle', 25], ['ego', 'car', 25], ['ego', 'motorcycle', 25], ['ego', 'airplane', 25], ['ego', 'bus', 25], ['ego', 'train', 25], ['ego', 'truck', 25], ['ego', 'boat', 25], ['ego', 'traffic light', 25], ['ego', 'fire hydrant', 25], ['ego', 'street sign', 25], ['ego', 'stop sign', 25], ['ego', 'parking meter', 25], ['ego', 'bench', 25]]


def process_to_junction_rsv(graph, proximity_thresholds=None, directional_thresholds=None, proximity_relations=None, directional_relations=None):
    if proximity_thresholds is None:
        proximity_thresholds = DEFAULT_PROXIMITY_THRESHOLDS
    if directional_thresholds is None:
        directional_thresholds = DEFAULT_DIRECTIONAL_THRESHOLDS
    if proximity_relations is None:
        proximity_relations = DEFAULT_PROXIMITY_RELATIONS
    if directional_relations is None:
        directional_relations = DEFAULT_DIRECTIONAL_RELATIONS
    driving_nodes = [node for node in graph.nodes if graph.nodes[node]['sem_class']  == 'Driving' and 
                     valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    actors = [node for node in graph.nodes if graph.nodes[node]['sem_class']  != 'Driving' and
                graph.nodes[node]['sem_class']  != 'Sidewalk']
    junction_road_lane = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    for node in driving_nodes:
        junction_road_lane[graph.nodes[node]['waypoint_junction_id']][graph.nodes[node]['waypoint_road_id']][graph.nodes[node]['waypoint_lane_id']].append(node)
    new_graph = nx.MultiDiGraph()
    waypoint_to_new = {}
    new_lane_nodes = {}
    for junction, road_lanes in junction_road_lane.items():
        junction_node = None
        if junction != -1:
            # this is actually a junction node
            junction_node = Node(name=f'Junction {junction}')
        for road, lane_nodes in road_lanes.items():
            road_node = Node(name=f'Road {road}')
            lane_map = {}
            if junction_node is not None:
                new_graph.add_edge(road_node, junction_node, label='isIn')
            for lane, nodes in lane_nodes.items():
                lane_node = Node(name=f"Lane {road}_{('+' if lane > 0 else '-')}{abs(lane)}")
                lane_map[lane] = lane_node
                new_lane_nodes[(junction, road, lane)] = lane_node
                new_graph.add_edge(lane_node, road_node, label='isIn')
                for node in nodes:
                    waypoint_to_new[node] = (junction_node, road_node, lane_node)
            sorted_lanes = sorted(lane_nodes.keys())
            # check for inter-lane relationships: which lanes are opposing each other and which are left/right
            # note that this uses the two lane points that are closest together to check
            for lane1, lane2 in zip(sorted_lanes, sorted_lanes[1:]):
                # check opposing
                if math.copysign(1, lane1) != math.copysign(1, lane2):
                    # if they have differing signs then they are in opposite directions
                    new_graph.add_edge(lane_map[lane1], lane_map[lane2], label='opposes')
                    new_graph.add_edge(lane_map[lane2], lane_map[lane1], label='opposes')
                # check left/right of
                for lane1_to_check, lane2_to_check in [(lane1, lane2), (lane2, lane1)]:
                    # find the pair with the minimum distance
                    _, l1, l2 = min([(get_node_distance(graph, l1, l2), l1, l2) \
                                         for l1, l2 in itertools.product(lane_nodes[lane1_to_check], lane_nodes[lane2_to_check])])
                    # compute left vs right
                    vert1 = copy.deepcopy(graph.nodes[l1]['vertex'])
                    vert2 = copy.deepcopy(graph.nodes[l2]['vertex'])
                    vert1_forward = copy.deepcopy(vert1 + graph.nodes[l1]['forward'])
                    new_actor1 = lane_map[lane1_to_check]
                    new_actor2 = lane_map[lane2_to_check]
                    vert1[0] *= -1
                    vert2[0] *= -1
                    vert1_forward[0] *= -1
                    v2 = vert2 - vert1
                    v1f = vert1_forward - vert1
                    angle1_to_2 = np.degrees(np.arctan2(v2[1], v2[0]) - np.arctan2(v1f[1], v1f[0]))
                    if angle1_to_2 < 0:
                        angle1_to_2 += 360
                    angle_to_check = angle1_to_2 + 90
                    if angle_to_check > 360:
                        angle_to_check -= 360
                    if angle_to_check < 0:
                        angle_to_check += 360
                    if angle1_to_2 <= 180:
                        # actor2 is to the left or actor1
                        new_graph.add_edge(new_actor2, new_actor1, label='toLeftOf')
                    else:
                        new_graph.add_edge(new_actor2, new_actor1, label='toRightOf')
    # add lane changes and inter-road relationships
    checked = defaultdict(list)
    for junction, road_lanes in junction_road_lane.items():
        for road, lane_nodes in road_lanes.items():
            for lane, nodes in lane_nodes.items():
                this_lane_node = new_lane_nodes[(junction, road, lane)]
                for node in nodes:
                    for u, v, a in graph.out_edges(node, data=True):
                        if v not in driving_nodes:
                            continue
                        other_lane_node = new_lane_nodes[(graph.nodes[v]['waypoint_junction_id'], graph.nodes[v]['waypoint_road_id'], graph.nodes[v]['waypoint_lane_id'])]
                        if this_lane_node == other_lane_node:
                            continue
                        pair_key = (this_lane_node, other_lane_node)
                        if a['lane_change']:
                            if 'laneChange' not in checked[pair_key]:
                                new_graph.add_edge(this_lane_node, other_lane_node, label='laneChange')
                                checked[pair_key].append('laneChange')
                        elif not (graph.nodes[v]['waypoint_junction_id'] == junction and graph.nodes[v]['waypoint_road_id'] == road and graph.nodes[v]['waypoint_lane_id'] == lane):
                            if 'travelsTo' not in checked[pair_key]:
                                new_graph.add_edge(this_lane_node, other_lane_node, label='travelsTo')
                                checked[pair_key].append('travelsTo')
    # add other entities
    actor_counter = defaultdict(int)
    node_map = {}
    added_actors = []
    offroad = Node(name='Off Road')
    for actor in actors:
        edges = [(u,v,a) for (u,v,a) in graph.out_edges(actor, data=True) if v in driving_nodes]
        sem_class = graph.nodes[actor]['sem_class']
        if sem_class == 'ego':
            ego_actor = actor
        # create the actor node
        if sem_class != 'ego':
            base_class = get_base_type(sem_class)
            special_class = get_special_type(sem_class)
            if base_class is None:
                continue
            name = f'{base_class}_{actor_counter[base_class]}'
            actor_counter[base_class] += 1
        else:
            name = 'ego'
            base_class='ego'
            special_class = 'ego'
        added_actors.append(actor)
        new_node = Node(name=name, base_class=base_class, attr=graph.nodes[actor])
        new_node.attr['special_class'] = special_class
        new_graph.add_node(new_node)
        node_map[actor] = new_node
        # now we need to find the lane that it is in, if we already have it
        checked = []
        if len(edges) == 0:
            new_graph.add_edge(new_node, offroad, label='isIn')
        else:
            for u, v, a in edges:
                other_lane_node = new_lane_nodes[(graph.nodes[v]['waypoint_junction_id'], graph.nodes[v]['waypoint_road_id'], graph.nodes[v]['waypoint_lane_id'])]
                check_key = (new_node, other_lane_node, a['dist_relationship'])
                if check_key not in checked:
                    checked.append(check_key)
                    new_graph.add_edge(new_node, other_lane_node, label=a['dist_relationship'])
    # add edges between the actors
    for actor1, actor2 in itertools.permutations(added_actors, 2):
        vert1 = copy.deepcopy(graph.nodes[actor1]['vertex'])
        vert2 = copy.deepcopy(graph.nodes[actor2]['vertex'])
        vert1_forward = copy.deepcopy(vert1 + graph.nodes[actor1]['forward'])
        new_actor1 = node_map[actor1]
        new_actor2 = node_map[actor2]
        actor1_base = new_actor1.base_class
        actor2_base = new_actor2.base_class
        distance = np.linalg.norm(vert1 - vert2)
        added_lr = False

        vert1[0] *= -1
        vert2[0] *= -1
        vert1_forward[0] *= -1

        v2 = vert2 - vert1
        v1f = vert1_forward - vert1
        angle1_to_2 = np.degrees(np.arctan2(v2[1], v2[0]) - np.arctan2(v1f[1], v1f[0]))
        # RSV uses a different angle convention, convert
        if angle1_to_2 < 0:
            angle1_to_2 += 360
        angle_to_check = angle1_to_2 + 90
        if angle_to_check > 360:
            angle_to_check -= 360
        if angle_to_check < 0:
            angle_to_check += 360
        for class1, class2, dist in proximity_relations:
            if ((class1 == actor1_base and class2 == actor2_base) or \
            (class1 == actor2_base and class2 == actor1_base)) and \
            distance <= dist:
                dist_enum = len(proximity_thresholds) - 1
                if distance <= proximity_thresholds[dist_enum][1]:
                    while dist_enum > 0 and distance <= proximity_thresholds[dist_enum-1][1]:
                        dist_enum -= 1
                    new_graph.add_edge(new_actor1, new_actor2, label=proximity_thresholds[dist_enum][0])
        for class1, class2, dist in directional_relations:
            if ((class1 == actor1_base and class2 == actor2_base) or \
            (class1 == actor2_base and class2 == actor1_base)) and \
            distance <= dist:
                if not added_lr:
                    added_lr = True
                    if angle1_to_2 <= 180:
                        # actor2 is to the left or actor1
                        new_graph.add_edge(new_actor2, new_actor1, label='toLeftOf')
                    else:
                        new_graph.add_edge(new_actor2, new_actor1, label='toRightOf')
                try:
                    for dir_rel, list_of_ranges in directional_thresholds:
                        for ranges in list_of_ranges:
                            if ranges[0] <= angle_to_check <= ranges[1]:
                                new_graph.add_edge(new_actor2, new_actor1, label=dir_rel)
                                raise StopIteration
                except StopIteration:
                    pass
    return new_graph

    



def process_to_rsv(graph, proximity_thresholds=None, directional_thresholds=None, proximity_relations=None, directional_relations=None,
                    filter_if_no_relation=False, curvature=Curvature.NONE, abstract=None, description=None,
                    gen_relationships=True, gen_lanes=True):
    if proximity_thresholds is None:
        proximity_thresholds = DEFAULT_PROXIMITY_THRESHOLDS
    if directional_thresholds is None:
        directional_thresholds = DEFAULT_DIRECTIONAL_THRESHOLDS
    if proximity_relations is None:
        proximity_relations = DEFAULT_PROXIMITY_RELATIONS
    if directional_relations is None:
        directional_relations = DEFAULT_DIRECTIONAL_RELATIONS
    driving_nodes = [node for node in graph.nodes if graph.nodes[node]['sem_class']  == 'Driving' and 
                     valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    sidewalk_nodes = [node for node in graph.nodes if graph.nodes[node]['sem_class']  == 'Sidewalk' and 
                     valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    ground_nodes = driving_nodes + sidewalk_nodes
    actors = [node for node in graph.nodes if graph.nodes[node]['sem_class']  != 'Driving' and
                    graph.nodes[node]['sem_class']  != 'Sidewalk' and
                    valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[node]['vertex'])]
    new_graph = nx.MultiDiGraph()
    root_road = Node(name='Root Road')
    new_graph.add_node(root_road)
    if curvature == Curvature.NONE:
        ego_lane = Node(name='Ego Lane')
    elif curvature == Curvature.SINGLE or curvature == Curvature.RELATIONAL_DISTANCES:
        if abstract is None:
            raise ValueError(f'Adding curvature to RSV graphs requires the abstract graph be provided.')
        ego_abstract = [node for node in abstract.nodes if node.name == 'ego'][0]
        ego_connected_nodes = [v for u,v in abstract.out_edges(ego_abstract)]
        ego_connected_set = set(ego_connected_nodes)
        if len(ego_connected_nodes) > 1:
            forward_node = [node for node in ego_connected_nodes if len(set([u for u,v,a in abstract.in_edges(node, data=True) if a['label']=='travels_to']).intersection(ego_connected_set)) == 0]
            if len(forward_node) != 1:
                raise RuntimeError('Cannot find forward node for ego lane')
            forward_node = forward_node[0]
        else:
            forward_node = ego_connected_nodes[0]
        curvatures = [get_curvature_from_node_name(forward_node)]
        if curvature == Curvature.RELATIONAL_DISTANCES:
            ego_loc = abstract.graph['node_locs'][ego_abstract]
            cur_node = forward_node
            cur_thresh = 0
            while cur_thresh < len(proximity_thresholds) and cur_node is not None:
                dist = np.linalg.norm(ego_loc - abstract.graph['node_locs'][cur_node])
                if dist >= proximity_thresholds[cur_thresh][1]:
                    curvatures.append(get_curvature_from_node_name(cur_node))
                    cur_thresh += 1
                next_node = [v for u,v,a in abstract.out_edges(cur_node, data=True) if a['label'] == 'travels_to']
                if len(next_node) > 1:
                    raise RuntimeError("Ego lane diverges")
                cur_node = None if len(next_node) == 0 else next_node[0]
        curvatures_str = ','.join([str(curve) for curve in curvatures])
        ego_lane = Node(name=f'Ego Lane@{curvatures_str}')
    else:
        raise ValueError(f'Curvature setting {curvature} not supported.')
    ego_lane_nodes = []
    for index, node in enumerate([v for u, v in graph.out_edges('ego') if v in ground_nodes]):
        ego_lane_nodes.append(node)
        cur_node = node
        # handle forward direction
        while cur_node is not None and len(graph.out_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
            potentials = [v for u, v, a in graph.out_edges(cur_node, data=True) if v in ground_nodes and (v not in ego_lane_nodes) and\
                           (not ('is_junction' in graph.nodes[v] and graph.nodes[v]['is_junction'])) and (not a['lane_change'])]
            if len(potentials) > 1:
                raise MultipleLaneChoicesException()
            if len(potentials) == 1:
                if index > 0:
                    raise EgoMultipleLaneException("Ego is in multiple lanes")
                cur_node = potentials[0]
                ego_lane_nodes.append(cur_node)
            else:
                cur_node = None
        # handle backward direction
        cur_node = node
        while cur_node is not None and len(graph.in_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
            potentials = [u for u, v, a in graph.in_edges(cur_node, data=True) if u in ground_nodes and (u not in ego_lane_nodes) and\
                           (not ('is_junction' in graph.nodes[u] and graph.nodes[u]['is_junction'])) and (not a['lane_change'])]
            if len(potentials) > 1:
                raise MultipleLaneChoicesException()
            if len(potentials) == 1:
                if index > 0:
                    raise EgoMultipleLaneException("Ego is in multiple lanes")
                cur_node = potentials[0]
                ego_lane_nodes.insert(0, cur_node)
            else:
                cur_node = None
    # check for other lanes
    lanes_with_ego = [(0, ego_lane_nodes)]
    offsets_found = [0]
    lane_map = {node: 0 for node in ego_lane_nodes}
    # these are lanes that travel in the same direction as ego and could (possibly through multiple steps) merge with ego
    # they are noted as (num_right, [nodes]) where num_right is the number of lanes you have to change to the right to get to that lane. negative for left
    explored = []  # since this starts empty, we will start by exploring (0, ego_lane_nodes)
    while len(explored) < len(lanes_with_ego):
        offset, working_lane = [(lane_offset, lane_nodes) for lane_offset, lane_nodes in lanes_with_ego if lane_offset not in explored][0]
        explored.append(offset)
        for node in working_lane:
            lane_changes = [v for u, v, a in graph.out_edges(node, data=True) if v in ground_nodes and v not in lane_map and 'lane_change' in a and a['lane_change'] and (not ('is_junction' in graph.nodes[v] and graph.nodes[v]['is_junction']))]
            for lane_change in lane_changes:
                # if this node is already part of any of the lanes we have already found, then we don't want to explore further
                if lane_change in lane_map:
                    continue
                new_lane = []
                new_lane.append(lane_change)
                cur_node = lane_change
                # handle forward direction
                while cur_node is not None and len(graph.out_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
                    potentials = [v for u, v, a in graph.out_edges(cur_node, data=True) if v in ground_nodes and (v not in lane_map) and\
                                (not ('is_junction' in graph.nodes[v] and graph.nodes[v]['is_junction'])) and (not a['lane_change'])]
                    if len(potentials) > 1:
                        raise MultipleLaneChoicesException()
                    if len(potentials) == 1:
                        cur_node = potentials[0]
                        new_lane.append(cur_node)
                    else:
                        cur_node = None
                # handle backward direction
                cur_node = lane_change
                while cur_node is not None and len(graph.in_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
                    potentials = [u for u, v, a in graph.in_edges(cur_node, data=True) if u in ground_nodes and (u not in lane_map) and\
                                (not ('is_junction' in graph.nodes[u] and graph.nodes[u]['is_junction'])) and (not a['lane_change'])]
                    if len(potentials) > 1:
                        raise MultipleLaneChoicesException()
                    if len(potentials) == 1:
                        cur_node = potentials[0]
                        new_lane.insert(0, cur_node)
                    else:
                        cur_node = None
                # first step is to determine if this lane change is right or left.
                # Sometimes there can be really tight curves or other oddities that mean using a single measurement for left/right is not good.
                # Instead, we average across the whole lane
                left_or_right = []
                angles = []
                for cur_lane_node in working_lane:
                    cur_vert = graph.nodes[cur_lane_node]['vertex']
                    next = [v for u, v, a in graph.out_edges(cur_lane_node, data=True) if v in ground_nodes and not ('lane_change' in a and a['lane_change'])]
                    if len(next) == 0:
                        continue
                    next_vert = graph.nodes[next[0]]['vertex']
                    possible_change_verts = [v for u, v in graph.out_edges(cur_lane_node) if v in new_lane]
                    if len(possible_change_verts) == 0:
                        continue
                    lane_change_vert = graph.nodes[possible_change_verts[0]]['vertex']
                    v1 = next_vert - cur_vert
                    v2 = lane_change_vert - cur_vert
                    angle1_to_2 = np.degrees(np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0]))
                    if angle1_to_2 < -180:
                        angle1_to_2 = 360 + angle1_to_2
                    if angle1_to_2 > 180:
                        angle1_to_2 = angle1_to_2 - 360
                    angles.append(angle1_to_2)
                    left_shift = angle1_to_2 < 0
                    left_or_right.append(-1 if left_shift else 1)
                vote = int(np.sign(np.mean(left_or_right)))
                if vote == 0:
                    left_shift = np.mean(angles) < 0
                    vote = -1 if left_shift else 1
                new_offset = offset + vote
                if new_offset in offsets_found:
                    raise ValueError(f'Found offset that has already been found. This should have been caught before this point. {new_offset} {offsets_found}')
                offsets_found.append(new_offset)
                lanes_with_ego.append((new_offset, new_lane))
                lane_map.update({n: new_offset for n in new_lane})
    new_graph.add_node(ego_lane)
    new_graph.add_edge(ego_lane, root_road, label='isIn')
    other_lanes_w_ego = {offset: Node(name=f"{'Right' if offset > 0 else 'Left'} {abs(offset)} of Ego Lane") for offset in offsets_found if offset is not 0}
    opposing_lane_map = {}
    for offset, other_lane in other_lanes_w_ego.items():
        new_graph.add_node(other_lane)
        new_graph.add_edge(other_lane, root_road, label='isIn')
    actor_counter = defaultdict(int)
    node_map = {}
    added_actors = []
    for actor in actors:
        edges = [(u,v,a) for (u,v,a) in graph.out_edges(actor, data=True) if v in ground_nodes]
        if len(edges) == 0:
            continue
        # create the actor node
        sem_class = graph.nodes[actor]['sem_class']
        if sem_class != 'ego':
            base_class = get_base_type(sem_class)
            if base_class is None:
                continue
            name = f'{base_class}_{actor_counter[base_class]}'
            actor_counter[base_class] += 1
        else:
            name = 'ego'
            base_class='ego'
        added_actors.append(actor)
        new_node = Node(name=name, base_class=base_class, attr=graph.nodes[actor])
        new_graph.add_node(new_node)
        node_map[actor] = new_node
        # now we need to find the lane that it is in, if we already have it
        for u, v, a in edges:
            if v not in ground_nodes:
                if valid_gradient(graph.nodes['ego']['vertex'], graph.nodes[v]['vertex']):
                    raise AssertionError("Found invalid edge when parsing the road graph for rsv")
                else:
                    continue  # the original graph generated added an edge that matches a change in the Z plane, skip
            if v in lane_map:
                lane = lane_map[v]
                target = other_lanes_w_ego[lane] if lane != 0 else ego_lane
            elif v in opposing_lane_map:
                target = opposing_lane_map[v]
            else:
                # need to calc other opposing lane
                if actor_counter["Opposing Lane"] == 4:
                    raise AssertionError("Found too many opposing lanes")
                name = f'Opposing Lane {actor_counter["Opposing Lane"]}'
                actor_counter["Opposing Lane"] += 1
                starting_node = v
                new_lane_node = Node(name=name)
                new_lane = []
                new_lane.append(v)
                cur_node = starting_node
                # handle forward direction
                while cur_node is not None and len(graph.out_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
                    potentials = [v for u, v, a in graph.out_edges(cur_node, data=True) if v in ground_nodes and (v not in opposing_lane_map) and\
                                (not ('is_junction' in graph.nodes[v] and graph.nodes[v]['is_junction'])) and (not a['lane_change'])]
                    if len(potentials) > 1:
                        raise MultipleLaneChoicesException()
                    if len(potentials) == 1:
                        cur_node = potentials[0]
                        new_lane.append(cur_node)
                    else:
                        cur_node = None
                # handle backward direction
                cur_node = starting_node
                while cur_node is not None and len(graph.in_edges(cur_node)) > 0 and not ('is_junction' in graph.nodes[cur_node] and graph.nodes[cur_node]['is_junction']):
                    potentials = [u for u, v, a in graph.in_edges(cur_node, data=True) if u in ground_nodes and (u not in opposing_lane_map) and\
                                (not ('is_junction' in graph.nodes[u] and graph.nodes[u]['is_junction'])) and (not a['lane_change'])]
                    if len(potentials) > 1:
                        raise MultipleLaneChoicesException()
                    if len(potentials) == 1:
                        cur_node = potentials[0]
                        new_lane.insert(0, cur_node)
                    else:
                        cur_node = None
                new_graph.add_node(new_lane_node)
                new_graph.add_edge(new_lane_node, root_road, label='isIn')
                opposing_lane_map.update({n: new_lane_node for n in new_lane})
                target = new_lane_node
            if not new_graph.has_edge(new_node, target):
                new_graph.add_edge(new_node, target, label='isIn')
    if 'ego' not in added_actors:
        raise EgoNotInLaneException
    # add edges between the actors
    if gen_relationships:
        for actor1, actor2 in itertools.permutations(added_actors, 2):
            vert1 = copy.deepcopy(graph.nodes[actor1]['vertex'])
            vert2 = copy.deepcopy(graph.nodes[actor2]['vertex'])
            vert1_forward = copy.deepcopy(vert1 + graph.nodes[actor1]['forward'])
            new_actor1 = node_map[actor1]
            new_actor2 = node_map[actor2]
            actor1_base = new_actor1.base_class
            actor2_base = new_actor2.base_class
            distance = np.linalg.norm(vert1 - vert2)
            added_lr = False

            vert1[0] *= -1
            vert2[0] *= -1
            vert1_forward[0] *= -1

            v2 = vert2 - vert1
            v1f = vert1_forward - vert1
            angle1_to_2 = np.degrees(np.arctan2(v2[1], v2[0]) - np.arctan2(v1f[1], v1f[0]))
            # RSV uses a different angle convention, convert
            if angle1_to_2 < 0:
                angle1_to_2 += 360
            angle_to_check = angle1_to_2 + 90
            if angle_to_check > 360:
                angle_to_check -= 360
            if angle_to_check < 0:
                angle_to_check += 360
            for class1, class2, dist in proximity_relations:
                if ((class1 == actor1_base and class2 == actor2_base) or \
                (class1 == actor2_base and class2 == actor1_base)) and \
                distance <= dist:
                    dist_enum = len(proximity_thresholds) - 1
                    if distance <= proximity_thresholds[dist_enum][1]:
                        while dist_enum > 0 and distance <= proximity_thresholds[dist_enum-1][1]:
                            dist_enum -= 1
                        new_graph.add_edge(new_actor1, new_actor2, label=proximity_thresholds[dist_enum][0])
            for class1, class2, dist in directional_relations:
                if ((class1 == actor1_base and class2 == actor2_base) or \
                (class1 == actor2_base and class2 == actor1_base)) and \
                distance <= dist:
                    if not added_lr:
                        added_lr = True
                        if angle1_to_2 <= 180:
                            # actor2 is to the left or actor1
                            new_graph.add_edge(new_actor2, new_actor1, label='toLeftOf')
                        else:
                            new_graph.add_edge(new_actor2, new_actor1, label='toRightOf')
                    try:
                        for dir_rel, list_of_ranges in directional_thresholds:
                            for ranges in list_of_ranges:
                                if ranges[0] <= angle_to_check <= ranges[1]:
                                    new_graph.add_edge(new_actor2, new_actor1, label=dir_rel)
                                    raise StopIteration
                    except StopIteration:
                        pass
        if filter_if_no_relation:
            to_remove = []
            ego_node = node_map['ego']
            for actor in added_actors:
                if actor == 'ego':
                    continue
                new_actor = node_map[actor]
                if not (new_graph.has_edge(new_actor, ego_node) or new_graph.has_edge('ego', ego_node)):
                    to_remove.append(new_actor)
            for actor in to_remove:
                new_graph.remove_node(actor)
            to_remove = []
            for u, v in new_graph.in_edges(root_road):
                if new_graph.in_degree(u) == 0:
                    to_remove.append(u)
            for u in to_remove:
                new_graph.remove_node(u)
    if not gen_lanes:
        to_remove = []
        # remove the lanes from the graph
        for node in new_graph:
            if 'Lane' in node.name or 'Road' in node.name:
                to_remove.append(node)
        for node in to_remove:
            new_graph.remove_node(node)
    # reorganize all of the opposing lanes for consistency
    opposing_lane_nodes = [node for node in new_graph if 'Opposing Lane' in node.name]
    reverse_opp_lane_map = {node:[orig for orig, new_lane in opposing_lane_map.items() if new_lane == node] for node in opposing_lane_nodes}
    opp_lane_min_dists = {node: min([np.linalg.norm(graph.nodes['ego']['vertex'] - graph.nodes[road_segment]['vertex']) for road_segment in road_segments]) for node, road_segments in reverse_opp_lane_map.items()}
    sorted_opp_lanes = sorted(opposing_lane_nodes, key=lambda x: (opp_lane_min_dists[x], x.name))
    for index, node in enumerate(sorted_opp_lanes):
        node.name = f'Opposing Lane {index}'
    return new_graph

def entities(graph, abstract_graph=None, description=None):
    return process_to_rsv(graph,
                            filter_if_no_relation=False,
                            abstract=abstract_graph,
                            description=description,
                            gen_relationships=False)
    
def semgraph(graph, description=None):
    return process_to_rsv(graph,
                            filter_if_no_relation=False,
                            curvature=Curvature.NONE,
                            description=description,
                            gen_relationships=False,
                            gen_lanes=False)

def semgraphrel(graph, description=None):
    return process_to_rsv(graph,
                            filter_if_no_relation=False,
                            curvature=Curvature.NONE,
                            description=description,
                            gen_relationships=True,
                            gen_lanes=False)
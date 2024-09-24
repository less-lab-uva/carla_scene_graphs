from functools import lru_cache, partial
from multiprocessing import Pool
import multiprocessing
import carla
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from timeit import default_timer as timer 
import networkx.drawing.nx_pydot as nx_pydot
import networkx as nx


def plot_graph(graph: nx.Graph, file_path: str):
    # Convert the NetworkX graph to a Pydot graph
    pydot_graph = nx_pydot.to_pydot(graph)

    # Save the Pydot graph to a file
    pydot_graph.write_png(file_path)


def get_waypoint_location(waypoint):
    return get_vert_from_location(waypoint.transform.location)


def get_vert_from_location(location):
    return np.array([location.x, location.y, location.z])


def hashable_location(vert, waypoint_precision, inflation=1000.0):
    waypoint_precision /= inflation
    expanded = np.round(vert / waypoint_precision)
    return f'x: {expanded[0]}, y: {expanded[1]}, z: {expanded[2]}'


def get_node_distance(graph, node_id1, node_id2, plane_projection=True):
    vert2 = graph.nodes[node_id2]['vertex']
    return get_distance(graph, node_id1, vert2, plane_projection=plane_projection)



def get_distance(graph, node, location, plane_projection=True):
    vert1 = graph.nodes[node]['vertex']
    vert2 = location
    if plane_projection:
        vert1 = vert1[0:2]
        vert2 = vert2[0:2]
    return np.linalg.norm(vert1 - vert2)  # calc euclidean distance


def vector_cosine(vect1, vect2):
    return np.dot(vect1, vect2) / (np.linalg.norm(vect1) * np.linalg.norm(vect2))


def get_average_vertex(graph, nodes):
    vertices = np.array([graph.nodes[node]['vertex'] for node in nodes])
    avg = np.mean(vertices, axis=0)
    return avg


def is_left(start, end, middle):
    # adapted from https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
    return (end[0] - start[0])*(middle[1] - start[1]) - \
           (end[1] - start[1])*(middle[0] - start[0]) > 0


def get_rotation_pointers(rotation):
    vectors = [rotation.get_forward_vector(), rotation.get_right_vector(), rotation.get_up_vector()]
    to_return = []
    for vector in vectors:
        to_return.append(np.array([vector.x, vector.y, vector.z]))
    return to_return[0], to_return[1], to_return[2]


def carla_bb_points_to_matplotlib(bb_points):
    points = [
        (bb_points[i].x, bb_points[i].y) for i in [0, 2, 6, 4]
    ]
    return points if not np.any(np.isnan(points)) else None


# fraction of 1 - the max the US generally allows is 0.06, steepest in the world is 0.348
# https://en.wikipedia.org/wiki/Grade_(slope)
# We use 0.1 because empirically this was acceptable for use in CARLA towns
__MAX_ROAD_GRADE = 0.1
__MAX_Z_THRESH = 2
def valid_gradient(ego_loc, other_loc):
    z_dist = abs(ego_loc[-1] - other_loc[-1])
    xy_dist = np.linalg.norm(ego_loc[0:2] - other_loc[0:2])
    if np.isclose(xy_dist, 0):  # don't divide by 0
        if z_dist < __MAX_Z_THRESH:
            return True  # comparing ego to itself would return False otherwise for div by 0
        else:
            return False
    road_grade = z_dist / xy_dist
    return road_grade < __MAX_ROAD_GRADE or z_dist < __MAX_Z_THRESH


def filter_by_valid_gradient(graph, nodes, ego_vertex, sem_classes):
    return [node for node in nodes if (graph.nodes[node]['sem_class'] not in sem_classes) or \
                (graph.nodes[node]['sem_class']  in sem_classes and \
                    valid_gradient(ego_vertex, graph.nodes[node]['vertex']))]



def carla_bb_to_matplotlib(carla_bb):
    local_points = carla_bb.get_local_vertices()
    return carla_bb_points_to_matplotlib(local_points)


def vertex_to_point(vertex):
    return Point(vertex[0], vertex[1])


def get_nodes_in_polygon(graph, nodes, polygon):
    return [node for node in nodes if polygon.contains(graph.nodes[node]['vertex_point'])]

def get_bbox_polygon(graph, node):
    if not 'bbox_polygon' in graph.graph:
        graph.graph['bbox_polygon'] = {}
    if node not in graph.graph['bbox_polygon']:
        graph.graph['bbox_polygon'][node] = Polygon(graph.graph['bbox'][node])
    return graph.graph['bbox_polygon'][node]


def get_bbox_intersects_poly(graph, bbox, node):
    poly = get_bbox_polygon(graph, node)
    if poly is None:
        return False
    return bbox.intersects(poly)
    

def intersects_func(polygon, box):
    return polygon.intersects(box)

def get_intersecting_nodes(graph, nodes, polygon, threaded=False):
    if threaded:
        # informal experiments showed that non-threaded was faster, but there may be cases this is useful 
        with multiprocessing.Pool() as pool:
            return [node for node, keep in zip(nodes, pool.map(partial(intersects_func, polygon), [graph.nodes[node]['waypoint_bbox_polygon'] for node in nodes])) if keep]
    return [node for node in nodes if polygon.intersects(graph.nodes[node]['waypoint_bbox_polygon'])]


def contains_func(point, bbox):
    return bbox.contains(point)


def get_containing_nodes(graph, nodes, point, threaded=False):
    if threaded:
        # informal experiments showed that non-threaded was faster, but there may be cases this is useful 
        with multiprocessing.Pool() as pool:
            return [node for node, keep in zip(nodes, pool.map(partial(contains_func, point), [graph.nodes[node]['waypoint_bbox_polygon'] for node in nodes])) if keep]
    return [node for node in nodes if graph.nodes[node]['waypoint_bbox_polygon'].contains(point)]

def filter_semantics_func(sem_class, node_sem_class):
    return node_sem_class == sem_class or (hasattr(sem_class, '__iter__') and node_sem_class in sem_class)

def filter_semantics(graph, nodes, sem_class=None, threaded=False):
    if sem_class is None:
        return nodes
    if threaded:
        # informal experiments showed that non-threaded was faster, but there may be cases this is useful 
        with multiprocessing.Pool() as pool:
            return [node for node, keep in zip(nodes, pool.map(partial(filter_semantics_func, sem_class), [graph.nodes[node]['sem_class'] for node in nodes])) if keep]
    return [node for node in nodes if sem_class is None or graph.nodes[node]['sem_class'] == sem_class or (hasattr(sem_class, '__iter__') and graph.nodes[node]['sem_class'] in sem_class)]


def add_sidewalk_edge(graph, u, v):
    graph.add_edge(u, v,
                    distance=get_node_distance(graph, u, v),
                    lane_change=False,
                    sem_class='sidewalk')
    

def get_location_from_vert(vert):
    return carla.Location(*vert)


def get_actor_transform(world_snapshot, actor):
    # for some reason, traffic lights have string ids
    temp_actor = world_snapshot.find(actor.id if type(actor.id) == int else int(actor.id)) if world_snapshot is not None else actor
    if temp_actor is None:
        return None
    return temp_actor.get_transform()


def get_actor_bb_matplotlib(actor, actor_transform):
    if actor_transform is None:
        return None
    return carla_bb_points_to_matplotlib(actor.bounding_box.get_world_vertices(actor_transform))


def numpy3dbb(locs):
    return np.array([[loc.x, loc.y, loc.z] for loc in locs])

def parse3dbb(actor, actor_transform):
    if actor_transform is None:
        return None
    return numpy3dbb(actor.bounding_box.get_world_vertices(actor_transform))


def vertex_to_point(vertex):
    return Point(vertex[0], vertex[1])


class LRUDict(dict):
    def __init__(self, max_size):
        if max_size <= 0:
            raise ValueError("max_size must be positive")
        self._keys = []
        self._max_size = max_size

    @property
    def max_size(self):
        return self._max_size

    def __set_item__(self, key, value):
        if key in self._keys:
            # if it was in the list, move it to the front
            self._keys.remove(key)
        else:
            if len(self._keys) == self._max_size:
                # evict
                last_key = self._keys[-1]
                self._keys.remove(last_key)
                super().__delitem__(last_key)
        self._keys.insert(0, key)
        super().__setitem__(key, value)


def get_curvature_from_node_name(node):
    name = node.name
    return int(name[name.find('@')+1:name.rfind('_')])

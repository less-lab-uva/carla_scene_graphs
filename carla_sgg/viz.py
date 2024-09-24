import matplotlib
import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing import nx_pydot

def get_road_label(graph, node):
    ret = ''
    ret += '' if 'waypoint_road_id' not in graph.nodes[node] else f"{graph.nodes[node]['waypoint_road_id']}"
    ret += '' if 'waypoint_lane_id' not in graph.nodes[node] else f", {graph.nodes[node]['waypoint_lane_id']}"
    return ret


def draw_graph(graph, title=None, colors=None, show=False, anim=False, edgecolors='k', road_labels=False):
    graph = nx.induced_subgraph(graph, [node for node in graph.nodes if 'landmark' not in graph.nodes[node]['sem_class']])
    if 'node_size' in graph.graph:
        node_size = [graph.graph['node_size'][node] for node in graph.nodes]
    else:
        node_size = 20
    if not anim:
        plt.figure()
    pos = {node: (xyz[0], xyz[1]) for node, xyz in nx.get_node_attributes(graph, 'vertex').items() if node in graph.nodes}
    if colors is not None or 'node_colors' in graph.graph:
        if colors is None and graph.graph['node_colors'] is not None:
            colors = ['yellow' if 'junction_boundary' in graph.nodes[node]['sem_class'] else graph.graph['node_colors'][node] for node in graph.nodes]
        nx.draw(graph, pos, node_color=colors, node_size=node_size, edgecolors=edgecolors)
    else:
        nx.draw(graph, pos, node_size=node_size, edgecolors=edgecolors)
    # if 'bbox' in graph.graph:
    #     print('drawing bbox')
    #     # have_done_driving = False
    #     for iter, (node, bbox) in enumerate(graph.graph['bbox'].items()):
    #         if node not in graph.nodes:
    #             continue
    #         # if graph.nodes[node]['sem_class'] == 'Driving':
    #         #     if have_done_driving:
    #         #         continue
    #         #     else:
    #         #         have_done_driving = True
    #         # edgecolor = f'C{iter%10}'
            
    #         plt.gca().add_patch(matplotlib.patches.Polygon(bbox,
    #                                                           edgecolor=f'C{iter%10}',
    #                                                           fill=False,
    #                                                           linestyle='--',
    #                                                           hatch='/'))
    if road_labels:
        nx.draw_networkx_labels(graph, pos, {node: get_road_label(graph, node) for node, _ in nx.get_node_attributes(graph, 'vertex').items()})
    if title is not None:
        print('saving fig')
        plt.savefig(f"{title}.png", format="PNG", dpi=1000)
    if show:
        print('showing fig')
        plt.show(block=not anim)
    if not anim:
        plt.close()


def export_to_jpg(graph, file):
    if not ('jpg' in file or 'jpeg' in file):
        raise ValueError("Method only supports saving as jpg or jpeg.")
    A = nx_pydot.to_pydot(graph)
    A.write_jpg(file)
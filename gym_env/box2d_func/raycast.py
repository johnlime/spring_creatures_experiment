from Box2D import *
import numpy as np
from copy import *

def raycast(origin_fixture, origin_position, raycast_relative_angle, raycast_distance):
    """
    Find out the edge that the knob is located on
    """
    # get normal of the knob position on base body
    # get edge that the knob is located on -> manually calculate normal...
    vertices = origin_fixture.shape.vertices
    total_vertices = len(vertices)
    detected_edge = {
        "vertex 0": None,
        "vertex 1": None,
        "horizontal": False,
        "slope": None,
    }
    for i, vertex in enumerate(vertices):
        prev_vertex = vertices[(i - 1) % total_vertices]
        detected_edge["vertex 0"] = prev_vertex
        detected_edge["vertex 1"] = vertex

        vertical = (vertex[0] - prev_vertex[0]) == 0
        detected_edge["horizontal"] = (vertex[1] - prev_vertex[1]) == 0

        border_condition = lambda x, y: \
            x >= np.sort([vertex[0], prev_vertex[0]])[0] and \
            x <= np.sort([vertex[0], prev_vertex[0]])[1] and \
            y >= np.sort([vertex[1], prev_vertex[1]])[0] and \
            y <= np.sort([vertex[1], prev_vertex[1]])[1]

        if vertical:
            if border_condition(origin_position[0], origin_position[1]):
                break # edge is on the vertical edge
            pass

        else:
            slope = (vertex[1] - prev_vertex[1]) / (vertex[0] - prev_vertex[0])
            detected_edge["slope"] = slope
            # edge function with boundary conditions taken into account
            on_edge = lambda x, y: slope * (x - vertex[0]) - (y - vertex[1]) == 0
            assert on_edge(prev_vertex[0], prev_vertex[1])

            if on_edge(origin_position[0], origin_position[1]):
                break

    """
    Calculate the normal vector and the direction of the joint
    """
    # get the normal vector
    normal_vector = np.empty(2)
    if detected_edge["horizontal"]:
        normal_vector[0], normal_vector[1] = 0, 1
    elif detected_edge["slope"] == 0:
        if origin_position[0] - origin_fixture.body.position[0]:
            normal_vector[0], normal_vector[1] = 1, 0
        else:
            normal_vector[0], normal_vector[1] = -1, 0
    else:
        normal_vector = np.array([1, - 1 / detected_edge["slope"]])
        normal_vector_norm = np.linalg.norm(normal_vector, 2)
        normal_vector = normal_vector / normal_vector_norm

    assert -np.pi/2 < raycast_relative_angle and raycast_relative_angle < np.pi/2

    directional_angle = np.arccos(normal_vector[0]) - raycast_relative_angle
    directional_vector = np.array([np.cos(directional_angle), np.sin(directional_angle)])

    # raycast from knob to a set angle
    input = b2RayCastInput(p1 = origin_position,
                           p2 = np.array(origin_position) + directional_vector,
                           maxFraction = raycast_distance)
    return input

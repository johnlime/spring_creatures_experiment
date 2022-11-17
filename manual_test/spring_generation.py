from Box2D import *
from Box2D.examples.framework import (Framework, Keys, main)

from math import sin, pi, sqrt
import numpy as np
from copy import copy, deepcopy

from settings import *

class SpringGeneration (Framework):
    name = "SpringGeneration"
    description = 'g to stop/go'
    count = 800

    def __init__(self):
        Framework.__init__(self)
        self.world.gravity = (0, 0)

        pos_x, pos_y = 0, 0
        dim_x, dim_y = 5, 5
        assert dim_x > 0 and dim_y > 0

        limb_base = self.world.CreateDynamicBody(
            position = (pos_x, pos_y),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2PolygonShape(box = (dim_x, dim_y)),
                                    ),
        )

        # create base body
        # limb_base = self.world.CreateStaticBody(
        #     position = (pos_x, pos_y),
        #     # (5, 5) in all directions
        #     # dimension is (10, 10)
        #     shapes = b2PolygonShape(box = (dim_x, dim_y))
        # )

        # create external body (for collision test)
        # limb_extension = self.world.CreateDynamicBody(
        #     position = (20, 20),
        #     fixtures = b2FixtureDef(density = 2.0,
        #                             friction = 0.6,
        #                             shape = b2PolygonShape(
        #                                 box = (5, 5)
        #                                 ),
        #                             ),
        # )

        self.generate_joint(limb_base, 0.99, 1)
        self.generate_joint(limb_base, 0.10, 1)

        """
        Body Aggregation (After body generation, to save processing time)
        """
        Framework.Step(self, default_settings)      # collision or overlap are only detected simulation step
        for i, contact in enumerate(self.world.contacts):
            worldManifold = b2WorldManifold()
            worldManifold.Initialize(contact.manifold,
                                     contact.fixtureA.body.transform,
                                     contact.fixtureA.shape.radius,
                                     contact.fixtureB.body.transform,
                                     contact.fixtureB.shape.radius)
            points = [worldManifold.points[i] for i in range(contact.manifold.pointCount)]
            for point in points:
                self.world.CreateRevoluteJoint(
                    bodyA = contact.fixtureA.body,
                    bodyB = contact.fixtureB.body,
                    anchor = point,
                    lowerAngle = 0,
                    upperAngle = 0,
                    enableLimit = True
                )
            print(points)

        self.go = False
        self.time = 0.0

    def generate_joint(self, base_body,
            # joint extension...
            knob_x_ratio = 1, knob_y_ratio = 1,     # joint node location
            joint_angle = np.pi/4,                  # angle of the prismatic joint
            joint_set_distance = 20,                # how far the prismatic joint goes
            #
            # ifelse shaperaycast hit detection lower than extension
            # just try it out with rigid bodies possibly colliding
            #
            # body extension...
            ext_pos_x = 20, ext_pos_y = 20,                     # (new body's position) we don't need this past testing
            ext_dim_x = 5, ext_dim_y = 5,                       # new body's dimensions
            ext_knob_x_ratio = -1, ext_knob_y_ratio = -1,       # new body's revolute node location
            ext_angle = 0,                                      # new body's rotation
            #
            # prismatic-distance joint settings
            prismatic_translation_low = -5, prismatic_translation_high = 5,
            ):

        """
        Generate Knob for Prismatic-Distance Joint
        """
        # otherwise, the knob would end up inside the base body
        assert \
        (-1 < knob_x_ratio and knob_x_ratio < 1 \
            and abs(knob_y_ratio) == 1) or \
        (-1 < knob_y_ratio and knob_y_ratio < 1 \
            and abs(knob_x_ratio) == 1)

        # create revolute joint on base body
        dim_x = abs(base_body.fixtures[0].shape.vertices[0][0])
        dim_y = abs(base_body.fixtures[0].shape.vertices[0][1])

        knob = self.world.CreateDynamicBody(
            position = (base_body.position[0] + dim_x * knob_x_ratio,
                        base_body.position[1] + dim_y * knob_y_ratio),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2CircleShape(pos=(0, 0), radius=0.5),
                                    ),
        )

        revolute_joint = self.world.CreateRevoluteJoint(
            bodyA = base_body,
            bodyB = knob,
            anchor = knob.worldCenter,
            lowerAngle = -0.5 ** b2_pi,
            upperAngle = 0.5 ** b2_pi,
            enableLimit = True
        )

        """
        Find out the edge that the knob is located on
        """
        # get normal of the knob position on base body
        # get edge that the knob is located on -> manually calculate normal...
        vertices = base_body.fixtures[0].shape.vertices
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
                if border_condition(knob.position[0], knob.position[1]):
                    break # edge is on the vertical edge
                pass

            else:
                slope = (vertex[1] - prev_vertex[1]) / (vertex[0] - prev_vertex[0])
                detected_edge["slope"] = slope
                # edge function with boundary conditions taken into account
                on_edge = lambda x, y: slope * (x - vertex[0]) - (y - vertex[1]) == 0
                assert on_edge(prev_vertex[0], prev_vertex[1])

                if on_edge(knob.position[0], knob.position[1]):
                    break

        """
        Calculate the normal vector and the direction of the joint
        """
        # get the normal vector
        normal_vector = np.empty(2)
        if detected_edge["horizontal"]:
            normal_vector[0], normal_vector[1] = 0, 1
        else:
            normal_vector = np.array([1, - 1 / detected_edge["slope"]])
            normal_vector_norm = np.linalg.norm(normal_vector, 2)
            normal_vector = normal_vector / normal_vector_norm

        assert -np.pi/2 < joint_angle and joint_angle < np.pi/2

        directional_angle = np.arccos(normal_vector[0]) - joint_angle
        directional_vector = np.array([np.cos(directional_angle), np.sin(directional_angle)])

        # raycast from knob to a set angle
        input = b2RayCastInput(p1 = knob.position,
                               p2 = np.array(knob.position) + directional_vector,
                               maxFraction = joint_set_distance)
        output = b2RayCastOutput()

        """
        Body Extension Generation/Detection -> Joint Generation
        """
        limb_extension = None
        spring_joint_anchor = None
        hit_once = False
        min_hit_distance = copy(joint_set_distance)
        for tmp_body in self.world.bodies:
            #####
            # Detect any collision between the ray and each body
            #####
            try:
                hit = tmp_body.fixtures[0].RayCast(output, input, 0)
                if hit:
                    hit_once = True
                    hit_distance = output.fraction * (input.p2 - input.p1)
                    hit_distance = sqrt(hit_distance[0] ** 2 + hit_distance[1] ** 2)
                    if hit_distance < min_hit_distance:
                        print("lower_hit_distance")
                        min_hit_distance = hit_distance
                        limb_extension = tmp_body
                        spring_joint_anchor = input.p1 + output.fraction * (input.p2 - input.p1)
                        print(tmp_body)           # debugging
                        print(hit_point, min_hit_distance)
            except:
                pass

        if not hit_once:
            #####
            # Generate Body Extension and Connect Spring
            #####
            limb_extension = self.world.CreateDynamicBody(
                position = (ext_pos_x, ext_pos_y),
                fixtures = b2FixtureDef(density = 2.0,
                                        friction = 0.6,
                                        shape = b2PolygonShape(
                                            box = (ext_dim_x, ext_dim_y)
                                            ),
                                        ),
            )
            spring_joint_anchor = (limb_extension.worldCenter[0]
                                       + ext_dim_x * ext_knob_x_ratio,
                                   limb_extension.worldCenter[1]
                                       + ext_dim_y * ext_knob_y_ratio)

        assert limb_extension != None and spring_joint_anchor != None

        prismatic_joint = self.world.CreatePrismaticJoint(
            bodyA = knob,
            bodyB = limb_extension,
            anchor = spring_joint_anchor,
            axis = (spring_joint_anchor[0] - knob.worldCenter[0],
                    spring_joint_anchor[1] - knob.worldCenter[1]),
            lowerTranslation = prismatic_translation_low,
            upperTranslation = prismatic_translation_high,
            enableLimit = True,
            #motorForce = 1.0, #(Doesn't work)
            motorSpeed = 0.0,
            enableMotor = True,
        )

        distance_joint = self.world.CreateDistanceJoint(
            bodyA = knob,
            bodyB = limb_extension,
            anchorA = knob.worldCenter,
            anchorB = spring_joint_anchor,
            # oscillations per second (for evey oscillator with 0 damping ratio)
            frequencyHz = 2.0,
            dampingRatio = 0.1,
            collideConnected = True
        )

    def Keyboard(self, key):
        if key == Keys.K_g:
            self.go = not self.go

    def Step(self, settings = default_settings):
        Framework.Step(self, settings)
        if self.go and settings.hz > 0.0:
            self.time += 1.0 / settings.hz

        print(len(self.world.contacts))

        renderer = self.renderer
        renderer.DrawPoint(renderer.to_screen((0, 0)),
                           4,
                           b2Color(0.9, 0.9, 0.9)
                           )

if __name__ == "__main__":
    main(SpringGeneration)

from Box2D import *
import numpy as np
from math import sin, cos, pi

from gym_env.box2d_func.generate_joint import *

TIMESTEP = 1.0 / 60
VEL_ITERS = 6
POS_ITERS = 2
BODY_LIMIT = 20

def spring_creature_generation(box2d_world, dim_x, dim_y,
                               morphogen_function = None):
    """
    Generate spring creature limbs recursively
    """
    limb_base = box2d_world.CreateDynamicBody(
        position = (0, 0),
        fixtures = b2FixtureDef(density = 2.0,
                                friction = 0.6,
                                shape = b2PolygonShape(box = (dim_x, dim_y)),
                                ),
    )
    bodies_to_scan = [limb_base]
    body_count = 1

    if morphogen_function == None:
        generate_joint(box2d_world, limb_base, 0.90, 1)
        generate_joint(box2d_world, limb_base, -0.90, 1)

        generate_joint(box2d_world,
                       generate_joint(box2d_world,
                                      limb_base,
                                      0.80,
                                      1)[1],
                       0.50,
                       1)

    else:
        """
        Recursively generate positions for morphogen functions (CPPN) to generate limbs from
        """
        while len(bodies_to_scan) != 0 or body_count < BODY_LIMIT:
            if len(bodies_to_scan) == 0 or body_count >= BODY_LIMIT:
                break
            body = bodies_to_scan[0]
            knob_x_ratio = 0
            knob_y_ratio = 0
            for x in range(1, 10):
                knob_x_ratio = 0.1 * x
                genome = morphogen_function([
                    body.position[0] + body.fixtures[0].shape.vertices[0][0] * knob_x_ratio,
                    body.position[1] + body.fixtures[0].shape.vertices[0][1] * 1
                    ])
                #print(genome)
                if genome[0]: # should dictate whether to generate genome or not
                    new_limb, limb_reference = generate_joint_from_genome(box2d_world,
                                                                          body,
                                                                          knob_x_ratio,
                                                                          1,
                                                                          genome)
                    if new_limb:
                        bodies_to_scan.append(limb_reference)
                        body_count += 1

            for y in range(1, 10):
                knob_y_ratio = 0.1 * y
                genome = morphogen_function([
                    body.position[0] + body.fixtures[0].shape.vertices[0][0] * 1,
                    body.position[1] + body.fixtures[0].shape.vertices[0][1] * knob_y_ratio
                    ])
                if genome[0]: # should dictate whether to generate genome or not
                    new_limb, limb_reference = generate_joint_from_genome(box2d_world,
                                                                          body,
                                                                          1,
                                                                          knob_y_ratio,
                                                                          genome)
                    if new_limb:
                        bodies_to_scan.append(limb_reference)
                        body_count += 1
            bodies_to_scan.pop(0)

    """
    Body Aggregation (After body generation, to save processing time)
    """
    box2d_world.Step(TIMESTEP, VEL_ITERS, POS_ITERS) # collision or overlap are only detected during simulation step
    for i, contact in enumerate(box2d_world.contacts):
        if type(contact.fixtureA.shape) == b2PolygonShape and \
            type(contact.fixtureB.shape) == b2PolygonShape:
            worldManifold = b2WorldManifold()
            worldManifold.Initialize(contact.manifold,
                                     contact.fixtureA.body.transform,
                                     contact.fixtureA.shape.radius,
                                     contact.fixtureB.body.transform,
                                     contact.fixtureB.shape.radius)
            points = [worldManifold.points[i] for i in range(contact.manifold.pointCount)]
            for point in points:
                box2d_world.CreateRevoluteJoint(
                    bodyA = contact.fixtureA.body,
                    bodyB = contact.fixtureB.body,
                    anchor = point,
                    lowerAngle = 0,
                    upperAngle = 0,
                    enableLimit = True
                )
            # print(points)

    for body in box2d_world.bodies:
        body.linearDamping = 0.1
        body.angularDamping = 0.1

    return limb_base
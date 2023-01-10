from Box2D import *
import numpy as np
from copy import *
from math import sqrt

from gym_env.box2d_func.raycast import *

def generate_joint_from_material(box2d_world, base_body, knob_x_ratio, knob_y_ratio, material):

    """
    Materials
    0: Whether to generate the limb or not
    1: Joint angle [- np.pi, np.pi]
    2: Joint distance
    3, 4: New body's dimensions > 0
    """
    if material[1] > np.pi:
        material[1] = np.pi
    elif material[1] < -np.pi:
        material[1] = -np.pi

    material[3] = abs(material[3])
    if material[3] == 0:
        return False, None
    material[4] = abs(material[4])
    if material[4] == 0:
        return False, None

    return generate_joint(box2d_world, base_body,
                          knob_x_ratio, knob_y_ratio,
                          joint_angle = material[1],
                          joint_set_distance = material[2],
                          ext_dim_x = material[3],
                          ext_dim_y = material[4],
                          prismatic_translation_high = material[2]
                          )

def generate_joint(box2d_world, base_body,
        # joint extension...
        knob_x_ratio = 1, knob_y_ratio = 1,     # joint node location
        joint_angle = np.pi/4,                  # angle of the prismatic joint
        joint_set_distance = 20,                # how far the prismatic joint goes
        # body extension...
        ext_dim_x = 5, ext_dim_y = 5,           # new body's dimensions
        ext_angle = 0,                          # new body's rotation
        # prismatic-distance joint settings
        prismatic_translation_low = -np.inf, prismatic_translation_high = 20,
        ):
    # print(b2_epsilon)
    # print((ext_dim_x) * (ext_dim_y))
    # does not generate joints
    assert ext_dim_x > 0 and ext_dim_y > 0
    if (ext_dim_x) * (ext_dim_y) > 10**(2):
        return False, None
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
    # the shape of the fixture is relative to the origin
    dim_x = abs(base_body.fixtures[0].shape.vertices[0][0])
    dim_y = abs(base_body.fixtures[0].shape.vertices[0][1])

    knob = box2d_world.CreateDynamicBody(
        position = (base_body.position[0] + dim_x * knob_x_ratio,
                    base_body.position[1] + dim_y * knob_y_ratio),
        fixtures = b2FixtureDef(density = 2.0,
                                friction = 0.6,
                                shape = b2CircleShape(pos=(0, 0), radius=0.5),
                                ),
    )

    revolute_joint = box2d_world.CreateRevoluteJoint(
        bodyA = base_body,
        bodyB = knob,
        anchor = knob.worldCenter,
        lowerAngle = -0.5 ** b2_pi,
        upperAngle = 0.5 ** b2_pi,
        enableLimit = True
    )

    input = raycast(origin_fixture = base_body.fixtures[0],
                    origin_position = knob.position,
                    raycast_relative_angle = joint_angle,
                    raycast_distance = joint_set_distance)

    """
    Body Extension Generation/Detection -> Joint Generation
    """
    limb_extension = None
    spring_joint_anchor = None
    hit_once = False
    min_hit_distance = copy(joint_set_distance)
    new_limb_boolean = True
    for tmp_body in box2d_world.bodies:
        #####
        # Detect any collision between the ray and each body
        # UNLESS it is a circle (joint)
        #####
        try:
            output = b2RayCastOutput()
            if type(tmp_body.fixtures[0].shape) is not b2CircleShape:
                    hit = tmp_body.fixtures[0].RayCast(output, input, 0)
                    if hit:
                        #hit_once = True
                        hit_distance = output.fraction * (input.p2 - input.p1)
                        hit_distance = sqrt(hit_distance[0] ** 2 + hit_distance[1] ** 2)
                        if hit_distance < min_hit_distance:
                            hit_once = True
                            # print("lower_hit_distance")
                            min_hit_distance = deepcopy(hit_distance)
                            limb_extension = copy(tmp_body)
                            spring_joint_anchor = input.p1 + output.fraction * (input.p2 - input.p1)
                            new_limb = False
                            # if limb_extension == None:
                            #     print("reference is nan", tmp_body)
                        # if limb_extension == None:
                        #     print(hit_distance)
                        #     print(min_hit_distance)
        except:
            raise


    if not hit_once:
        #####
        # Generate Body Extension and Connect Spring
        #####
        limb_extension_position = input.p1 + (input.p2 - input.p1) * joint_set_distance
        limb_extension = box2d_world.CreateDynamicBody(
            position = (limb_extension_position[0],
                        limb_extension_position[1]),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2PolygonShape(
                                        box = (ext_dim_x, ext_dim_y)
                                        ),
                                    ),
        )
        # cast ray to determine joint endpoint
        output = b2RayCastOutput()
        limb_extension.fixtures[0].RayCast(output, input, 0)
        spring_joint_anchor = input.p1 + output.fraction * (input.p2 - input.p1)

    if limb_extension == None or spring_joint_anchor == None:
        print(hit_once, limb_extension, spring_joint_anchor)

    assert limb_extension != None and spring_joint_anchor != None


    prismatic_joint = box2d_world.CreatePrismaticJoint(
        bodyA = knob,
        bodyB = limb_extension,
        anchor = spring_joint_anchor,
        axis = (spring_joint_anchor[0] - knob.worldCenter[0],
                spring_joint_anchor[1] - knob.worldCenter[1]),
        lowerTranslation = prismatic_translation_low,
        upperTranslation = prismatic_translation_high,
        enableLimit = True,
        #motorForce = 1.0, #(Doesn't work)
        maxMotorForce = 10 ** 4,
        motorSpeed = 0.0,
        enableMotor = True,
    )

    distance_joint = box2d_world.CreateDistanceJoint(
        bodyA = knob,
        bodyB = limb_extension,
        anchorA = knob.worldCenter,
        anchorB = spring_joint_anchor,
        # oscillations per second (for evey oscillator with 0 damping ratio)
        frequencyHz = 2.0,
        dampingRatio = 0.1,
        collideConnected = True
    )

    return new_limb_boolean, limb_extension

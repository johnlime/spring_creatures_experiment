from Box2D import *
from Box2D.examples.framework import (Framework, Keys, main)

from math import sin, pi, sqrt

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

        # limb_base = self.world.CreateDynamicBody(
        #     position = (pos_x, pos_y),
        #     fixtures = b2FixtureDef(density = 2.0,
        #                             friction = 0.6,
        #                             shape = b2PolygonShape(box = (dim_x, dim_y)),
        #                             ),
        # )

        # create base body
        limb_base = self.world.CreateStaticBody(
            position = (pos_x, pos_y),
            # (5, 5) in all directions
            # dimension is (10, 10)
            shapes = b2PolygonShape(box = (dim_x, dim_y))
        )

        self.generate_joint(limb_base)


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
        assert \
        (-1 < knob_x_ratio and knob_x_ratio < 1 \
            and abs(knob_y_ratio) = 1) or \     # otherwise, the knob would end up inside the base body
        (-1 < knob_x_ratio and knob_x_ratio < 1 \
            and abs(knob_x_ratio) = 1)

        # create revolute joint on base body
        knob = self.world.CreateDynamicBody(
            position = (pos_x + dim_x * knob_x_ratio,
                        pos_y + dim_y * knob_y_ratio),
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
        Raycast to see where the prismatic joint would lead to
        """
        ...

        """
        Generate Body Extension and Connect Spring
        """
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

        limb_extension = self.world.CreateDynamicBody(
            position = (pos_x + ext_pos_x, pos_y + ext_pos_y),
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

    def Keyboard(self, key):
        if key == Keys.K_g:
            self.go = not self.go

    def Step(self, settings):
        Framework.Step(self, settings)

        if self.go and settings.hz > 0.0:
            self.time += 1.0 / settings.hz

        renderer = self.renderer
        renderer.DrawPoint(renderer.to_screen((0, 0)),
                           4,
                           b2Color(0.9, 0.9, 0.9)
                           )

if __name__ == "__main__":
    main(SpringGeneration)

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

        self.generate_joint()

        self.go = False
        self.time = 0.0

    def generate_joint(self,
            pos_x = 0, pos_y = 0,
            dim_x = 5, dim_y = 5,
            knob_x_ratio = 1, knob_y_ratio = 1,
            ext_pos_x = 20, ext_pos_y = 20,
            ext_dim_x = 5, ext_dim_y = 5,
            ext_knob_x_ratio = -1, ext_knob_y_ratio = -1,
            prismatic_translation_low = -5, prismatic_translation_high = 5,
            ):

        """
        Generate Knob

        dim_x > 0;
        dim_y > 0;
        (-1 < knob_x_ratio < 1
            && abs(knob_y_ratio) = 1)
            ||
        (-1 < knob_y_ratio < 1
            && abs(knob_x_ratio) = 1)
        """
        # limb_base = self.world.CreateDynamicBody(
        #     position = (pos_x, pos_y),
        #     fixtures = b2FixtureDef(density = 2.0,
        #                             friction = 0.6,
        #                             shape = b2PolygonShape(box = (dim_x, dim_y)),
        #                             ),
        # )

        limb_base = self.world.CreateStaticBody(
            position = (pos_x, pos_y),
            # (5, 5) in all directions
            # dimension is (10, 10)
            shapes = b2PolygonShape(box = (dim_x, dim_y))
            )

        knob = self.world.CreateDynamicBody(
            position = (pos_x + dim_x * knob_x_ratio,
                        pos_y + dim_y * knob_y_ratio),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2CircleShape(pos=(0, 0), radius=0.5),
                                    ),
        )

        revolute_joint = self.world.CreateRevoluteJoint(
            bodyA = limb_base,
            bodyB = knob,
            anchor = knob.worldCenter,
            lowerAngle = -0.5 ** b2_pi,
            upperAngle = 0.5 ** b2_pi,
            enableLimit = True
        )

        """
        Generate Body Extension and Connect Spring
        """
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

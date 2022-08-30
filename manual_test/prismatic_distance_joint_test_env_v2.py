from Box2D import *
from Box2D.examples.framework import (Framework, Keys, main)

from math import sin, pi, sqrt

class PrismaticDistanceJointV2 (Framework):
    name = "PrismaticDistanceJointV2"
    description = 'g to stop/go'
    count = 800

    def __init__(self):
        Framework.__init__(self)
        self.world.gravity = (0, 0)

        body = self.world.CreateStaticBody(
            position = (10, 0),
            # (5, 5) in all directions
            # dimension is (10, 10)
            shapes = b2PolygonShape(box = (10, 5))
            )

        limb = self.world.CreateDynamicBody(
            position = (-10, 0),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2PolygonShape(box = (5, 5)),
                                    ),
        )

        self.prismatic_joint = self.world.CreatePrismaticJoint(
            bodyA = body,
            bodyB = limb,
            anchor = (0, 0),
            axis = (1, 0),
            lowerTranslation = -5.0,
            upperTranslation = 5,
            enableLimit = True,
            #motorForce = 1.0, #(Doesn't work)
            motorSpeed = 0.0,
            enableMotor = True,
        )

        self.distance_joint = self.world.CreateDistanceJoint(
            bodyA = body,
            bodyB = limb,
            anchorA = (0, 0),
            anchorB = (0, 0),
            # oscillations per second (for evey oscillator with 0 damping ratio)
            frequencyHz = 2.0,
            dampingRatio = 0.1,
            collideConnected = True
        )

        self.go = False
        self.time = 0.0

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
    main(PrismaticDistanceJointV2)

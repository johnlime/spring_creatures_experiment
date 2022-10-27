from Box2D import *
from Box2D.examples.framework import (Framework, Keys, main)

from math import sin, pi, sqrt

class RaycastTest (Framework):
    name = "RaycastTest"
    description = 'g to stop/go'
    count = 800

    def __init__(self):
        Framework.__init__(self)
        self.world.gravity = (0, 0)

        body = self.world.CreateStaticBody(
            position = (10, 0),
            # (5, 5) in all directions
            # dimension is (10, 10)
            shapes = b2PolygonShape(box = (10, 5)) # you might want to keep this
            )

        self.limb = self.world.CreateDynamicBody(
            position = (-5, 0),
            fixtures = b2FixtureDef(density = 2.0,
                                    friction = 0.6,
                                    shape = b2PolygonShape(box = (5, 5)),
                                    ),
        )

        joint = self.world.CreatePrismaticJoint(
            bodyA = body,
            bodyB = self.limb,
            anchor = (0, 0),
            axis = (1, 0),
            lowerTranslation = -5.0,
            upperTranslation = 0,
            enableLimit = True,
            #motorForce = 1.0,
            motorSpeed = 0.0,
            enableMotor = True,
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

        input = b2RayCastInput(p1 = (0, 0),
                               p2 = (-1, 0),
                               maxFraction = 4)
        output = b2RayCastOutput()
        transform = b2Transform()
        hit = self.limb.fixtures[0].RayCast(output, input, 0)
        if hit:
            hit_point = input.p1 + output.fraction * (input.p2 - input.p1)
            print(hit_point)


        renderer = self.renderer
        renderer.DrawPoint(renderer.to_screen((0, 0)),
                           4,
                           b2Color(0.9, 0.9, 0.9)
                           )

if __name__ == "__main__":
    main(RaycastTest)

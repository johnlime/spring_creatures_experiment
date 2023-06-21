from Box2D import *
import pygame
import numpy as np
from math import sin, cos

def spring_creature_render(obj):
    ####
    # Generate window and clock
    ####
    if obj.window is None and obj.render_mode == "human":
        pygame.init()
        pygame.display.init()
        obj.window = pygame.display.set_mode((obj.window_size, obj.window_size))
    if obj.clock is None and obj.render_mode == "human":
        obj.clock = pygame.time.Clock()

    canvas = pygame.Surface((obj.window_size, obj.window_size))
    canvas.fill((255, 255, 255))

    ####
    # Render items here
    ####
    for body in obj.world.bodies:
        for fixture in body.fixtures:
            if type(fixture.shape) == b2PolygonShape:
                fixture_vertices = []
                # rotate + translate vertices
                for i in range(len(fixture.shape.vertices)):
                    x = fixture.shape.vertices[i][0] * cos(-body.angle) - \
                        fixture.shape.vertices[i][1] * sin(-body.angle) + \
                        body.position[0]

                    y = fixture.shape.vertices[i][0] * sin(-body.angle) + \
                        fixture.shape.vertices[i][1] * cos(-body.angle) + \
                        body.position[1]

                    x = obj.scale * x + obj.window_size // 2 # half the window size
                    y = -obj.scale * y + obj.window_size // 2

                    fixture_vertices.append(tuple((x, y)))

                pygame.draw.polygon(
                    surface = canvas,
                    color = (0, 255, 150),
                    points = fixture_vertices,
                )

            elif type(fixture.shape) == b2CircleShape:
                pygame.draw.circle(
                    surface = canvas,
                    color = (0, 0, 255),
                    center = (obj.scale * (body.position[0] + fixture.shape.pos[0]) + \
                                obj.window_size // 2,
                              - obj.scale * (body.position[1] + fixture.shape.pos[1]) + \
                                obj.window_size // 2),
                    radius = obj.scale * fixture.shape.radius
                )
            else:
                pass

    for joint in obj.world.joints:
        if type(joint) == b2DistanceJoint:
            anchor_a = (obj.scale * joint.anchorA[0] + \
                            obj.window_size // 2,
                        - obj.scale * joint.anchorA[1] + \
                            obj.window_size // 2)
            anchor_b = (obj.scale * joint.anchorB[0] + \
                            obj.window_size // 2,
                        - obj.scale * joint.anchorB[1] + \
                            obj.window_size // 2)
            pygame.draw.line(
                surface = canvas,
                color = (255, 0, 0),
                start_pos = anchor_a,
                end_pos = anchor_b
            )


    ####
    # Display in a window or return an rgb array
    ####
    if obj.render_mode == "human":
        # The following line copies our drawings from `canvas` to the visible window
        obj.window.blit(canvas, canvas.get_rect())
        pygame.event.pump()
        pygame.display.update()

        # We need to ensure that human-rendering occurs at the predefined framerate.
        # The following line will automatically add a delay to keep the framerate stable.
        obj.clock.tick(obj.metadata["render_fps"])
    else:  # if obj.render_mode == "rgb_array":
        return np.transpose(
            np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
        )

from munch import DefaultMunch

default_settings = {
    'backend': 'pygame',
    'drawAABBs': False,
    'drawCOMs': False,
    'drawContactNormals': False,
    'drawContactPoints': False,
    'drawCoreShapes': False,
    'drawFPS': True,
    'drawJoints': True,
    'drawMenu': True,
    'drawOBBs': False,
    'drawPairs': False,
    'drawShapes': True,
    'drawStats': True,
    'enableContinuous': True,
    'enableSubStepping': False,
    'enableWarmStarting': True,
    'hz': 60,
    'maxContactPoints': 100,
    'onlyInit': False,
    'pause': False,
    'pointSize': 2.5,
    'positionIterations': 3,
    'singleStep': False,
    'velocityIterations': 8
}

default_settings = DefaultMunch.fromDict(default_settings)

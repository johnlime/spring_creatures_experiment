import numpy as np

x = np.fromfile('./run/models/sample.dat', dtype=float)
print(x.shape)
#sed = np.loadtxt('./run/models/sample.dat', unpack = True)
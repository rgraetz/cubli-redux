import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

sq_size = 1
x = [0, 0, sq_size]
y = [sq_size, 0, 0]
z = [0, sq_size, 0]

x = [0, 0, -sq_size]
y = [-sq_size, 0, 0]
z = [0, -sq_size, 0]

fig = plt.figure()
plt.clf()
ax = fig.add_subplot(projection='3d', proj_type='ortho')
ax.plot(x + [x[0]], y + [y[0]], z + [z[0]])
verts = [list(zip(x, y, z))]
ax.add_collection3d(Poly3DCollection(verts, alpha=0.5))

# plt.plot([0, 0.5], [0, 0.5], [0, 0], 'k--')
# plt.plot([0, 0], [0, 0.5], [0, 0.5], 'k--')
# plt.plot([0, 0.5], [0, 0], [0, 0.5], 'k--')

# board coordinate system
# plt.plot([0, -1.0/np.sqrt(1.5)], [0, 0.5/np.sqrt(1.5)], [0, 0.5/np.sqrt(1.5)], 'r--')
# plt.plot([0, 0.0], [0, -1.0/np.sqrt(2)], [0, 1.0/np.sqrt(2)], 'g--')
# plt.plot([0, 1.0/np.sqrt(3)], [0, 1.0/np.sqrt(3)], [0, 1.0/np.sqrt(3)], 'b--')

plt.plot([0, -0.5/np.sqrt(1.5)], [0, 1/np.sqrt(1.5)], [0, -0.5/np.sqrt(1.5)], 'r-.')
plt.plot([0, -1/np.sqrt(2)], [0, 0.0], [0, 1.0/np.sqrt(2)], 'g-.')
plt.plot([0, 1.0/np.sqrt(3)], [0, 1.0/np.sqrt(3)], [0, 1.0/np.sqrt(3)], 'b--')
plt.plot([0, -1.0/np.sqrt(3)], [0, -1.0/np.sqrt(3)], [0, -1.0/np.sqrt(3)], 'b-.')

# motor coordinate system and origin
ax.plot([0, 1], [0, 0], [0, 0], 'r')
ax.plot([0, 0], [0, 1], [0, 0], 'g')
ax.plot([0, 0], [0, 0], [0, 1], 'b')
ax.plot(0, 0, 0, 'k+')

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
plt.show(block=True)

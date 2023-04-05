import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import numpy as np

fig, ax = plt.subplots()

patches = []

wedge = mpatches.Wedge((.5, .5), 0.5, 270, 30, ec="none")
patches.append(wedge)

colors = np.linspace(0, 1, len(patches))
collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
collection.set_array(np.array(colors))
ax.add_collection(collection)

plt.show()
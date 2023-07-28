import numpy as np
from matplotlib import pyplot as plt

mat = np.load("results2.npy")
print(mat.shape)
# 350, 14, 10, 2
algs = [
    "AStar",
    "CBS",
    "CBSCAT",
    # "Coop",
    "ID",
    "IDCAT",
    "SID",
    "SIDCAT",
    "DSCBS",
    "StandardAStar",
    "EID",
    "SIDAStar",
    "SIDCBS",
    "SIDCATAStar",
    "SIDCATCBS",
    "DSCBSCAT",
]
nagents = np.arange(5, 55, 5, dtype=int)

plot = ["CBS", "CBSCAT", "DSCBS", "DSCBSCAT", "IDCAT"]

fig = plt.figure()
ax, ax2 = fig.subplots(2, 1)

ax.set_xlabel("# agents")
ax.set_ylabel("success rate")

for i, alg in enumerate(algs):
    if alg not in plot:
        continue
    successes = mat[:, i, :, 0].sum(axis=0)
    ax.plot(nagents, successes / 350, label=alg)
ax.legend()

ax2.set_xlabel("# agents")
ax2.set_ylabel("Runime [s]")

for i, alg in enumerate(algs):
    if alg not in plot:
        continue
    times = mat[:, i, :, 1].sum(axis=0)
    ax2.plot(nagents, times / 350, label=alg)
ax2.legend()

plt.show()

import numpy as np
from matplotlib import pyplot as plt

mat = np.load("results4.npy")
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
    "IDCBS",
    "IDCBSCAT",
]
nagents = np.arange(5, 55, 5, dtype=int)


def agents_vs_success(ax, plot, sharex=False):
    if sharex is False:
        ax.set_xlabel("# agents")
    else:
        ax.tick_params("x", labelbottom=False)
    ax.set_ylabel("Success rate [1]")
    ax.set_xticks(nagents)

    for i, alg in enumerate(algs):
        if alg not in plot:
            continue
        successes = mat[:, i, :, 0].sum(axis=0)
        ax.plot(nagents, successes / 350, label=alg)
    ax.legend()


def agents_vs_runtime(ax, plot, sharex=False):
    if sharex is False:
        ax.set_xlabel("# agents")
    else:
        ax.tick_params("x", labelbottom=False)
    ax.set_ylabel("Running time [s]")
    ax.set_xticks(nagents)

    for i, alg in enumerate(algs):
        if alg not in plot:
            continue
        times = mat[:, i, :, 1].sum(axis=0)
        ax.plot(nagents, times / 350, label=alg)
    ax.legend()


CBS = ["CBS", "CBSCAT", "DSCBS", "DSCBSCAT"]

fig = plt.figure()

fig.suptitle("CBS improvements comparison")

ax1, ax2 = fig.subplots(2, 1)
agents_vs_success(ax1, CBS, sharex=True)

agents_vs_runtime(ax2, CBS)

ID = [
    "ID",
    "IDCAT",
    "IDCBS",
    "IDCBSCAT",
    "SIDAStar",
    "SIDCBS",
    "SIDCATAStar",
    "SIDCATCBS",
]


fig = plt.figure()

fig.suptitle("ID variations comparison")

ax1, ax2 = fig.subplots(2, 1)
agents_vs_success(ax1, ID, sharex=True)

agents_vs_runtime(ax2, ID)

AStar = ["StandardAStar", "AStar"]

fig = plt.figure()

fig.suptitle("AStar with and without OD")

ax1, ax2 = fig.subplots(2, 1)
agents_vs_success(ax1, AStar, sharex=True)

agents_vs_runtime(ax2, AStar)

plt.show()

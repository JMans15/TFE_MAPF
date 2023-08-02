import numpy as np
from matplotlib import pyplot as plt
from matplotlib import use as mpuse
from itertools import product as iterprod

mpuse("QtAgg")

mat = np.load("all_results.npy")
print(mat.shape)
# 350, 14, 10, 2

objectives = ["SumOfCosts", "Fuel", "Makespan"]

heuristics = ["Optimal", "Manhattan"]

algs = [
    "A*",
    "A* OD",
    # "Coop A*",
    "ID A*",
    "ID A* CAT",
    "ID CBS",
    "ID CBS CAT",
    "SID A*",
    "SID A* CAT",
    "SID CBS",
    "SID CBS CAT",
    "CBS DS",
    "CBS DS CAT",
    "CBS",
    "CBS CAT",
]
nagents = np.arange(5, 55, 5, dtype=int)

# (350, 3, 2, 14, 10, 2)

# comparison between all obj and heur to determine what to use next

fig = plt.figure(figsize=(16, 8))
fig.suptitle("General comparison between objective functions and heuristics")
ax1, ax2 = fig.subplots(2, 1)
ax1.set_title("Success rate")
ax1.tick_params("x", labelbottom=False)
ax2.set_title("Running time")
ax2.set_xlabel("# agents")
ax1.set_ylabel("Success rate [1]")
ax2.set_ylabel("Running time [s]")
for (i, o), (j, h) in iterprod(enumerate(objectives), enumerate(heuristics)):
    color = ""
    match o:
        case "Makespan":
            color = "C0"
        case "SumOfCosts":
            color = "C1"
        case "Fuel":
            color = "C2"
    linestyle = ""
    match h:
        case "Manhattan":
            linestyle = "dashed"
        case "Optimal":
            linestyle = "solid"
    succ = mat[:, i, j, :, :, 0].sum(axis=0).sum(axis=0)
    time = mat[:, i, j, :, :, 1].sum(axis=0).sum(axis=0)
    ax1.plot(
        nagents, succ / 350 / 14, label=f"{o}, {h}", color=color, linestyle=linestyle
    )
    ax2.plot(
        nagents, time / 350 / 14, label=f"{o}, {h}", color=color, linestyle=linestyle
    )

box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])
box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])

# Put a legend below current axis
handles = []
handles.append(plt.Line2D([0], [0], label="Manhattan", color="k", linestyle="dashed"))
handles.append(plt.Line2D([0], [0], label="Optimal", color="k", linestyle="solid"))
handles.append(plt.Line2D([0], [0], label="Makespan", color="C0", linestyle="solid"))
handles.append(plt.Line2D([0], [0], label="SumOfCosts", color="C1", linestyle="solid"))
handles.append(plt.Line2D([0], [0], label="Fuel", color="C2", linestyle="solid"))
ax2.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.2),
    fancybox=True,
    shadow=True,
    ncol=6,
    handles=handles,
)

# Using Makespan, Optimal from now on

AStar = ["A*", "A* OD"]
CBS = ["CBS", "CBS CAT", "CBS DS", "CBS DS CAT"]
ID = ["ID A*", "ID A* CAT", "ID CBS", "ID CBS CAT"]
SID = ["SID A*", "SID A* CAT", "SID CBS", "SID CBS CAT"]

# A* vs OD

fig = plt.figure(figsize=(16, 8))
ax1, ax2 = fig.subplots(2, 1)
fig.suptitle(
    "Comparison between A* w/ and w/o OD, Optimal objective, Fuel and SumOfCosts heuristics"
)
ax1.set_title("Success rate")
ax1.tick_params("x", labelbottom=False)
ax2.set_title("Running time")
ax2.set_xlabel("# agents")
ax1.set_ylabel("Success rate [1]")
ax2.set_ylabel("Running time [s]")
for i, alg in enumerate(algs):
    if alg not in AStar:
        continue
    succ = mat[:, :2, 0, i, :, 0].sum(axis=(0, 1))
    time = mat[:, :2, 0, i, :, 1].sum(axis=(0, 1))
    ax1.plot(nagents, succ / 350 / 2, label=alg)
    ax2.plot(nagents, time / 350 / 2, label=alg)

box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])
box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])

ax2.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.2),
    fancybox=True,
    shadow=True,
    ncol=6,
)

# Comparison IDS

fig = plt.figure(figsize=(16, 8))
ax1, ax2 = fig.subplots(2, 1)
fig.suptitle("Global comparison between ID variations")
ax1.set_title("Success rate")
ax1.tick_params("x", labelbottom=False)
ax2.set_title("Running time")
ax2.set_xlabel("# agents")
ax1.set_ylabel("Success rate [1]")
ax2.set_ylabel("Running time [s]")
for i, alg in enumerate(algs):
    if alg not in ID and alg not in SID:
        continue
    linestyle = ""
    if alg in ID:
        linestyle = "solid"
    if alg in SID:
        linestyle = "dashed"
    succ = mat[:, :, :, i, :, 0].sum(axis=(0, 1, 2))
    time = mat[:, :, :, i, :, 1].sum(axis=(0, 1, 2))
    ax1.plot(nagents, succ / 350 / 6, label=alg, linestyle=linestyle)
    ax2.plot(nagents, time / 350 / 6, label=alg, linestyle=linestyle)

box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])
box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])

ax2.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.2),
    fancybox=True,
    shadow=True,
    ncol=8,
)


# Comparison CBS

fig = plt.figure(figsize=(16, 8))
ax1, ax2 = fig.subplots(2, 1)
fig.suptitle("Global comparison between CBS variations")
ax1.set_title("Success rate")
ax1.tick_params("x", labelbottom=False)
ax2.set_title("Running time")
ax2.set_xlabel("# agents")
ax1.set_ylabel("Success rate [1]")
ax2.set_ylabel("Running time [s]")
for i, alg in enumerate(algs):
    if alg not in CBS:
        continue
    succ = mat[:, :, :, i, :, 0].sum(axis=(0, 1, 2))
    time = mat[:, :, :, i, :, 1].sum(axis=(0, 1, 2))
    ax1.plot(nagents, succ / 350 / 6, label=alg)
    ax2.plot(nagents, time / 350 / 6, label=alg)

box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])
box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])

ax2.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.2),
    fancybox=True,
    shadow=True,
    ncol=8,
)
plt.show()

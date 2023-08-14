import numpy as np
from matplotlib import pyplot as plt
from matplotlib import use as mpuse
from itertools import product as iterprod

mpuse("QtAgg")

mat = np.load("all_results.npy")
mat_AStar = np.load("astar_results.npy")
mat_CBS = np.load("CBS_results.npy")
mat_IDAstar = np.load("IDastar_results.npy")
mat_IDCBS = np.load("IDCBS_results.npy")

mat100 = np.concatenate([mat_AStar, mat_CBS, mat_IDAstar, mat_IDCBS], axis=3)

# mat : pb x alg x nagents(10) x 2
# others : pb x 1 x 1 x alg x nagents(20) x 2

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
    mask = mat[:, i, j, :, :, 0] == 1
    time = np.mean(mat[:, i, j, :, :, 1], where=mask, axis=(0, 1))
    # time = mat[:, i, j, :, :, 1].sum(axis=0).sum(axis=0)
    ax1.plot(
        nagents, succ / 350 / 14, label=f"{o}, {h}", color=color, linestyle=linestyle
    )
    ax2.plot(nagents, time, label=f"{o}, {h}", color=color, linestyle=linestyle)

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

AStar = ["A*", "A* OD"]
CBS = ["CBS", "CBS CAT", "CBS DS", "CBS DS CAT"]
ID = ["ID A*", "ID A* CAT", "ID CBS", "ID CBS CAT"]
SID = ["SID A*", "SID A* CAT", "SID CBS", "SID CBS CAT"]

warehouse_maps = [14, 15, 16, 17]
maze_maps = [1, 2, 10, 11]
random_maps = [3, 4, 13]
city_maps = [6, 7, 18]
videogame_maps = [8, 9, 12]
empty_maps = [0]
room_maps = [5]

nagents = range(5, 105, 5)


def mkplot_big(mat, idx, labels, title, nagents, maps=list(range(19))):
    fig = plt.figure(figsize=(16, 8))
    ax1, ax2 = fig.subplots(2, 1)
    fig.suptitle(title)
    ax1.set_title("Success rate")
    ax1.tick_params("x", labelbottom=False)
    ax2.set_title("Running time")
    ax2.set_xlabel("# agents")
    ax1.set_ylabel("Success rate [1]")
    ax2.set_ylabel("Running time [s]")
    scens = np.arange(950, dtype=int).reshape((19, 50))[maps, :].flatten()
    for i, label in zip(idx, labels):
        succ = mat[scens, 0, 0, i, :, 0].sum(axis=0)
        mask = mat[scens, 0, 0, i, :, 0] == 1
        time = np.mean(mat[scens, 0, 0, i, :, 1], where=mask, axis=0)
        # time = mat[scens, 0, 0, i, :, 1].sum(axis=0)
        ax1.plot(nagents, succ / scens.size, label=label)
        ax2.plot(nagents, time, label=label)

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


def mkplot_for_map(
    maps, mapslabel, mat, algidx, ax1, ax2, linestyle="solid", runningtimes=True
):
    scens = np.arange(950, dtype=int).reshape((19, 50))[maps, :].flatten()
    succ = mat[scens, 0, 0, algidx, :, 0].sum(axis=0)
    mask = mat[scens, 0, 0, algidx, :, 0] == 1
    time = np.mean(mat_CBS[scens, 0, 0, 0, :, 1], where=mask, axis=0)
    ax1.plot(nagents, succ / scens.size, label=mapslabel, linestyle=linestyle)
    if not runningtimes:
        return
    ax2.plot(nagents, time, label=mapslabel, linestyle=linestyle)


def mkplot_for_maps(
    mat, algidx, title, algidx_dotted=None, mat_dotted=None, runningtimes=True
):
    fig = plt.figure(figsize=(16, 8))
    ax1, ax2 = fig.subplots(2, 1)
    fig.suptitle(title)
    ax1.set_title("Success rate")
    ax1.tick_params("x", labelbottom=False)
    ax2.set_title("Running time")
    ax2.set_xlabel("# agents")
    ax1.set_ylabel("Success rate [1]")
    ax2.set_ylabel("Running time [s]")

    mkplot_for_map(maze_maps, "Maze maps", mat, algidx, ax1, ax2)
    mkplot_for_map(warehouse_maps, "Warehouse maps", mat, algidx, ax1, ax2)
    mkplot_for_map(random_maps, "Random maps", mat, algidx, ax1, ax2)
    mkplot_for_map(city_maps, "City maps", mat, algidx, ax1, ax2)
    mkplot_for_map(videogame_maps, "Videogame maps", mat, algidx, ax1, ax2)
    mkplot_for_map(empty_maps, "Empty maps", mat, algidx, ax1, ax2)
    mkplot_for_map(room_maps, "Room maps", mat, algidx, ax1, ax2)

    ax1.set_prop_cycle(None)
    ax2.set_prop_cycle(None)
    if algidx_dotted is not None and mat_dotted is not None:
        mkplot_for_map(maze_maps, None, mat_dotted, algidx_dotted, ax1, ax2, "dashed")
        mkplot_for_map(
            warehouse_maps,
            None,
            mat_dotted,
            algidx_dotted,
            ax1,
            ax2,
            "dashed",
        )
        mkplot_for_map(random_maps, None, mat_dotted, algidx_dotted, ax1, ax2, "dashed")
        mkplot_for_map(city_maps, None, mat_dotted, algidx_dotted, ax1, ax2, "dashed")
        mkplot_for_map(
            videogame_maps,
            None,
            mat_dotted,
            algidx_dotted,
            ax1,
            ax2,
            "dashed",
        )
        mkplot_for_map(empty_maps, None, mat_dotted, algidx_dotted, ax1, ax2, "dashed")
        mkplot_for_map(room_maps, None, mat_dotted, algidx_dotted, ax1, ax2, "dashed")

    box = ax2.get_position()
    ax2.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])
    box = ax1.get_position()
    ax1.set_position([box.x0, box.y0 + box.height * 0.05, box.width, box.height * 0.95])

    if runningtimes:
        ax2.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, -0.2),
            fancybox=True,
            shadow=True,
            ncol=7,
        )
    else:
        ax1.set_xlabel("# agents")
        ax1.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, -0.2),
            fancybox=True,
            shadow=True,
            ncol=7,
        )
        ax2.remove()
        box = ax1.get_position()
        ax1.set_position(
            [box.x0, box.y0 + box.height * -1.05, box.width, box.height * 1.95]
        )


mkplot_for_maps(
    mat_CBS, 0, "CBS DS (solid) and CBS (dashed) in different maps", 2, mat_CBS
)

mkplot_for_maps(mat_CBS, 0, "CBS DS in different maps")

mkplot_for_maps(
    mat_IDAstar,
    1,
    "ID A* CAT (solid) and CBS DS CAT (dashed) in different maps",
    1,
    mat_CBS,
)

mkplot_big(
    mat_AStar,
    [0, 1],
    ["w/o OD", "w/ OD"],
    "A* w/ and w/o OD, 950 problems, 60s timeout, SumOfCosts objective, Optimal Heuristic",
    nagents,
)

mkplot_big(
    mat_IDAstar,
    list(range(4)),
    ["ID A*", "ID A* CAT", "SID A*", "SID A* CAT"],
    "Comparison between A* ID variants, 950 problems, \
60s timeout, SumOfCosts objective, Optimal Heuristic",
    nagents,
)

mkplot_for_maps(
    mat_IDAstar,
    1,
    "ID A* algorithm in different map types, SID (dashed) vs EID (solid)",
    3,
    mat_IDAstar,
    runningtimes=False,
)

mkplot_big(
    mat_IDCBS,
    list(range(4)),
    ["ID CBS", "ID CBS CAT", "SID CBS", "SID CBS CAT"],
    "Comparison between CBS ID variants, 950 problems, \
60s timeout, SumOfCosts objective, Optimal Heuristic",
    nagents,
)

mkplot_for_maps(
    mat_IDCBS,
    1,
    "ID CBS algorithm in different map types, SID (dashed) vs EID (solid)",
    3,
    mat_IDCBS,
    runningtimes=False,
)

mkplot_big(
    mat_CBS,
    list(range(4)),
    ["CBS DS", "CBS DS CAT", "CBS", "CBS CAT"],
    "Comparison between CBS variants, 950 problems, \
60s timeout, SumOfCosts objective, Optimal heuristic",
    nagents,
)

mkplot_big(
    mat100,
    [13, 9],
    ["SID CBS CAT", "SID A* CAT"],
    "Comparison between CBS and A* SID low-level searches, 950 problems, \
60s timeout, SumOfCosts objective, Optimal Heuristic",
    nagents,
)

mkplot_for_maps(
    mat100,
    9,
    "A* and CBS ID variants in different map types, SID CBS CAT (dashed) vs SID A* CAT (solid)",
    13,
    mat100,
    runningtimes=False,
)

mkplot_big(
    mat100,
    [1, 3, 7, 11],
    ["A* OD", "CBS DS CAT", "EID A* CAT", "EID CBS CAT"],
    "Comparison between best variants of each algorithm, \
950 problems, 60s timeout, SumOfCosts objective, Optimal heuristic",
    nagents,
)

plt.show()

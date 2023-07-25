import os
import subprocess
import time
from multiprocessing import Pool, current_process, freeze_support  # noqa: F401

from matplotlib import pyplot as plt

import utils

program = "../cmake-build/TFE_MAPF_visu"
num_threads = (
    1  # Number of threads to use (do not use too much because of RAM usage of mapf)
)
tmax = 240
nagents = 10

algs = ["AStar", "SIDAStar", "ID", "CBS", "IDCAT", "DSCBS", "CBSCAT"]

maps = [
    "../mapf-map/32-32-1",
    "../mapf-map/32-32-2",
    "../mapf-map/32-32-3",
    "../mapf-map/32-32-4",
    "../mapf-map/32-32-5",
    "../mapf-map/32-32-6",
    "../mapf-map/Paris",
]

problems = []
for mp in maps:
    wlk = os.walk(mp)
    tmp = wlk.__next__()
    mapfile = tmp[2][0]
    scendir = tmp[1][1]
    _ = wlk.__next__()
    scenfiles = wlk.__next__()[2]
    for scenfile in scenfiles:
        problems.append(
            [os.path.join(mp, mapfile), os.path.join(mp, scendir, scenfile)]
        )
print(f"{len(problems)} problems found")

errors = []


def run_program(a, i, algo, to):
    try:
        ok = subprocess.run(
            [
                program,
                "--scen",
                f"{problems[i][1]}",
                "--map",
                f"{problems[i][0]}",
                "-n",
                f"{a}",
                "--outfile",
                f"../Plotting/result_{algo}_{current_process().name}_{i}.txt",
                "-a",
                f"{algo}",
            ],
            timeout=to,
        )
        # Check if process ran succesfully
        try:
            ok.check_returncode()
        except subprocess.CalledProcessError:
            print("There was an error")
            return False
        # Check if solution is correct
        with open(f"result_{algo}_{current_process().name}_{i}.txt", "r") as file:
            if not utils.check_solution(file):
                print(f"failed with scenario {problems[i][1]}")
                os.remove(f"result_{algo}_{current_process().name}_{i}.txt")
                return False
            if not utils.check_start_and_target(file, problems[i][1], a):
                print(f"failed with scenario {problems[i][1]}")
                os.remove(f"result_{algo}_{current_process().name}_{i}.txt")
                return False
        os.remove(f"result_{algo}_{current_process().name}_{i}.txt")
        return True
    except subprocess.TimeoutExpired:
        return False


def get_alg_perf(alg, timeout):
    print(f"========== Starting with {alg}, {timeout} ==========")
    start = time.time()
    nprob = len(problems)
    elapsed = 0
    timestamps = [0.0]
    for i in range(nprob):
        run_program(nagents, i, alg, timeout - elapsed)
        now = time.time()
        elapsed = now - start
        if elapsed >= timeout:
            return timestamps
        # if (i + 1) % 5 == 0 or i + 1 == nprob:
        timestamps.append(elapsed)
        print(f"# {alg}: {i+1}/{nprob}")
    return timestamps


if __name__ == "__main__":
    freeze_support()
    # with Pool(num_threads) as p:
    # results = [p.apply_async(get_alg_perf, (alg, tmax)) for alg in algs]
    # for r in results:
    #     r.wait()
    # results = list(map(lambda x: list(x.get()), results))
    results = [get_alg_perf(alg, tmax) for alg in algs]

    fig = plt.figure()
    ax = fig.add_subplot()
    fig.suptitle(f"{nagents} agents, 175 problems")
    ax.set_xlabel("Timout [s]")
    ax.set_ylabel("Problems solved")
    for i, a in enumerate(algs):
        ax.plot(range(len(results[i])), results[i], label=a, linestyle="dashed")
    ax.legend()
    plt.show()

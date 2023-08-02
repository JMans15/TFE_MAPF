import os
import sys
import numpy as np
import subprocess
import time
from multiprocessing import Pool, Value, freeze_support  # noqa: F401

import utils

program = "./cpp-projects/TFE_MAPF/cmake-build/TFE_MAPF_visu"


counter = Value("i", 0)


def extract_problems(maps):
    problems = []
    for mp in maps:
        print(mp)
        wlk = os.walk(mp)
        tmp = wlk.__next__()
        mapfile = tmp[2][0]
        tmp = wlk.__next__()
        scendir = tmp[0]
        scenfiles = tmp[2]
        for scenfile in scenfiles:
            problems.append(
                [os.path.join(mp, mapfile), os.path.join(scendir, scenfile)]
            )
        tmp = wlk.__next__()
        scendir = tmp[0]
        scenfiles = tmp[2]
        for scenfile in scenfiles:
            problems.append(
                [os.path.join(mp, mapfile), os.path.join(scendir, scenfile)]
            )
    return problems


def run_and_check(a, i, algo, to, problem, obj, heu):
    try:
        # filename = f"./result_{algo}_{i}_{a}_{obj}_{heu}.txt"
        filename = (
            f"./cpp-projects/TFE_MAPF/Plotting/result_{algo}_{i}_{a}_{obj}_{heu}.txt"
        )
        ok = subprocess.run(
            [
                program,
                "--scen",
                f"{problem[1]}",
                "--map",
                f"{problem[0]}",
                "-n",
                f"{a}",
                "-o",
                obj,
                "--heuristic",
                heu,
                "--outfile",
                filename,
                "-a",
                f"{algo}",
            ],
            timeout=to,
        )
        # Check if process ran succesfully
        try:
            ok.check_returncode()
        except subprocess.CalledProcessError:
            print(
                f"There was an error with {algo} {obj} {heu} {a} {problem[1]}",
                file=sys.stderr,
            )
            return False
        # Check if solution is correct
        with open(filename, "r") as file:
            if not utils.check_solution(file):
                print(f"failed with scenario {problem[1]}")
                os.remove(filename)
                return False
            if not utils.check_start_and_target(file, problem[1], a):
                print(f"failed with scenario {problem[1]}")
                os.remove(filename)
                return False
        os.remove(filename)
        return True
    except subprocess.TimeoutExpired:
        return False


def get_alg_perf(alg, timeout, i, problem, nagents, obj, heu):
    results = []
    global counter
    counter.value += 1
    print(
        f"{counter.value:<5}",
        f"{alg:<11}",
        f"{obj:<10}",
        f"{heu:<9}",
        problem[1].split("/")[-1],
    )
    for n in nagents:
        start = time.time()
        ok = run_and_check(n, i, alg, timeout, problem, obj, heu)
        now = time.time()
        results.append((ok, now - start))
        if not ok:
            break
    for _ in range(len(nagents) - len(results)):
        results.append((False, timeout))
    return results


def init(cntr):
    global counter
    counter = cntr


if __name__ == "__main__":
    freeze_support()
    counter = Value("i", 0)

    nagents = np.arange(5, 55, 5, dtype=int)
    tmax = 30
    num_threads = 10

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

    maps = [
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-1",
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-2",
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-3",
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-4",
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-5",
        "./cpp-projects/TFE_MAPF/mapf-map/32-32-6",
        "./cpp-projects/TFE_MAPF/mapf-map/Paris",
    ]
    problems = extract_problems(maps)
    # problems = problems[:4]
    print(f"{len(problems)} problems found")
    pool = Pool(num_threads, initializer=init, initargs=(counter,))
    results = np.empty(
        (len(problems), len(objectives), len(heuristics), len(algs), len(nagents), 2)
    )
    presults = np.array(
        [
            [
                [
                    [
                        pool.apply_async(
                            get_alg_perf, args=(a, tmax, i, p, nagents, o, h)
                        )
                        for a in algs
                    ]
                    for h in heuristics
                ]
                for o in objectives
            ]
            for i, p in enumerate(problems)
        ]
    )
    for pi, p in enumerate(presults):
        for oi, o in enumerate(p):
            for hi, h in enumerate(o):
                for ai, a in enumerate(h):
                    results[pi, oi, hi, ai] = a.get()
    print(results.shape)
    np.save("./cpp-projects/TFE_MAPF/Plotting/all_results.npy", results)

from multiprocessing import Pool, freeze_support
import os
import subprocess
from matplotlib import pyplot as plt
import time

program = "../cmake-build/TFE_MAPF_visu"
num_threads = (
    1  # Number of threads to use (do not use too much because of RAM usage of mapf)
)
nagents = 20

maps = [
    "../mapf-map/Paris",
    "../mapf-map/32-32-1",
    "../mapf-map/32-32-2",
    "../mapf-map/32-32-3",
    "../mapf-map/32-32-4",
    "../mapf-map/32-32-5",
    "../mapf-map/32-32-6",
]

problems = []
for map in maps:
    wlk = os.walk(map)
    mapfile = wlk.__next__()[2][0]
    _ = wlk.__next__()
    scenfiles = wlk.__next__()[2]
    for scenfile in scenfiles:
        problems.append([mapfile, scenfile])
print(f"{len(problems)} problems found")


def run_program(a, i, algo, to):
    try:
        # print(
        #     f"Running {algo} with {a} agents with timeout {to} for problem {problems[i]}"
        # )
        subprocess.run(
            [
                program,
                "--scen",
                f"{problems[i][1]}",
                "--map",
                f"{problems[i][0]}",
                "-n",
                f"{a}",
                "--outfile",
                f"../Plotting/result_{i}.txt",
                "-a",
                f"{algo}",
            ],
            timeout=to,
        )
        # print(
        #     f"Finished {algo} with {a} agents with timeout {to} for problem {problems[i]}"
        # )
        # with open(f"result_{i}.txt", "r") as file:
        #     if not check_solution(file):
        #         print(f"failed with scenario {file_path}")
        #         return False
        #     if not check_start_and_target(file, file_path, a):
        #         print(f"failed with scenario {file_path}")
        #         return False
        return True
    except subprocess.TimeoutExpired:
        return False


def get_alg_perf(alg, timeout):
    print(f"========== Starting with {alg}, {timeout} ==========")
    start = time.time()
    nprob = len(problems)
    elapsed = 0
    for i in range(nprob):
        run_program(nagents, i, alg, timeout - elapsed)
        now = time.time()
        elapsed = now - start
        if elapsed >= timeout:
            return i
        if i + 1 % 10 == 0 or i + 1 == nprob:
            print(f"#### {alg}: {i}/{nprob}")
    return nprob


algs = ["AStar", "ID", "CBS"]

with Pool(num_threads) as p:
    results = []
    for to in range(1, 6):
        row = []
        for alg in algs:
            row.append(get_alg_perf(alg, to))
        results.append(row)

    for to, row in zip(range(1, 6), results):
        for alg, result in zip(algs, row):
            print(f"{alg}, {to}: {result}/{len(problems)}")
    # results = [
    #     [p.apply_async(get_alg_perf, (alg, to)) for alg in algs] for to in range(1, 6)
    # ]
    # for rw in results:
    #     for r in rw:
    #         r.wait()
    # for to, row in zip(range(1, 6), results):
    #     for alg, result in zip(algs, row):
    #         print(f"{alg}, {to}: {result.get()}/{len(problems)}")

# def check_solution(file):
#     file.seek(0)
#     agents, timesteps = list(map(int, file.readline().strip().split(" ")))
#     lines = list(map(lambda line: line.strip(), file.readlines()))
#     # positions = list(map(lambda l: list(map(int, l.strip().split(','))), lines))
#     per_timestep = np.array(lines).reshape((timesteps, agents))
#     # Vertex conflicts
#     for step in per_timestep:
#         uniques, c = np.unique(step, return_counts=True)
#         doubles = uniques[c > 1]
#         if doubles.size > 0:
#             print("vertex conflict")
#             print(doubles)
#             return False
#
#     # Edge conflicts
#     def sort_edges(arr):
#         result = np.empty_like(arr[1:], dtype=object)
#         for i, (el, nel) in enumerate(zip(arr[:-1], arr[1:])):
#             big, small = (el, nel) if el > nel else (nel, el)
#             result[i] = f"{small};{big}"
#         return result
#
#     sorted_edges = np.apply_along_axis(sort_edges, axis=0, arr=per_timestep)
#     for move in sorted_edges:
#         uniques, c = np.unique(move, return_counts=True)
#         doubles = uniques[c > 1]
#         if doubles.size > 0:
#             print("edge conflict")
#             print(doubles)
#             return False
#     return True
#
#
# def check_start_and_target(file, scenario_filename, nagents):
#     file.seek(0)
#     agents, timesteps = list(map(int, file.readline().strip().split(" ")))
#     starts, targets = [], []
#     with open(scenario_filename, "r") as scenario:
#         scenario.readline()
#         lines = np.array(
#             list(map(lambda line: line.strip().split("\t"), scenario.readlines()))
#         )
#         starts = np.char.add(
#             np.char.add(lines[:, 4], np.full_like(lines[:, 4], fill_value=",")),
#             lines[:, 5],
#         )[:nagents]
#         targets = np.char.add(
#             np.char.add(lines[:, 6], np.full_like(lines[:, 6], fill_value=",")),
#             lines[:, 7],
#         )[:nagents]
#     lines = list(map(lambda line: line.strip(), file.readlines()))
#     per_timestep = np.array(lines).reshape((timesteps, agents))[:, :nagents]
#
#     if sorted(per_timestep[0, :]) != sorted(starts):
#         print("Starts are not right")
#         return False
#     if sorted(per_timestep[-1, :]) != sorted(targets):
#         print("Targets are not right")
#         return False
#     return True

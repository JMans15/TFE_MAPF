import os
import subprocess
import numpy as np
from matplotlib import pyplot as plt
from multiprocessing import freeze_support
import time
import random

program = "../cmake-build/TFE_MAPF_visu"
timeout = 5  # Timeout in seconds


def check_solution(file):
    file.seek(0)
    agents, timesteps = list(map(int, file.readline().strip().split(" ")))
    lines = list(map(lambda l: l.strip(), file.readlines()))
    # positions = list(map(lambda l: list(map(int, l.strip().split(','))), lines))
    per_timestep = np.array(lines).reshape((timesteps, agents))
    # Vertex conflicts
    for step in per_timestep:
        uniques, c = np.unique(step, return_counts=True)
        doubles = uniques[c > 1]
        if doubles.size > 0:
            print("vertex conflict")
            print(doubles)
            return False

    # Edge conflicts
    def sort_edges(arr):
        result = np.empty_like(arr[1:], dtype=object)
        for i, (el, nel) in enumerate(zip(arr[:-1], arr[1:])):
            big, small = (el, nel) if el > nel else (nel, el)
            result[i] = f"{small};{big}"
        return result

    sorted_edges = np.apply_along_axis(sort_edges, axis=0, arr=per_timestep)
    for move in sorted_edges:
        uniques, c = np.unique(move, return_counts=True)
        doubles = uniques[c > 1]
        if doubles.size > 0:
            print("edge conflict")
            print(doubles)
            return False
    return True


def check_start_and_target(file, scenario_filename, nagents):
    file.seek(0)
    agents, timesteps = list(map(int, file.readline().strip().split(" ")))
    starts, targets = [], []
    with open(scenario_filename, "r") as scenario:
        scenario.readline()
        lines = np.array(
            list(map(lambda l: l.strip().split("\t"), scenario.readlines()))
        )
        starts = np.char.add(
            np.char.add(lines[:, 4], np.full_like(lines[:, 4], fill_value=",")),
            lines[:, 5],
        )[:nagents]
        targets = np.char.add(
            np.char.add(lines[:, 6], np.full_like(lines[:, 6], fill_value=",")),
            lines[:, 7],
        )[:nagents]
    lines = list(map(lambda l: l.strip(), file.readlines()))
    per_timestep = np.array(lines).reshape((timesteps, agents))[:, :nagents]

    if sorted(per_timestep[0, :]) != sorted(starts):
        print("Starts are not right")
        return False
    if sorted(per_timestep[-1, :]) != sorted(targets):
        print("Targets are not right")
        return False
    return True


def run_program(file_path, a, i, algo, args):
    try:
        start_time = time.time()
        subprocess.run(
            [
                program,
                "--scen",
                f"{file_path}",
                "-n",
                f"{a}",
                "--outfile",
                f"../Plotting/result_{i}.txt",
                "-a",
                f"{algo}",
            ]
            + args,
            timeout=timeout,
        )
        end_time = time.time()
        with open(f"result_{i}.txt", "r") as file:
            if not check_solution(file):
                print(f"failed with scenario {file_path}")
                return False, None
            if not check_start_and_target(file, file_path, a):
                print(f"failed with scenario {file_path}")
                return False, None
        return True, (end_time - start_time) * 1000  # millisecondes
    except subprocess.TimeoutExpired:
        return False, None


a = [
    random.randint(20, 100) for _ in range(10000)
]  # random number between 20 and 100 for the number of agents


def data_for_algo(algo):
    data = []
    number_of_instances = 0

    print("------------------- NEW ALGO")

    args = ["--map", "../mapf-map/32-32-1/empty-32-32.map"]
    directory = "../mapf-map/32-32-1/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-2/maze-32-32-2.map"]
    directory = "../mapf-map/32-32-2/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-3/maze-32-32-4.map"]
    directory = "../mapf-map/32-32-3/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-4/random-32-32-10.map"]
    directory = "../mapf-map/32-32-4/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-5/random-32-32-20.map"]
    directory = "../mapf-map/32-32-5/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-6/room-32-32-4.map"]
    directory = "../mapf-map/32-32-6/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    data.sort()
    return data, len(data) * 100.0 / number_of_instances


def data_for_algo2(algo):
    data = []
    number_of_instances = 0

    print("------------------- NEW ALGO")

    args = ["--map", "../mapf-map/32-32-1/empty-32-32.map"]
    directory = "../mapf-map/32-32-1/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1
    # directory = '../mapf-map/32-32-1/scen-even'
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-2/maze-32-32-2.map"]
    directory = "../mapf-map/32-32-2/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1
    directory = "../mapf-map/32-32-2/scen-even"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-3/maze-32-32-4.map"]
    directory = "../mapf-map/32-32-3/scen-random"
    files = os.listdir(directory)
    for i, filename in enumerate(files):
        bool, time = run_program(
            os.path.join(directory, filename), a[number_of_instances], i, algo, args
        )
        if bool:
            data.append(time)
        number_of_instances += 1
    directory = "../mapf-map/32-32-3/scen-even"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-4/random-32-32-10.map"]
    directory = "../mapf-map/32-32-4/scen-random"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1
    directory = "../mapf-map/32-32-4/scen-even"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    args = ["--map", "../mapf-map/32-32-5/random-32-32-20.map"]
    directory = "../mapf-map/32-32-5/scen-random"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1
    # directory = '../mapf-map/32-32-5/scen-even'
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    # args = ['--map', '../mapf-map/32-32-6/room-32-32-4.map']
    # directory = '../mapf-map/32-32-6/scen-random'
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1
    directory = "../mapf-map/32-32-6/scen-even"
    # files = os.listdir(directory)
    # for i, filename in enumerate(files):
    #     bool, time = run_program(os.path.join(directory, filename), a[number_of_instances], i, algo, args)
    #     if bool:
    #         data.append(time)
    #     number_of_instances += 1

    print(number_of_instances)
    print(len(data) * 100.0 / number_of_instances, " %")

    data.sort()
    return data, len(data) * 100.0 / number_of_instances


if __name__ == "__main__":
    freeze_support()
    fig = plt.figure(figsize=(16, 8))
    ax = fig.add_subplot()

    ax.set_title(
        f"Performance curves (timeout={timeout}s, random scenarios in several 32x32 grids)"
    )
    ax.set_xlabel("Number of solved problems")
    ax.set_ylabel("Time [ms]")

    data, success_rate = data_for_algo2("StandardAStar")
    if len(data) > 0:
        ax.text(
            len(data),
            data[-1],
            str(success_rate) + " %",
            ha="left",
            va="bottom",
        )
    if len(data) > 0:
        ax.plot(len(data), data, marker="x", label="S")

    data, success_rate = data_for_algo2("AStar")
    if len(data) > 0:
        ax.text(
            len(data),
            data[-1],
            str(success_rate) + " %",
            ha="left",
            va="bottom",
        )
    if len(data) > 0:
        ax.plot(len(data), data, marker="x", label="OD")

    data, success_rate = data_for_algo2("SIDAStar")
    if len(data) > 0:
        ax.text(
            len(data),
            data[-1],
            str(success_rate) + " %",
            ha="left",
            va="bottom",
        )
    if len(data) > 0:
        ax.plot(len(data), data, marker="x", label="SID+OD")

    # data = data_for_algo("SIDCATAStar")
    # ax.plot(range(15, 30), data, marker='x', label="(SID+CAT)+A*")

    # data, succes_rate = data_for_algo("SIDCBS")
    # ax.plot(list(range(1, len(data) + 1)), data, marker='x', label="SID+CBS")

    # data = data_for_algo("SIDCATCBS")
    # ax.plot(range(15, 30), data, marker='x', label="(SID+CAT)+CBS")

    # data = data_for_algo("EID")
    # ax.plot(list(range(1, len(data)+1)), data, marker='x', label="Enhanced version of ID (EID)")

    data, success_rate = data_for_algo2("ID")
    if len(data) > 0:
        ax.text(
            len(data),
            data[-1],
            str(success_rate) + " %",
            ha="left",
            va="bottom",
        )
    if len(data) > 0:
        ax.plot(len(data), data, marker="x", label="Refinement1=EID")

    data, success_rate = data_for_algo2("IDCAT")
    if len(data) > 0:
        ax.text(
            len(data),
            data[-1],
            str(success_rate) + " %",
            ha="left",
            va="bottom",
        )
    if len(data) > 0:
        ax.plot(len(data), data, marker="x", label="Refinement2=EID+CAT")

    # data, succes_rate = data_for_algo("CBS")
    # ax.plot(list(range(1, len(data) + 1)), data, marker='x', label="CBS")

    # data = data_for_algo("CBSCAT")
    # ax.plot(range(10, 25), data, marker='x', label="CBS+CAT")

    # data = data_for_algo("DSCBS")
    # ax.plot(list(range(1, len(data)+1)), data, marker='x', label="Disjoint Splitting CBS")
    # ax.set_yscale("log")

    ax.grid(axis="both")

    plt.legend()
    plt.show()

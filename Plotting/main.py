import os
import subprocess
import numpy as np
from matplotlib import pyplot as plt
from multiprocessing import Pool
import sys, os

directory = '../mapf-map/Paris/scen-random'
program = '../cmake-build-debug/TFE_MAPF_visu'
timeout = 2  # Timeout in seconds
args = ['-m', '--map', '../mapf-map/Paris/Paris_1_256.map']
num_threads = 1  # Number of threads to use (do not use too much because of RAM usage of mapf)

def check_solution(file):
    file.seek(0)
    agents, timesteps = list(map(int, file.readline().strip().split(' ')))
    lines = list(map(lambda l: l.strip(), file.readlines()))
    # positions = list(map(lambda l: list(map(int, l.strip().split(','))), lines))
    per_timestep = np.array(lines).reshape((timesteps, agents))
    # Vertex conflicts
    for step in per_timestep:
        uniques, c = np.unique(step, return_counts=True)
        doubles = uniques[c > 1]
        if doubles.size > 0:
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
            print(doubles)
            return False
    return True

def check_start_and_target(file, scenario_filename, nagents):
    file.seek(0)
    agents, timesteps = list(map(int, file.readline().strip().split(' ')))
    starts, targets = [], []
    with open(scenario_filename, 'r') as scenario:
        scenario.readline()
        lines = np.array(list(map(lambda l: l.strip().split('\t'), scenario.readlines())))
        starts = np.char.add(np.char.add(lines[:, 4], np.full_like(lines[:, 4], fill_value=',')), lines[:, 5])[:nagents]
        targets = np.char.add(np.char.add(lines[:, 6], np.full_like(lines[:, 6], fill_value=',')), lines[:, 7])[:nagents]
    lines = list(map(lambda l: l.strip(), file.readlines()))
    per_timestep = np.array(lines).reshape((timesteps, agents))[:, :nagents]

    if sorted(per_timestep[0, :]) != sorted(starts):
        print("Starts are not right")
        return False
    if sorted(per_timestep[-1, :]) != sorted(targets):
        print("Targets are not right")
        return False
    return True

def run_program(file_path, a, i, algo):
    try:
        subprocess.run([program, '--scen', f'{file_path}', '-a', f'{a}', '--outfile', f'../Plotting/result_{i}.txt', algo] + args, timeout=timeout)
        with open(f"result_{i}.txt", 'r') as file:
            if not check_solution(file):
                print(f"failed with scenario {file_path}")
                return False
            if not check_start_and_target(file, file_path, a):
                print(f"failed with scenario {file_path}")
                return False
        return True
    except subprocess.TimeoutExpired:
        return False

def data_for_algo(algo):
    data = []
    for a in range(1, 26): #max 1002
        files = os.listdir(directory)
        with Pool(num_threads) as p:
            results = [p.apply_async(run_program, (os.path.join(directory, filename), a, i, algo)) for i, filename in enumerate(files)]
            timeouts = sum(not result.get() for result in results)
        success = (1-timeouts/len(files))*100
        data.append(success)
        print(f"Success rate with {a} agents = {success}")
        if success == 0.0:
            break

    return data

fig = plt.figure(figsize=(16, 8))
ax = fig.subplots(1,1)

data = data_for_algo('')

ax.set_title(f"Timeout = {timeout}, random scenarios")
ax.set_xlabel("Number of agents")
ax.set_ylabel("Success rate [%]")

ax.plot(list(range(1, len(data)+1)), data, marker='x', label="A*")

data = data_for_algo("--sid")
ax.plot(list(range(1, len(data)+1)), data, marker='x', label="Simple Independence Detection")

data = data_for_algo("--id")
ax.plot(list(range(1, len(data)+1)), data, marker='x', label="Independence Detection")

ax.grid(axis='both')

plt.legend()
plt.show()

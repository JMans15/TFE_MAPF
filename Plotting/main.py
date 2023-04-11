import os
import subprocess
import numpy as np
from matplotlib import pyplot as plt
from multiprocessing import Pool
import sys, os

directory = '../mapf-map/Paris/scen-random'
program = '../cmake-build-debug/TFE_MAPF_visu'
timeout = 10  # Timeout in seconds
args = ['-m', '--map', '../mapf-map/Paris/Paris_1_256.map']
num_threads = 4  # Number of threads to use (do not use too much because of RAM usage of mapf)

data = []

def check_solution(filename):
    with open(filename, 'r') as file:
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

def run_program(file_path, a, i):
    try:
        subprocess.run([program, '--scen', f'{file_path}', '-a', f'{a}', '--outfile', f'../Plotting/result_{i}.txt'] + args, timeout=timeout)
        check_solution(f'result_{i}.txt')
        return True
    except subprocess.TimeoutExpired:
        return False

for a in range(1, 1002):
    files = os.listdir(directory)
    with Pool(num_threads) as p:
        results = [p.apply_async(run_program, (os.path.join(directory, filename), a, i)) for i, filename in enumerate(files)]
        timeouts = sum(not result.get() for result in results)
    success = (1-timeouts/len(files))*100
    data.append(success)
    print(f"Success rate with {a} agents = {success}")
    if success == 0.0:
        break

print(f"\nDatapoints : {data}")

fig = plt.figure(figsize=(16, 8))
ax = fig.subplots(1,1)
ax.set_title(f"Timeout = {timeout}, random scenarios")
ax.set_xlabel("Number of agents")
ax.set_ylabel("Success rate [%]")
ax.plot(list(range(1, len(data)+1)), data, marker='x')
plt.show()

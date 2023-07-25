import numpy as np


def check_solution(file):
    file.seek(0)
    agents, timesteps = list(map(int, file.readline().strip().split(" ")))
    lines = list(map(lambda line: line.strip(), file.readlines()))
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
            list(map(lambda line: line.strip().split("\t"), scenario.readlines()))
        )
        starts = np.char.add(
            np.char.add(lines[:, 4], np.full_like(lines[:, 4], fill_value=",")),
            lines[:, 5],
        )[:nagents]
        targets = np.char.add(
            np.char.add(lines[:, 6], np.full_like(lines[:, 6], fill_value=",")),
            lines[:, 7],
        )[:nagents]
    lines = list(map(lambda line: line.strip(), file.readlines()))
    per_timestep = np.array(lines).reshape((timesteps, agents))[:, :nagents]

    if sorted(per_timestep[0, :]) != sorted(starts):
        print("Starts are not right")
        return False
    if sorted(per_timestep[-1, :]) != sorted(targets):
        print("Targets are not right")
        return False
    return True

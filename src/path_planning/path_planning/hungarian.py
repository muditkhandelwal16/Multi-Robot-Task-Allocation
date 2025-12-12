#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hungarian-based task allocation (original core logic preserved),
plus distance-aware variants for integration with A* path lengths.
"""

import numpy as np
from collections import deque
import sys

# ---------------------------------------------------------------------------
# Example robots and tasks (you can ignore/override these when importing)
# ---------------------------------------------------------------------------

robots = [
    {'id': 'R1', 'c_robot': np.array([1, 1, 0, 1, 0]), 'battery': 75, 'status': 0, 'position': [0, 0]},
    {'id': 'R2', 'c_robot': np.array([0, 0, 1, 1, 1]), 'battery': 40, 'status': 0, 'position': [0, 0]},
    {'id': 'R3', 'c_robot': np.array([1, 0, 0, 1, 0]), 'battery': 82, 'status': 0, 'position': [0, 0]},
    {'id': 'R4', 'c_robot': np.array([0, 1, 1, 0, 1]), 'battery': 60, 'status': 0, 'position': [0, 0]},
    {'id': 'R5', 'c_robot': np.array([1, 1, 1, 1, 0]), 'battery': 90, 'status': 0, 'position': [0, 0]},
]

tasks = [
    {'id': 'T1', 'r_tasks': np.array([1, 1, 0, 1, 0]), 'coordinate': [10, 12], 'dist': 50,  't_ser': 5,  'priority': 1, 'status': 0, 'assigned_robot': None},
    {'id': 'T2', 'r_tasks': np.array([0, 0, 1, 1, 0]), 'coordinate': [35, 5],  'dist': 70,  't_ser': 10, 'priority': 0, 'status': 0, 'assigned_robot': None},
    {'id': 'T3', 'r_tasks': np.array([1, 0, 0, 1, 0]), 'coordinate': [50, 40], 'dist': 150, 't_ser': 12, 'priority': 2, 'status': 0, 'assigned_robot': None},
    {'id': 'T4', 'r_tasks': np.array([1, 1, 1, 0, 0]), 'coordinate': [90, 25], 'dist': 300, 't_ser': 4,  'priority': 1, 'status': 0, 'assigned_robot': None},
    {'id': 'T5', 'r_tasks': np.array([1, 1, 1, 0, 0]), 'coordinate': [70, 50], 'dist': 100, 't_ser': 7,  'priority': 0, 'status': 0, 'assigned_robot': None},
    {'id': 'T6', 'r_tasks': np.array([0, 1, 1, 1, 0]), 'coordinate': [20, 80], 'dist': 180, 't_ser': 8,  'priority': 2, 'status': 0, 'assigned_robot': None},
    {'id': 'T7', 'r_tasks': np.array([1, 0, 1, 0, 1]), 'coordinate': [60, 10], 'dist': 120, 't_ser': 6,  'priority': 1, 'status': 0, 'assigned_robot': None},
    {'id': 'T8', 'r_tasks': np.array([0, 1, 0, 1, 1]), 'coordinate': [85, 60], 'dist': 250, 't_ser': 15, 'priority': 2, 'status': 0, 'assigned_robot': None},
]

# ---------------------------------------------------------------------------
# Global parameters (same as your original)
# ---------------------------------------------------------------------------

v_max = 1.0    # travel speed (distance units per time unit)
q_max = 100.0
q_min = 10.0
t_max = 60.0
t_min = 5.0
w1 = 1.5
w2 = 2.0
w3 = 1.0
w4 = 1.0
hi = 9999.0

# ---------------------------------------------------------------------------
# Normalization & penalty functions (unchanged)
# ---------------------------------------------------------------------------

def capability_penalty(c_i, rho_j):
    req = np.sum(rho_j)
    if req == 0:
        return 1.0
    return 1.0 - (np.dot(c_i, rho_j) / req)

def t_nor(t):
    return (t - t_min) / (t_max - t_min)

def q_nor(q):
    return (q - q_min) / (q_max - q_min)

def b_nor(b):
    return 1 if b > 26 else 0

def euclidean_dist(r_coord, t_coord):
    # NOTE: legacy function (buggy as in original file, left untouched)
    xr, yr = r_coord
    xt, yt = t_coord
    dist = np.sqrt((xt - xr)**2 + (yt - xr)**2)
    return dist

# ---------------------------------------------------------------------------
# Original cost-matrix generation (kept as-is)
# ---------------------------------------------------------------------------

def compute_cost_matrix(available_robots, candidate_tasks):
    nr = len(available_robots)
    nt = len(candidate_tasks)
    cost = np.zeros((nr, nt))
    for i_idx, i in enumerate(available_robots):
        for j_idx, j in enumerate(candidate_tasks):
            if i['battery'] > 25:
                cap_penalty = capability_penalty(i['c_robot'], j['r_tasks'])
                cost_val = (
                    w1 * cap_penalty +
                    w2 * (2 - j['priority']) +
                    w3 * t_nor(j['t_ser']) +
                    w4 * q_nor(euclidean_dist(i['position'], j['coordinate']))
                )
                cost[i_idx, j_idx] = cost_val
            else:
                cost[i_idx, j_idx] = hi
    return cost

# ---------------------------------------------------------------------------
# NEW: cost matrix that uses an external distance function
#   distance_fn(robot_dict, task_dict) -> float (path length)
# ---------------------------------------------------------------------------

def compute_cost_matrix_with_distance_fn(available_robots, candidate_tasks, distance_fn):
    """
    Same structure as compute_cost_matrix, but uses a supplied distance_fn
    instead of built-in Euclidean distance. This is what we use with A*.
    """
    nr = len(available_robots)
    nt = len(candidate_tasks)
    cost = np.zeros((nr, nt), dtype=float)

    for i_idx, i in enumerate(available_robots):
        for j_idx, j in enumerate(candidate_tasks):
            if i['battery'] > 25:
                cap_penalty = capability_penalty(i['c_robot'], j['r_tasks'])
                try:
                    dist_val = float(distance_fn(i, j))
                except Exception:
                    # any failure in distance_fn -> treat as very large distance
                    dist_val = q_max
                cost_val = (
                    w1 * cap_penalty +
                    w2 * (2 - j['priority']) +
                    w3 * t_nor(j['t_ser']) +
                    w4 * q_nor(dist_val)
                )
                cost[i_idx, j_idx] = cost_val
            else:
                cost[i_idx, j_idx] = hi
    return cost

# ---------------------------------------------------------------------------
# Hungarian algorithm (unchanged)
# ---------------------------------------------------------------------------

def labelIt(cost, lx):
    n = len(cost)
    for i in range(n):
        for j in range(n):
            lx[i] = max(lx[i], cost[i][j])

def addTree(x, prevX, inTreeX, prev, slack, slackX, lx, ly, cost):
    n = len(slack)
    inTreeX[x] = True
    prev[x] = prevX
    for y in range(n):
        if lx[x] + ly[y] - cost[x][y] < slack[y]:
            slack[y] = lx[x] + ly[y] - cost[x][y]
            slackX[y] = x

def updateLabels(inTreeX, inTreeY, slack, lx, ly):
    n = len(slack)
    delta = sys.maxsize
    for y in range(n):
        if not inTreeY[y]:
            delta = min(delta, slack[y])
    for x in range(n):
        if inTreeX[x]:
            lx[x] -= delta
    for y in range(n):
        if inTreeY[y]:
            ly[y] += delta
        else:
            slack[y] -= delta

def augment(cost, match, inTreeX, inTreeY, prev, xy, yx, slack, slackX, lx, ly):
    n = len(cost)
    if match[0] == n:
        return
    q = deque()
    root = -1
    for i in range(n):
        if xy[i] == -1:
            root = i
            q.append(root)
            prev[i] = -2
            inTreeX[i] = True
            break
    for y in range(n):
        slack[y] = lx[root] + ly[y] - cost[root][y]
        slackX[y] = root
    while True:
        while q:
            x = q.popleft()
            for y in range(n):
                if lx[x] + ly[y] - cost[x][y] == 0 and not inTreeY[y]:
                    if yx[y] == -1:
                        cx, cy = x, y
                        match[0] += 1
                        while cx != -2:
                            ty = xy[cx]
                            xy[cx] = cy
                            yx[cy] = cx
                            cx = prev[cx]
                            cy = ty
                        for i in range(n):
                            inTreeX[i] = inTreeY[i] = False
                        augment(cost, match, inTreeX, inTreeY, prev, xy, yx, slack, slackX, lx, ly)
                        return
                    else:
                        inTreeY[y] = True
                        q.append(yx[y])
                        addTree(yx[y], x, inTreeX, prev, slack, slackX, lx, ly, cost)
        updateLabels(inTreeX, inTreeY, slack, lx, ly)
        for y in range(n):
            if not inTreeY[y] and slack[y] == 0:
                if yx[y] == -1:
                    cx, cy = slackX[y], y
                    match[0] += 1
                    while cx != -2:
                        ty = xy[cx]
                        xy[cx] = cy
                        yx[cy] = cx
                        cx = prev[cx]
                        cy = ty
                    for i in range(n):
                        inTreeX[i] = inTreeY[i] = False
                    augment(cost, match, inTreeX, inTreeY, prev, xy, yx, slack, slackX, lx, ly)
                    return
                else:
                    inTreeY[y] = True
                    if not inTreeX[yx[y]]:
                        q.append(yx[y])
                        addTree(yx[y], slackX[y], inTreeX, prev, slack, slackX, lx, ly, cost)

def run_hungarian(cost):
    n = len(cost)
    # convert to maximization problem by negation
    cost_copy = cost.copy().astype(float)
    for i in range(n):
        for j in range(n):
            cost_copy[i][j] = -cost_copy[i][j]
    match = [0]
    xy = [-1] * n
    yx = [-1] * n
    lx = [0] * n
    ly = [0] * n
    slack = [0] * n
    slackX = [0] * n
    prev = [0] * n
    inTreeX = [False] * n
    inTreeY = [False] * n
    labelIt(cost_copy, lx)
    augment(cost_copy, match, inTreeX, inTreeY, prev, xy, yx, slack, slackX, lx, ly)
    # compute result (un-negate)
    total = 0
    assignments = []
    for i in range(n):
        if xy[i] != -1:
            total += cost[i][xy[i]]
            assignments.append((i, xy[i], cost[i][xy[i]]))
        else:
            assignments.append((i, -1, None))
    return total, assignments

def hungarian_rectangular(cost_rect):
    nr, nt = cost_rect.shape
    n = max(nr, nt)
    cost_pad = np.full((n, n), hi, dtype=float)
    cost_pad[:nr, :nt] = cost_rect
    total, assignments = run_hungarian(cost_pad)
    # filter only real robot rows and real task columns
    filtered = []
    for (r_idx, t_idx, c) in assignments[:nr]:
        if t_idx >= 0 and t_idx < nt and c is not None and c < hi:
            filtered.append((r_idx, t_idx, c))
        else:
            filtered.append((r_idx, -1, None))
    return filtered

# ---------------------------------------------------------------------------
# Original dynamic reallocation simulation (unchanged)
# ---------------------------------------------------------------------------

def simulate_dynamic_reallocation(robots, tasks):
    # initialize robot runtime state
    for r in robots:
        r['busy_until'] = 0.0
        r['current_task'] = None

    # task state: status: 0 = unassigned, 1 = assigned/in-progress, 2 = completed
    current_time = 0.0
    timeline = []

    # helper to get uncompleted tasks
    def remaining_tasks():
        return [t for t in tasks if t['status'] != 2]

    # main loop: continue until all tasks completed
    while any(t['status'] != 2 for t in tasks):
        # find available robots at current_time
        available_robots = [r for r in robots if r['busy_until'] <= current_time and r['battery'] >= 25]
        unassigned_tasks = [t for t in tasks if t['status'] == 0]

        # if there are available robots and unassigned tasks -> assign now
        if available_robots and unassigned_tasks:
            cost_rect = compute_cost_matrix(available_robots, unassigned_tasks)
            assign_list = hungarian_rectangular(cost_rect)
            # process assignments
            for (r_idx, t_idx, c) in assign_list:
                robot = available_robots[r_idx]
                if t_idx == -1:
                    continue  # no feasible task for this robot now
                task = unassigned_tasks[t_idx]
                # assign and compute times
                dist = euclidean_dist(robot['position'], task['coordinate'])
                travel_time = dist / v_max if v_max > 0 else 0
                service_time = task['t_ser']
                start_time = current_time + travel_time
                finish_time = start_time + service_time

                # set robot and task state
                robot['busy_until'] = finish_time
                robot['current_task'] = task['id']
                task['status'] = 1
                task['assigned_robot'] = robot['id']
                task['start_time'] = start_time
                task['finish_time'] = finish_time

                timeline.append((current_time, f"Assigned {task['id']} -> {robot['id']} (start @ {start_time:.1f}, finish @ {finish_time:.1f}, cost {c:.3f})"))
                print(f"[{current_time:.1f}] Assigned {task['id']} -> {robot['id']}; start {start_time:.1f}, finish {finish_time:.1f}, cost {c:.3f}")

        # Find next event time: earliest finish of an in-progress task
        busy_times = [r['busy_until'] for r in robots if r['busy_until'] > current_time]
        if busy_times:
            next_event = min(busy_times)
        else:
            if not any(t['status'] == 0 for t in tasks):
                break
            next_event = current_time + 0.1

        # advance time
        current_time = next_event

        # mark finished tasks and free robots
        for r in robots:
            if r['current_task'] is not None and r['busy_until'] <= current_time:
                finished_task = next((t for t in tasks if t['id'] == r['current_task']), None)
                if finished_task is not None and finished_task['status'] != 2:
                    finished_task['status'] = 2
                    r['position'] = finished_task['coordinate']
                    timeline.append((current_time, f"Completed {finished_task['id']} by {r['id']} at {current_time:.1f}"))
                    print(f"[{current_time:.1f}] Completed {finished_task['id']} by {r['id']}")
                r['current_task'] = None

    print("\nAll tasks completed. Timeline:")
    for time, event in timeline:
        print(f"[{time:.1f}] {event}")

# ---------------------------------------------------------------------------
# NEW: Build per-robot schedule from final tasks list
# ---------------------------------------------------------------------------

def build_schedule_by_robot(tasks):
    """
    Build a dict: { 'R1': [{'task_id': 'T1', 'start_time': ..., 'finish_time': ...}, ...], ... }
    """
    schedule = {}
    for t in tasks:
        rid = t.get('assigned_robot', None)
        if rid is None:
            continue
        start = float(t.get('start_time', 0.0))
        finish = float(t.get('finish_time', start))
        entry = {
            'task_id': t['id'],
            'start_time': start,
            'finish_time': finish,
        }
        schedule.setdefault(rid, []).append(entry)

    # sort by start time for each robot
    for rid in schedule:
        schedule[rid].sort(key=lambda e: e['start_time'])
    return schedule

# ---------------------------------------------------------------------------
# NEW: dynamic reallocation that uses distance_fn (A* path length)
# ---------------------------------------------------------------------------

def simulate_dynamic_reallocation_with_distance_fn(robots, tasks, distance_fn):
    """
    Same logic as simulate_dynamic_reallocation, but:
      - uses compute_cost_matrix_with_distance_fn for costs
      - uses distance_fn(robot, task) for travel_time.
    Returns a dict with timeline, robots, tasks, and per-robot schedule.
    """

    # initialize robot runtime state
    for r in robots:
        r['busy_until'] = 0.0
        r['current_task'] = None

    current_time = 0.0
    timeline = []

    while any(t['status'] != 2 for t in tasks):
        available_robots = [r for r in robots if r['busy_until'] <= current_time and r['battery'] >= 25]
        unassigned_tasks = [t for t in tasks if t['status'] == 0]

        if available_robots and unassigned_tasks:
            cost_rect = compute_cost_matrix_with_distance_fn(available_robots, unassigned_tasks, distance_fn)
            assign_list = hungarian_rectangular(cost_rect)

            for (r_idx, t_idx, c) in assign_list:
                robot = available_robots[r_idx]
                if t_idx == -1:
                    continue
                task = unassigned_tasks[t_idx]

                # distance & travel time from distance_fn
                try:
                    dist = float(distance_fn(robot, task))
                except Exception:
                    dist = q_max
                travel_time = dist / v_max if v_max > 0 else 0.0
                service_time = task['t_ser']
                start_time = current_time + travel_time
                finish_time = start_time + service_time

                robot['busy_until'] = finish_time
                robot['current_task'] = task['id']
                task['status'] = 1
                task['assigned_robot'] = robot['id']
                task['start_time'] = start_time
                task['finish_time'] = finish_time

                timeline.append(
                    (current_time,
                     f"Assigned {task['id']} -> {robot['id']} (start @ {start_time:.1f}, finish @ {finish_time:.1f}, cost {c:.3f})")
                )
                print(f"[{current_time:.1f}] Assigned {task['id']} -> {robot['id']}; "
                      f"start {start_time:.1f}, finish {finish_time:.1f}, cost {c:.3f}")

        busy_times = [r['busy_until'] for r in robots if r['busy_until'] > current_time]
        if busy_times:
            next_event = min(busy_times)
        else:
            if not any(t['status'] == 0 for t in tasks):
                break
            next_event = current_time + 0.1

        current_time = next_event

        for r in robots:
            if r['current_task'] is not None and r['busy_until'] <= current_time:
                finished_task = next((t for t in tasks if t['id'] == r['current_task']), None)
                if finished_task is not None and finished_task['status'] != 2:
                    finished_task['status'] = 2
                    r['position'] = finished_task['coordinate']
                    timeline.append(
                        (current_time, f"Completed {finished_task['id']} by {r['id']} at {current_time:.1f}")
                    )
                    print(f"[{current_time:.1f}] Completed {finished_task['id']} by {r['id']}")
                r['current_task'] = None

    print("\nAll tasks completed. Timeline:")
    for time, event in timeline:
        print(f"[{time:.1f}] {event}")

    schedule = build_schedule_by_robot(tasks)
    return {
        "timeline": timeline,
        "robots": robots,
        "tasks": tasks,
        "schedule": schedule,
    }


if __name__ == "__main__":
    # legacy Euclidean test
    simulate_dynamic_reallocation(robots, tasks)
    for t in tasks:
        print(t)
    for r in robots:
        print(r)

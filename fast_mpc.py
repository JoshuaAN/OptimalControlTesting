import math
import pylab as plt
from constants import constants
import collections
import numpy as np

def clamp(x, low, high):
    return min(high, max(low, x))

# Given an intial starting velocity and time, and ending velocity and time,
# solve the optimal velocity at the time midpoint (t0 + t1) / 2.
# TODO: add lagrange multipliers to show full extent of recursive generation since this heuristic may not work in all cases
def solve_midpoint_velocity(states, v0, v1, t0, t1, reference, depth):
    if depth == 0:
        return
    dt = (t1 - t0) / 2
    velocity = lambda x, u, t: (x - u / constants.kv) * math.exp(-constants.kv / constants.ka * t) + u / constants.kv
    min_velocity = velocity(v0, -12, dt)
    max_velocity = velocity(v0, 12, dt)
    v = clamp(reference, min_velocity, max_velocity)
    states[(t0 + t1) / 2] = v
    solve_midpoint_velocity(states, v0, v, t0, (t0 + t1) / 2, reference, depth - 1)
    solve_midpoint_velocity(states, v, v1, (t0 + t1) / 2, t1, reference, depth - 1)

def solve_trajectory(initial_velocity, reference):
    T = 5
    search_depth = 7
    states = {0:initial_velocity,T:reference}
    solve_midpoint_velocity(states, initial_velocity, reference, 0, T, reference, search_depth)
    sorted_states = collections.OrderedDict(sorted(states.items(), key=lambda kv: kv[0]))
    # TODO: make key search more efficient b/c this seems stupid
    items = []
    keys = []
    for item in sorted_states.items():
        keys.append(item[0])
        items.append(item[1])
    plt.plot(keys, items, "--", label="Fast MPC")
    plt.legend()
# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
# Modified by Rahul Kunji (rahulsk2@illinois.edu) on 01/16/2019

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""

from maze import *
import queue as queue
import heapq
from collections import defaultdict
import math
from itertools import permutations
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,greedy,astar)

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "dfs": dfs,
        "greedy": greedy,
        "astar": astar,
    }.get(searchMethod)(maze)

def bfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    # getNeighbors(maze, row, col) --> give us coordinates to place into the queue/path
    # getStart(maze) --> gives us first coordinates
    start = maze.getStart()
    q = queue.Queue()
    num_states_explored = 0
    visited = []
    prev = {}
    q.put(start)
    prev[start] = None          # no previous for starting coordinate
    visited.append(start)
    dot_coord = None            # coordinate of dot (objective)
    while q.empty() is not True:
        v = q.get()
        if maze.isObjective(v[0], v[1]):   # check if dot is found
            dot_coord = v                   # update coordinate of dot
            break
        for i in maze.getNeighbors(v[0], v[1]):
            if i not in visited:
                q.put(i)
                prev[i] = v    # set previous node for each visited node
                visited.append(i)

    path = get_path(dot_coord, prev)
    num_states_explored = len(visited)
    return path, num_states_explored

def dfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    # getNeighbors(maze, row, col) --> give us coordinates to place into the queue/path
    # getStart(maze) --> gives us first coordinates
    start = maze.getStart()
    stack = []
    num_states_explored = 0
    visited = []
    prev = {}
    stack.append(start)
    prev[start] = None          # no previous for starting coordinate
    visited.append(start)
    dot_coord = None            # coordinate of dot (objective)
    while len(stack):
        v = stack.pop()
        if maze.isObjective(v[0], v[1]):    # check if dot is found
            dot_coord = v                   # update coordinate of dot
            break
        for i in maze.getNeighbors(v[0], v[1]):
            if i not in visited:
                stack.append(i)
                prev[i] = v    # set previous node for each visited node
                visited.append(i)

    path = get_path(dot_coord, prev)
    num_states_explored = len(visited)
    return path, num_states_explored

def greedy(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    start = maze.getStart()
    dot_coord = maze.getObjectives()[0] # only 1 dot, so first entry is correct
    start_node = (manhattan(start, dot_coord), start)
    h = []
    h.append(start_node)
    heapq.heapify(h)    # priority queue initialized
    # hold parents dictionary: if key exists, node has been visited. val is parent
    parents = {}
    parents[start] = None
    while h:
        min = heapq.heappop(h)[1]
        if maze.isObjective(min[0], min[1]):
            break
        for i in maze.getNeighbors(min[0], min[1]):
            if i not in parents.keys():
                add_node = (manhattan(i, dot_coord), i)
                heapq.heappush(h, add_node)
                parents[i] = min

    path = get_path(dot_coord, parents)
    num_states_explored = len(parents)
    return path, num_states_explored

def astar(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    if len(maze.getObjectives()) > 1:
        return mult_astar(maze)

    start = maze.getStart()
    dot_coord = maze.getObjectives()[0] # only 1 dot, so first entry is correct
    closedSet = []
    openSet = []
    start_node = (manhattan(start, dot_coord), start)
    openSet.append(start_node)
    heapq.heapify(openSet)
    parents = {}
    parents[start] = None
    gScore = defaultdict(lambda: float('inf'))
    gScore[start] = 0
    fScore = defaultdict(lambda: float('inf'))
    fScore[start] = manhattan(start, dot_coord)
    while len(openSet) != 0:
        curr = heapq.heappop(openSet)[1]
        if maze.isObjective(curr[0], curr[1]):
            break
        closedSet.append(curr)
        for i in maze.getNeighbors(curr[0], curr[1]):
            if i in closedSet:
                continue
            tentative_gScore = gScore[curr] + manhattan(curr, i)
            if i not in openSet:
                parents[i] = curr
                gScore[i] = tentative_gScore
                fScore[i] = gScore[i] + manhattan(i, dot_coord)
                add_node = (fScore[i], i)
                openSet.append(add_node)
                heapq.heapify(openSet)
            elif tentative_gScore >= gScore[i]:
                continue;

    path = get_path(dot_coord, parents)
    num_states_explored = len(parents)
    return path, num_states_explored

def mult_astar(maze):
    dots = maze.getObjectives()
    perm = permutations(dots)
    min_dist = math.inf
    min_path = []
    min_num_states = 0
    for i in perm:
        i = list(i)
        path, length, num_states_explored = mult_astar_helper(maze, i)
        if min_dist > length:
            min_path = path
            min_num_states = num_states_explored
    return min_path, num_states_explored

def mult_astar_helper(maze, dot_list):
    dot_idx = 0
    dot_coord = dot_list[dot_idx]
    start = maze.getStart()
    closedSet = []
    openSet = []
    start_node = (manhattan(start, dot_coord), start)
    openSet.append(start_node)
    heapq.heapify(openSet)
    parents = {}
    parents[start] = None
    gScore = defaultdict(lambda: float('inf'))
    gScore[start] = 0
    fScore = defaultdict(lambda: float('inf'))
    fScore[start] = manhattan(start, dot_coord)
    path = []
    while len(openSet) != 0:
        curr = heapq.heappop(openSet)[1]
        if (curr[0], curr[1]) == dot_coord:
            path.extend(get_path(dot_coord, parents))
            parents[dot_coord] = None   # so get_path has a new spot to stop at
            dot_idx += 1
            if dot_idx == len(dot_list)-1:
                break
            dot_coord = dot_list[dot_idx]
        closedSet.append(curr)
        for i in maze.getNeighbors(curr[0], curr[1]):
            if i in closedSet:
                continue
            tentative_gScore = gScore[curr] + manhattan(curr, i)
            if i not in openSet:
                parents[i] = curr
                gScore[i] = tentative_gScore
                fScore[i] = gScore[i] + manhattan(i, dot_coord)
                add_node = (fScore[i], i)
                openSet.append(add_node)
                heapq.heapify(openSet)
            elif tentative_gScore >= gScore[i]:
                continue;

    #path = get_path(dot_coord, parents)
    num_states_explored = len(parents)
    return path, len(path), num_states_explored

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_path(dot, parents):
    path = []
    path.append(dot)
    p = parents[dot]
    while p is not None:
        path.append(p)
        p = parents[p]
    path = path[::-1]
    return path

"""
returns closest dot to param node start
"""
def closest_obj(start, objs):
    min = math.inf
    ret = None
    for obj in objs:
        dist = manhattan(start, obj)
        if min > dist:
            min = dist
            ret = obj
    return ret

def numNodes(maze):
    num = maze.getDimensions()[0]*maze.getDimensions()[1]

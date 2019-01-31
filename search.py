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
import Queue as queue
from pythonds.basic.stack import Stack
import heapq
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

    path = []
    path.append(dot_coord)
    p = prev[dot_coord]
    while p is not None:
        path.append(p)
        p = prev[p]
    path = path[::-1]
    num_states_explored = len(visited)
    return path, num_states_explored

def dfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    # getNeighbors(maze, row, col) --> give us coordinates to place into the queue/path
    # getStart(maze) --> gives us first coordinates
    start = maze.getStart()
    stack = Stack()
    num_states_explored = 0
    visited = []
    prev = {}
    stack.push(start)
    prev[start] = None          # no previous for starting coordinate
    visited.append(start)
    dot_coord = None            # coordinate of dot (objective)
    while stack.isEmpty() is not True:
        v = stack.pop()
        if maze.isObjective(v[0], v[1]):    # check if dot is found
            dot_coord = v                   # update coordinate of dot
            break
        for i in maze.getNeighbors(v[0], v[1]):
            if i not in visited:
                stack.push(i)
                prev[i] = v    # set previous node for each visited node
                visited.append(i)

    path = []
    path.append(dot_coord)
    p = prev[dot_coord]
    while p is not None:
        path.append(p)
        p = prev[p]
    path = path[::-1]
    num_states_explored = len(visited)
    return path, num_states_explored

def greedy(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    start = maze.getStart()
    dot_coord = maze.getObjectives()[0] # only 1 dot, so first entry is correct
    start_node = (start, manhattan(start, dot_coord))
    h = []
    h.append(start_node)
    heapq.heapify(h)    # priority queue initialized
    # hold parents dictionary: if key exists, node has been visited. val is parent
    parents = {}
    parents[start] = None
    while h:
        min = heapq.heappop(h)[0]
        if maze.isObjective(min[0], min[1]):
            break
        for i in maze.getNeighbors(min[0], min[1]):
            if i not in parents.keys():
                add_node = (i, manhattan(i, dot_coord))
                heapq.heappush(h, add_node)
                parents[i] = min
    path = []
    path.append(dot_coord)
    p = parents[dot_coord]
    while p is not None:
        path.append(p)
        p = parents[p]
    path = path[::-1]
    num_states_explored = len(parents)
    return path, num_states_explored

def astar(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    return [], 0

def manhattan(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

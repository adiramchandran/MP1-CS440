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
    """
    // A* Search Algorithm
    1.  Initialize the open list
    2.  Initialize the closed list
        put the starting node on the open
        list (you can leave its f at zero)

    3.  while the open list is not empty
        a) find the node with the least f on
           the open list, call it "q"

        b) pop q off the open list

        c) generate q's 8 successors and set their
           parents to q

        d) for each successor
            i) if successor is the goal, stop search
              successor.g = q.g + distance between
                                  successor and q
              successor.h = distance from goal to
              successor (This can be done using many
              ways, we will discuss three heuristics-
              Manhattan, Diagonal and Euclidean
              Heuristics)

              successor.f = successor.g + successor.h

            ii) if a node with the same position as
                successor is in the OPEN list which has a
               lower f than successor, skip this successor

            iii) if a node with the same position as
                successor  is in the CLOSED list which has
                a lower f than successor, skip this successor
                otherwise, add  the node to the open list
         end (for loop)

        e) push q on the closed list
        end (while loop)
    """
    start = maze.getStart()
    dot_coord = maze.getObjectives()[0] # only 1 dot, so first entry is correct
    closedSet = []
    openSet = []
    start_node = (start, manhattan(start, dot_coord))
    openSet.append(start_node)
    heapq.heapify(openSet) 
    parents = {}
    parents[start] = None
    gScore = defaultdict(lambda: float('inf'))
    gScore[start] = 0
    fScore = defaultdict(lambda: float('inf'))
    fScore[start] = manhattan(start, dot_coord)
    while len(openSet) != 0:
        curr = heapq.heappop(openSet)[0]
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
                add_node = (i, fScore[i])
                openSet.append(add_node)
                heapq.heapify(openSet)
            elif tentative_gScore >= gScore[i]:
                continue;

    path = []
    path.append(dot_coord)
    p = parents[dot_coord]
    while p is not None:
        path.append(p)
        p = parents[p]
    path = path[::-1]
    num_states_explored = len(parents)
    return path, num_states_explored

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


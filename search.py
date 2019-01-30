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

"""
    # Function to print a BFS of graph
    def BFS(self, s):

        # Mark all the vertices as not visited
        visited = [False] * (len(self.graph))

        # Create a queue for BFS
        queue = []

        # Mark the source node as
        # visited and enqueue it
        queue.append(s)
        visited[s] = True

        while queue:

            # Dequeue a vertex from
            # queue and print it
            s = queue.pop(0)
            print (s, end = " ")

            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it
            for i in self.graph[s]:
                if visited[i] == False:
                    queue.append(i)
                    visited[i] = True
https://crab.rutgers.edu/~guyk/BFS.pdf
"""
def bfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    # getNeighbors(maze, row, col) --> give us coordinates to place into the queue/path
    # getStart(maze) --> gives us first coordinates
    start = getStart(maze)
    q = queue.Queue()
    num_states_explored = 0
    visited = []
    prev = []
    q.put(start)
    prev[start] = None          # no previous for starting coordinate
    visited.append(start)
    dot_coord = None            # coordinate of dot (objective)
    while q.empty() is not True:
        v = q.get()
        if isObjective(maze, v[0], v[1]):   # check if dot is found
            dot_coord = v                   # update coordinate of dot
            break
        for i in getNeighbors(maze, v[0], v[1]):
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
    return [], 0


def greedy(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    return [], 0


def astar(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    return [], 0

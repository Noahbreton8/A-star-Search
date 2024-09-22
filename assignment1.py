import heapq #in standard library
import pandas as pd

class Point:
    def __init__(self, pointType, heuristic):
        self.parent = (-1, -1)
        self.h = heuristic
        self.path_cost = float('inf') #start at infinity so first path cost added will be lower
        self.type = pointType

    def getPriorityValue(self):
        return self.path_cost + self.h
    
    #possible to use for heapq
    def __lt__(self, other):
        return self.getPriorityValue() < other.getPriorityValue()
    
    #for debugging purposes
    def __str__(self):
        return f"My heuristic is {self.h}, my path cost is {self.path_cost}, my type is {self.type}, my parent is {self.parent}"

#Returns the grid from the csv file as a 2D array
def readFile(filepath):
    return pd.read_csv(filepath, header=None).values

#Going to use the manhattan distance as the heuristic since we cannot move diagonally
def addHeuristic(grid, goal):
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            heuristicDistance = abs(row - goal[0]) + abs(col - goal[1])
            pointType = grid[row][col]
            grid[row][col] = Point(pointType, heuristicDistance)

    return grid

def findStartAndGoal(grid):
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            if grid[row][col] == "S":
                start = (row, col)
            elif grid[row][col] == "G":
                goal = (row, col)

    return start, goal

def getNeighbours(grid, currPosition):
    directions = [(1,0), (0,1), (0,-1), (-1, 0)]

    for direction in directions:
        #get valid direction
        #check if in bounds
        #check for doors and how many keys it has
        #maybe pass the number of keys into the function
        pass


# The pathfinding function must implement A* search to find the goal state
def pathfinding(filepath):
    grid = readFile(filepath)
    start, goal = findStartAndGoal(grid)
    grid = addHeuristic(grid, goal)

    # optimal_path is a list of coordinate of squares visited (in order)
    optimal_path = []
    # optimal_path_cost is the cost of the optimal path
    optimal_path_cost = 0
    # num_states_explored is the number of states explored during A* search
    num_states_explored = 0

    #basic graph search from lecture, change it to use heuristics and other
    frontier = [start]
    explored = []

    while True:
        if frontier == []:
            return -1, -1, -1
        
        #use heapq
        leaf = heapq.heappop(frontier)

        if leaf == goal:
            break

        explored.append(leaf)
        for node in getNeighbours(grid, leaf):

            curr_path_cost = leaf.path_cost + 1 #weight between two nodes will always be one
            if (node not in frontier and node not in explored) or (node in frontier and curr_path_cost < node.path_cost):
                node.parent = leaf
                node.path_cost = curr_path_cost

                #this part is not complete yet, just an idea
                #use heapq
                priority_value = grid[node[0]][node[1]].getPriorityValue()
                heapq.heappush(frontier, (priority_value, node[0], node[1]))

    return optimal_path, optimal_path_cost, num_states_explored

pathfinding('Examples/Example0/grid.csv')
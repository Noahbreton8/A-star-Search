import heapq #in standard library
import copy #in standard library
import pandas as pd

class State:
    def __init__(self, grid, heuristic, curr_location, num_keys = 0, path_cost = 0, path = []):
        self.path = path + [curr_location]
        self.h = heuristic
        self.path_cost = path_cost
        self.grid = grid
        self.location = curr_location
        self.num_keys = num_keys

    def getPriorityValue(self):
        return self.path_cost + self.h
    
    def __eq__(self, other):
        return self.location == other.location and self.num_keys == other.num_keys
    
    def __lt__(self, other):
        # Uncomment if we want to order by heuristic (distance to goal) when they're tied
        # if self.getPriorityValue() == other.getPriorityValue():
        #     return self.h < other.h
        return self.getPriorityValue() < other.getPriorityValue()
    
    #for debugging purposes
    def __str__(self):
        return f"My heuristic is {self.h}, my path cost is {self.path_cost}, my path is {self.path}"
    
#Returns the grid from the csv file as a 2D array
def readFile(filepath):
    return pd.read_csv(filepath, header=None).values

def findStartAndGoal(grid):
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            if grid[row][col] == "S":
                start = (row, col)
            elif grid[row][col] == "G":
                goal = (row, col)

    return start, goal

def getHeuristic(curr_position, goal):
    return abs(curr_position[0] - goal[0]) + abs(curr_position[1] - goal[1])

def valid_point(point, row_length, col_length):
    if point[0] < 0 or point[1] < 0 or point[0] >= row_length or point[1] >= col_length:
        return False
    return True

def getNeighbours(grid, curr_location, has_key):
    directions = [(1,0), (0,1), (0,-1), (-1, 0)]

    neighbours = []

    for direction in directions:
        new_point = tuple(a + b for a, b in zip(direction, curr_location))
        if valid_point(new_point, len(grid), len(grid[0])) and (grid[new_point[0]][new_point[1]] != 'D' or has_key):
            neighbours.append(new_point)
    
    return neighbours


# The pathfinding function must implement A* search to find the goal state
def pathfinding(filepath):
    start_grid = readFile(filepath)
    start, goal = findStartAndGoal(start_grid)

    # optimal_path is a list of coordinate of squares visited (in order)
    optimal_path = []
    # optimal_path_cost is the cost of the optimal path
    optimal_path_cost = 0
    # num_states_explored is the number of states explored during A* search
    num_states_explored = 0

    start_heuristic = getHeuristic(start, goal)
    start_state = State(start_grid, start_heuristic, start)

    #basic graph search from lecture, change it to use heuristics and other
    frontier = [start_state]
    explored = set() #for easy lookup

    while True:
        if frontier == []:
            #there should be a solution
            return False
        
        #use heapq to pop off best priority
        #leaf = State object
        leaf = heapq.heappop(frontier)
        num_states_explored += 1

        if (leaf.location) == goal:
            optimal_path_cost = leaf.path_cost
            optimal_path = leaf.path
            break

        explored.add(leaf.location + (leaf.num_keys, ))

        curr_grid = leaf.grid

        for node in getNeighbours(leaf.grid, leaf.location, leaf.num_keys > 0):
            
            new_grid = copy.deepcopy(curr_grid)
            new_grid[node[0]][node[1]] = 'O'

            new_heuristic = getHeuristic(node, goal)
            
            curr_keys = leaf.num_keys
            if curr_grid[node[0]][node[1]] == 'K':
                curr_keys += 1
            elif curr_grid[node[0]][node[1]] == 'D':
                curr_keys -= 1
            
            new_path_cost = leaf.path_cost + 1 #weight between two nodes will always be one
            
            new_state = State(new_grid, new_heuristic, node, curr_keys, new_path_cost, leaf.path)

            same_state = False
            for state in frontier:
                if new_state == state and new_path_cost < state.path_cost:
                    same_state = True
                    del new_state
                    new_state = state
                    #Since the path we're taking right now is best update the path cost and path
                    new_state.path_cost = new_path_cost 
                    new_state.path = leaf.path + [node]
                    break

            if same_state or (new_state not in frontier and node + (curr_keys, ) not in explored):
                heapq.heappush(frontier, new_state)

    return optimal_path, optimal_path_cost, num_states_explored

print(pathfinding('Examples/Example0/grid.csv'))
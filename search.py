# Basic searching algorithms
import heapq
from math import inf

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h=0, parent=None):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = 0         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = 0      # total cost (depend on the algorithm)
        self.parent = parent    # previous node

    def __lt__(self, other):
        return self.cost < other.cost

def calcost(goal, node):
    return abs(goal.row - node.row) + abs(goal.col - node.col)


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    #declaring a a queue, using a heapstack 
    q = []
    visited = set() #visited nodes set
    #creating a start and goal node
    start_node = Node(start[0], start[1], grid[start[0]][start[1]], 0)
    goal_node = Node(goal[0], goal[1], grid[goal[0]][goal[1]])
    start_node.cost = 0
    start_node.g = 0
    
    heapq.heappush(q, (start_node.g, start_node))#pushing the start node to the heap
    
    goal_node.g = inf
    goal_node.cost = inf
    
    #Order for exploration 
    
    row = [0, 1, 0, -1]
    col = [1, 0, -1, 0]
    

    while len(q) > 0:
        #if the start or goal are obstacle
        if (start_node.is_obs or goal_node.is_obs):
            break 
        #if the start and the goal node are the same 
        if (start_node.row == goal_node.row and start_node.col == start_node.row):
            found = True
            break
        currnode = heapq.heappop(q)[1]
        if (currnode.row, currnode.col) in visited:
            continue    
        visited.add((currnode.row, currnode.col))
        steps += 1
        
        if currnode.row == goal_node.row and currnode.col == goal_node.col:
            goal_node.g = currnode.g
            goal_node.parent = currnode.parent
            found = True
            break
        for i in range(len(row)):
            #To check if the position is not valid or is not in the grid
            if not (0 <= currnode.row + row[i] < len(grid) and 0 <= currnode.col + col[i] < len(grid[0])and grid[currnode.row + row[i]][currnode.col + col[i]] == 0):
                continue 
            #creating nodes for the neghibours
            negh = Node(currnode.row + row[i], currnode.col + col[i], grid[currnode.row + row[i]][currnode.col + col[i]], 0)
            if grid[negh.row][negh.col] == 1 or (negh.row, negh.col) in visited:
                continue
            #calculating the cost for travelling, adding 1 as the cost to travel to neghibouring nodes
            negh.g = currnode.g + 1
            negh.cost = negh.g
            negh.parent = currnode
            heapq.heappush(q,(negh.g, negh))
                  
    if found:
        node = goal_node
        while node is not None:
            path.insert(0, [node.row, node.col])
            node = node.parent
            
        #print(path)
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    #declaring a a queue, using a heapstack 
    q = []
    visited = set() #visited nodes set
    #creating a start and goal node
    
    start_node = Node(start[0], start[1], grid[start[0]][start[1]], 0)
    goal_node = Node(goal[0], goal[1], grid[goal[0]][goal[1]])
    start_node.cost = 0
    start_node.g = 0
    heapq.heappush(q, (start_node.cost, start_node))#pushing the start node into the heap
    
    goal_node.g = inf
    goal_node.cost = inf
    
    #exploration of neghibours
    row = [0, 1, 0, -1]
    col = [1, 0, -1, 0]
    
    while q:
        #if the start or goal are obstacle
        if (start_node.is_obs or goal_node.is_obs):
            break 
        #if the start and the goal node are the same 
        if (start_node.row == goal_node.row and start_node.col == start_node.row):
            found = True
            break
        currnode = heapq.heappop(q)[1]
        if (currnode.row, currnode.col) in visited:
            continue    
        visited.add((currnode.row, currnode.col))
        steps += 1
        if currnode.row == goal_node.row and currnode.col == goal_node.col:
            goal_node.g = currnode.g
            goal_node.parent = currnode.parent
            found = True
            break
        for i in range(len(row)):
            if not (0 <= currnode.row + row[i] < len(grid) and 0 <= currnode.col + col[i] < len(grid[0])and grid[currnode.row + row[i]][currnode.col + col[i]] == 0):
                continue
            negh = Node(currnode.row + row[i], currnode.col + col[i], grid[currnode.row + row[i]][currnode.col + col[i]], 0)
            if 0 <= negh.row < len(grid) and 0 <= negh.col < len(grid[0]):
                if grid[negh.row][negh.col] == 1 and (negh.row, negh.col) in visited:
                    continue    
                #for A* the heuristics are also calculated. The total cost is g+h
                negh.g = currnode.g + 1
                negh.h = calcost(negh,goal_node)
                negh.cost = negh.g+negh.h
                negh.parent = currnode
                heapq.heappush(q,(negh.cost, negh))
                
                
    
    if found:
        node = goal_node
        while node is not None:
            path.insert(0, [node.row, node.col])
            node = node.parent
        #print(path)
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()

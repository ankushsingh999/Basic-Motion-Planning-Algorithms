# Basic-Motion-Planning-Algorithms
Implemented Dijkstra and A* algorithms with Python3. 

* Dijkstra:

The Dijkstra algorithm is essentially a BFS algorithm but with weights. The algorithm uses
the attribute “cost to travel to other node” as the base line to get path. It choses the least
weighted path to reach the goal

* A* :

A* is a similar algorithm to the Dijkstra, except for calculating the cost it takes in to
consideration the cost to come and also the heuristic too. The heuristic taken in the algorithm
is the Manhattan distance. The nodes are sorted by f(x):
f(x) = g(x)+ h(x)


Files included:

• search.py is the file where you will find the implemented algorithms.

• main.py is the script that provides helper functions that load the map
from csv files and visualize the map and path.

• map.csv is the map file you could modify to create your own map.

• test_map.csv restores a test map for doc test purpose only. Do not
modify this file.
* Note : The order “right, down, left, up” to explore the nearby nodes in the map, which means “[0, +1], [+1, 0], [0, -1], [-1, 0]” in coordinates.

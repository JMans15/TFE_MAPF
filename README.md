# TFE_MAPF

## Files *.cpp and c.h
<ul>
<li> Parser: Takes a map file and returns a Graph instance. </li>
<li> Graph: Represents a graph with adjacency lists. </li>
<li> Problem: Represents an MAPF problem with a graph, a list of start positions, a list of target positions and an objective function. </li>
<li> library: Contains a single function aStarSearch() which takes a Problem and returns a Solution to that problem. </li>
<li> State: Represents a state for the A* search. A state = (the positions for every agent (unassigned agents AND assigned agents) + the time)</li>
<li> Node: Node class used in the A* search. A node englobe a state with more information like the cost g(n), the parent node, ...</li>
<li> Solution: A class to represent a solution to a MAPF solution. Contains the cost of the solution and the position of each agent at each time, ... </li>
<li> test: Small file with very simple tests with the "pipeline". </li>
</ul>

# TFE_MAPF

## Files *.cpp and *.h
<ul>
<li> Parser: Takes a map file and returns a Graph instance. </li>
<li> Graph: Represents a graph with adjacency lists. </li>
<li> Problem: Abstract class for a problem. Used in the A* search.</li>
<li> SingleAgentProblem: Daughter class of Problem. Simplest problem with a single agent, its start position and its target position. </li>
<li> MultiAgentProblem: Daughter class of Problem. Problem with several agents. Contains a list of start positions, a list of target positions, an objective function and an optional set of constraints (a,p,t). </li>
<li> Heuristic: File containing the heuristics : an abstract class for a heuristic used in the A* search and its daughter classes. </li>
<li> library: Contains a single function aStarSearch() which takes a Problem and a Heuristic and returns a Solution to that problem. </li>
<li> State: Abstract class for a state in the A* search. Used in the A* search. </li>
<li> SingleAgentState: Daughter class of State. A state = (the position of the agent) </li>
<li> MultiAgentState: Daughter class of State. A state = (the positions for every agent (unassigned agents AND assigned agents)) </li>
<li> Node: Node class used in the A* search. A node englobes a state with more information like the cost g(n), the parent node, ...</li>
<li> Solution: A class to represent a solution to a MAPF solution. Contains the cost of the solution and the position of each agent at each time, ... </li>
<li> test: Small file with very simple tests with the "pipeline". </li>
</ul>

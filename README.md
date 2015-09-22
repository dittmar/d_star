# d_star
Text-based visualization of the D* pathfinding algorithm.
This application simply uses the D* pathfinding algorithm and prints out a grid at each step for the world.

Word of Warning:
Do not run a map like map5.  My implementation of the D* algorithm does not handle maps like map5 well.  You will get a
text-file output with a size measured in gigabytes by the time the program runs out of heap space.  The problem probably
lies in using an ArrayList to store Nodes and node information along with the sheer number of calculations D* tries to do
on a grid that size when a path between the start and goal is totally blocked by unforeseen obstacles.

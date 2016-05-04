# Experimental Robotics: Team Sketch
## Usage
Copy these files into your directory for Homework 3, replacing the original homework code. Make sure to copy the KUKA model into the specs directory, and to create a symlink to that the directory in your Homework 3 directory.

Build the code using: 
 ```$ sh make_rel.sh```.
 
 Run the simulation with: 
 ```$ ./scl_orientation_control```
 
 Run the simulation and a python client that sketches the expected output using: ```$ python client.py```.
 
 ## Graph Format 
 
 This simulation creates sketch created by a directed graph data structure. One can specify the graph to sketch by editing ``graph_representation.txt``. The nodes in the graph are specified line by line
 
 ```python
 number_of_nodes_in_graph
 Node_id_1 x_1 y_1 number_of_neighbors_1 first_neighbor_id second_neighbor_id ...
 Node_id_2 x_2 y_2 number_of_neighbors_2 first_neighbor_id second_neighbor_id ...
 ...
 ```
 
 **For now, keep x and y of each node between -.2 and .2 to avoid moving out of the robot workspace.**
 
 ## Todo
 Currently the simulating does not account for nodes with multiple neighborsIt also ignores user-specified node ids, and just increments the node id in order





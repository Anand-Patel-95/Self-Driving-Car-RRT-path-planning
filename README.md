# Self-Driving-Car-RRT-path-planning
Rapidly-exploring random tree algorithm for path planning an autonomous car, with vehicle dynamics, around static obstacles. 

1) Visualization code by me, executed in the same code as algorithm.

2) Main code is RRT_anand_V8.m, execute in matlab section by section

3) In the RESULTS images:
Blue 'x' are sample points.
Green circle is start node, Magenta circle is goal region, Black circles are obstacles.
Black lines are edges to nodes, red line is path found.          

How it works: 
1. Pick a random node q_rand.
2. Find the closest node q_near from nodes list to branch out from
towards q_rand.
3. Move from q_near towards q_rand: interpolate if node is too far away,
reach q_new. Check for collisions.
4. Update cost of reaching q_new from q_near, Cmin. q_near
acts as the parent node of q_new.
5. Add q_new to node list.
6. Continue until maximum number of samples is reached or goal region is
entered.


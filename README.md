This project was done as part of research internship in NUS Singapore. It focuses on multi agent exploration with limited field of view sensors.

The work was developed using the following resources as reference as cited - 
Nazif, A.N., Davoodi, A., Pasquier, P. (2010). Multi-Agent Area Coverage Using a Single Query Roadmap: A Swarm Intelligence Approach. In: Bai, Q., Fukuta, N. (eds) Advances in Practical Multi-Agent Systems. Studies in Computational Intelligence, vol 325. Springer, Berlin, Heidelberg. https://doi.org/10.1007/978-3-642-16098-1_7

OVERVIEW of WMARRT (Weighted Multi Agent RRT)

The approach to the environment coverage problem, presented in this paper, can be divided into two processes. Firstly, a special roadmap called WMA-RRT is constructed and embedded in the environment to discretely represent the free space. Secondly, WMA-RRT is distributed among all the agents. Each agent
independently bears the responsibility of locally traversing its associated part of the WMA-RRT roadmap.The agents employ the WMA-RRT both as a roadmap and an information ex-change channel. This channel is used by the agents to interact with the environ-ment and indirectly communicate to one another. In the other words, the agents
change the situation of the environment by altering the states of the edges and nodes of the roadmap while traversing the roadmap tree. Meanwhile, this information is used by the agents in order to decide which action to take, where togo next, how to return to the origin, how to support other working agents and
handle the agents’ failures during the operation

LOGIC AND EXPLANATION

Tree Structure:

The algorithm builds a tree structure (WMARRT) to represent the explored space.
Nodes (WMARRTNode) represent positions in the environment.
Edges connect nodes, representing paths between positions.


Exploration Strategy:

The algorithm starts with a main root and creates secondary roots for each agent.
Agents are assigned to different branches (secondary roots) to explore.
The tree is grown by randomly sampling points and connecting them to the nearest existing node.


Path Planning:

Uses the RRT (Rapidly-exploring Random Tree) algorithm for path planning.
New nodes are added by steering from the nearest existing node towards a randomly sampled point.


Collision Detection:

Uses Shapely library for geometric operations.
Checks for collisions between planned paths and obstacles or unexplored areas.


Node Evaluation:

Nodes are evaluated based on the amount of unexplored area nearby and the number of agents exploring the branch.
Uses a penalty system to balance exploration between branches.


Branch Completion:

Tracks the completion of branches and reassigns agents when a branch is fully explored.


Math Used:

Vector operations for steering and distance calculations.
Euclidean distance for finding nearest nodes.
Convex hull and centroid calculations for determining the main root position.


Randomization:

Uses random sampling for tree growth and exploration, typical in RRT algorithms.


MATHEMATICS


1. Euclidean Distance Calculation:

    Purpose: To find the nearest node in the tree to a randomly sampled point and to steer from one node to another.
    Formula:
    distance=(x2−x1)2+(y2−y1)2
    distance=(x2​−x1​)2+(y2​−y1​)2

    ​
    Code Usage: In get_nearest_vertex, this formula is used to calculate the distance between a sampled point and all existing nodes in the tree, selecting the node with the smallest distance.

2. Steering Logic:

    Purpose: To create a new node in the direction of the sampled point but within a maximum step size to ensure smooth exploration.
    Logic and Equation:
        Find the vector pointing from the current node to the sampled point.
        direction=(samplex−currentx,sampley−currenty)
        direction=(samplex​−currentx​,sampley​−currenty​)
        Calculate the length of this vector.
        length=directionx2+directiony2
        length=directionx2​+directiony2​
3. Convex Hull Calculation:

    Purpose: To determine the boundary encompassing all agents' starting positions, ensuring that the exploration starts from a common central area.
    Logic:
        Use the Convex Hull algorithm to find the smallest convex polygon that contains all starting points.
        Compute the centroid of this polygon to find the main root.
    Code Usage: In find_main_root, the ConvexHull from scipy.spatial is used to find the boundary around the agents, and the centroid of the hull points is calculated to serve as the main root for exploration.

4. Branch Allocation and Management:

    Purpose: To allocate branches of the exploration tree to different agents to maximize coverage.
    Logic:
        Each agent is assigned a secondary root (branch) based on their proximity to free cells within the environment.
        The algorithm ensures that agents do not collide by assigning branches that do not intersect with known obstacles.
    Code Usage: assign_secondary_roots method uses the Euclidean distance to distribute agents around the main root. allocate_branch and deallocate_branch track which agents are exploring which branches.



RESULTS
ENV 1
![0_1_samples](https://github.com/user-attachments/assets/58e91529-5e13-4bea-9c3d-9b1ecf9f6204)
![0_18_samples](https://github.com/user-attachments/assets/78f377d8-5696-4d88-8167-4c3ca76a77d2)
![0_100_samples](https://github.com/user-attachments/assets/aba45076-a99e-4e7d-9e08-f602db67d8c3)



ENV 2
![1_1_samples](https://github.com/user-attachments/assets/c1f4fce6-d8c3-41b3-8ec7-d47a84fdd5ba)
![1_4_samples](https://github.com/user-attachments/assets/8d266059-275c-40cb-980f-06cd46fb9f6c)
![1_7_samples](https://github.com/user-attachments/assets/d51621da-c454-447b-92e8-f9725f432452)
![1_18_samples](https://github.com/user-attachments/assets/4b9c579e-c42b-40b5-9213-aebe660a9a19)





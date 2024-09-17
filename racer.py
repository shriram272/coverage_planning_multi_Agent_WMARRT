# import imageio
# import csv
# import os
# import numpy as np
# import random
# import shapely.geometry
# import matplotlib.pyplot as plt
# from NBVP_env import Env
# from test_parameter import *

# gifs_path = f'results/mcts'

# def check_collision(start, end, robot_belief):
#     collision = False
#     line = shapely.geometry.LineString([start, end])

#     sortx = np.sort([int(start[0]), int(end[0])])
#     sorty = np.sort([int(start[1]), int(end[1])])

#     # print(robot_belief.shape)
#     robot_belief = robot_belief[sorty[0]:sorty[1] + 1, sortx[0]:sortx[1] + 1]

#     occupied_area_index = np.where(robot_belief == 1)
#     occupied_area_coords = np.asarray(
#             [occupied_area_index[1] + sortx[0], occupied_area_index[0] + sorty[0]]).T
#     unexplored_area_index = np.where(robot_belief == 127)
#     unexplored_area_coords = np.asarray(
#             [unexplored_area_index[1] + sortx[0], unexplored_area_index[0] + sorty[0]]).T
#     unfree_area_coords = occupied_area_coords

#     # obstacles = []
#     for i in range(unfree_area_coords.shape[0]):
#         coords = ([(unfree_area_coords[i][0] -5, unfree_area_coords[i][1] -5),
#                (unfree_area_coords[i][0] + 5, unfree_area_coords[i][1] -5),
#                (unfree_area_coords[i][0] - 5, unfree_area_coords[i][1] + 5),
#                (unfree_area_coords[i][0] + 5, unfree_area_coords[i][1] + 5)])
#         obstacle = shapely.geometry.Polygon(coords)
#         if abs(end[0] - unfree_area_coords[i][0] <= 8) and abs(end[1] - unfree_area_coords[i][1] <= 8):
#             collision = True
#         if not collision:
#             collision = line.intersects(obstacle)
#         if collision:
#             break

#     if not collision:
#         unfree_area_coords = unexplored_area_coords
#         for i in range(unfree_area_coords.shape[0]):
#             coords = ([(unfree_area_coords[i][0], unfree_area_coords[i][1]),
#                        (unfree_area_coords[i][0] + 1, unfree_area_coords[i][1]),
#                        (unfree_area_coords[i][0], unfree_area_coords[i][1]),
#                        (unfree_area_coords[i][0] + 1, unfree_area_coords[i][1] + 1)])
#             obstacle = shapely.geometry.Polygon(coords)
#             collision = line.intersects(obstacle)
#             if collision:
#                 break


#     return collision


# class TreeRRT:
#     def __init__(self, start_coords, env):
#         self.vertices = dict()
#         self.env = env
#         self.robot_belief = env.robot_belief
#         self.add_vertex(0, -1, start_coords)

#     def add_vertex(self, vertex_id, parent_id, coords):
#         self.vertices[vertex_id] = {'parent_id': parent_id, 'coords': coords}
    
#     def get_nearest_vertex(self, sample_coords):
#         vertices_coords = np.array([v['coords'] for v in self.vertices.values()])
#         dist_list = np.linalg.norm(sample_coords - vertices_coords, axis=-1)
#         nearest_vertex_id = np.argmin(dist_list)
#         return nearest_vertex_id, vertices_coords[nearest_vertex_id]

# class RRT_worker:
#     def __init__(self, metaAgentID, global_step, save_image=False):
#         self.metaAgentID = metaAgentID
#         self.global_step = global_step
#         self.save_image = save_image
#         self.step_length = 20
#         self.pre_best_path = []
#         self.planned_paths = []

#         self.env = Env(map_index=self.global_step, plot=save_image, test=True)

#     def free_area(self, robot_belief):
#         index = np.where(robot_belief == 255)
#         free = np.asarray([index[1], index[0]]).T
#         return free

#     def sample_free_area(self):
#         free_area = self.free_area(self.env.robot_belief)
#         sample_index = random.randint(0, len(free_area) - 1)
#         return free_area[sample_index]

#     def steer(self, start_coords, goal_coords):
#         direction = (goal_coords - start_coords)
#         distance = np.linalg.norm(direction)
#         step_size = min(self.step_length, distance)
#         new_coords = start_coords + (direction / distance) * (step_size+1)
#         return new_coords
    
#     def goal_reached(self):
#         # Check if the exploration has met the threshold
#         return self.env.explored_rate >= self.env.finish_percent

#     def extract_route(self, tree, vertex_id):
#         route = []
#         current_id = vertex_id
#         while current_id != -1:
#             route.append(tree.vertices[current_id]['coords'])
#             current_id = tree.vertices[current_id]['parent_id']
#         return route[::-1]

#     def find_next_best_viewpoints(self):
#         max_iter_steps = 50
#         tree = TreeRRT(self.env.robot_position, self.env)
#         self.planned_paths = []
#         best_route = []

#         for i in range(max_iter_steps):
#             sample_coords = self.sample_free_area()
#             nearest_vertex_id, nearest_vertex_coords = tree.get_nearest_vertex(sample_coords)
            
#             new_vertex_coords = self.steer(nearest_vertex_coords, sample_coords)
#             if not check_collision(nearest_vertex_coords, new_vertex_coords, self.env.robot_belief):
#                 new_vertex_id = len(tree.vertices)
#                 tree.add_vertex(new_vertex_id, nearest_vertex_id, new_vertex_coords)
#                 best_route = self.extract_route(tree, new_vertex_id)
#                 self.planned_paths.append(best_route)

#             if self.goal_reached():
#                 break

#         return best_route[1], self.planned_paths

#     def run_episode(self, currEpisode):
#         perf_metrics = dict()
#         done = False
#         self.env.begin()
#         i = 0
#         while not done:
#             i += 1
#             next_node_coords, planned_route = self.find_next_best_viewpoints()
#             done = self.env.step(next_node_coords)

#             if self.save_image:
#                 if not os.path.exists(gifs_path):
#                     os.makedirs(gifs_path)
#                 self.env.plot_env(self.global_step, gifs_path, i, planned_route)

#             if SAVE_TRAJECTORY:
#                 if not os.path.exists(trajectory_path):
#                     os.makedirs(trajectory_path)
#                 csv_filename = f'results/trajectory/rrt_trajectory_result.csv'
#                 new_file = False if os.path.exists(csv_filename) else True
#                 field_names = ['dist', 'area']
#                 with open(csv_filename, 'a') as csvfile:
#                     writer = csv.writer(csvfile)
#                     if new_file:
#                         writer.writerow(field_names)
#                     csv_data = np.array([self.env.travel_dist, np.sum(self.env.robot_belief == 255)]).reshape(1, -1)
#                     writer.writerows(csv_data)

#             if done:
#                 perf_metrics['travel_dist'] = self.env.travel_dist
#                 perf_metrics['explored_rate'] = self.env.explored_rate
#                 perf_metrics['success_rate'] = True
#                 perf_metrics['relax_success_rate'] = True if self.env.explored_rate > self.env.finish_percent else False
#                 break

#         if SAVE_LENGTH:
#             if not os.path.exists(length_path):
#                 os.makedirs(length_path)
#             csv_filename = f'results/length/RRT_length_result.csv'
#             new_file = False if os.path.exists(csv_filename) else True
#             field_names = ['dist']
#             with open(csv_filename, 'a') as csvfile:
#                 writer = csv.writer(csvfile)
#                 if new_file:
#                     writer.writerow(field_names)
#                 csv_data = np.array([self.env.travel_dist]).reshape(-1,1)
#                 writer.writerows(csv_data)

#         if self.save_image:
#             self.make_gif(gifs_path, currEpisode)

#         return perf_metrics

#     def work(self, currEpisode):
#         self.currEpisode = currEpisode
#         self.perf_metrics = self.run_episode(currEpisode)

#     def make_gif(self, path, n):
#         with imageio.get_writer(f'{path}/{n}_explored_rate_{self.env.explored_rate:.4g}_length_{self.env.travel_dist:.4g}.gif', mode='I', duration=0.5) as writer:
#             for frame in self.env.frame_files:
#                 image = imageio.imread(frame)
#                 writer.append_data(image)
#         print('gif complete\n')

#         for filename in self.env.frame_files[:-1]:
#             os.remove(filename)


# if __name__ == "__main__":
#     total_episode = 40
#     total_dist = 0
#     for i in range(total_episode):
#         worker = RRT_worker(metaAgentID=0, global_step=i, save_image=SAVE_GIFS)
#         performance = worker.run_episode(i)
#         total_dist += performance["travel_dist"]
#         mean_dist = total_dist / (i + 1)
#         print(mean_dist)




import numpy as np
import shapely.geometry
import random
from scipy.spatial import ConvexHull
import os
import csv
import imageio


from NBVP_env import Env# Assuming the Env class is in a file named env.py

SAVE_GIFS = True  # Set to True if you want to save GIFs
SAVE_TRAJECTORY = False  # Set to True if you want to save trajectory data
SAVE_LENGTH = False  # Set to True if you want to save length data

gifs_path = 'results/gifs'
trajectory_path = 'results/trajectory'
length_path = 'results/length'


class WMARRTNode:
    def __init__(self, coords, parent_id=-1):
        self.coords = coords
        self.parent_id = parent_id
        self.eo = []
        self.ei = None
        self.wo = {}
        self.wi = 0
        self.c = False
        self.lo = None
        self.co = []
        self.agents_exploring = 0
        self.is_completed = False

class WMARRT:
    def __init__(self, start_coords, env, num_agents):
        self.env = env
        self.num_agents = num_agents
        self.completed_branches = 0
        self.vertices = {0: WMARRTNode(start_coords)}
        self.main_root = 0
        self.secondary_roots = []
        self.next_vertex_id = 1
        self.next_edge_id = 0

    def add_vertex(self, coords, parent_id):
        new_id = self.next_vertex_id
        self.vertices[new_id] = WMARRTNode(coords, parent_id)
        self.next_vertex_id += 1
        return new_id

    def mark_branch_completed(self, branch_id):
        self.vertices[branch_id].is_completed = True
        self.completed_branches += 1

    def is_exploration_completed(self):
        return self.completed_branches == len(self.secondary_roots)    

    def add_edge(self, parent_id, child_id):
        edge_id = self.next_edge_id
        self.next_edge_id += 1

        parent_node = self.vertices[parent_id]
        child_node = self.vertices[child_id]

        parent_node.eo.append(edge_id)
        parent_node.wo[edge_id] = 0

        child_node.ei = edge_id
        child_node.wi = 0

    def get_nearest_vertex(self, sample_coords):
        vertices_coords = np.array([v.coords for v in self.vertices.values()])
        dist_list = np.linalg.norm(sample_coords - vertices_coords, axis=-1)
        nearest_vertex_id = np.argmin(dist_list)
        return nearest_vertex_id, vertices_coords[nearest_vertex_id]

    def steer(self, start_coords, goal_coords):
     direction = np.array(goal_coords) - np.array(start_coords)
     distance = np.linalg.norm(direction)
     if distance == 0:
        return start_coords  # If start and goal are the same, return start
     step_size = min(10, distance)  # Reduced from 20 to 10
     new_coords = np.array(start_coords) + (direction / distance) * (step_size + 1)
     return tuple(new_coords)

    def extract_path(self, vertex_id):
     path = []
     current_id = vertex_id
     while current_id != -1:
        path.append(self.vertices[current_id].coords)
        current_id = self.vertices[current_id].parent_id
     path = path[::-1]  # Reverse the path
    
    # Ensure all coordinates are 2D
     path = [coord[:2] if len(coord) > 2 else coord for coord in path]
    
    # If path is empty, add the start position twice to create a valid path
     if not path:
        start_pos = self.vertices[self.main_root].coords[:2]
        path = [start_pos, start_pos]
    
    # If path has only one point, duplicate it to create a valid path
     elif len(path) == 1:
        path.append(path[0])
    
     return path

    def find_main_root(self, agent_positions):
        agent_positions = np.array(agent_positions)
        if np.all(agent_positions == agent_positions[0]):
            # All agents start at the same position
            self.main_root = 0
            return

        hull = ConvexHull(agent_positions)
        hull_points = agent_positions[hull.vertices]
        polygon = shapely.geometry.Polygon(hull_points)
        
        diameters = []
        for i in range(len(hull.vertices)):
            for j in range(i+1, len(hull.vertices)):
                line = shapely.geometry.LineString([hull_points[i], hull_points[j]])
                diameters.append((line.length, line))
        
        diameters.sort(reverse=True)
        
        if len(diameters) >= 2:
            intersection = diameters[0][1].intersection(diameters[1][1])
            
            if intersection.is_empty:
                main_root_coords = polygon.centroid.coords[0]
            else:
                main_root_coords = (intersection.x, intersection.y)
        else:
            main_root_coords = agent_positions[0]

        if self.check_collision(self.vertices[0].coords, main_root_coords):
            main_root_coords = polygon.centroid.coords[0]

        self.main_root = self.add_vertex(main_root_coords, -1)

    def assign_secondary_roots(self, agent_positions):
     if np.all(agent_positions == agent_positions[0]):
        # All agents start at the same position
        main_root_coords = self.vertices[self.main_root].coords
        for i in range(self.num_agents):
            angle = 2 * np.pi * i / self.num_agents
            direction = np.array([np.cos(angle), np.sin(angle)])
            candidate = tuple(np.array(main_root_coords) + direction * 10)  # 10 is the epsilon value
            
            if not self.check_collision(main_root_coords, candidate):
                secondary_root = self.add_vertex(candidate, self.main_root)
                self.secondary_roots.append(secondary_root)
                self.add_edge(self.main_root, secondary_root)
            else:
                # If collision, try to find a nearby free cell
                for dist in range(11, 20):  # Increase search radius if needed
                    candidate = tuple(np.array(main_root_coords) + direction * dist)
                    if not self.check_collision(main_root_coords, candidate):
                        secondary_root = self.add_vertex(candidate, self.main_root)
                        self.secondary_roots.append(secondary_root)
                        self.add_edge(self.main_root, secondary_root)
                        break
     else:
        # Original logic for when agents start at different positions
        for agent_pos in agent_positions:
            direction = np.array(agent_pos) - np.array(self.vertices[self.main_root].coords)
            distance = np.linalg.norm(direction)
            if distance > 10:  # epsilon = 10
                direction = direction / distance * 10
            
            candidate = tuple(np.array(self.vertices[self.main_root].coords) + direction)
            
            if not self.check_collision(self.vertices[self.main_root].coords, candidate):
                secondary_root = self.add_vertex(candidate, self.main_root)
                self.secondary_roots.append(secondary_root)
                self.add_edge(self.main_root, secondary_root)
            else:
                if self.secondary_roots:
                    nearest = min(self.secondary_roots, 
                                  key=lambda x: np.linalg.norm(np.array(self.vertices[x].coords) - np.array(agent_pos)))
                    self.secondary_roots.append(nearest)
                else:
                    secondary_root = self.add_vertex(tuple(agent_pos), self.main_root)
                    self.secondary_roots.append(secondary_root)
                    self.add_edge(self.main_root, secondary_root)

     print(f"Number of secondary roots: {len(self.secondary_roots)}")
    def check_collision(self, start, end):
        return check_collision(start, end, self.env.robot_belief)

    def allocate_branch(self, branch_id):
        self.vertices[branch_id].agents_exploring += 1

    def deallocate_branch(self, branch_id):
        self.vertices[branch_id].agents_exploring -= 1    

        

class WMARRTWorker:
    def __init__(self, metaAgentID, global_step, save_image=False, num_agents=5):
        self.metaAgentID = metaAgentID
        self.global_step = global_step
        self.save_image = save_image
        self.exploration_completed = False
        self.num_agents = num_agents
        self.step_length = 20
        self.pre_best_path = []
        self.planned_paths = []
        
        self.env = Env(map_index=self.global_step, plot=save_image, test=False, num_agents=num_agents)
        self.wmarrt = None
        self.agent_positions = []
        self.agent_assigned_branches = [None] * num_agents

    def run_episode(self):
        perf_metrics = dict()
        done = False
        self.env.begin()
        

        # Initialize WMA-RRT
        self.wmarrt = WMARRT(self.env.robot_position, self.env, self.num_agents)
        self.agent_positions = [self.env.robot_position] * self.num_agents
        self.wmarrt.find_main_root(self.agent_positions)
        self.wmarrt.assign_secondary_roots(self.agent_positions)

        i = 0
        while not done:
            i += 1
            # next_node_coords, planned_routes = self.find_next_best_viewpoints()
            # done = self.env.step(next_node_coords)
            next_node_coords, planned_routes = self.find_next_best_viewpoints()
            done = self.env.step(next_node_coords) or self.exploration_completed
            self.agent_positions = next_node_coords

            if self.save_image:
                if not os.path.exists('results/gifs'):
                    os.makedirs('results/gifs')
                self.env.plot_env(self.global_step, 'results/gifs', i, planned_routes)

            if done:
                perf_metrics['travel_dist'] = self.env.travel_dist
                perf_metrics['explored_rate'] = self.env.explored_rate
                perf_metrics['success_rate'] = True
                perf_metrics['relax_success_rate'] = True if self.env.explored_rate > self.env.finish_percent else False
                break

        return perf_metrics


    def work(self, currEpisode):
        self.currEpisode = currEpisode
        self.perf_metrics = self.run_episode(currEpisode)

    def finish_branch_exploration(self, agent_id):
        branch = self.agent_assigned_branches[agent_id]
        if branch is not None:
            self.wmarrt.deallocate_branch(branch)
            self.wmarrt.mark_branch_completed(branch)
            self.agent_assigned_branches[agent_id] = None
            print(f"Agent {agent_id} completed branch {branch}")

            # Check if all branches are completed
            if self.wmarrt.is_exploration_completed():
                self.exploration_completed = True
                print("All branches explored. Exploration completed.")    

    def make_gif(self, path, n):
        with imageio.get_writer(f'{path}/{n}_explored_rate_{self.env.explored_rate:.4g}_length_{self.env.travel_dist:.4g}.gif', mode='I', duration=0.5) as writer:
            for frame in self.env.frame_files:
                image = imageio.imread(frame)
                writer.append_data(image)
        print('gif complete\n')

        for filename in self.env.frame_files[:-1]:
            os.remove(filename)

    def check_branch_completion(self, agent_id):
        branch = self.agent_assigned_branches[agent_id]
        if branch is not None:
            if self.is_branch_fully_explored(branch):
                self.finish_branch_exploration(agent_id)
                return True
        return False
    def generate_agent_positions(self):
        free_cells = self.env.free_cells()
        return [free_cells[np.random.randint(len(free_cells))] for _ in range(self.num_agents)]

    # def find_branch(self, agent_id):
    #     for secondary_root in self.wmarrt.secondary_roots:
    #         if secondary_root not in self.agent_assigned_branches:
    #             return secondary_root
    #     return None   
    def find_branch(self, agent_id):
        # Enumerate all secondary roots (branches)
        for secondary_root in self.wmarrt.secondary_roots:
            if self.wmarrt.vertices[secondary_root].agents_exploring == 0:
                # Found an unallocated branch
                self.wmarrt.allocate_branch(secondary_root)
                return secondary_root
        
        # If no unallocated branch, find the branch with minimum agents
        min_agents = float('inf')
        min_branch = None
        for secondary_root in self.wmarrt.secondary_roots:
            agents = self.wmarrt.vertices[secondary_root].agents_exploring
            if agents < min_agents:
                min_agents = agents
                min_branch = secondary_root
        
        if min_branch is not None:
            self.wmarrt.allocate_branch(min_branch)
        return min_branch 

    def find_least_explored_branch(self):
     branch_exploration = {branch: self.calculate_branch_exploration(branch) for branch in self.wmarrt.secondary_roots}
     return min(branch_exploration, key=branch_exploration.get)    

    def find_next_best_viewpoints(self):
        # Grow the tree with 100 new nodes
        for _ in range(100):
            dice = np.random.random()
            if dice > 0.2:
                # Sample from free area
                free_cells = self.env.free_cells()
                if free_cells.size == 0:  # Check if the array is empty
                    continue
                sample = free_cells[np.random.randint(free_cells.shape[0])]
            else:
                # Sample from frontiers
                frontiers = self.env.frontiers  # Changed from self.env.frontiers()
                if len(frontiers) == 0:  # Check if the list is empty
                    continue
                sample = frontiers[np.random.randint(len(frontiers))]

            nearest_id, nearest_coords = self.wmarrt.get_nearest_vertex(sample)
            new_coords = self.wmarrt.steer(nearest_coords, sample)
            
            # Check collision with environment
            if not self.wmarrt.check_collision(nearest_coords, new_coords):
                new_id = self.wmarrt.add_vertex(new_coords, nearest_id)
                self.wmarrt.add_edge(nearest_id, new_id)

        next_positions = []
        planned_routes = []

        for agent_id in range(self.num_agents):
            if self.agent_assigned_branches[agent_id] is None:
                # Agent is at Main Root, needs to find a branch
                branch = self.find_branch(agent_id)
                if branch is not None:
                    print(f"Agent {agent_id} assigned to branch: {branch}")
                    self.agent_assigned_branches[agent_id] = branch
                    best_node = self.wmarrt.vertices[branch]
                else:
                    print(f"Agent {agent_id} couldn't find a branch, staying at Main Root")
                    best_node = self.wmarrt.vertices[self.wmarrt.main_root]
            else:
                # Check if the current branch is completed
                if self.check_branch_completion(agent_id):
                    # Branch completed, find a new branch
                    branch = self.find_branch(agent_id)
                    if branch is not None:
                        print(f"Agent {agent_id} assigned to new branch: {branch}")
                        self.agent_assigned_branches[agent_id] = branch
                        best_node = self.wmarrt.vertices[branch]
                    else:
                        print(f"Agent {agent_id} couldn't find a new branch, staying at Main Root")
                        best_node = self.wmarrt.vertices[self.wmarrt.main_root]
                else:
                    # Continue exploring current branch
                    best_node = self.explore_branch(self.agent_assigned_branches[agent_id])
                    print(f"Agent {agent_id} exploring branch: {self.agent_assigned_branches[agent_id]}")

            current_position = self.agent_positions[agent_id]
            path_to_best_node = self.calculate_path(current_position, best_node.coords)
            
            # Move agent incrementally along the path
            if len(path_to_best_node) > 1:
                next_position = path_to_best_node[1]  # Move to the next point on the path
            else:
                next_position = current_position
            
            next_positions.append(next_position)
            planned_routes.append(path_to_best_node)

        # Update agent positions
        self.agent_positions = next_positions

        return self.agent_positions, planned_routes





    # def find_branch(self, agent_id):
    #     for secondary_root in self.wmarrt.secondary_roots:
    #         if secondary_root not in self.agent_assigned_branches:
    #             return secondary_root
    #     return None

    def find_branch(self, agent_id):
        # Enumerate all secondary roots (branches)
        for secondary_root in self.wmarrt.secondary_roots:
            if self.wmarrt.vertices[secondary_root].agents_exploring == 0:
                # Found an unallocated branch
                self.wmarrt.allocate_branch(secondary_root)
                return secondary_root
        
        # If no unallocated branch, find the branch with minimum agents
        min_agents = float('inf')
        min_branch = None
        for secondary_root in self.wmarrt.secondary_roots:
            agents = self.wmarrt.vertices[secondary_root].agents_exploring
            if agents < min_agents:
                min_agents = agents
                min_branch = secondary_root
        
        if min_branch is not None:
            self.wmarrt.allocate_branch(min_branch)
        return min_branch

    def find_minimum_agent_branch(self):
        branch_counts = {branch: self.agent_assigned_branches.count(branch) for branch in self.wmarrt.secondary_roots}
        return min(branch_counts, key=branch_counts.get)

    def explore_branch(self, branch_id):
    # Find the best node in the branch to explore next
     print(f"Exploring branch {branch_id}")  # Debug branch being explored
     branch_nodes = self.get_branch_nodes(branch_id)
     best_node = max(branch_nodes, key=lambda node: self.evaluate_node(node))
     print(f"Best node in branch {branch_id}: {best_node.coords}")  # Debug best node in the branch
     return best_node

    def is_node_fully_explored(self, node):
        # Define a small area around the node to check for unexplored cells
        x, y = int(node.coords[0]), int(node.coords[1])
        area = self.env.robot_belief[y-5:y+6, x-5:x+6]
        
        # If there are any unexplored cells (value 127) in this area, the node is not fully explored
        return np.all(area != 127)

    def get_branch_nodes(self, branch_id):
        branch_nodes = [self.wmarrt.vertices[branch_id]]
        queue = [branch_id]
        while queue:
            current_id = queue.pop(0)
            for edge_id in self.wmarrt.vertices[current_id].eo:
                child_id = next(vid for vid, vertex in self.wmarrt.vertices.items() if vertex.ei == edge_id)
                branch_nodes.append(self.wmarrt.vertices[child_id])
                queue.append(child_id)
        return branch_nodes

    def calculate_path(self, start, end):
        # Find the nearest vertices in the tree for start and end points
        start_id, _ = self.wmarrt.get_nearest_vertex(start)
        end_id, _ = self.wmarrt.get_nearest_vertex(end)

        # Extract path from start to end through the tree
        path_to_end = self.wmarrt.extract_path(end_id)
        path_from_start = self.wmarrt.extract_path(start_id)

        # Combine paths: reverse path from start, add direct connection, then path to end
        full_path = path_from_start[::-1] + [start, end] + path_to_end
        
        # Remove duplicates while preserving order
        full_path = list(dict.fromkeys(map(tuple, full_path)))
        
        return full_path    

    # def evaluate_node(self, node):
    #     unexplored = np.sum(self.env.robot_belief[int(node.coords[1])-10:int(node.coords[1])+11, 
    #                                               int(node.coords[0])-10:int(node.coords[0])+11] == 127)
    #     return unexplored - sum(node.wo.values())

    def evaluate_node(self, node):
        unexplored = np.sum(self.env.robot_belief[int(node.coords[1])-10:int(node.coords[1])+11, 
                                                  int(node.coords[0])-10:int(node.coords[0])+11] == 127)
        
        # Prioritize branches with fewer agents
        branch_penalty = self.wmarrt.vertices[node.parent_id].agents_exploring * 10
        
        return unexplored - sum(node.wo.values()) - branch_penalty

    def check_branch_completion(self, agent_id):
        branch = self.agent_assigned_branches[agent_id]
        if branch is not None:
            if self.is_branch_fully_explored(branch):
                self.finish_branch_exploration(agent_id)
                return True
        return False

    def is_branch_fully_explored(self, branch):
        branch_nodes = self.get_branch_nodes(branch)
        for node in branch_nodes:
            if not self.is_node_fully_explored(node):
                return False
        
        # Check overall exploration rate
        current_exploration_rate = self.env.explored_rate
        if current_exploration_rate < 0.95:  # Set a threshold, e.g., 95%
            return False
        
        return True  

def check_collision(start, end, robot_belief):
    collision = False
    line = shapely.geometry.LineString([start, end])

    sortx = np.sort([int(start[0]), int(end[0])])
    sorty = np.sort([int(start[1]), int(end[1])])

    robot_belief = robot_belief[sorty[0]:sorty[1] + 1, sortx[0]:sortx[1] + 1]

    occupied_area_index = np.where(robot_belief == 1)
    occupied_area_coords = np.asarray(
            [occupied_area_index[1] + sortx[0], occupied_area_index[0] + sorty[0]]).T
    unexplored_area_index = np.where(robot_belief == 127)
    unexplored_area_coords = np.asarray(
            [unexplored_area_index[1] + sortx[0], unexplored_area_index[0] + sorty[0]]).T
    unfree_area_coords = occupied_area_coords

    for i in range(unfree_area_coords.shape[0]):
        coords = ([(unfree_area_coords[i][0] -5, unfree_area_coords[i][1] -5),
               (unfree_area_coords[i][0] + 5, unfree_area_coords[i][1] -5),
               (unfree_area_coords[i][0] - 5, unfree_area_coords[i][1] + 5),
               (unfree_area_coords[i][0] + 5, unfree_area_coords[i][1] + 5)])
        obstacle = shapely.geometry.Polygon(coords)
        if abs(end[0] - unfree_area_coords[i][0] <= 8) and abs(end[1] - unfree_area_coords[i][1] <= 8):
            collision = True
        if not collision:
            collision = line.intersects(obstacle)
        if collision:
            break

    if not collision:
        unfree_area_coords = unexplored_area_coords
        for i in range(unfree_area_coords.shape[0]):
            coords = ([(unfree_area_coords[i][0], unfree_area_coords[i][1]),
                       (unfree_area_coords[i][0] + 1, unfree_area_coords[i][1]),
                       (unfree_area_coords[i][0], unfree_area_coords[i][1]),
                       (unfree_area_coords[i][0] + 1, unfree_area_coords[i][1] + 1)])
            obstacle = shapely.geometry.Polygon(coords)
            collision = line.intersects(obstacle)
            if collision:
                break

    return collision

if __name__ == "__main__":
    total_episode = 40
    total_dist = 0
    for i in range(total_episode):
        worker = WMARRTWorker(metaAgentID=0, global_step=i, save_image=True, num_agents=5)
        performance = worker.run_episode()
        total_dist += performance["travel_dist"]
        mean_dist = total_dist / (i + 1)
        print(f"Episode {i+1}: Explored rate: {performance['explored_rate']:.4f}, Travel distance: {performance['travel_dist']:.4f}")
        print(f"Mean distance so far: {mean_dist:.4f}")
from scipy import spatial
from skimage import io
import numpy as np
import time
import sys
from scipy import ndimage
import matplotlib.pyplot as plt
import os
import copy
from skimage.measure import block_reduce

from sensor import *
from parameter import *


class Env():
    def __init__(self, map_index, plot=False, test=False, num_agents=5):
        self.test = test
        if self.test:
            self.map_dir = f'DungeonMaps/medium'
        else:
            self.map_dir = f'DungeonMaps/train'

        self.map_list = os.listdir(self.map_dir)
        self.map_list.sort(reverse=True)
        self.map_index = map_index % np.size(self.map_list)
        self.ground_truth, self.start_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
        self.ground_truth_size = np.shape(self.ground_truth)
        self.robot_beliefs = [np.ones(self.ground_truth_size) * 127 for _ in range(num_agents)] 
        self.map_index = map_index % np.size(self.map_list)
        self.ground_truth, self.robot_position = self.import_ground_truth(self.map_dir + '/' + self.map_list[self.map_index])
        self.ground_truth_size = np.shape(self.ground_truth) # (480, 640)
        self.robot_belief = np.ones(self.ground_truth_size) * 127 # unexplored 127
        
        self.finish_percent = 0.985
        self.resolution = 4
        self.sensor_range = 80
        self.old_robot_belief = copy.deepcopy(self.robot_belief)

        self.plot = plot
        self.frame_files = []
        self.num_agents = num_agents
        self.agent_paths = [[] for _ in range(num_agents)]
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'orange', 'purple', 'pink']

        if self.plot:
            # initialize the route
            self.xPoints = [self.robot_position[0]]
            self.yPoints = [self.robot_position[1]]
        self.frontiers = None
        self.travel_dist = 0
        self.explored_rate = 0
        self.route_node = [self.robot_position]
        self.frontiers = None
        self.downsampled_belief = None

    def begin(self):
        
        self.robot_belief = self.update_robot_belief(self.robot_position, self.sensor_range, self.robot_belief, self.ground_truth)
        self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)

        self.frontiers = self.find_frontier()
        
        self.old_robot_belief = copy.deepcopy(self.robot_belief)

        # Initialize all agent paths with the starting position
        for i in range(self.num_agents):
            self.agent_paths[i].append(self.robot_position)


    def step(self, next_node_coords):
     if isinstance(next_node_coords, np.ndarray) and next_node_coords.ndim == 1:
        # Single agent case
        next_node_coords = [next_node_coords]
    
     for i, coords in enumerate(next_node_coords):
        if i >= len(self.agent_paths):
            self.agent_paths.append([])  # Initialize new agent path if needed
        
        prev_pos = self.agent_paths[i][-1] if self.agent_paths[i] else self.robot_position
        
        # Convert coords and prev_pos to numpy arrays
        coords = np.array(coords)
        prev_pos = np.array(prev_pos)
        
        dist = np.linalg.norm(coords - prev_pos)
        
        self.travel_dist += dist
        self.agent_paths[i].append(coords)
    
    # Update robot_belief based on all agents' new positions
     for coords in next_node_coords:
        self.robot_belief = self.update_robot_belief(coords, self.sensor_range, self.robot_belief, self.ground_truth)
    
     self.downsampled_belief = block_reduce(self.robot_belief.copy(), block_size=(self.resolution, self.resolution), func=np.min)
     frontiers = self.find_frontier()
     self.explored_rate = self.evaluate_exploration_rate()
     self.frontiers = frontiers
    
     done = self.check_done()
     return done

    def import_ground_truth(self, map_index):
        # occupied 1, free 255, unexplored 127

        ground_truth = (io.imread(map_index, 1) * 255).astype(int)
        robot_location = np.nonzero(ground_truth == 208)
        robot_location = np.array([np.array(robot_location)[1, 127], np.array(robot_location)[0, 127]])
        ground_truth = (ground_truth > 150)
        ground_truth = ground_truth * 254 + 1
        return ground_truth, robot_location

    def free_cells(self):
        index = np.where(self.ground_truth == 255)
        free = np.asarray([index[1], index[0]]).T
        return free

    def update_robot_belief(self, robot_position, sensor_range, robot_belief, ground_truth):
        robot_belief = sensor_work(robot_position, sensor_range, robot_belief, ground_truth)
        return robot_belief

    def check_done(self):
        done = False
        #if self.node_utility.sum() == 0:
        #if self.explored_rate >= 0.99 or self.node_utility.sum() == 0:
        if np.sum(self.ground_truth == 255) - np.sum(self.robot_belief == 255) <= 250:
            done = True
        return done

    def evaluate_exploration_rate(self):
        rate = np.sum(self.robot_belief == 255) / np.sum(self.ground_truth == 255)
        return rate

    def calculate_new_free_area(self):
        old_free_area = self.old_robot_belief == 255
        current_free_area = self.robot_belief == 255

        new_free_area = (current_free_area.astype(np.int) - old_free_area.astype(np.int)) * 255

        return new_free_area, np.sum(old_free_area)

    def calculate_utility_along_path(self, path, nodes_list):
        observable_frontiers = []
        for index in path:
            observable_frontiers += nodes_list[index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

        return unique_frontiers.shape[0]

    def calculate_node_gain_over_path(self, node_index, path, nodes_list):
        observable_frontiers = []
        for index in path:
            observable_frontiers += nodes_list[index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        pre_unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)
        observable_frontiers += nodes_list[node_index].observable_frontiers
        np_observable_frontiers = np.array(observable_frontiers).reshape(-1,2)
        unique_frontiers = np.unique(np_observable_frontiers[:, 0] + np_observable_frontiers[:, 1]*1j)

        return unique_frontiers.shape[0] - pre_unique_frontiers.shape[0]

    def calculate_dist_path(self, path, node_list):
        dist = 0
        start = path[0]
        end = path[-1]
        for index in path:
            if index == end:
                break
            dist += np.linalg.norm(node_list[start].coords - node_list[index].coords)
            start = index
        return dist

    def find_frontier(self):
        y_len = self.downsampled_belief.shape[0]
        x_len = self.downsampled_belief.shape[1]
        mapping = self.downsampled_belief.copy()
        belief = self.downsampled_belief.copy()
        # 0-1 unknown area map
        mapping = (mapping == 127) * 1
        mapping = np.lib.pad(mapping, ((1, 1), (1, 1)), 'constant', constant_values=0)
        fro_map = mapping[2:][:, 1:x_len + 1] + mapping[:y_len][:, 1:x_len + 1] + mapping[1:y_len + 1][:, 2:] + \
                  mapping[1:y_len + 1][:, :x_len] + mapping[:y_len][:, 2:] + mapping[2:][:, :x_len] + mapping[2:][:,
                                                                                                      2:] + \
                  mapping[:y_len][:, :x_len]
        ind_free = np.where(belief.ravel(order='F') == 255)[0]
        ind_fron_1 = np.where(1 < fro_map.ravel(order='F'))[0]
        ind_fron_2 = np.where(fro_map.ravel(order='F') < 8)[0]
        ind_fron = np.intersect1d(ind_fron_1, ind_fron_2)
        ind_to = np.intersect1d(ind_free, ind_fron)

        map_x = x_len
        map_y = y_len
        x = np.linspace(0, map_x - 1, map_x)
        y = np.linspace(0, map_y - 1, map_y)
        t1, t2 = np.meshgrid(x, y)
        points = np.vstack([t1.T.ravel(), t2.T.ravel()]).T

        f = points[ind_to]
        f = f.astype(int)

        f = f * self.resolution

        return f

    # def plot_env(self, n, path, step, planned_routes=None):
    #     plt.switch_backend('agg')
    #     plt.cla()
    #     combined_belief = np.min(self.robot_beliefs, axis=0)
    #     plt.imshow(combined_belief, cmap='gray')
    #     plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))
    #     if planned_routes:
    #         for agent_id, agent_routes in enumerate(planned_routes):
    #             for p in agent_routes:
    #                 planned_x, planned_y = [], []
    #                 for coords in p:
    #                     print(f"coords: {coords}")
    #                     planned_x.append(coords[0])
    #                     planned_y.append(coords[1])
    #                 plt.plot(planned_x, planned_y, c=['r', 'g', 'b', 'y'][agent_id % 4], linewidth=2, zorder=2)
    #     for i in range(self.num_agents):
    #         plt.plot(self.xPoints[i], self.yPoints[i], ['b', 'g', 'r', 'y'][i % 4], linewidth=2)
    #         plt.plot(self.robot_position[i][0], self.robot_positions[i][1], 'mo', markersize=8)
    #     for i, start_pos in enumerate(self.robot_position):
    #         plt.plot(start_pos[0], start_pos[1], 'co', markersize=8)
    #     plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='r', s=2, zorder=3)
    #     plt.suptitle('Explored ratio: {:.4g}  Travel distances: {}'.format(self.explored_rate, [round(d, 2) for d in self.travel_dists]))
    #     plt.tight_layout()
    #     plt.savefig('{}/{}_{}_samples.png'.format(path, n, step, dpi=150))
    #     frame = '{}/{}_{}_samples.png'.format(path, n, step)
    #     self.frame_files.append(frame)
    


    def plot_env(self, n, path, step, planned_routes=None):
     plt.switch_backend('agg')
     plt.figure(figsize=(10, 10))
     plt.imshow(self.robot_belief, cmap='gray')
     plt.axis((0, self.ground_truth_size[1], self.ground_truth_size[0], 0))

     for i, agent_path in enumerate(self.agent_paths):
        if agent_path:
            try:
                agent_path = np.array(agent_path)
                if agent_path.ndim == 1:
                    agent_path = agent_path.reshape(-1, 2)
                elif agent_path.ndim > 2:
                    agent_path = agent_path[:, :2]
                
                x, y = agent_path[:, 0], agent_path[:, 1]
                plt.plot(x, y, c=self.colors[i % len(self.colors)], linewidth=2, label=f'Agent {i+1}')
                plt.plot(x[-1], y[-1], c=self.colors[i % len(self.colors)], marker='o', markersize=8)
            except Exception as e:
                print(f"Error plotting path for Agent {i+1}: {e}")
                print(f"Problematic path: {agent_path}")

     if planned_routes:
        for i, route in enumerate(planned_routes):
            if route:
                try:
                    route = np.array(route)
                    if route.ndim == 1:
                        route = route.reshape(-1, 2)
                    elif route.ndim > 2:
                        route = route[:, :2]
                    
                    planned_x, planned_y = route[:, 0], route[:, 1]
                    plt.plot(planned_x, planned_y, c=self.colors[i % len(self.colors)], linewidth=1, linestyle='--')
                except Exception as e:
                    print(f"Error plotting planned route for Agent {i+1}: {e}")
                    print(f"Problematic route: {route}")

    # Plot starting position
     if self.agent_paths and self.agent_paths[0]:
        start_pos = np.array(self.agent_paths[0][0])
        plt.plot(start_pos[0], start_pos[1], 'co', markersize=10, markerfacecolor='none', markeredgewidth=2)

    # Plot frontiers
     if hasattr(self, 'frontiers') and self.frontiers is not None:
        plt.scatter(self.frontiers[:, 0], self.frontiers[:, 1], c='red', s=2, zorder=3)

     plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
     plt.title('Multi-Agent Exploration')
     plt.suptitle(f'Explored ratio: {self.explored_rate:.4g} Travel distance: {self.travel_dist:.4g}')
     plt.tight_layout()
     plt.savefig(f'{path}/{n}_{step}_samples.png', dpi=150, bbox_inches='tight')
     self.frame_files.append(f'{path}/{n}_{step}_samples.png')
     plt.close()
   


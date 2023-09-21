import random
import matplotlib.pyplot as plt
import math


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacle_list, max_iter, step_size,min_x,max_x,min_y,max_y):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacle_list = obstacle_list
        self.max_iter = max_iter
        self.step_size = step_size
        self.nodes = [self.start]
        self.min_x=min_x
        self.min_y=min_y
        self.max_x=max_x
        self.max_y=max_y

    def is_collision_free(self, new_node):
        for obstacle in self.obstacle_list:
            obstacle_node=Node(obstacle[0],obstacle[1])
            if distance(new_node, obstacle_node) < 0.5:  
                return False
        return True

    def extend(self, rand_node):
        nearest_node = self.nodes[0]
        for node in self.nodes:
            if distance(node, rand_node) < distance(nearest_node, rand_node):
                nearest_node = node

        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_x = nearest_node.x + self.step_size * math.cos(theta)
        new_y = nearest_node.y + self.step_size * math.sin(theta)
        new_node = Node(new_x, new_y)
        new_node.parent = nearest_node

        if self.is_collision_free(new_node):
            self.nodes.append(new_node)
            return new_node
        return None

    def plan(self):
        for _ in range(self.max_iter):
            rand_x = random.uniform(self.min_x, self.max_x)
            rand_y = random.uniform(self.min_y, self.max_y)
            rand_node = Node(rand_x, rand_y)
            nearest_node = self.extend(rand_node)
            
            if nearest_node and distance(nearest_node, self.goal) < 0.5:
                print("YAY you found it 1 ")
                self.goal.parent = nearest_node
                return self.goal

        return None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

# basic information
min_x=min_y=0
max_x=max_y=10
start = (1,1)
goal = (9,8)
obstacle_list = [(2, 2), (2, 3), (2, 4), (5, 6),(5,5),(6,6),(7,3),(7,4),(7,5),(7,6),(8,6)]

# RRT
rrt = RRT(start, goal, obstacle_list,2000,0.5,min_x,max_x,min_y,max_y)
result = rrt.plan()

# path planning
if result:
    path = []
    node = result
    while node:
        path.append((node.x, node.y))
        node = node.parent
    
    # generate image of path and obstacles
    plt.figure(1)
    plt.plot([node[0] for node in path], [node[1] for node in path], '-o')
    for obstacle in obstacle_list:
        plt.plot(obstacle[0], obstacle[1], 'ro')
    plt.plot(start[0], start[1], 'go')
    plt.plot(goal[0], goal[1], 'bo')
    plt.xlim(min_x,max_x)
    plt.ylim(min_y,max_y)
    plt.grid(True)
    plt.show()
else:
    print("failed to find path")

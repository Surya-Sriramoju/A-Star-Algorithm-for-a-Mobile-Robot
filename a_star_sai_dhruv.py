'''authors : Sai Surya Sriramoju, Dhruv Sharma
github - https://github.com/Surya-Sriramoju/A-Star-Algorithm-for-a-Mobile-Robot
'''
import numpy as np
import cv2 
from queue import PriorityQueue
import time

def shapes(map, image, clearance, radius):
    clearance_radius = clearance+radius
    ##hexagon##
    hexagon_vertices = [[300,50],[365,88],[365,162],[300,200],[235,162],[235,88]]
    hexagon_vertices = np.array(hexagon_vertices)
    map = cv2.fillPoly(map, [hexagon_vertices], color=255)
    map = cv2.polylines(map, [hexagon_vertices], isClosed=True, color=255, thickness=clearance_radius)
    image = cv2.fillPoly(image, [hexagon_vertices], color=(255,255,0))
    image = cv2.polylines(image, [hexagon_vertices], isClosed=True, color=(255,255,255), thickness=clearance_radius)

    ##rectangles##
    rectangle_vertices_1 = np.array([[100,0],[100,100],[150,100],[150,0]])
    map = cv2.fillPoly(map, [rectangle_vertices_1], color=255)
    map = cv2.polylines(map, [rectangle_vertices_1], isClosed=True, color=255, thickness=clearance_radius)
    image = cv2.fillPoly(image, [rectangle_vertices_1], color=(255,255,0))
    image = cv2.polylines(image, [rectangle_vertices_1], isClosed=True, color=(255,255,255), thickness=clearance_radius)

    rectangle_vertices_2 = np.array([[100,250],[100,150],[150,150],[150,250]])
    map = cv2.fillPoly(map, [rectangle_vertices_2], color=255)
    map = cv2.polylines(map, [rectangle_vertices_2], isClosed=True, color=255, thickness=clearance_radius)
    image = cv2.fillPoly(image, [rectangle_vertices_2], color=(255,255,0))
    image = cv2.polylines(image, [rectangle_vertices_2], isClosed=True, color=(255,255,255), thickness=clearance_radius)

    ##triangles##
    triangle_vertices = np.array([[460,125-100],[460,125+100],[510,125]])
    map = cv2.fillPoly(map, [triangle_vertices], color=255)
    map = cv2.polylines(map, [triangle_vertices], isClosed=True, color=255, thickness=clearance_radius)
    image = cv2.fillPoly(image, [triangle_vertices], color=(255,255,0))
    image = cv2.polylines(image, [triangle_vertices], isClosed=True, color=(255,255,255), thickness=clearance_radius)
    image = cv2.copyMakeBorder(image, clearance_radius, clearance_radius, clearance_radius, clearance_radius, cv2.BORDER_CONSTANT, value=[255, 255, 255])
    map = cv2.copyMakeBorder(map, clearance_radius, clearance_radius, clearance_radius, clearance_radius, cv2.BORDER_CONSTANT, value=[255])

    return image, map

def populate_nodes(current,free_points,step_size):
    nodes = []
    actions = [-60, -30, 0, 30, 60]
    for action in actions:
        new_angle = current[-1] + action
        new_x = int(current[0]+(step_size*np.cos(np.radians(new_angle))))
        new_y = int(current[1]+(step_size*np.sin(np.radians(new_angle))))
        
        if (new_x, new_y) in free_points:
            nodes.append((new_x, new_y, new_angle))
    return nodes

def calculate_distance(node_1, node_2):
    distance = abs(np.linalg.norm(np.asarray(node_1[:2]) - np.asarray(node_2[:2])))
    return distance

def calculate_cost(new_node, current_node, node_cost, parents, step_size, goal, thresh, open):
    reached = False
    distance = calculate_distance(new_node, current_node)
    new_cost = node_cost[current_node] + step_size + distance
    temp_cost = node_cost.get(new_node)

    if not temp_cost or (temp_cost > new_cost):
        node_cost[new_node] = new_cost
        parents[new_node[:2]] = current_node[:2]
        open.put((new_cost,new_node))
    if calculate_distance(goal, new_node) < thresh:
        reached = True
    return reached, open, node_cost, parents, new_node

def backtrack(parents, start, visited):
    current_node = visited[-1]
    parent_node = parents[current_node]
    path = [current_node]
    while not parent_node == start[:2]:
        current_node = parent_node
        parent_node = parents[current_node]
        path.append(current_node)
    path.append(start[:2])
    return path[::-1]

def astar(start, goal, free_points, step_size, thresh,img):
    open = PriorityQueue()
    open.put((0,start))
    visited = []
    node_cost = {}
    parents = {}
    node_cost[start] = 0
    reached = False

    while not reached:

        _, current_node = open.get()
        visited.append(current_node[:2])
        if calculate_distance(current_node, goal) < thresh:
            print("Goal Reached!")
            reached = True
            parents[goal[0:2]] = start[0:2]
            break
        new_nodes = populate_nodes(current_node,free_points,step_size)
        # print(new_nodes)
        for node in new_nodes:
            # print(node)
            
            if node[:2] not in visited:
                # visited.append(node[:2])
                # print(node[:2])
                a = time.time()
                reached, open, node_cost, parents, new_node = calculate_cost(node, current_node, node_cost, parents, step_size, goal, thresh, open)
                
            if reached:
                print("Goal Reached!")
                visited.append(new_node[:2])
                break
    return parents, visited

def video_save_demo(path, visited ,image, value):
    temp = image.copy()
    total = visited + path
    visited_len = len(visited)
    path_len = len(path)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video = cv2.VideoWriter("A_star_implementation.avi", fourcc, 1000, (image.shape[1], image.shape[0]))
    i = 0
    for index in range(len(total)-1):
        if i<visited_len:
            image[total[index][1], total[index][0]] = (0,255,0)
        else:
            cv2.line(image, (total[index][1], total[index][0]),(total[index+1][1], total[index+1][0]), [0,0,255], 2)
            time.sleep(0.1)
        temp_image = image[::-1,:,:]
        i += 1
        video.write(temp_image)
        if value == 1:
            cv2.imshow("Demo", temp_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.imshow("Demo", temp_image)
    cv2.waitKey(0)
        
def main():
    map = np.zeros((250, 600))
    image = np.zeros((250, 600,3), dtype=np.uint8)
    clearance = int(input("Enter the clearance value: "))
    radius = int(input("Enter the robot radius: "))
    step_size = int(input("Enter the step size: "))

    img, map = shapes(map, image, clearance, radius)
    y,x = np.where(map==0)
    free_points = []
    for i,j in zip(x,y):
        free_points.append((i,j))
    
    action_set = [-60, -30, 0, 30, 60]

    while True:
        points_start = input('Enter the start_node in the format x,y,theta: ')
        x_start = int(points_start.split(",")[0])
        y_start = int(points_start.split(",")[1])
        theta_start = int(points_start.split(",")[2])
        
        points_goal = input('Enter the goal_node in the format x,y,theta: ')
        x_goal = int(points_goal.split(",")[0])
        y_goal = int(points_goal.split(",")[1])
        theta_goal = int(points_start.split(",")[2])
        
        try:
            if ((x_start, y_start) in free_points and (x_goal, y_goal) in free_points) and ((theta_start in action_set) and (theta_goal in action_set)):
                break
            else:
                print("Please enter the points which are in free space and right orientation!")
                continue
        except:
            print("Dimensions more than the given map, try again!")
    thresh = 1.5
    
    start = (x_start,y_start,theta_start)
    goal = (x_goal, y_goal,theta_goal)
    for i in range(0,5):
        for j in range(0,5):
            img[y_goal+j, x_goal+i] = (0,0,255)
    print("Starting the algorithm....")
    a = time.time()
    parents, visited = astar(start, goal, free_points, step_size, thresh, img)
    print('time taken to find the goal: ', time.time()-a)

    path = backtrack(parents, start, visited)
    
    value = int(input("for viewing and saving the video press - 1, for just saving the video press - 2: "))

    video_save_demo(path, visited, img, value)

if __name__ == '__main__':
    main()
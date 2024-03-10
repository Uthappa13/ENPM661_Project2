# import the required libraries 
import time
import cv2
import heapq as hq
import copy
import numpy as np


#################################### Functions to define the action sets ########################################

# Function definition to move the robot up
def move_up(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)
    
    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]-1 > 0) and (map[position[1]-1][position[0]][0]<255):
        position[1] = position[1] - 1 
        return True,tuple(position)
    else:
        return False,tuple(position)



# Function definition to move the robot down
def move_down(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]+1 < map.shape[0]) and (map[position[1]+1][position[0]][0]<255):
        position[1] = position[1] + 1 
        return True,tuple(position)
    else:
        return False,tuple(position)



# Function definition to move the robot left
def move_left(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[0]-1 > 0) and (map[position[1]][position[0]-1][0]<255):
        position[0] = position[0] - 1 
        return True,tuple(position)
    else:
        return False,tuple(position)
    



# Function definition to move the robot right
def move_right(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[0]+1 < map.shape[1]) and (map[position[1]][position[0]+1][0]<255):
        position[0] = position[0] + 1 
        return True,tuple(position)
    else:
        return False,tuple(position)



# Function definition to move the robot up-left
def move_up_left(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]-1 > 0) and (position[0]-1 >0) and (map[position[1]-1][position[0]-1][0]<255):
        position[1] = position[1] - 1
        position[0] = position[0] - 1 
        return True,tuple(position)
    else:
        return False,tuple(position)
    


# Function definition to move the robot up-right
def move_up_right(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]-1 > 0) and (position[0]+1 <map.shape[1]) and (map[position[1]-1][position[0]+1][0]<255):
        position[1] = position[1] - 1
        position[0] = position[0] + 1 
        return True,tuple(position)
    else:
        return False,tuple(position)
    


# Function definition to move the robot down-left
def move_down_left(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)
    
    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]+1 < map.shape[0]) and (position[0]-1 >0) and (map[position[1]+1][position[0]-1][0]<255):
        position[1] = position[1] + 1
        position[0] = position[0] - 1 
        return True,tuple(position)
    else:
        return False,tuple(position)
    


# Function definition to move the robot down-right
def move_down_right(current_node_location,map):
    # make a copy of the current position so that the original position remains unchanged
    position = copy.deepcopy(current_node_location)

    # check if the new position is within limits and not in the obstacle space (check the blue color channel)
    if(position[1]+1 < map.shape[0]) and (position[0]+1 <map.shape[1]) and (map[position[1]+1][position[0]+1][0]<255):    
        position[1] = position[1] + 1
        position[0] = position[0] + 1 
        return True,tuple(position)
    else:
        return False,tuple(position)
    
    

####################################### Function for backtracking ###############################################

def function_backtracking(initial_state, goal_state, list_close, map):
    
    # Create a videowriter object to visualize the path finding process
    out = cv2.VideoWriter('Code_running_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 1000, (map.shape[1], map.shape[0]))

    path_stack = []
    
    # Iterate through all the keys and set the color to white to mark the explored nodes
    for key in list_close.keys():    
        map[key[1]][key[0]] = [255, 255, 255]
        
        cv2.imshow("Project 2 output", map)
        cv2.waitKey(1)
        out.write(map)

    # Backtracking from the goal state to the initial state   
    parent_node_current_location = list_close[tuple(goal_state)]
    path_stack.append(goal_state)
    
    while parent_node_current_location != initial_state:
        path_stack.append(parent_node_current_location)
        parent_node_current_location = list_close[tuple(parent_node_current_location)]
    
    #Defining initial state for visualization
    cv2.circle(map, tuple(initial_state), 5, (0, 255, 0), -1)
    
    #Defining goal state for visualization
    cv2.circle(map, tuple(goal_state), 5, (0, 255, 0), -1)
    path_stack.append(initial_state)
    
    # Pop points from the path stack and color them to show the shortest path
    while len(path_stack) > 0:
        current_loaction_path = path_stack.pop()
        map[current_loaction_path[1]][current_loaction_path[0]] = [0, 0, 255]
        out.write(map)
        cv2.imshow("Project 2 output", map)
    out.release() 


############################################### Map definition ##################################################
    
# Create an empty map
map = np.ones((500,1200,3),dtype="uint8") 

# Input the array for the first obstacle
obstacle_1 =[np.array([[100,0],[100,400],[175,400],[175,0]])]
cv2.fillPoly(map, obstacle_1, color =(255,0, 0))

# Define the clearance around the obstacle space
cv2.line(map, (95,0), (95,405), (255,0, 0), thickness = 1)
cv2.line(map, (95,405), (180,405), (255,0, 0), thickness = 1)
cv2.line(map, (180,0), (180,405), (255,0, 0), thickness = 1) 

# Input the array for the second obstacle
obstacle_2 = [np.array([[275,100],[275,500],[350,500],[350,100]])]
cv2.fillPoly(map, obstacle_2, color=(255,0, 0))

# Define the clearance around the obstacle space
cv2.line(map, (270,95), (270,500), (255,0, 0), thickness = 1)
cv2.line(map, (270,95), (355,95), (255,0, 0), thickness = 1)
cv2.line(map, (355,95), (355,500), (255,0, 0), thickness = 1) 

# Input the array for the third obstacle
obstacle_3 = [np.array([[800, 250], [725, 379], [575, 379], [500, 250], [574, 120], [724, 120]],np.int32)]
cv2.fillPoly(map, obstacle_3, color=(255,0, 0))
obstacle_3_c = [np.array([[805, 250], [727, 384], [572, 384], [495, 250], [572, 115], [727, 115]],np.int32)]
cv2.polylines(map,obstacle_3_c,True,color=(255,0, 0),thickness=1)

# Input the array for the fourth obstacle
obstacle_4_1 = [np.array([[900,50],[900,125],[1100,125],[1100,50]])]
cv2.fillPoly(map, obstacle_4_1, color=(255,0, 0))

obstacle_4_2 = [np.array([[1020,125],[1020,375],[1100,375],[1100,125]])]
cv2.fillPoly(map, obstacle_4_2, color=(255,0, 0))

obstacle_4_3 = [np.array([[900,375],[900,450],[1100,450],[1100,375]])]
cv2.fillPoly(map, obstacle_4_3, color=(255,0, 0))

# Define the clearance around the obstacle space
cv2.line(map, (895,45), (1105,45), (255,0, 0), thickness = 1)
cv2.line(map, (1105,45), (1105,455), (255,0, 0), thickness = 1)
cv2.line(map, (1105,455), (895,455), (255,0, 0), thickness = 1)
cv2.line(map, (895,455), (895,370), (255,0, 0), thickness = 1)
cv2.line(map, (895,370), (1015,370), (255,0, 0), thickness = 1)
cv2.line(map, (1015,370), (1015,130), (255,0, 0), thickness = 1) 
cv2.line(map, (1015,130), (895,130), (255,0, 0), thickness = 1)
cv2.line(map, (895,130), (895,45), (255,0, 0), thickness = 1) 

# Define the clearance around the boundary wall
boundary_clearance = [np.array([[0,0], [1200,0], [1200,500], [0,500]])]
cv2.polylines(map,boundary_clearance,True,color=(255,0, 0),thickness=5)


########################### Use of semi algebraic models for obstacle space definition########################



###################################### Inputs and initializations ############################################

# Lists to store the initial and goal states
initial_state = []
goal_state = []

# Function definition for user input and relevancy check
def user_input(type_of_point):
    while True:
        x = input(f"Please enter X Coordinate of {type_of_point}: ")
        y = input(f"Please enter Y Coordinate of {type_of_point}: ")
        # check if the initial state is valid to the map dimensions
        if not (0 <= int(x) < map.shape[1] and 0 <= int(y) < map.shape[0]):
            print("Enter valid coordinates as per map dimensions")
            
        #check if the initial state overlaps the obstacle space 
        elif map[map.shape[0] - int(y) - 1][int(x)][0] == 255:
            print(f"ERROR !!! {type_of_point.lower()} overlaps with obstacle")
        else:
            return [int(x), int(y)]


# Function call for the user inputs
initial_state = user_input("Initial state")
goal_state = user_input("Goal state")


goal_state[1] = map.shape[0]-1 - goal_state[1]
initial_state[1] = map.shape[0]-1 - initial_state[1]

# Define a time variable to print the runtime
start_time = time.time()

# Define a open list
list_open = []

# Define a closed list
list_close = {}



################################################ Implementation #######################################################  

# Initialize a flag to indicate whether the goal has been reached and backtracking should occur
back_tracking_flag = False

# Open list is turned into a heap for priority queue operations
hq.heapify(list_open)

# Initial state is pushed into the queue with cost 0 and itself as the parent node
hq.heappush(list_open,[0,initial_state,initial_state])

# Run the algorithm as long as there are nodes in the open list
while(len(list_open)>0):
    # Pop the node with the lowest cost
    current_node_location = hq.heappop(list_open)
    # The current node is added to closed list and keeps track of the explored nodes
    list_close[(current_node_location[2][0],current_node_location[2][1])] = current_node_location[1]
    #initailize the current cost
    current_node_location_cost = current_node_location[0]
    
    # If the current node is the goal state, set the flag to True
    if list(current_node_location[2]) == goal_state:
        back_tracking_flag = True
        print("Back Track")
        break
    
    # Exploring new nodes and adding them to the open list
    for motion in [(move_up, 1),(move_down, 1),(move_left, 1),(move_right, 1),(move_up_left, 1.4),(move_up_right, 1.4),(move_down_left, 1.4),(move_down_right, 1.4)]:
        flag, position = motion[0](current_node_location[2], map)
        # Check if the new node explored is not in the open list
        if flag and position not in list_close:
            temp = False
        
            # Check if the new node explored is already in the open list
            for i in range(len(list_open)):
                if list_open[i][2] == list(position):
                    temp = True
                    # Check if the new node will result in a lower cost
                    if (current_node_location_cost + motion[1]) < list_open[i][0]:
                        list_open[i][0] = current_node_location_cost + motion[1]
                        list_open[i][1] = current_node_location[2]
                        hq.heapify(list_open)
                    break
                
            # Add the new explored node to open list if not already present
            if not temp:
                hq.heappush(list_open, [current_node_location_cost + motion[1], current_node_location[2], list(position)])
                hq.heapify(list_open)

# Backtracking function is called if the flag is true 
if(back_tracking_flag):
    function_backtracking(initial_state,goal_state,list_close,map)
else:
    print("ERROR!!! The given path cannot be found")

goal_reached_time = time.time() 

cv2.waitKey(0) 
#end all windows
cv2.destroyAllWindows() 

print("The total time taken is : ",goal_reached_time-start_time) 

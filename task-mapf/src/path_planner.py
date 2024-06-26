#!/usr/bin/env python3
from __future__ import annotations
import math
import numpy as np
import cv2
from copy import deepcopy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point32, Point, Pose, PoseStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from tf.transformations import euler_from_quaternion
from priority_queue import PriorityQueue




class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        # rospy.Subscriber('/odom', Odometry, self.update_odometry)


        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")

        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service("plan_path", GetPlan, self.plan_path)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.a_star_pub = rospy.Publisher("/a_star", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("/apath", Path, queue_size=10)
        ## Create publishers for C-space (inflated obstacles)
        self.cspace_pub = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)
        ## Initialize the request counter
        self.request_counter = 0
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")
        self.map=self.request_map()
        self.cspaced=self.calc_cspace(self.map, 1)
        self.baseCspaced=deepcopy(self.cspaced)




    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: [int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return p[1] * mapdata.info.width + p[0]
    
    @staticmethod
    def index_to_grid(mapdata: OccupancyGrid, i: int) -> tuple[int, int]:
        """
        Returns the (x,y) coordinates corresponding to the given index in the occupancy grid.
        :param i [int] The index.
        :return  [(int, int)] The cell coordinate.
        """
        return (i % mapdata.info.width, int(i / mapdata.info.width))

    @staticmethod
    def euclidean_distance(p1: [float, float], p2: [float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        return math.dist(p1, p2)
    
    @staticmethod
    def euclidean_distance_index(mapdata: OccupancyGrid, p1: int, p2: int) -> float:
        """
        Calculates the Euclidean distance between two points.
        :p1 index of first point
        :p2 index of second point
        :return   [float]          distance.
        """

        point1 = PathPlanner.index_to_grid(mapdata, p1)
        point2 = PathPlanner.index_to_grid(mapdata, p2)

        return PathPlanner.euclidean_distance(point1, point2)
    
    @staticmethod
    def path_length(mapdata: OccupancyGrid, path: Path) -> float:
        """
        Calculates the length of a path in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param path    [(int)]         The path as a list of indices.
        :return        [float]         The length of the path.
        """
        length = 0
        for i in range(len(path.poses) - 1):
            p1 = [path.poses[i].pose.position.x, path.poses[i].pose.position.y]
            p2 = [path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y]
            length += PathPlanner.euclidean_distance(p1, p2)
        return length

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: [int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """

        cellSize=mapdata.info.resolution
        gridPos=(p[0]*cellSize,p[1]*cellSize)
        origin_quat=mapdata.info.origin.orientation
        (roll, pitch, yaw) = euler_from_quaternion([origin_quat.x, origin_quat.y, origin_quat.z, origin_quat.w])
        origin_pos=deepcopy(mapdata.info.origin.position)
        origin_rot=np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw),  math.cos(yaw)]])
        worldPosArr=np.add(np.matmul(origin_rot,np.vstack(np.array([gridPos[0],gridPos[1]]))),np.vstack(np.array([origin_pos.x,origin_pos.y])))
        worldPoint=origin_pos
        worldPoint.x=worldPosArr[0]+cellSize/2
        worldPoint.y=worldPosArr[1]+cellSize/2
        
        return worldPoint
        
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> [int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        cellSize=mapdata.info.resolution
        origin_pos=mapdata.info.origin.position
        origin_quat=mapdata.info.origin.orientation
        (roll, pitch, yaw) = euler_from_quaternion([origin_quat.x, origin_quat.y, origin_quat.z, origin_quat.w])
        wx_relOr=wp.x-origin_pos.x
        wy_relOr=wp.y-origin_pos.y
        wp_relOr=np.vstack(np.array([wx_relOr,wy_relOr]))
        origin_rot=np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw),  math.cos(yaw)]])
        gridPoint=np.matmul(np.linalg.inv(origin_rot),wp_relOr)
        gridx=int((gridPoint[0])/cellSize)
        gridy=int((gridPoint[1])/cellSize)
        return (gridx, gridy)       
   
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: [[int, int]]) -> [PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        # REQUIRED CREDIT
        #TODO!!!!!!!!!
        pass  

    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: [int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        width = mapdata.info.width
        height = mapdata.info.height
        
        if p[0] < 0 or p[0] >= width or p[1] < 0 or p[1] >= height:
            return False
        index=PathPlanner.grid_to_index(mapdata,p)
        
        val = mapdata.data[index]
        # print(p,val)
        return val < 100 and val != -1         

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: [int, int]) -> [[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        walkable_neighbors = []
        if PathPlanner.is_cell_walkable(mapdata, (p[0]-1, p[1])):
            walkable_neighbors.append((p[0]-1, p[1]))
        if PathPlanner.is_cell_walkable(mapdata, (p[0]+1, p[1])):
            walkable_neighbors.append((p[0]+1, p[1]))
        if PathPlanner.is_cell_walkable(mapdata, (p[0], p[1]-1)):
            walkable_neighbors.append((p[0], p[1]-1))
        if PathPlanner.is_cell_walkable(mapdata, (p[0], p[1]+1)):
            walkable_neighbors.append((p[0], p[1]+1))
        return walkable_neighbors

    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: [int, int]) -> [[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        walkable_neighbors = []
        
        if PathPlanner.is_cell_walkable(mapdata, (p[0]-1, p[1]-1)):
            walkable_neighbors.append((p[0]-1, p[1]-1))
        if PathPlanner.is_cell_walkable(mapdata, (p[0]+1, p[1]+1)):
            walkable_neighbors.append((p[0]+1, p[1]+1))
        if PathPlanner.is_cell_walkable(mapdata, (p[0]-1, p[1]+1)):
            walkable_neighbors.append((p[0]-1, p[1]+1))
        if PathPlanner.is_cell_walkable(mapdata, (p[0]+1, p[1]-1)):
            walkable_neighbors.append((p[0]+1, p[1]-1))
        walkable_neighbors+=(PathPlanner.neighbors_of_4(mapdata, p))
        return walkable_neighbors
    
    @staticmethod
    def neighbors_within_dist(mapdata: OccupancyGrid, p: tuple[int, int], distance: int) -> list[tuple[int, int]]:
        """
        Returns the walkable "neighbor" cells of (x,y) within padding distance in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :param padding [int]           The distance from the cell.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        walkable_neighbors = []
        # Loop through all cells within distance of p
        for i in range(p[0]-distance, p[0]+distance+1):
            for j in range(p[1]-distance, p[1]+distance+1):
                if PathPlanner.is_cell_walkable(mapdata, (i, j)) and round(math.dist(p, (i, j))) <= distance:
                    # If the cell is walkable and within distance, add it to the list
                    walkable_neighbors.append((i, j))
        return walkable_neighbors

    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service("static_map")
        rospy.loginfo("got map")
        get_map=rospy.ServiceProxy("static_map",GetMap)
        map=get_map().map
        if map is None:
            rospy.logerr("Could not request the map")
            return None
        return map

    @staticmethod
    def calc_cspace(mapdata: OccupancyGrid, paddingVal: float = 1.2) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")

        # Convert existing map data to a NumPy array for easier manipulation
        map_array = np.array(mapdata.data, dtype=np.int8).reshape(
            (mapdata.info.height, mapdata.info.width))

        # get the robot radius, convert to cell count
        # add 25% padding to the radius
        robot_radius_cells = math.ceil(
            (0.178 * 0.5*paddingVal) / mapdata.info.resolution)
        print(
            f"The robot's radius is {robot_radius_cells} map cells wide (Resolution: {mapdata.info.resolution})")
        additional_clearance_cells = 0  # Adjust as needed for additional clearance
        padding = robot_radius_cells + additional_clearance_cells

        # Find obstacles in the map and dilate them
        # obstacles = map_array < 255 # Remove unknown cells
        # Assuming obstacles are represented by values greater than 50
        obstacles = map_array > 50
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * padding + 1, 2 * padding + 1))
        dilated_obstacles = cv2.dilate(obstacles.astype(np.uint8), kernel)

        # Set the inflated obstacles to 100 in the map data
        map_array[dilated_obstacles > 0] = 100
        map_array[obstacles > 0] = 0
        # Convert map array back to a flat list
        new_map_data = map_array.flatten().tolist()
        
        newmapdata = OccupancyGrid()
        newmapdata.header = mapdata.header
        newmapdata.info = mapdata.info
        # Convert list back to tuple so we can publish it
        newmapdata.data = tuple(new_map_data)

        # Create a GridCells message and publish it
        cspace = GridCells()
        cspace.header.frame_id = "map"
        cspace.cell_width = mapdata.info.resolution
        cspace.cell_height = mapdata.info.resolution
        cspace.cells = []

        for i in range(len(newmapdata.data)):
            if newmapdata.data[i] == 100:
                # Publish only the inflated cells
                cspace.cells.append(PathPlanner.grid_to_world(
                    newmapdata, (i % newmapdata.info.width, int(i / newmapdata.info.width))))

        # rospy.loginfo("Publishing C-Space")
        # PathPlanner.cspace_pub.publish(cspace)

        # Return the C-space
        return newmapdata
    
    @staticmethod
    def calc_robots(mapdata: OccupancyGrid, px: list[float], py: list[float], paddingVal: float = 2, ignoreRobot=-1) -> OccupancyGrid:
        mapthedata=list(mapdata.data)
        robot_radius_cells = math.ceil(
                    (0.178 * 0.5) / mapdata.info.resolution)
        robot_radius_cells_padded = math.ceil(
                    (0.178 * 0.5*paddingVal) / mapdata.info.resolution)
        for i in range(len(px)):
            if i!=ignoreRobot-1:
                cellCord=PathPlanner.world_to_grid(mapdata,Point(px[i],py[i],0))               
                mapthedata[PathPlanner.grid_to_index(mapdata,cellCord)]=100
                cellsBlocked=PathPlanner.neighbors_within_dist(mapdata,cellCord,robot_radius_cells_padded)
                for cell in cellsBlocked:
                    mapthedata[PathPlanner.grid_to_index(mapdata,cell)]=100
            else:
                cellCord=PathPlanner.world_to_grid(mapdata,Point(px[i],py[i],0))
                mapthedata[PathPlanner.grid_to_index(mapdata,cellCord)]=0
                cellsBlocked=PathPlanner.neighbors_within_dist(mapdata,cellCord,robot_radius_cells)
                for cell in cellsBlocked:
                    mapthedata[PathPlanner.grid_to_index(mapdata,cell)]=0
        mapdata.data=tuple(mapthedata)  
           
       
        return mapdata


    @staticmethod
    def calc_gradspace(mapdata: OccupancyGrid, padding: int = 2) -> OccupancyGrid:
        """
        Calculates the C-Space with a gradient indicating the distance from obstacles.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space with a distance gradient.
        """
        rospy.loginfo("Calculating C-Space with Gradient")

        # Convert existing map data to a NumPy array for easier manipulation
        map_array = np.array(mapdata.data, dtype=np.uint8).reshape(
            (mapdata.info.height, mapdata.info.width))

        # Create a binary mask indicating obstacle cells
        # Assuming obstacles are represented by values less than 50
        obstacle_mask = map_array < 50

        # Calculate the distance transform
        distance_transform = cv2.distanceTransform(
            obstacle_mask.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

        # Normalize the distance transform to a range between 0 and 100 (adjust as needed)
        max_distance = np.max(distance_transform)
        normalized_distance = (distance_transform / max_distance) * 100

        # Set the inflated obstacles to the normalized distance values in the map data
        # normalized_distance[obstacle_mask]
        map_array[obstacle_mask] = distance_transform[obstacle_mask]

        # Convert map array back to a flat list
        new_map_data = map_array.flatten().tolist()

        newmapdata = OccupancyGrid()
        newmapdata.header = mapdata.header
        newmapdata.info = mapdata.info
        # Convert list back to tuple so we can publish it
        newmapdata.data = tuple(new_map_data)
        # self.grad_pub.publish(mapdata)
        # Return the C-space with gradient
        return newmapdata

    @staticmethod
    def a_star(mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int], gradSpace: OccupancyGrid=None,otherPaths=None,initialHeading=None) -> list[tuple[int, int]]:
        """
        Calculates the Optimal path using the A* algorithm.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param start [int]           The starting grid location to pathfind from.
        :param goal [int]           The target grid location to pathfind to.
        :return        [list[tuple(int, int)]] The Optimal Path from start to goal.
        """
        robot_radius_cells = math.ceil(
                    (0.178 * 0.5) / mapdata.info.resolution)

        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" %
                      (start[0], start[1], goal[0], goal[1]))

        # Check if start and goal are walkable
        # startindex=PathPlanner.grid_to_index(mapdata,start)
        # print(start,startindex)
        if (not PathPlanner.is_cell_walkable(mapdata, start)):
            # print(mapdata.data[self.grid_to_index(mapdata,start)])
            rospy.loginfo('start blocked')
            i=1
            newstart=start
            while not PathPlanner.is_cell_walkable(mapdata, newstart):
                newstart=PathPlanner.neighbors_within_dist(mapdata,start,i)[0]
                i+=1
                if i>robot_radius_cells*1.5:
                    rospy.loginfo('no start')
                    return []

        if (not PathPlanner.is_cell_walkable(mapdata, goal)):
            # print(mapdata.data[self.grid_to_index(mapdata,goal)])
            rospy.loginfo('goal blocked')
            return []

        # Priority queue for the algorithm
        q = PriorityQueue()
        # element ((Cords),(Prev),g)

        # dictionary of all the explored points keyed by their coordinates tuple
        explored = {}
        exppoints = []
        checkPaths=otherPaths!=None
        checkFat=gradSpace!=None
        q.put((start, None, 0,0), PathPlanner.euclidean_distance(start, goal))
        while not q.empty():
            element = q.get()
            cords = element[0]
            prev = element[1]
            g = element[2]  # cost sof far at this element
            ts = element[3] # time step
            if cords == start:
                prev = cords
            heading = math.atan2(cords[1]-prev[1], cords[0]-prev[0])
            if initialHeading!=None and ts == 0:
                heading=initialHeading
            explored[(cords[0],cords[1])] = element
            exppoints.append(cords)

            if cords == goal:
                goalts=ts
                # Once we've hit the goal, reconstruct the path and then return it
                return PathPlanner.reconstructPath(explored, start, goal,goalts)

            neighbors = PathPlanner.neighbors_of_8(mapdata, cords)

            for i in range(len(neighbors)):
                neighbor = neighbors[i]
                manhatdis = abs(neighbor[0]-cords[0])+abs(neighbor[1]-cords[1])
                if manhatdis > 1:  # Oridinal Neighbors
                    gfactor = 1.4
                else:  # Cardinal Neighbors
                    gfactor = 1
                skip=False
                turningFactor = 0
                newHeading = math.atan2(neighbor[1]-cords[1], neighbor[0]-cords[0])
                turnAngle =newHeading-heading
                if turnAngle<-math.pi:
                    turnAngle+=2*math.pi
                elif turnAngle>math.pi:
                    turnAngle-=2*math.pi

                turningFactor = math.degrees(turnAngle)*.01#np.rad2deg(np.unwrap([turnAngle]))[0]*.01
                
                if initialHeading!=None and ts==0:
                    if abs(turnAngle)>math.pi/3:
                        skip=True
                cspaceFactor = 0
                
                if checkFat:
                    dis = gradSpace.data[PathPlanner.grid_to_index(gradSpace, neighbor)]
                    
                    if dis < 15:
                        cspaceFactor = (gfactor*(16-dis)**2)*.5
                    elif dis < 20:
                        cspaceFactor = gfactor*(20-dis)*.05
                if checkPaths:         
                    for path in otherPaths:
                        if ts+1<len(path):
                            pathpoint = path[ts+1]
                        elif len(path)>0:
                            pathpoint = path[-1]
                        else:
                            pathpoint = [math.inf,math.inf]
                        dis=PathPlanner.euclidean_distance(neighbor,pathpoint) 
                        if dis < robot_radius_cells*3.5:
                            skip=True
                            break                  

                # print(explored)
                if (not skip) and (explored.get((neighbor[0],neighbor[1])) is None or explored.get((neighbor[0],neighbor[1]))[2] > g+gfactor+cspaceFactor):
                    f = g+gfactor + \
                        PathPlanner.euclidean_distance(neighbor, goal) + cspaceFactor + turningFactor
                    # print((neighbor,cords,g+1),f)
                    q.put((neighbor, cords, g+gfactor+cspaceFactor+turningFactor,ts+1), f)

        rospy.loginfo('Could not reach goal')
        return []

    @staticmethod
    def reconstructPath(explored: dict, start: tuple[int, int], goal: tuple[int, int], goalts) -> list[tuple[int, int]]:   
        """
        A helper function to reconstruct the path from the explored dictionary
        :param explored [dict] The dictionary of explored nodes
        :param start [tuple(int, int)] The starting point
        :param goal [tuple(int, int)] The goal point
        :return        [list[tuple(int, int)]] The Optimal Path from start to goal.
        """
        cords = goal
        ts=goalts
        path = []
        # rospy.loginfo('Reconstructing path')
        # print(explored.keys())
        # Loops backwards through the explored dictionary to reconstruct the path
        while cords != start:
            element = explored[(cords[0],cords[1])]
            path = [cords] + path
            cords = element[1]
            ts-=1
            if cords == None:
                # This should never happen given the way the algorithm is implemented
                rospy.loginfo('Could not reconstruct')
                return []
        # path = [start] + path
        return path
    
    @staticmethod
    def optimize_path(path: [[int, int]]) -> [[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        rospy.loginfo("Optimizing path")
        initLen = len(path)
        index = 0
        # Use while loop so we can modify the list as we go
        while index < len(path) - 2:
            # Stop after the third to last point
            current = path[index]
            # Angle to next point
            angleToNext = math.atan2(path[index+1][1] - current[1], path[index+1][0] - current[0])
            # Keep checking until the angle changes
            nextIndex = index + 2
            while nextIndex < len(path):
                # Check if the angle to the next point is the same as the angle to the next next point
                angleToNextNext = math.atan2(path[nextIndex][1] - current[1], path[nextIndex][0] - current[0])
                if angleToNextNext == angleToNext:
                    # If it is, remove the next point
                    # rospy.loginfo(f"Removing point {path[nextIndex - 1]} from between {path[index]} and {path[nextIndex]}")
                    path.pop(nextIndex - 1)
                    # Don't increment nextIndex because we just removed the previous point, 
                    # so the next point is now at the same index
                else:
                    # If it isn't, break out of the loop
                    break
            index += 1
        rospy.loginfo(f"Optimized path from {initLen} to {len(path)} points")
        rospy.loginfo(f"Returning optimized path: {path}")
        return path

    @staticmethod
    def path_to_message(mapdata: OccupancyGrid, path: [[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        # rospy.loginfo("Returning a Path message")
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = []
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = PathPlanner.grid_to_world(mapdata, p)
            path_msg.poses.append(pose)

        return path_msg
     
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        rospy.loginfo("Starting planning")
        # print(self.map)
        ## Request the map
        ## In case of error, return an empty path
        mapdata = self.map#PathPlanner.request_map()
        
        rospy.loginfo("Map Req")
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.cspaced#self.calc_cspace(mapdata, .95)
        ## Calculate the Gradient Space
        gradSpace= self.gradSpace#PathPlanner.calc_gradspace(mapdata)
        ## Execute A*
        start = PathPlanner.world_to_grid(cspacedata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(cspacedata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal,gradSpace)
        ## Optimize waypoints
        waypoints = path#PathPlanner.optimize_path(path)
        pathpoints=[self.grid_to_world(cspacedata,start)]
        for point in waypoints:
            pathpoints.append(self.grid_to_world(cspacedata,point))
        
        path_msg=GridCells()
        path_msg.cell_height=cspacedata.info.resolution
        path_msg.cell_width=cspacedata.info.resolution
        path_msg.header.frame_id="map"
        path_msg.cells=pathpoints
        self.a_star_pub.publish(path_msg)
        ## Return a Path message
        pthmsg=PathPlanner.path_to_message(cspacedata, waypoints)
        self.path_pub.publish(pthmsg)
        return pthmsg

    @staticmethod
    def poseToMatrix(pose:Pose):
        '''
        Converts a Pose message to a 4x4 homogenous transformation matrix
        '''
        quat=pose.orientation
        q1,q2,q3,q0=quat.x,quat.y,quat.z,quat.w
        pos=pose.position
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        matrix = np.array([[r00, r01, r02, pos.x],
                           [r10, r11, r12, pos.y],
                           [r20, r21, r22, pos.z],
                           [  0,   0,   0,     1]])
        return matrix
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.map = PathPlanner.request_map()
        self.cspaced=self.calc_cspace(self.map, 1.25)
        # self.gradSpace=self.calc_gradspace(self.map)
       
        rospy.spin()

if __name__ == '__main__':
    PathPlanner().run()

def testPathOptimize():
    """
    Real quick test script to manually confirm path optimization was working
    """
    path = [(0,0), (0,1), (0,2), (0,3), (1,2), (2,1), (1,1), (0,1)]
    print(path)
    print(PathPlanner.optimize_path(path))

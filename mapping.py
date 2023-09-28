#!/usr/bin/env python3

"""
    # Andreas Naoum
    # 19990831-T690
    # anaoum@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs, ceil, floor

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap

DEBUG = False # Set true for debug/info messages


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False
    
    def print_debug(self, msg):
        if DEBUG:
            print(msg)

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """

        if DEBUG:
            print("--------- Update Map Start ---------")

        # Position offset between the robot's position and the origin of the map
        offset_x = pose.pose.position.x - origin.position.x 
        offset_y = pose.pose.position.y - origin.position.y

        min_map_x, min_map_y = grid_map.get_width() + 1, grid_map.get_height() + 1
        max_map_x, max_map_y = -1, -1

        obstacles = []

        if DEBUG:
            print("----- Update map called -----")
            print("Offset x = ", offset_x, " y = ", offset_y)

        for i, range_cur in enumerate(scan.ranges):
            # Satisfy the condition values <= range_min or >= range_max
            if scan.range_min < range_cur < scan.range_max:

                # The bearing for the first range in ranges (ranges[0]) is angle_min.
                # The bearing for the second range in ranges (ranges[1]) is angle_min + angle_increment and so on.
                # This way you can get both the bearing and the range.
                bearing = scan.angle_min + scan.angle_increment * i  

                # transform to map frame 
                x, y = range_cur * cos(bearing + robot_yaw), range_cur * sin(bearing + robot_yaw) 
                map_x, map_y = int((x + offset_x)/resolution), int((y + offset_y)/resolution)

                # Supportive messages
                if DEBUG:
                    print("----------------------")
                    print("Laser angle: ", bearing)
                    print("x = ", x + pose.pose.position.x, ", y = ", y + pose.pose.position.y)
                    print("map x = ", map_x, ", map y = ", map_y)

                # Obstacles
                if DEBUG:
                    print("Add Obstacle: ", (map_x, map_y))
                
                if self.add_to_map(grid_map, map_x, map_y, self.occupied_space):
                    obstacles.append((map_x, map_y))
                    min_map_x, min_map_y = min(min_map_x, map_x), min(min_map_y, map_y)   
                    max_map_x, max_map_y = max(max_map_x, map_x), max(max_map_y, map_y)
                    free_area = self.raytrace((offset_x/resolution, offset_y/resolution), (map_x, map_y))
                    for point in free_area:
                        point_x, point_y = point
                        if self.add_to_map(grid_map, point_x, point_y, self.free_space):
                            if DEBUG:
                                print("Add Free: ", (point_x, point_y))
                        

        # Set occupied space again, not to be overwritten
        list(map(lambda obstacle: self.add_to_map(grid_map, obstacle[0], obstacle[1], self.occupied_space), obstacles))

        """
        For C only!
        Fill in the update correctly below.
        """ 

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        # update.x = min(value[0] for value in obstacles)
        # The minimum y index in 'grid_map' that has been updated
        # update.y = min(value[1] for value in obstacles)
        # Maximum x index - minimum x index + 1
        # update.width = max(value[0] for value in obstacles) - update.x + 1
        # Maximum y index - minimum y index + 1
        # update.height = max(value[1] for value in obstacles) - update.y + 1

        # The minimum x index in 'grid_map' that has been updated
        update.x = min_map_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_map_y
        # Maximum x index - minimum x index + 1
        update.width = max_map_x - update.x + 1
        # Maximum y index - minimum y index + 1
        update.height = max_map_y - update.y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        if DEBUG:
            print("----------------------")
            print("update x = ", update.x, " update.y = ", update.y)
            print("update width = ", update.width, " update height = ", update.height) 

        for grid_y in range(0, update.height):
            list(map(lambda grid_x: update.data.append(grid_map[update.x + grid_x, update.y + grid_y]), range(0, update.width)))

        # list(map(lambda grid_x: update.data.append(grid_map[update.x + grid_x, update.y + grid_y]), range(0, update.width)))

        if DEBUG:
            print("--------- Update Map End ---------")

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update


    def is_occupied(self, grid_map, pos):
        if self.is_in_bounds(grid_map, pos[0], pos[1]) and grid_map[pos[0], pos[1]] == self.occupied_space:
            return True
        return False
    

    def is_in_radius_coverage(self, pos):
        if pos[0] ** 2 + pos[1] ** 2 <= self.radius ** 2:
            return True
        return False
        

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here

        """

        if DEBUG:
            print("--------- Inflate Map Start ---------")

        # Check for every cell in the map if there is a occupied cell in the radius coverage
        for test_y in range(grid_map.get_height()):
            for test_x in range(grid_map.get_width()):
                if not(self.is_occupied(grid_map, (test_x, test_y))): 
                    # self.is_in_bounds(grid_map, test_x + add_x, test_y + add_y) and
                    for add_y in range(-floor(self.radius), ceil(self.radius)):
                        for add_x in range(-floor(self.radius), ceil(self.radius)):
                            if self.is_occupied(grid_map, (test_x + add_x, test_y + add_y)) and self.is_in_radius_coverage((add_x, add_y)):
                                self.add_to_map(grid_map, test_x, test_y, self.c_space)
                                if DEBUG:
                                    print("Occupied space expanded, x = ", test_x, ", y = ", test_y)
                                break
                                                                                                                                                                              
                    # if any((self.is_occupied(grid_map, (test_x + add_x, test_y + add_y)) and self.is_in_radius_coverage((add_x, add_y))) for add_x in range(-floor(self.radius), ceil(self.radius)) for add_y in range(-floor(self.radius), ceil(self.radius))):
                    #    self.add_to_map(grid_map, test_x, test_y, self.c_space)
                    #    if DEBUG:
                    #        print("Occupied space expanded, x = ", test_x, ", y = ", test_y)


        if DEBUG:
            print("--------- Inflate Map End ---------")
        
        # Return the inflated map
        return grid_map

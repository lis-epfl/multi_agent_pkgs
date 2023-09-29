import math
import numpy as np
import random as rd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class VoxelGrid:

    def __init__(self, dimension, voxel_size, origin):
        """
        Initialize a VoxelGrid.

        Args:
            dimension (tuple): 3D vector (size in all 3 directions) in meters .
            voxel_size (float): Scalar (size/side length of each voxel/cube).
            origin (tuple): 3D vector (origin of the voxel grid relative to a world frame centered at 0.0,0.0,0.0).
        """
        self.dimension = dimension
        self.voxel_size = voxel_size
        self.origin = origin
        self.grid_size = self.to_voxel_coordinates(dimension[0] + self.origin[0], 
                                                   dimension[1] + self.origin[1], 
                                                   dimension[2] + self.origin[2]) # Create the size of the grid
        self.data = [0] * (self.grid_size[0] * self.grid_size[1] * self.grid_size[2])
        self.obstacles = [] # List of shapes that will be obstacles. A random volume and a wall are also shapes.

    def set_voxel(self, i, j, k, value):
        """
        Set the value of a voxel at integer coordinates (i, j, k).

        Args:
            i (int): X-coordinate.
            j (int): Y-coordinate.
            k (int): Z-coordinate.
            value (int): Value to set for the voxel.
        """
        index = i + self.grid_size[0] * j + self.grid_size[0] * self.grid_size[1] * k
        self.data[index] = value

    def get_voxel(self, i, j, k):
        """
        Get the value of a voxel at integer coordinates (i, j, k).

        Args:
            i (int): X-coordinate.
            j (int): Y-coordinate.
            k (int): Z-coordinate.

        Returns:
            int: Value of the voxel at the specified coordinates.
        """
        index = i + self.grid_size[0] * j + self.grid_size[0] * self.grid_size[1] * k
        return self.data[index]

    def clear(self):
        """Clear the voxel grid by setting all voxels to 0."""
        self.data = [0] * (self.grid_size[0] * self.grid_size[1] * self.grid_size[2])

    def is_valid_coordinate(self, i, j, k):
        """
        Check if the given coordinates are within the bounds of the voxel grid.

        Args:
            i (int): X-coordinate.
            j (int): Y-coordinate.
            k (int): Z-coordinate.

        Returns:
            bool: True if the coordinates are within bounds, False otherwise.
        """
        return 0 <= i < self.grid_size[0] and 0 <= j < self.grid_size[1] and 0 <= k < self.grid_size[2]

    def to_world_coordinates(self, i, j, k):
        """
        Convert voxel coordinates to world coordinates.

        Args:
            i (int): X-coordinate in voxel space.
            j (int): Y-coordinate in voxel space.
            k (int): Z-coordinate in voxel space.

        Returns:
            tuple: 3D vector representing world coordinates.
        """
        x = self.origin[0] + i * self.voxel_size
        y = self.origin[1] + j * self.voxel_size
        z = self.origin[2] + k * self.voxel_size
        return (x, y, z)

    def to_voxel_coordinates(self, x, y, z):
        """
        Convert world coordinates to voxel coordinates.

        Args:
            x (float): X-coordinate in world space.
            y (float): Y-coordinate in world space.
            z (float): Z-coordinate in world space.

        Returns:
            tuple: 3D vector representing voxel coordinates.
        """
        i = int((x - self.origin[0]) / self.voxel_size)
        j = int((y - self.origin[1]) / self.voxel_size)
        k = int((z - self.origin[2]) / self.voxel_size)
        return (i, j, k)
    
    def visualize(self):
        """
        Visualize the voxel grid using Matplotlib.

        This function creates a 3D scatter plot to represent the voxel grid.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Collect voxel positions where data is not zero
        x, y, z = [], [], []
        for i in range(0, len(self.occupied_voxels), 3):
            x.append(self.occupied_voxels[i])
            y.append(self.occupied_voxels[i+1])
            z.append(self.occupied_voxels[i+2])

        ax.scatter(x, y, z, c='b', marker='o', s=100)  # Blue markers for occupied voxels

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Voxel Grid Visualization')

        ax.set_box_aspect([1, 1, 1]) # To have an orthonormal view

        plt.show()


    def add_shape(self, shape):
        self.obstacles.append(shape)

    def compute_occupancy(self):
        self.occupied_voxels = []
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                for k in range(self.grid_size[2]):       
                    point = np.array(self.to_world_coordinates(i,j,k))
                    for shape in self.obstacles :
                        if shape.includes(point):
                            self.set_voxel(i,j,k,100)
                            self.occupied_voxels.append(point[0])
                            self.occupied_voxels.append(point[1])
                            self.occupied_voxels.append(point[2])
                            break

    def get_occupied_voxels(self):
        """
        Returns two lists : a list of size of obstacles (voxel size) by triplets, and a list that will give the x,y,z positions of given obstacles. Function made to fill out the YAML file
        """
        self.compute_occupancy()
        return [self.voxel_size]*len(self.occupied_voxels), self.occupied_voxels


class Shape:
    """
    Superclass to implement obstacles.
    """
    def __init__(self):
        pass

    def includes(self, point):
        """
        Determines whether a 3D point belongs to the given shape
    
        Args :
        point (tuple) : 3D point under study

        Returns :
        boolean indicating if the wall includes this point
        """
        pass

class Cylinder(Shape):
    """
    Class that implements the cylinder shape
    """
    def __init__(self, axis_origin, axis_direction, radius, cylinder_height = float('Inf')):
        self.axis_origin = np.array(axis_origin) # the starting point of the cylinder 
        self.axis_direction = np.array(axis_direction) # its direction, need not to be a unit vector, but should not be null vector.
        self.axis_direction = self.axis_direction/np.linalg.norm(self.axis_direction)
        self.radius = radius
        self.cylinder_height = cylinder_height

    def includes(self, point):
        """
        Returns True if the given point is inside the cylinder using projection of point onto the axis.
        
        Args :
        point (tuple) : 3D point under study

        Returns :
        boolean indicating if the wall includes this point
        """
        point = np.array(point)
        OP = point - self.axis_origin # relative vector
        if -self.cylinder_height/2 <= np.dot(OP, self.axis_direction) < self.cylinder_height/2: # point is not too far from origin
            projection = np.dot(OP, self.axis_direction) * self.axis_direction # projection on the cylinder axis
            distance_vector = OP - projection # distance to cylinder axis
            if np.linalg.norm(distance_vector) <= self.radius : # distance to cylinder is smaller than radius : point belongs to the cylinder.
                return True
        return False



class Loop(Shape):
    """
    Class that implements the loop obstacle
    """
    def __init__(self, axis_origin, angle, inside_radius, outside_radius, thickness = 1):
        self.axis_origin = np.array(axis_origin)
        self.axis_direction = np.array([np.cos(angle), np.sin(angle), 0])
        # A loop consists only of two cylinders of same thickness and same revolution axis
        self.inner_cylinder = Cylinder(self.axis_origin, self.axis_direction, inside_radius,thickness)
        self.outer_cylinder = Cylinder(self.axis_origin, self.axis_direction, outside_radius,thickness)

    def includes(self, point):
        """
        Determines if a given point stands inside the loop

        Args :
        point (tuple) : 3D point under study

        Returns :
        boolean indicating if the loop includes this point
        """
        return self.outer_cylinder.includes(point) and not(self.inner_cylinder.includes(point)) # Point belongs to loop if it belongs to the outer cylinder without belonging to inner cylinder.

                    
class Wall(Shape):
    """
    Implements the wall shape
    """
    def __init__(self, origin, direction1, direction2 = (0,0,1), length = float('Inf'), width = 2.0, height = float('Inf')):
        """
        Initialize a wall object

        Args :
        origin (tuple) : the starting point for the wall
        direction1 (tuple) : the direction the wall will have (X direction of the plane)
        direction2 (tuple) : the second direction that defines the plane the wall is on (should not be colinear to direction 1)
        length (float) : the length of the wall
        width (float) : the wall's thickness
        heigth (float) : the wall's height
        """
        self.origin = np.array(origin)

        # Two directions define a plane, they should not be colinear
        self.direction1 = np.array(direction1)
        self.direction1 = self.direction1 / np.linalg.norm(self.direction1) # to make it a unit vector

        self.direction2 = np.array(direction2)
        self.direction2 = self.direction2 / np.linalg.norm(self.direction2) # to make it a unit vector

        self.normal = np.cross(self.direction1, self.direction2) # normal vector defined as the third dimension.
        self.normal = self.normal /np.linalg.norm(self.normal) # make it unit

        self.direction2 = np.cross(self.normal, self.direction1) #make the base orthonormal for the wall.

        self.length = length
        self.width = width
        self.height = height
        self.gaps = []

    def includes(self, point):
        """
        Determines if a given point stands inside the wall

        Args :
        point (tuple) : point under study

        Returns :
        boolean indicating if the wall includes this point
        """
        point = np.array(point)
        OP = point - self.origin
        if 0 <= np.dot(self.direction1, OP) < self.length and -self.width < np.dot(self.normal, OP) <= 0 and 0 <= np.dot(self.direction2, OP) < self.height : # is point in wall ?
            for gap in self.gaps :
                if gap.includes(point): # is point in a gap ?
                    return False
            return True
        return False


    def add_square_gap(self, rel_origin, length, height):
        """
        Adds a square gap in the wall

        Args : 
        rel_origin (2D coordinates) : the x and y coordinates of the gap on the wall
        length (float)
        height (float)
        """
        gap_origin = self.origin + self.direction1 * rel_origin[0] + self.direction2 * rel_origin[1]
        gap = Wall(gap_origin, self.direction1, self.direction2, length, self.width, height)
        self.gaps.append(gap)


class RandomVolume(Shape):

    def __init__(self, origin_range, seed = 0):
        self.shapes = []
        self.origin_range = np.array(origin_range)
        self.origin_range[1,:] += self.origin_range[0,:]
        rd.seed(seed)


    def add_random_cylinders(self, 
                            nb_cylinder, 
                            direction_range = [[-1.0,-1.0,-1.0], [1.0,1.0,1.0]], 
                            radius_range = [0.5, 2.0], 
                            height_range = [0.5, 100.0]):
        
        for _ in range(nb_cylinder):
            # Create the parameters for the random cylinder
            origin = self.random_in_range(self.origin_range)
            direction = self.random_in_range(direction_range)
            radius = self.random_in_range(radius_range)
            height = self.random_in_range(height_range)

            cylinder = Cylinder(origin,direction, radius, height)
            self.shapes.append(cylinder)


    def add_random_loops(self, 
                         nb_loops, 
                         angle_range = [0, np.pi], 
                         radius_range = [0.0, 6.0], 
                         thickness_range = [0.5, 2.0]):
        

        for _ in range(nb_loops):
            # Create the parameters for the random cylinder
            origin = self.random_in_range(self.origin_range)
            angle = self.random_in_range(angle_range)

            radius1 = self.random_in_range(radius_range)
            radius2 = self.random_in_range(radius_range)
            inner_radius = min(radius1, radius2)
            outer_radius = max(radius1, radius2)

            thickness = self.random_in_range(thickness_range)

            loop = Loop(origin, angle, inner_radius, outer_radius, thickness)
            self.shapes.append(loop)
        

    def random_in_range(self, bounds):
        """
        Returns a random tuple from same length as the provided range.
        """
        bounds = np.array(bounds)
        if bounds.ndim == 1 : # In that case, we have a single number to output. Example of input : [0.0,1.0]
            return rd.uniform(bounds[0], bounds[1])
        
        # else, the input looks something like this : [[0.0, 0.0], [1.0, 1.0]] -> should output 2 random numbers.
        result = []
        for i in range(len(bounds[0])):
            result.append(rd.uniform(bounds[0,i], bounds[1,i]))
        return result


    def includes(self, point):
        if (self.origin_range[0, 0] <= point[0] < self.origin_range[1,0] 
            and self.origin_range[0, 1] <= point[1] < self.origin_range[1,1]
            and self.origin_range[0, 2] <= point[2] < self.origin_range[1,2]): # Check if point is inside the volume defined by the random volume (x between x_min and x_max, y between ... etc)
            for shape in self.shapes : # then tests whether point belongs a least too one of the shapes in the collection.
                if shape.includes(point):
                    return True
        return False
        

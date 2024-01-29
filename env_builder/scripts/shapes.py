import math
import time
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
                                                   dimension[2] + self.origin[2])  # Create the size of the grid
        self.data = [False] * (self.grid_size[0] *
                               self.grid_size[1] * self.grid_size[2])
        self.occupied_voxels = []
        # List of shapes that will be obstacles. A random volume and a wall are also shapes.
        self.obstacles = []

    def set_voxel(self, i, j, k, value):
        """
        Set the value of a voxel at integer coordinates (i, j, k).

        Args:
            i (int): X-coordinate.
            j (int): Y-coordinate.
            k (int): Z-coordinate.
            value (int): Value to set for the voxel.
        """
        index = i + self.grid_size[0] * j + \
            self.grid_size[0] * self.grid_size[1] * k
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
        index = i + self.grid_size[0] * j + \
            self.grid_size[0] * self.grid_size[1] * k
        return self.data[index]
    

    def clear(self):
        """Clear the voxel grid by setting all voxels to 0."""
        self.data = [0] * (self.grid_size[0] *
                           self.grid_size[1] * self.grid_size[2])

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

    def point_to_voxel_coordinates(self, point):
        """
        Convert world coordinates to voxel coordinates.

        Args:
            point (vector): point in world space.

        Returns:
            tuple: 3D vector representing voxel coordinates.
        """
        return self.to_voxel_coordinates(point[0], point[1], point[2])

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

        # Blue markers for occupied voxels
        ax.scatter(x, y, z, c='b', marker='o', s=30)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Voxel Grid Visualization')

        ax.set_box_aspect([1, 1, 1])  # To have an orthonormal view

        plt.show()

    def add_shape(self, shape):
        self.obstacles.append(shape)
    
    def compute_occupancy(self):
        start_time = time.time()
        print('Start generation of obstacles ...')
        # Go through all the obstacles to fill grid
        for obstacle in self.obstacles:
            obstacle.occupy_voxels(self, mesh_size=self.voxel_size/2)
        # Run through grid to output taken obstacles
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                for k in range(self.grid_size[2]):
                    if self.get_voxel(i, j, k):
                        point = np.array(self.to_world_coordinates(i, j, k))
                        self.occupied_voxels.append(
                            point[0] - self.origin[0])
                        self.occupied_voxels.append(
                            point[1] - self.origin[1])
                        self.occupied_voxels.append(
                            point[2] - self.origin[2])
        print("Done : generation of obstacles took %.2f seconds." %
              (time.time()-start_time))

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

    def occupy_voxels(self, voxel_grid, container_volume, mesh_size):
        """
        Mark all the voxels from the grid that the shape occupies to True.
        """
        pass


class Cylinder(Shape):
    """
    Class that implements the cylinder shape
    """

    def __init__(self, axis_origin, axis_direction, radius, cylinder_height=12.0):
        # the starting point of the cylinder
        self.axis_origin = np.array(axis_origin)
        # its direction, need not to be a unit vector, but should not be null vector.
        self.axis_direction = np.array(axis_direction)
        self.axis_direction = self.axis_direction / \
            np.linalg.norm(self.axis_direction)
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
        OP = point - self.axis_origin  # relative vector

        # point is not too far from origin
        if 0 <= abs(np.dot(OP, self.axis_direction)) < (self.cylinder_height)/2:
            # projection on the cylinder axis
            projection = np.dot(OP, self.axis_direction) * self.axis_direction
            distance_vector = OP - projection  # distance to cylinder axis
            # distance to cylinder is smaller than radius : point belongs to the cylinder.
            if np.linalg.norm(distance_vector) <= self.radius:
                return True
        return False
    
    def occupy_voxels(self, voxel_grid, container_volume=None, mesh_size=0.05):

        # Find the normals to the axis
        first_normal = np.cross(self.axis_direction, np.array([0, 1, 0]))
        if np.linalg.norm(first_normal) < 1e-2:
            first_normal = np.cross(self.axis_direction, np.array([1, 0, 0]))
        first_normal = first_normal/np.linalg.norm(first_normal)
        second_normal = np.cross(self.axis_direction, first_normal)

        # Create mesh grid and fill occupancy in the voxel grid
        for radius in np.arange(self.radius, mesh_size/4, -mesh_size):
            angle_step = mesh_size / (2*np.pi*radius)
            for angle in np.arange(0, 2*np.pi, angle_step):
                x_rel = radius*np.cos(angle)
                y_rel = radius*np.sin(angle)
                for z_rel in np.arange(-self.cylinder_height/2, self.cylinder_height/2, mesh_size):
                    point = self.axis_origin + x_rel*first_normal + \
                        y_rel*second_normal + z_rel*self.axis_direction
                    if (container_volume is None or container_volume.volume_includes(point)):
                        voxel_coordinates = voxel_grid.point_to_voxel_coordinates(
                            point)
                        if voxel_grid.is_valid_coordinate(voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2]):
                            voxel_grid.set_voxel(
                                voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2], True)


class Loop(Shape):
    """
    Class that implements the loop obstacle
    """

    def __init__(self, axis_origin, angle, inside_radius, outside_radius, thickness=1):
        self.axis_origin = np.array(axis_origin)
        self.axis_direction = np.array([np.cos(angle), np.sin(angle), 0])
        self.inside_radius = inside_radius
        self.outside_radius = outside_radius
        self.thickness = thickness
        # A loop consists only of two cylinders of same thickness and same revolution axis
        self.inner_cylinder = Cylinder(
            self.axis_origin, self.axis_direction, inside_radius, thickness)
        self.outer_cylinder = Cylinder(
            self.axis_origin, self.axis_direction, outside_radius, thickness)

    def includes(self, point):
        """
        Determines if a given point stands inside the loop

        Args :
        point (tuple) : 3D point under study

        Returns :
        boolean indicating if the loop includes this point
        """
        return self.outer_cylinder.includes(point) and not (self.inner_cylinder.includes(point))  # Point belongs to loop if it belongs to the outer cylinder without belonging to inner cylinder.

    def occupy_voxels(self, voxel_grid, container_volume=None, mesh_size=0.05):

        # Find the normals to the axis
        first_normal = np.cross(self.axis_direction, np.array([0, 1, 0]))
        if np.linalg.norm(first_normal) < 1e-2:
            first_normal = np.cross(self.axis_direction, np.array([1, 0, 0]))
        first_normal = first_normal/np.linalg.norm(first_normal)
        second_normal = np.cross(self.axis_direction, first_normal)

        # Create mesh grid and fill occupancy in the voxel grid
        for radius in np.arange(self.outside_radius, self.inside_radius, -mesh_size):
            angle_step = mesh_size / (2*np.pi*radius)
            for angle in np.arange(0, 2*np.pi, angle_step):
                x_rel = radius*np.cos(angle)
                y_rel = radius*np.sin(angle)
                for z_rel in np.arange(0.0, self.thickness, mesh_size):
                    point = self.axis_origin + x_rel*first_normal + \
                        y_rel*second_normal + z_rel*self.axis_direction
                    if (container_volume is None or container_volume.volume_includes(point)):
                        voxel_coordinates = voxel_grid.point_to_voxel_coordinates(
                            point)
                        if voxel_grid.is_valid_coordinate(voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2]):
                            voxel_grid.set_voxel(
                                voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2], True)

class Wall(Shape):
    """
    Implements the wall shape
    """

    def __init__(self, origin, direction1, direction2=(0, 0, 1), length=100.0, width=1.0, height=100.0):
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
        self.direction1 = self.direction1 / \
            np.linalg.norm(self.direction1)  # to make it a unit vector

        self.direction2 = np.array(direction2)
        self.direction2 = self.direction2 / \
            np.linalg.norm(self.direction2)  # to make it a unit vector

        # normal vector defined as the third dimension.
        self.normal = np.cross(self.direction1, self.direction2)
        self.normal = self.normal / np.linalg.norm(self.normal)  # make it unit

        # make the base orthonormal for the wall.
        self.direction2 = np.cross(self.normal, self.direction1)

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
        if 0 <= np.dot(self.direction1, OP) < self.length and -self.width < np.dot(self.normal, OP) <= 0 and 0 <= np.dot(self.direction2, OP) < self.height:  # is point in wall ?
            for gap in self.gaps:
                if gap.includes(point):  # is point in a gap ?
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
        gap_origin = self.origin + self.direction1 * \
            rel_origin[0] + self.direction2 * rel_origin[1]
        gap = Wall(gap_origin, self.direction1,
                   self.direction2, length, self.width, height)
        self.gaps.append(gap)

    def occupy_voxels(self, voxel_grid, container_volume=None, mesh_size=0.05):
        # Create mesh grid and fill occupancy in the voxel grid
        for x_rel in np.arange(0.0, self.length, mesh_size):
            for y_rel in np.arange(0.0, self.height, mesh_size):
                for z_rel in np.arange(-self.width, 0.0, mesh_size):
                    point = self.origin + x_rel*self.direction1 + \
                        y_rel*self.direction2 + z_rel*self.normal
                    voxel_coordinates = voxel_grid.point_to_voxel_coordinates(
                        point)
                    # is inside the voxelGrid
                    if voxel_grid.is_valid_coordinate(voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2]):
                        is_in_wall = True
                        for gap in self.gaps:
                            if gap.includes(point):
                                is_in_wall = False
                                break
                        if is_in_wall:  # No gap includes the point, it is part of the wall
                            voxel_coord_world = voxel_grid.to_world_coordinates(
                                voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2])
                            if voxel_grid.is_valid_coordinate(voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2]):
                                voxel_grid.set_voxel(
                                    voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2], True)


class RandomVolume(Shape):

    def __init__(self, origin_range, seed=0):
        self.shapes = []
        self.origin_range = np.array(origin_range)
        self.origin_range[1, :] += self.origin_range[0, :]
        rd.seed(seed)

    def add_random_cylinders(self,
                             nb_cylinder,
                             direction_range=[
                                 [-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]],
                             radius_range=[0.5, 2.0],
                             height_range=[0.5, 100.0]):

        for _ in range(nb_cylinder):
            # Create the parameters for the random cylinder
            origin = self.random_in_range(self.origin_range)
            direction = self.random_in_range(direction_range)
            radius = self.random_in_range(radius_range)
            height = self.random_in_range(height_range)

            cylinder = Cylinder(origin, direction, radius, height)
            self.shapes.append(cylinder)

    def add_random_loops(self,
                         nb_loops,
                         angle_range=[0, np.pi],
                         radius_range=[0.0, 6.0],
                         thickness_range=[0.5, 2.0],
                         height_range=[-2, 2]):
        # the height range is the position offset from the center height of the grid
        for _ in range(nb_loops):
            # Create the parameters for the random cylinder
            origin = self.random_in_range(self.origin_range)
            origin[2] = (self.origin_range[1, 2] + self.origin_range[0, 2]
                         )/2 + self.random_in_range(height_range)
            angle = self.random_in_range(angle_range)

            radius1 = radius_range[0]
            radius2 = radius_range[1]
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
        # In that case, we have a single number to output. Example of input : [0.0,1.0]
        if bounds.ndim == 1:
            return rd.uniform(bounds[0], bounds[1])

        # else, the input looks something like this : [[0.0, 0.0], [1.0, 1.0]] -> should output 2 random numbers.
        result = []
        for i in range(len(bounds[0])):
            result.append(rd.uniform(bounds[0, i], bounds[1, i]))
        return result
    
    def volume_includes(self, point):
        return self.origin_range[0, 0] <= point[0] < self.origin_range[1, 0] \
                and self.origin_range[0, 1] <= point[1] < self.origin_range[1, 1] \
                and self.origin_range[0, 2] <= point[2] < self.origin_range[1, 2]

    def volume_includes(self, point):
        return self.origin_range[0, 0] <= point[0] < self.origin_range[1, 0] \
            and self.origin_range[0, 1] <= point[1] < self.origin_range[1, 1] \
            and self.origin_range[0, 2] <= point[2] < self.origin_range[1, 2]

    def includes(self, point):
        # Check if point is inside the volume defined by the random volume (x between x_min and x_max, y between ... etc)
        if self.volume_includes(point):
            # then tests whether point belongs a least too one of the shapes in the collection.
            for shape in self.shapes:
                if shape.includes(point):
                    return True
        return False

    def occupy_voxels(self, voxel_grid, container_volume=None, mesh_size=0.05):
        for shape in self.shapes:
            shape.occupy_voxels(voxel_grid, self, mesh_size)

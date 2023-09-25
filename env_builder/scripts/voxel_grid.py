import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class VoxelGrid:
    def __init__(self, dimension, voxel_size, origin):
        """
        Initialize a VoxelGrid.

        Args:
            dimension (tuple): 3D integer vector (number of voxels in each direction).
            voxel_size (float): Scalar (size/side length of each voxel/cube).
            origin (tuple): 3D vector (origin of the voxel grid relative to a world frame centered at 0,0,0).
        """
        self.dimension = dimension
        self.voxel_size = voxel_size
        self.origin = origin
        self.data = [0] * (dimension[0] * dimension[1] * dimension[2])

    def set_voxel(self, i, j, k, value):
        """
        Set the value of a voxel at integer coordinates (i, j, k).

        Args:
            i (int): X-coordinate.
            j (int): Y-coordinate.
            k (int): Z-coordinate.
            value (int): Value to set for the voxel.
        """
        index = i + self.dimension[0] * j + self.dimension[0] * self.dimension[1] * k
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
        index = i + self.dimension[0] * j + self.dimension[0] * self.dimension[1] * k
        return self.data[index]

    def clear(self):
        """Clear the voxel grid by setting all voxels to 0."""
        self.data = [0] * (self.dimension[0] * self.dimension[1] * self.dimension[2])

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
        return 0 <= i < self.dimension[0] and 0 <= j < self.dimension[1] and 0 <= k < self.dimension[2]

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
        for i in range(self.dimension[0]):
            for j in range(self.dimension[1]):
                for k in range(self.dimension[2]):
                    if self.data[i + self.dimension[0] * j + self.dimension[0] * self.dimension[1] * k] != 0:
                        x.append(self.origin[0] + i * self.voxel_size)
                        y.append(self.origin[1] + j * self.voxel_size)
                        z.append(self.origin[2] + k * self.voxel_size)

        ax.scatter(x, y, z, c='b', marker='o', s=100)  # Blue markers for occupied voxels
        ax.set_xlim([self.origin[0], self.origin[0] + self.voxel_size*self.dimension[0]])
        ax.set_ylim([self.origin[1], self.origin[1] + self.voxel_size*self.dimension[1]])
        ax.set_zlim([self.origin[2], self.origin[2] + self.voxel_size*self.dimension[2]])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Voxel Grid Visualization')
        ax.set_box_aspect([1, 1, 1])

        plt.show()
    
    def add_cylinder(self, axis_origin, axis_direction, radius, cylinder_height = float('Inf')):
        """
        Adds a cylinder in the voxel grid using an axis_origin, the direction of the axis and the radius.

        Args :
            axis_origin (3D tuple)   : Origin of the cylinder in world coordinates
            axis_direction (3D tuple): Direction of the cylinder
            radius (float)           : Radius of the cylinder in world coordinates
            cylinder_height(float)   : Height of the cylinder in world coordinates
        """
        axis_origin = np.array(axis_origin)
        axis_direction = np.array(axis_direction)
        axis_direction = axis_direction/ np.linalg.norm(axis_direction)

        for i in range(dimension[0]):
            for j in range(dimension[1]):
                for k in range(dimension[2]):
                    point = np.array(self.to_world_coordinates(i,j,k))
                    OP = point - axis_origin
                    if 0 < np.dot(OP, axis_direction) < cylinder_height :
                        projection = np.dot(OP, axis_direction) * axis_direction
                        distance_vector = OP - projection

                        if np.linalg.norm(distance_vector) <= radius :
                            self.set_voxel(i,j,k, 100)


    def add_loop(self, axis_origin, angle, inside_radius, outside_radius, thickness = 1):
        """
        Adds a loop in the voxel grid using an axis_origin, the direction of the axis and the radius.

        Args :
            axis_origin (3D tuple)  : Origin of the cylinder in world coordinates
            angle (float)           : Direction of the loop, in rad
            inside_radius (float)   : Inside radius of the loop in world coordinates
            outside_radius (float)  : Outside radius of the loop in world coordinates
            thickness(float)        : Thickness of the loop in world coordinates
        """

        axis_origin = np.array(axis_origin)
        axis_direction = np.array([np.cos(angle), np.sin(angle), 0])

        for i in range(dimension[0]):
            for j in range(dimension[1]):
                for k in range(dimension[2]):
                    point = np.array(self.to_world_coordinates(i,j,k))
                    OP = point - axis_origin
                    if np.abs(np.dot(OP, axis_direction)) < thickness :
                        projection = np.dot(OP, axis_direction) * axis_direction
                        distance_vector = OP - projection

                        if inside_radius <= np.linalg.norm(distance_vector) <= outside_radius:
                            self.set_voxel(i,j,k, 100)


    def add_wall(self, wall):
        for i in range(dimension[0]):
            for j in range(dimension[1]):
                for k in range(dimension[2]):
                    point = np.array(self.to_world_coordinates(i,j,k))
                    if wall.includes(point):
                        self.set_voxel(i,j,k, 100)

    def get_occupied_voxels(self):
        voxel_list = []
        for i in range(dimension[0]):
            for j in range(dimension[1]):
                for k in range(dimension[2]):
                    if self.get_voxel(i,j,k) > 0 :
                        voxel_list.append(i)
                        voxel_list.append(j)
                        voxel_list.append(k)
        return voxel_list


        

                    
class Wall:

    def __init__(self, origin, direction, length = float('Inf'), width = 2.0, height = float('Inf')):
        """
        Initialize a wall object

        Args :
        origin (3D point) : the starting point for the wall
        direction (3D vector) : the direction the wall will have (cannot be solely vertical, i.e. on axis Z)
        length (float) : the length of the wall
        width (float) : the wall's thickness
        heigth (float) : the wall's height
        """
        self.origin = np.array(origin)
        self.direction = np.array(direction)
        self.direction = self.direction / np.linalg.norm(self.direction) # to make it a unit vector
        self.normal = np.cross(self.direction, np.array([0,0,1]))
        self.normal = self.normal /np.linalg.norm(self.normal) # make it unit
        self.direction2 = np.cross(self.normal, self.direction) #,unit by construction
        self.length = length
        self.width = width
        self.height = height
        self.gaps = []

    def includes(self, point):
        """
        Determines if a given point stands inside the wall

        Args :
        point (3D point) : point under study

        Returns :
        boolean indicating if the wall includes this point
        """
        point = np.array(point)
        OP = point - self.origin
        if 0 <= np.dot(self.direction, OP) < self.length and -self.width < np.dot(self.normal, OP) <= 0 and 0 <= np.dot(self.direction2, OP) < self.height :
            for gap in self.gaps :
                if gap.includes(point):
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

        gap_origin = self.origin + self.direction * rel_origin[0] + self.direction2 * rel_origin[1]
        gap = Wall(gap_origin, self.direction, length, self.width, height)
        self.gaps.append(gap)



        

# Example usage:
if __name__ == "__main__":
    dimension = (30, 30, 30)
    voxel_size = 0.33
    origin = (0.0, 0.0, 0.0)

    voxel_grid = VoxelGrid(dimension, voxel_size, origin)
    voxel_grid.add_cylinder((8.0,2.0,5.0), (0.5,0,1.0), 1.0)
    voxel_grid.add_loop((2.0,8.0,5.0), -np.pi/4, 2.0, 3.0)
    wall = Wall((1,1,1), (1.0,0.3,0.5), width=0.5)
    wall.add_square_gap((1,2), 1,5)
    wall.add_square_gap((3,2), 1,5)
    wall.add_square_gap((2,4), 1,1)

    wall.add_square_gap((5,2), 1,3)
    wall.add_square_gap((5,6), 1,1)

    wall.add_square_gap((7,2), 1,1)
    wall.add_square_gap((7,4), 1,2)

    # voxel_grid.add_wall(wall)
    voxel_grid.get_occupied_voxels()
    voxel_grid.visualize()  # Visualize the voxel grid


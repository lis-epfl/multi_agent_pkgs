import numpy as np
from shapes import VoxelGrid, Wall, RandomVolume, Cylinder, Loop


def write_config_file(filename, voxel_grid, seed):
    voxel_grid.compute_occupancy()
    config_yaml = """env_builder_node: 
  ros__parameters:
    origin_grid: %s # origin of the voxel grid
    dimension_grid: %s # dimensions in meters
    vox_size: %f # voxel size of the grid
    free_grid: true # if true the grid is initialized to free instead of unknown
  
    multi_obst_size: false # if false, use size_obst as a common size for all obstacles; otherwise use size_obst_multi to define the size of each obstacle
    multi_obst_position: true # if false, generate positions randomly using normal distribution, if true use position_obst_multi to define the position of each obstacle
    range_obst: [30.0, 30.0, 0.0] # area on the map where to generate obstacles, always positive numbers to move it around, use the origin_obst variable
    origin_obst: [0.0, 0.0, 0.0] # origin of the area where we generate the obstacles
    size_obst: %s # height x width x length
    n_obst: 90 # 225, 180, 135, 90, number of obstacles (only used if we want to use random generation, otherwise the number of obstacles is inferred from position_obst_vec)
    rand_seed: %i #seed for the random generation of the obstacles
    size_obst_vec: %s # x,y,z size of each obstacle concatenated
    position_obst_vec: %s # x,y,z position of each obstacle concatenated


    env_pc_topic: "environment_poincloud" # topic on which to publish the env pointcloud
    env_vg_topic: "environment_voxel_grid" # topic on which to publish the env voxel grid 
    env_pc_frame: "world" # origin frame of the published poinctloud
    get_grid_service_name: "get_voxel_grid" # name of the service to get the voxel grid of each agent

    save_obstacles: true # if true, save the obstacle position in pos_obs.csv as well as the pointcloud in a file""" % (
        voxel_grid.origin,
        voxel_grid.dimension,
        voxel_grid.voxel_size,
        [0.01]*3,
        seed,
        [0.01]*3,
        voxel_grid.occupied_voxels

    )

    # Specify the filename for the YAML config file
    config_filename = '../config/%s.yaml' % filename

    # Write the configuration to a YAML file
    with open(config_filename, 'w') as yaml_file:
        yaml_file.write(config_yaml)

    print(f"Configuration saved to '{config_filename}'")


if __name__ == "__main__":
    ##################### PARAMETERS FOR YOUR RANDOM ENVIRONMENT HERE ##################
    config_filename = "env_long_config"
    seed = 0 

    # Voxel Grid parameters
    dimension = [100.0, 30.0, 15.0]  # meters
    voxel_size = 0.3  # meters
    origin = [0.0, 0.0, -6.0]  # meters

    # Create the voxel grid
    voxel_grid = VoxelGrid(dimension, voxel_size, origin)

    # # Add shapes : Cylinder, Loop, Walls or Random volumes
    # # Create a cylinder
    # cylinder = Cylinder((2.0,5.0,5.0), (0.3,0.5,1.0), 1.5)
    # voxel_grid.add_shape(cylinder)

    # # Create a loop
    # loop = Loop((12.0,5.0,5.0), -np.pi/4, 2.0, 3.0)
    # voxel_grid.add_shape(loop)

    # # # Create a wall
    wall = Wall(origin=(48, 0, -6), direction1=(0.0, 1.0, 0.0), direction2=(0.0,
                0.0, 1.0), width=0.3)  # Two directions to define the plane
    wall.add_square_gap(rel_origin=(2.5, 5), length=1.5, height=2)
    wall.add_square_gap(rel_origin=(5, 7.5), length=3, height=3)
    wall.add_square_gap(rel_origin=(7.5, 5), length=3, height=2)
    wall.add_square_gap(rel_origin=(10, 9.5), length=2, height=3)
    wall.add_square_gap(rel_origin=(12.5, 4), length=2, height=2)
    wall.add_square_gap(rel_origin=(15, 12.5), length=2, height=2)
    wall.add_square_gap(rel_origin=(17.5, 3), length=2, height=1.5)
    wall.add_square_gap(rel_origin=(20, 8.5), length=3, height=1.5)
    wall.add_square_gap(rel_origin=(22.5, 12), length=1.5, height=2)
    wall.add_square_gap(rel_origin=(25, 5.5), length=2, height=2)
    wall.add_square_gap(rel_origin=(27, 4), length=1.2, height=2)
    wall.add_square_gap(rel_origin=(30, 7), length=2, height=2)
    wall.add_square_gap(rel_origin=(32.5, 12), length=3, height=1.5)
    wall.add_square_gap(rel_origin=(35, 9), length=2, height=2)
    wall.add_square_gap(rel_origin=(37.5, 7), length=1.5, height=1.5)
    voxel_grid.add_shape(wall)

    # Create a Random volume
    # The containing volume of randomly generated cylinders.
    rd_volume_cylinders = RandomVolume([[3, 0, -6], [30, 30, 15]], seed)
    rd_volume_cylinders.add_random_cylinders(90,
                                             direction_range=[
                                                 [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
                                             radius_range=[0.05, 0.05],
                                             height_range=[20.0, 20.0])  # Choose to add 10 cylinders with parameters as default
    # rd_volume_cylinders = RandomVolume([[3, 0, -6], [90, 30, 15]], seed)
    # rd_volume_cylinders.add_random_loops(75,
    #                                      angle_range=[0, np.pi/2],
    #                                      radius_range=[1.5, 2],
    #                                      thickness_range=[0.3, 0.3])
    voxel_grid.add_shape(rd_volume_cylinders)

    rd_volume_cylinders = RandomVolume([[63, 0, -6], [30, 30, 15]], seed)
    rd_volume_cylinders.add_random_cylinders(180,
                                             direction_range=[
                                                 [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
                                             radius_range=[0.05, 0.05],
                                             height_range=[20.0, 20.0])  # Choose to add 10 cylinders with parameters as default

    voxel_grid.add_shape(rd_volume_cylinders)

    # Or with loops (can do both in same volume)
    # rd_volume_loops = RandomVolume([[33, 0, 0], [40,10,10]], seed)
    # rd_volume_loops.add_random_loops(5)
    # voxel_grid.add_shape(rd_volume_loops)

    ########################### END OF PARAMETRIZING ##################################
    # Write the YAML config file
    write_config_file(config_filename, voxel_grid, seed)

    # voxel_grid.visualize()  # Visualize the voxel grid if you wish (needs matplotlib)

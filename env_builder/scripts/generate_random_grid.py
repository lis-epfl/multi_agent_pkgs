from shapes import VoxelGrid, Wall, RandomVolume, Cylinder, Loop

def write_config_file(filename, voxel_grid, seed):
    size_obst_vec, position_obst_vec = voxel_grid.get_occupied_voxels()
    config_yaml = """env_builder_node: 
  ros__parameters:
    origin_grid: %s # origin of the voxel grid
    dimension_grid: %s # dimensions in meters
    vox_size: %f # voxel size of the grid
    free_grid: true # if true the grid is initialized to free instead of unknown
    inflation_dist: 0.3 # all voxels within that distance to an occupied voxel are set as occupied
  
    multi_obst_size: true # if false, use size_obst as a common size for all obstacles; otherwise use size_obst_multi to define the size of each obstacle
    multi_obst_position: false # if false, generate positions randomly using normal distribution, if true use position_obst_multi to define the position of each obstacle
    range_obst: [30.0, 30.0, 0.0] # area on the map where to generate obstacles, always positive numbers to move it around, use the origin_obst variable
    origin_obst: [6.5, 6.5, 0.0] # origin of the area where we generate the obstacles
    size_obst: [0.1, 0.1, 10.0] # height x width x length
    n_obst: 135 # 225, 180, 135, 90, number of obstacles (only used if we want to use random generation, otherwise the number of obstacles is inferred from position_obst_vec)
    rand_seed: %i #seed for the random generation of the obstacles
    size_obst_vec: %s # x,y,z size of each obstacle concatenated
    position_obst_vec: %s # x,y,z position of each obstacle concatenated


    env_pc_topic: "environment_poincloud" # topic on which to publish the env pointcloud
    env_vg_topic: "environment_voxel_grid" # topic on which to publish the env voxel grid 
    env_pc_frame: "world" # origin frame of the published poinctloud
    get_grid_service_name: "get_voxel_grid" # name of the service to get the voxel grid of each agent

    save_obstacles: true # if true, save the obstacle position in pos_obs.csv as well as the pointcloud in a file""" %(
        voxel_grid.origin,
        voxel_grid.dimension,
        voxel_grid.voxel_size,
        seed,
        size_obst_vec,
        position_obst_vec

    )

    # Specify the filename for the YAML config file
    config_filename = '../config/%s.yaml' %filename

    # Write the configuration to a YAML file
    with open(config_filename, 'w') as yaml_file:
        yaml_file.write(config_yaml)

    print(f"Configuration saved to '{config_filename}'")


if __name__ == "__main__":
    ##################### PARAMETERS FOR YOUR RANDOM ENVIRONMENT HERE ##################
    config_filename = "new_config"
    seed = 50

    ##################### Voxel Grid parameters
    dimension = [10, 10, 10]
    voxel_size = 0.5
    origin = [-5.0, -5.0, -5.0]

    voxel_grid = VoxelGrid(dimension, voxel_size, origin) # Create the voxel grid

    ##################### Add shapes : Cylinder, Loop, Walls or Random volumes
    ## Create a cylinder
    # cylinder = Cylinder((0.0,0.0,-5.0), (0.0,0.5,1.0), 1.0)
    # voxel_grid.add_shape(cylinder)

    ## Create a loop
    # loop = Loop((2.0,8.0,5.0), -np.pi/4, 2.0, 3.0)
    # voxel_grid.add_shape(loop)

    ## Create a wall
    # wall = Wall((80,0,0), direction1=(0.0,1.0,0.0), direction2 = (0.5, 0.0, 1.0), width=3)
    # wall.add_square_gap((20,20),10,10)
    # voxel_grid.add_shape(wall)

    ## Create a Random volume
    # rd_volume_cylinders = RandomVolume([[0,0,0], [20,40,40]], seed)
    # rd_volume_cylinders.add_random_cylinders(10)
    # voxel_grid.add_shape(rd_volume_cylinders)

    ## Or with loops (can do both in same volume)
    # rd_volume_loops = RandomVolume([[40, 0, 0], [60,40,40]], seed)
    # rd_volume_loops.add_random_loops(5)
    # voxel_grid.add_shape(rd_volume_loops)

    ########################### END OF PARAMETRIZING ##################################
    write_config_file(config_filename, voxel_grid, seed)

    # voxel_grid.visualize()  # Visualize the voxel grid if you wish
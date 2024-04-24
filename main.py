# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


import random
import carla


def main():
    try:
        # create a client
        client = carla.Client('localhost', 2000)

        # load the map in carla
        client.load_world('Town01')

        # set numbers
        num_walkers = 200
        num_vehicle = 20

        # percentage of running and crossing people in pedestrians
        percentage_pedestrians_running = 0.01
        percentage_pedestrians_crossing = 0.1

        # check the world
        world = client.get_world()

        # Obtain observers in this world
        spectator = world.get_spectator()

        # Obtain observer's directional information
        transform = spectator.get_transform()

        # Set a new orientation based on the observer's default orientation
        location = transform.location + carla.Location(x=-30, z=20)
        rotation = carla.Rotation(pitch=-20, yaw=-20, roll=0)
        new_transform = carla.Transform(location, rotation)

        # Set the observer to a new orientation
        spectator.set_transform(new_transform)

        # get all vehicles
        vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

        # if you want to get the pedestrian, just use
        ped_blueprints = world.get_blueprint_library().filter('*pedestrian*')

        # get all the places that can generate vehicles by world
        vehicle_spawn_points = world.get_map().get_spawn_points()

        # # get all the places that can generate vehicles by world and store these places
        ped_spawn_points = []
        for i in range(num_walkers):
            # create a new carla.Transform object , the object will be used to define
            # the possible location where can generate
            spawn_point = carla.Transform()

            # find a random location in the world that's
            # considered part of the navigation mesh (i.e., suitable for walking).
            # Stores this random location in the loc variable.
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                # if the valid location (loc) was found, its location data is assigned to the spawn_point
                spawn_point.location = loc
                ped_spawn_points.append(spawn_point)

        # Randomly generate num_vehicle vehicles on the map,
        # with each vehicle being a random vehicle in the vehicle blueprint library
        for i in range(0, num_vehicle):
            world.try_spawn_actor(random.choice(vehicle_blueprints),
                          random.choice(vehicle_spawn_points))

        # Create a list to store pedestrians, pedestrian speed settings and pedestrian controllers
        walker_batch = []
        walker_speed = []
        walker_ai_batch = []

        # Randomly generate num_walkers pedestrians on the map, each pedestrian is a random pedestrian in the pedestrian
        # blueprint library, and set the pedestrian's moving speed
        for j in range(num_walkers):
            walker_bp = random.choice(ped_blueprints)

            # There are six attribute of pedestrians in carla
            # age,gender,generation,is_invincible,role_name,speed

            # Cancel pedestrian invincibility status
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # Set pedestrian movement speed
            if walker_bp.has_attribute('speed'):

                # the percentage_pedestrians_running is the probability of pedestrian is running
                if random.random() > percentage_pedestrians_running:
                    # Set the corresponding pedestrian speed to the walking speed
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # Set the corresponding pedestrian speed to the running speed
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])

            # Randomly select and generate random pedestrians from spawn points that can spawn pedestrians, and then add
            # the generated pedestrians to the same batch
            walker_batch.append(world.try_spawn_actor(walker_bp, random.choice(ped_spawn_points)))

        # Searching for controllers that control pedestrian movement logic from the blueprint library
        walker_ai_blueprint = world.get_blueprint_library().find('controller.ai.walker')

        # Generate corresponding controllers for each batch of pedestrians and add them to the list representing batch controllers
        for walker in world.get_actors().filter('*pedestrian*'):
            walker_ai_batch.append(world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker))

        # Batch start pedestrian controller and set controller parameters
        for i in range(len(walker_ai_batch)):

            # start pedestrian controller
            walker_ai_batch[i].start()

            # Set pedestrian target points through the controller
            walker_ai_batch[i].go_to_location(world.get_random_location_from_navigation())

            # Set pedestrian walking speed through the controller
            walker_ai_batch[i].set_max_speed(float(walker_speed[i]))

        # Set parameters for pedestrians crossing the road
        world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)

        # Get vehicles from the world and set each vehicle's autopilot mode on
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.set_autopilot()

        # Randomly select a location from where a vehicle can be spawned and record it
        ego_spawn_point = random.choice(vehicle_spawn_points)

        # Select the main car blueprint we need from the blueprint library
        ego_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')

        # Set the character name in the properties of the main vehicle blueprint
        ego_bp.set_attribute('role_name', 'hero')

        # Generate main car
        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)

        # Set the main vehicle to autonomous driving mode
        ego_vehicle.set_autopilot()

        # Find rgb camera from blueprint library, there are many types of sensor in carla including
        # 1.sensor.camera.rgb: Captures images similar to a standard camera,
        # providing color information (Red, Green, Blue channels).
        # 2.sensor.camera.depth : Measures the distance to each pixel in the scene, providing a depth map. Darker
        # values typically represent objects closer to the camera.
        # 3.sensor.camera.semantic_segmentation : Assigns a category label to each pixel in the image
        # (e.g., "road", "vehicle", "pedestrian", "building"). This provides a high-level understanding of the scene.

        # other types like Lidar (sensor.lidar.ray_cast),IMU (sensor.other.imu),GNSS (sensor.other.gnss) can be found in
        # https://carla.readthedocs.io/en/latest/bp_library/#sensor
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

        # Set the orientation information of the rgb camera
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                   carla.Rotation(pitch=10, yaw=0, roll=0))
        # Generate an rgb camera and bind it to the main car using SpringArmGhost
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle,
                           attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        while True:
            # Obtain the observer's perspective from the world and set the orientation information of the observer's
            # perspective to the corresponding orientation information of the camera.
            world.get_spectator().set_transform(camera.get_transform())
            # Wait for the server to update the world status
            world.wait_for_tick()

    finally:
        # Stop and destroy all controllers
        for controller in world.get_actors().filter('*controller*'):
            controller.stop()

        # Destroy all vehicles
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.destroy()

        # Destroy all pedestrians
        for walker in world.get_actors().filter('*walker*'):
            walker.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')



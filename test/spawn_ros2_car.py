import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # 1. Setup Vehicle (using 'hero' to trigger internal ROS logic)
    veh_bp = bp_lib.filter('model3')[0]
    veh_bp.set_attribute('role_name', 'hero')

    # Try different spawn points until one works
    spawn_points = world.get_map().get_spawn_points()
    vehicle = None
    
    for pt in spawn_points:
        pt.location.z += 0.5 # Lift it slightly to avoid floor collision
        vehicle = world.try_spawn_actor(veh_bp, pt)
        if vehicle is not None:
            print(f"Spawned Vehicle ID: {vehicle.id}")
            break
    
    if not vehicle:
        print("Could not find an empty spawn point. Try restarting CARLA.")
        return

    # 2. Setup RGB Camera with Native ROS 2 attributes
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('ros_name', 'front_cam')
    cam_bp.set_attribute('image_size_x', '160')
    cam_bp.set_attribute('image_size_y', '120')
    cam_bp.set_attribute('sensor_tick', '0.1') # 10 Hz to save CPU

    # Attach to the front of the car
    cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    print("Success! Keep this script running and check 'ros2 topic list' now.")
    
    try:
        while True:
            world.wait_for_tick()
    except KeyboardInterrupt:
        print("Cleaning up...")
        camera.destroy()
        vehicle.destroy()

if __name__ == '__main__':
    main()
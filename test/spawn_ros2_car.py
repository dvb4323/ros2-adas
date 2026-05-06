import carla
import time

def main():
    print("Connecting to CARLA...")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    print("Connected to CARLA")

    bp_lib = world.get_blueprint_library()

    # =============================
    # Spawn vehicle
    # =============================
    veh_bp = bp_lib.filter('model3')[0]
    veh_bp.set_attribute('role_name', 'hero')

    spawn_points = world.get_map().get_spawn_points()
    vehicle = None

    for pt in spawn_points:
        pt.location.z += 0.5
        vehicle = world.try_spawn_actor(veh_bp, pt)
        if vehicle:
            # 1. Connect to the Traffic Manager
            tm = client.get_trafficmanager(8005)
            # 2. Tell the vehicle to listen to the TM
            vehicle.set_autopilot(True, tm.get_port())
            print(f"Spawned vehicle ID: {vehicle.id}")
            break

    if not vehicle:
        print("ERROR: Failed to spawn vehicle")
        return

    # =============================
    # Spawn camera
    # =============================
    cam_bp = bp_lib.find('sensor.camera.rgb')

    cam_bp.set_attribute('ros_name', 'front_cam')
    cam_bp.set_attribute('image_size_x', '160')
    cam_bp.set_attribute('image_size_y', '120')
    cam_bp.set_attribute('sensor_tick', '0.1')
    cam_bp.set_attribute('enable_postprocess_effects', 'false')

    cam_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4)
    )

    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    print("Camera attached")

    # Activate sensor
    def dummy_callback(image):
        pass

    camera.listen(dummy_callback)

    print("Sensor running")
    print("Now check: ros2 topic list")

    try:
        while True:
            world.wait_for_tick()  # IMPORTANT: async mode safe
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        print("Cleaning up...")

        camera.stop()
        camera.destroy()
        vehicle.destroy()

        print("Done.")


if __name__ == '__main__':
    main()
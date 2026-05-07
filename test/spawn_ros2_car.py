import carla
import time

def main():
    print("Connecting to CARLA...")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # world = client.get_world()
    world = client.load_world('Town04')
    print("Connected to CARLA")
    
    settings = world.get_settings()
    settings.synchronous_mode = True # Server waits for script
    settings.fixed_delta_seconds = 0.05 # 20 FPS
    world.apply_settings(settings)
    
    # Clean up all existing vehicles and sensors before starting
    actors = world.get_actors()
    for actor in actors.filter('vehicle.*'):
        actor.destroy()
    for actor in actors.filter('sensor.*'):
        actor.destroy()
    print("Cleaned up old actors.")

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
            tm.set_synchronous_mode(True)
            # 2. Tell the vehicle to listen to the TM
            vehicle.set_autopilot(False, tm.get_port())
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
        print("Monitoring vehicle telemetry... (Ctrl+C to stop)")
        while True:
            world.tick() # Advance the simulation
            v = vehicle.get_velocity()
            l = vehicle.get_location()
            import math
            speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
            
            # Check if the car is being held by a traffic light
            state = vehicle.get_traffic_light_state() 
            # Returns: Red, Yellow, Green, Off, or Unknown
            
            print(f"Loc: {l.x:.1f}, {l.y:.1f} | Speed: {speed:.2f} km/h | Light: {state}", end="\r")
            
            # Use for asynchronous mode:
            # world.wait_for_tick()
            # time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        print("Cleaning up...")
        
        # 1. ALWAYS disable synchronous mode before exiting
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8005)
        tm.set_synchronous_mode(False)

        # 2. Destroy actors
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        print("Done.")

if __name__ == '__main__':
    main()

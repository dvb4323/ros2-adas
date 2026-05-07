import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = False
world.apply_settings(settings)
print("Server unfrozen.")
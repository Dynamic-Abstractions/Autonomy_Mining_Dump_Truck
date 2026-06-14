import carla
client = carla.Client('localhost', 2000)
print(client.get_available_maps())

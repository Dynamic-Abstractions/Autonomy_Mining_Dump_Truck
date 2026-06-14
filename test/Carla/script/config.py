import carla

# Connect to the server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load the Synkrotron mining map
client.load_world('Mine01')

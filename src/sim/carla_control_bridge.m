
client = py.carla.Client('localhost', int32(2000));
client.set_timeout(2.0);
world = client.get_world();

% Read an state

vehicle = world.get_actors().filter('vehicle.*'){1};

transform = vehicle.get_transform();
vel = vehicle.get_velocity();

x = double(transform.location.x);
y = double(transform.location.y);
psi = double(transform.rotation.yaw);
v = sqrt(double(vel.x)^2 + double(vel.y)^2);

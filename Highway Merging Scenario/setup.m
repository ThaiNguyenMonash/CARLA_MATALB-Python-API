port = int16(2000);
client = py.carla.Client('localhost', port);
client.set_timeout(10.0);
world = client.load_world('Town04_Opt');
settings = world.get_settings();
settings.no_rendering_mode = true;
settings.synchronous_mode = true;
settings.fixed_delta_seconds = 0.05;
world.apply_settings(settings);

disp("Connected")


waypoints = csvread('Long_ego_waypoints.csv');
waypoints = waypoints(:,1:2);
NPC_waypoints = csvread('NPC_waypoints.csv');
NPC_waypoints = NPC_waypoints(:,1:2);

disp("Waypoint Loaded")


%% POINTS
%{

NPC:
200 8.5 7.2
155 7.4 9.5
99.5 6.5 10.8

200, 8.2, 7.2
220, 8.3, 6



EGO Vehicle:
105, -1.5, 10.8 150r

130, -35, 9.6 105r


Merging Area:

98, 2.1, 10.8 155r


%}

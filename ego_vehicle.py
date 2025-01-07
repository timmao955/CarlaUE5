import carla
from get_sensors import GetSensors
from get_waypoints import GetWaypoints

def main():
    # 启动carla客户端
    client = carla.Client('localhost',2000)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # 设置天气
    weather = carla.WeatherParameters(
        cloudiness = 100.0,
        fog_density = 10.0,
        precipitation = 50.0,
        wetness = 80.0,
        wind_intensity = 80.0
    )
    world.set_weather(weather)

    # 设置ego_vehicle & spawn_point
    ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz')
    ego_vehicle_bp.set_attribute('role_name', 'hero')
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[129]
    ego_vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

    # 设置观察者
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(spawn_point.location + carla.Location(z = 50), carla.Rotation(pitch = -90)))

    # 加载、绘制waypoints
    xml_file = '/home/moj5wx/Desktop/bench2drive/Bench2Drive/leaderboard/data/bench2drive220.xml'
    way_points = GetWaypoints.get_way_points(xml_file)
    GetWaypoints.draw_way_points(world, way_points, 0.1)

    # 设置传感器（bev大小是512x512）
    sensors_config = [
        {'type': 'sensor.camera.rgb', 'x': 0.27, 'y': -0.55, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0, 'width': 1600, 'height': 900, 'fov': 70, 'id': 'CAM_FRONT_LEFT'},
        {'type': 'sensor.camera.rgb', 'x': 0.8, 'y': 0.0, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 1600, 'height': 900, 'fov': 70, 'id': 'CAM_FRONT'},
        {'type': 'sensor.camera.rgb', 'x': 0.27, 'y': 0.55, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0, 'width': 1600, 'height': 900, 'fov': 70, 'id': 'CAM_FRONT_RIGHT'},
        {'type': 'sensor.camera.rgb', 'x': 0.0, 'y': 0.0, 'z': 50.0, 'roll': 0.0, 'pitch': -90.0, 'yaw': 0.0, 'width': 1600, 'height': 900, 'fov': 50.0, 'id': 'bev'},
        {'type': 'sensor.camera.rgb', 'x': -0.32, 'y': -0.55, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': -110.0, 'width': 1600, 'height': 900, 'fov': 70, 'id': 'CAM_BACK_LEFT'},
        {'type': 'sensor.camera.rgb', 'x': -2.0, 'y': 0.0, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0, 'width': 1600, 'height': 900, 'fov': 110, 'id': 'CAM_BACK'},
        {'type': 'sensor.camera.rgb', 'x': -0.32, 'y': 0.55, 'z': 1.6, 'roll': 0.0, 'pitch': 0.0, 'yaw': 110.0, 'width': 1600, 'height': 900, 'fov': 70, 'id': 'CAM_BACK_RIGHT'}
        # {'type': 'sensor.other.imu', 'x': -1.4, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'sensor_tick': 0.05, 'id': 'IMU'},
        # {'type': 'sensor.other.gnss', 'x': -1.4, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'sensor_tick': 0.01, 'id': 'GPS'},
        # {'type': 'sensor.speedometer', 'reading_frequency': 20, 'id': 'SPEED'}
    ]
    visualizer = GetSensors(ego_vehicle, sensors_config)
 
    while True:
        world.tick()
        visualizer.render()

if __name__ == '__main__':
    main()
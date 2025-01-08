import carla
import cv2
import queue
import threading
import time
import torch
from get_sensors import GetSensors
from get_waypoints import GetWaypoints
from parked_vehicle import GetParking
from vehicle_control import VehicleControl

class BufferQueue:
    def __init__(self, maxsize = 10):
        self.queue = queue.Queue(maxsize)
    
    def put(self, data):
        self.queue.put(data)
    
    def get(self):
        return self.queue.get()

    def is_empty(self):
        return self.queue.empty()
    
    def is_full(self):
        return self.queue.full()


class ConsumerThread(threading.Thread):     
    def __init__(self, buffer_queue):
        super().__init__()
        self.buffer_queue = buffer_queue
        self.running = True
    
    def run(self):
        while self.running:
            if not self.buffer_queue.is_empty():
                image = self.buffer_queue.get()

                # 离线保存图像
                timestamp = time.time()
                filename = f"frame_{timestamp:.3f}.png"
                cv2.imwrite(filename, image)
                
    def stop(self):
        self.running = False


def main():
    # 启动carla客户端
    client = carla.Client('localhost',2000)
    # client.load_world('Mine_01')
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

    # 初始化停车位
    parking_manager = GetParking(
        route = way_points,
        world = world,
        map_name = world.get_map(),
        list_scenarios = []
    )

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
    
    buffer_queue = BufferQueue()
    sensor_manager = GetSensors(ego_vehicle, sensors_config, buffer_queue)
    consumer = ConsumerThread(buffer_queue)
    VehicleControl.move_to_waypoints(ego_vehicle, way_points, world)

    consumer.start()
    sensor_manager.start()


    try:
        while True:
            sensor_manager.render()
            time.sleep(0.1)
 
            # world.tick()
            # visualizer.render()
    
    except KeyboardInterrupt:
        pass

    finally:
        consumer.stop()
        consumer.join()
        # visualizer.reset()
        sensor_manager.stop()

        ego_vehicle.destroy()


if __name__ == '__main__':
    main()
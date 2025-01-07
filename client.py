import carla
import socket
import json
import struct
import numpy as np
import copy
import logging
import time
import cv2
import random
 
# class SensorDataSender:
#     # 初始化连接
#     def __init__(self, host = 'localhost', port = 5001):
#         self.host = host
#         self.port = port
#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         try:
#             self.socket.connect((self.host, self.port))
#             print(f"Connected to server at {self.host}:{self.port}")
#         except Exception as e:
#             logging.error(f"Could not connect to server: {e}")
#             self.socket = None
    
#     # 发送数据
#     def send_data(self, data):
#         if self.socket is None:
#             logging.error("Socket is not connected.")
#             return
#         try:
#             # 将数据转换为JSON字符串
#             json_data = json.dumps(data)
#             encoded_data = json_data.encode('utf-8')
#             # 发送数据
#             self.socket.sendall(encoded_data)
#         except Exception as e:
#             logging.error(f"Error sending data: {e}")
    
#     # 关闭连接
#     def close(self):
#         if self.socket:
#             self.socket.close()
 
# class SensorDataHandler:
#     def __init__(self, data_provider, tag, sender):
#         self._data_provider = data_provider
#         self._tag = tag
#         self._sender = sender
#     def __call__(self, data):
#         """
#         Call function to handle incoming sensor data.
#         """
#         if isinstance(data, carla.Image):
#             self._parse_image_cb(data, self._tag)
#         elif isinstance(data, carla.LidarMeasurement):
#             self._parse_lidar_cb(data, self._tag)
#         elif isinstance(data, carla.RadarMeasurement):
#             self._parse_radar_cb(data, self._tag)
#         elif isinstance(data, carla.GnssMeasurement):
#             self._parse_gnss_cb(data, self._tag)
#         elif isinstance(data, carla.IMUMeasurement):
#             self._parse_imu_cb(data, self._tag)
#         else:
#             logging.error('No callback method for this sensor.')
#     def _parse_image_cb(self, image, tag):
#         """
#         Parses camera images and sends them to the server.
#         """
#         array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#         array = copy.deepcopy(array)
#         array = np.reshape(array, (image.height, image.width, 4))
#         array = array[:, :, :3]  # 只取RGB通道
#         data = {
#             'sensor': 'image',
#             'tag': tag,
#             'frame': image.frame,
#             'data': array.tolist()
#         }
#         self._sender.send_data(data)
#     def _parse_lidar_cb(self, lidar_data, tag):
#         """
#         Parses Lidar data and sends them to the server.
#         """
#         points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
#         points = copy.deepcopy(points)
#         points = np.reshape(points, (int(points.shape[0] / 4), 4))
#         points = points[:, :3]  # 只取x, y, z
#         data = {
#             'sensor': 'lidar',
#             'tag': tag,
#             'frame': lidar_data.frame,
#             'data': points.tolist()
#         }
#         self._sender.send_data(data)
#     def _parse_radar_cb(self, radar_data, tag):
#         """
#         Parses Radar data and sends them to the server.
#         """
#         points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
#         points = copy.deepcopy(points)
#         points = np.reshape(points, (int(points.shape[0] / 4), 4))
#         points = np.flip(points, 1)  # [depth, azimuth, altitude, velocity] -> [velocity, altitude, azimuth, depth]
#         data = {
#             'sensor': 'radar',
#             'tag': tag,
#             'frame': radar_data.frame,
#             'data': points.tolist()
#         }
#         self._sender.send_data(data)
#     def _parse_gnss_cb(self, gnss_data, tag):
#         """
#         Parses GNSS data and sends them to the server.
#         """
#         array = np.array([gnss_data.latitude,
#                           gnss_data.longitude,
#                           gnss_data.altitude], dtype=np.float64)
#         data = {
#             'sensor': 'gnss',
#             'tag': tag,
#             'frame': gnss_data.frame,
#             'data': array.tolist()
#         }
#         self._sender.send_data(data)
#     def _parse_imu_cb(self, imu_data, tag):
#         """
#         Parses IMU data and sends them to the server.
#         """
#         array = np.array([imu_data.accelerometer.x,
#                           imu_data.accelerometer.y,
#                           imu_data.accelerometer.z,
#                           imu_data.gyroscope.x,
#                           imu_data.gyroscope.y,
#                           imu_data.gyroscope.z,
#                           imu_data.compass,
#                           ], dtype=np.float64)
#         data = {
#             'sensor': 'imu',
#             'tag': tag,
#             'frame': imu_data.frame,
#             'data': array.tolist()
#         }
#         self._sender.send_data(data)
 
class CameraVisualizer:
    """
    Camera Visualizer for CARLA
    Provides a birdeye camera and a front bumper camera with OpenCV visualization.
 
    Args:
        actor (carla.Actor): Vehicle actor to attach cameras.
    """
 
    def __init__(self, actor):
        self._actor = actor
        self._cv_image_bird = None
        self._cv_image_front = None
        self._camera_bird = None
        self._camera_front = None
 
        # bev
        bird_bp = actor.get_world().get_blueprint_library().find('sensor.camera.rgb')
        bird_bp.set_attribute('image_size_x', '1000')
        bird_bp.set_attribute('image_size_y', '400')
        bird_transform = carla.Transform(carla.Location(x=0.0, z=50.0), carla.Rotation(pitch=-90))
        self._camera_bird = actor.get_world().spawn_actor(bird_bp, bird_transform, attach_to=actor)
        self._camera_bird.listen(lambda image: self._on_camera_update(image, birdseye=True))
 
        # front
        front_bp = actor.get_world().get_blueprint_library().find('sensor.camera.rgb')
        front_bp.set_attribute('image_size_x', '1000')
        front_bp.set_attribute('image_size_y', '400')
        front_transform = carla.Transform(carla.Location(x=2.0, z=1.5))
        self._camera_front = actor.get_world().spawn_actor(front_bp, front_transform, attach_to=actor)
        self._camera_front.listen(lambda image: self._on_camera_update(image, birdseye=False))
 
    def _on_camera_update(self, image, birdseye):
        """
        Update OpenCV image based on camera input.
        """
        if not image:
            return
 
        image_data = np.frombuffer(image.raw_data, dtype=np.uint8)
        np_image = np.reshape(image_data, (image.height, image.width, 4))
        np_image = np_image[:, :, :3][:, :, ::-1]  # Convert BGRA to RGB
        if birdseye:
            self._cv_image_bird = np_image
        else:
            self._cv_image_front = np_image
 
    def render(self):
        """
        Render images with OpenCV.
        """
        if self._cv_image_bird is not None and self._cv_image_front is not None:
            combined_image = cv2.vconcat([self._cv_image_front, self._cv_image_bird])
            # Correct speed calculation
            velocity = self._actor.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # Convert m/s to km/h
            speed_text = f"{int(speed):3d} kph"
            cv2.putText(combined_image, speed_text, (850, 370), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("CARLA Camera Visualizer", combined_image)
            cv2.waitKey(1)
 
    def reset(self):
        """
        Destroy cameras.
        """
        if self._camera_bird:
            self._camera_bird.destroy()
            self._camera_bird = None
        if self._camera_front:
            self._camera_front.destroy()
            self._camera_front = None

def main():
    """
    Main script to spawn a vehicle and visualize cameras.
    """
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
 
        world = client.get_world()
        # blueprint_library = world.get_blueprint_library()
 
        # vehicle_bp = blueprint_library.filter('vehicle.*')[0]
        # spawn_points = world.get_map().get_spawn_points()
        # vehicle = world.spawn_actor(vehicle_bp, random.choice(spawn_points))

        vehicles = world.get_actors().filter('vehicle.*')
        print(len(vehicles))
        player_vehicle = None

        # for vehicle in vehicles:
        #     # print(vehicle.id)
        #     if vehicle.attributes.get('role_name') == 'hero':
        #         player_vehicle = vehicle
        #         break
 
        visualizer = CameraVisualizer(vehicles[0])
 
        for _ in range(2000):  # Run for a fixed number of frames
            world.tick()
            visualizer.render()
 
    finally:
        if 'visualizer' in locals():
            visualizer.reset()
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.destroy()
        print("Actors destroyed.")

# def main():

#     # sender = SensorDataSender(host = 'localhost', port = 5001)
#     # if sender.socket is None:
#     #     print("Failed to connect to the server. Exiting.")
#     #     return
    
#     try:
#         client = carla.Client('localhost', 2000)
#         client.set_timeout(10.0)

#         world = client.get_world()
#         blueprint_library = world.get_blueprint_library()

#         vehicle_bp = blueprint_library.filter('vehicle.*')

#     # 获取车辆
#     vehicles = world.get_actors().filter('vehicle.*')
#     if not vehicles:
#         print("No vehicles found in the world.")
#         return
#     vehicle = vehicles[0]


    

#     # 添加摄像头传感器
#     camera_bp = blueprint_library.find('sensor.camera.rgb')
#     camera_bp.set_attribute('image_size_x', '1000')
#     camera_bp.set_attribute('image_size_y', '400')
#     spawn_point = carla.Transform(carla.Location(x=2.3, z=1.0))

#     # # 创建传感器并绑定回调
#     # camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)
#     # camera_handler = SensorDataHandler(data_provider=None, tag='front_camera', sender=sender)
#     # camera.listen(lambda data: camera_handler(data))

    
#     # try:
#     #     print("Sensors are listening. Press Ctrl+C to exit.")
#     #     while True:
#     #         # 运行主循环，等待传感器数据
#     #         time.sleep(1)
#     #         # 暂时不接收控制信号
#     # except KeyboardInterrupt:
#     #     print("Stopping...")
#     # finally:
#     #     sender.close()
#     #     camera.stop()
#     #     camera.destroy()
#     #     lidar.stop()
#     #     lidar.destroy()
#     #     # 销毁其他传感器
 
if __name__ == "__main__":
    main()
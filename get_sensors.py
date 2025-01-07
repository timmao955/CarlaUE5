import carla
import cv2
import numpy as np
import threading

class GetSensors:
    def __init__(self, actor, sensors_config):
        self._actor = actor
        self._cv_images = {}
        self._sensors = []

        world = actor.get_world()
        blueprint_library = world.get_blueprint_library()

        for sensor_config in sensors_config:
            blueprint = blueprint_library.find(sensor_config['type'])
            for attr, value in sensor_config.get('attributes', {}).items():
                blueprint.set_attribute(attr, str(value))
            
            # 创建 Transform
            transform = carla.Transform(
                location=carla.Location(x=sensor_config['x'], y=sensor_config['y'], z=sensor_config['z']),
                rotation=carla.Rotation(roll=sensor_config['roll'], pitch=sensor_config['pitch'], yaw=sensor_config['yaw'])
            )

            # 生成传感器并附加到车辆
            sensor = world.spawn_actor(blueprint, transform, attach_to = actor)
            sensor.listen(lambda data, id=sensor_config['id']: self._on_sensor_update(data, id))
            self._sensors.append(sensor)
 
    def _on_sensor_update(self, data, sensor_id):
        if isinstance(data, carla.Image):
            image_data = np.frombuffer(data.raw_data, dtype=np.uint8)
            np_image = np.reshape(image_data, (data.height, data.width, 4))
            self._cv_images[sensor_id] = np_image[:, :, :3][:, :, ::-1]  # Convert BGRA to RGB
 
    def render(self):
        display = []

        for sensor_id in [
            'CAM_FRONT_LEFT',
            'CAM_FRONT',
            'CAM_FRONT_RIGHT'
        ]:
            if sensor_id in self._cv_images:
                display.append(self._cv_images[sensor_id])

        black_image = np.zeros_like(self._cv_images.get('bev', None))
        display.append(black_image)
        display.append(self._cv_images.get('bev', None))
        display.append(black_image)

        for sensor_id in [
            'CAM_BACK_LEFT',
            'CAM_BACK',
            'CAM_BACK_RIGHT'
        ]:
            if sensor_id in self._cv_images:
                display.append(self._cv_images[sensor_id])

        if len(display) == 9:
            top_row = cv2.hconcat([display[0], display[1], display[2]])
            middle_row = cv2.hconcat([display[3], display[4], display[5]])
            bottom_row = cv2.hconcat([display[6], display[7], display[8]])
            combined_image = cv2.vconcat([top_row, middle_row, bottom_row])

            velocity = self._actor.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6
            speed_text = f"{int(speed):3d} kph"
            cv2.putText(combined_image, speed_text, (850, 370), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            cv2.imshow("Carla Camera Visualizer", combined_image)
            cv2.resizeWindow("Carla Camera Visualizer", 800, 600)
            cv2.waitKey(1)
 
    def reset(self):
        for sensor in self._sensors:
            sensor.destroy()
        self._sensors.clear()
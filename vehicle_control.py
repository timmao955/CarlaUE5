import carla
import math
import time

class VehicleControl:
    def move_to_waypoints(vehicle, waypoints, world, speed = 20.0):

        current_index = 0
    
        while current_index < len(waypoints):

            # 当前路径点
            target_waypoint = waypoints[current_index][0].location
    
            # 获取车辆当前位置
            vehicle_transform = vehicle.get_transform()
            current_location = vehicle_transform.location
    
            # 计算距离
            distance = current_location.distance(target_waypoint)
    
            if distance < 2.0:  # 如果车辆距离路径点很近，则切换到下一个路径点
                current_index += 1
                continue
    
            # 计算方向
            direction_vector = carla.Vector3D(
                x = target_waypoint.x - current_location.x,
                y = target_waypoint.y - current_location.y,
                z = 0.0
            )
            norm = math.sqrt(direction_vector.x**2 + direction_vector.y**2)
            if norm != 0:
                direction_vector.x /= norm
                direction_vector.y /= norm
    
            # 计算目标方向的角度
            target_yaw = math.degrees(math.atan2(direction_vector.y, direction_vector.x))
            current_yaw = vehicle_transform.rotation.yaw
            steering_angle = target_yaw - current_yaw
    
            # 确保方向盘角度在 [-1, 1] 之间
            steering_angle = max(min(steering_angle / 45.0, 1.0), -1.0)
    
            # 设置控制命令
            control = carla.VehicleControl()
            control.throttle = 0.5 if distance > 5 else 0.2  # 根据距离调整油门
            control.steer = steering_angle
            control.brake = 0.0
    
            # 应用控制命令
            vehicle.apply_control(control)
    
            # 渲染车辆传感器数据和其他逻辑
            world.tick()
            time.sleep(0.05)  # 控制刷新频率
    
        # 停止车辆
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
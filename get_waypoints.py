import carla
import xml.etree.ElementTree as ET

class GetWaypoints:
    # 获取waypoints
    def get_way_points(xml_file):
        way_points = []

        tree = ET.parse(xml_file)
        root = tree.getroot()

        for route in root.findall('route'):
            waypoints = route.find('waypoints')
            for position in waypoints.findall('position'):
                x = float(position.get('x'))
                y = float(position.get('y'))
                z = float(position.get('z'))
                waypoint = carla.Transform(location = carla.Location(x = x, y = y, z = z))
                way_points.append((waypoint, None))
        
        return way_points

    # 绘制waypoints
    def draw_way_points(world, way_points, size):
        for i, (waypoint, road_option) in enumerate(way_points):
            color = carla.Color(0, 128, 0)
            world.debug.draw_point(waypoint.location,
                                size = size,
                                color = color,
                                life_time = float('inf'))
        # 起点 
        world.debug.draw_point(way_points[0][0].location,
                            size = 2*size,
                            color = carla.Color(0, 0, 255),
                            life_time = float('inf'))
        # 终点
        world.debug.draw_point(way_points[-1][0].location,
                            size = 2*size,
                            color = carla.Color(255, 0, 0),
                            life_time = float('inf'))
import carla

class GetParking:
    def __init__(self, route, world, map_name, list_scenarios):
        self.route = route
        self.world = world,
        self.map = map_name
        self.list_scenarios = list_scenarios

    def get_parking_slots(self, max_distance = 1, route_step = 1):
        # 判断停车位是否靠近路线
        def is_close(slot_location):
            for i in range(0, len(self.route), route_step):
                route_transform = self.route[i][0]
                if route_transform.location.distance(slot_location) < max_distance:
                    return True
            return False
        
        # 计算路径范围边线
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        for route_transform, _ in self.route:
            min_x = min(min_x, route_transform.location.x - max_distance)
            min_y = min(min_y, route_transform.location.y - max_distance)
            max_x = max(max_x, route_transform.location.x + max_distance)
            max_y = max(max_y, route_transform.location.y + max_distance)

        # 获取已占用停车位
        occupied_parking_locations = []
        for scenario in self.list_scenarios:
            occupied_parking_locations.extend(scenario.get_parking_slots())

        # 获取所有停车位
        available_parking_locations = []
        map_name = self.map.name.split('/')[-1]
        available_parking_locations = getattr(parked_vehicles, map_name, [])

        # Exclude parking slots that are too far from the route
        for slot in available_parking_locations:
            slot_transform = carla.Transform(
                location=carla.Location(slot["location"][0], slot["location"][1], slot["location"][2]),
                rotation=carla.Rotation(slot["rotation"][0], slot["rotation"][1], slot["rotation"][2])
            )

            in_area = (min_x < slot_transform.location.x < max_x) and (min_y < slot_transform.location.y < max_y)
            close_to_route = is_close(slot_transform.location)
            if not in_area or not close_to_route:
                available_parking_locations.remove(slot)
                continue

        self.available_parking_locations = available_parking_locations
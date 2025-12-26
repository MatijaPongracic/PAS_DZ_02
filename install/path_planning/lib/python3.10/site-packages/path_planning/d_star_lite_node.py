import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import time

from .d_star_lite import DStarLite


class DStarLiteNode(Node):
    def __init__(self):
        super().__init__('d_star_lite_node')

        self.get_logger().info('Inicijalizacija D* Lite čvora...')

        # Parametri
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/visualization_marker_array')
        self.declare_parameter('discretization', 0.25)

        self.map_topic = self.get_parameter('map_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.discretization = float(self.get_parameter('discretization').value)

        # Varijable za mapu
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_frame = 'map'
        self.map_data = None

        self.grid_width = None
        self.grid_height = None

        self.d_star_lite = None

        # Start/goal u world koordinatama
        self.start_x = None
        self.start_y = None
        self.goal_x = None
        self.goal_y = None
        self.have_start = False
        self.have_goal = False

        # QoS za mapu
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscriber na mapu
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile=map_qos
        )

        # QoS za markere
        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher za markere
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            qos_profile=marker_qos
        )

        # Subscriber za 2D Pose Estimate (start)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # Subscriber za 2D Nav Goal (goal)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.get_logger().info(
            f'Čvor pokrenut. Čeka mapu na {self.map_topic}, '
            f'start na /initialpose, goal na /goal_pose.'
        )


    def map_callback(self, msg: OccupancyGrid):
        """Spremi mapu i pripremi grid, ali NE pokreći planiranje dok nema start+goal."""
        self.get_logger().info('Mapa primljena!')

        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_frame = msg.header.frame_id or 'map'
        self.map_data = msg.data

        self.get_logger().info(
            f'Veličina mape: {self.map_width} x {self.map_height}'
        )
        self.get_logger().info(
            f'Rezolucija mape: {self.map_resolution} m/cell'
        )

        # Diskretizacija grida
        discretization_step = int(self.discretization / self.map_resolution)
        self.grid_width = (self.map_width + discretization_step - 1) // discretization_step
        self.grid_height = (self.map_height + discretization_step - 1) // discretization_step

        self.get_logger().info(
            f'Diskretizacija: {self.discretization}m ({discretization_step} px)'
        )
        self.get_logger().info(
            f'Diskretizirani grid: {self.grid_width} x {self.grid_height}'
        )

        # Prepreke
        obstacle_coords = self.extract_obstacles_from_map(discretization_step)
        self.get_logger().info(f'Pronađeno prepreka: {len(obstacle_coords)}')

        self.d_star_lite = DStarLite(
            self.grid_width,
            self.grid_height,
            obstacle_coords
        )

        # Ako su start i goal zadani, pokreni planiranje
        self.maybe_run_planning(discretization_step)

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        self.have_start = True
        self.get_logger().info(f'Start postavljen preko RViz-a: ({self.start_x:.2f}, {self.start_y:.2f})')

        # Plavi marker na startu
        if self.map_frame is not None:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.start_x
            marker.pose.position.y = self.start_y
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.discretization * 1.2
            marker.scale.y = self.discretization * 1.2
            marker.scale.z = self.discretization * 1.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.9
            ma = MarkerArray()
            ma.markers.append(marker)
            self.marker_publisher.publish(ma)

        if self.map_width is not None:
            discretization_step = int(self.discretization / self.map_resolution)
            self.maybe_run_planning(discretization_step)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.have_goal = True
        self.get_logger().info(f'Goal postavljen preko RViz-a: ({self.goal_x:.2f}, {self.goal_y:.2f})')

        # Zuti marker na goalu
        if self.map_frame is not None:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.goal_x
            marker.pose.position.y = self.goal_y
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.discretization * 1.2
            marker.scale.y = self.discretization * 1.2
            marker.scale.z = self.discretization * 1.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            ma = MarkerArray()
            ma.markers.append(marker)
            self.marker_publisher.publish(ma)

        if self.map_width is not None:
            discretization_step = int(self.discretization / self.map_resolution)
            self.maybe_run_planning(discretization_step)
            

    def maybe_run_planning(self, discretization_step: int):
        """Pokreće planiranje samo ako su dostupni: mapa + start + goal + D* Lite."""
        if self.map_width is None or self.d_star_lite is None:
            return
        if not (self.have_start and self.have_goal):
            return

        # Konverzija iz world u grid
        start_grid = self.world_to_grid(self.start_x, self.start_y, discretization_step)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, discretization_step)

        self.get_logger().info(f'Start (grid): {start_grid}')
        self.get_logger().info(f'Goal  (grid): {goal_grid}')

        if not self.is_valid_grid_position(start_grid):
            self.get_logger().error('Start je prepreka ili izvan mape (grid)!')
            return

        if not self.is_valid_grid_position(goal_grid):
            self.get_logger().error('Goal je prepreka ili izvan mape (grid)!')
            return

        # Pretraga
        self.get_logger().info('Započinjanje D* Lite pretraživanja...')
        start_time = time.time()

        path, examined_nodes = self.d_star_lite.search(start_grid, goal_grid)

        elapsed_time = time.time() - start_time
        self.get_logger().info(f'Pretraživanje gotovo za {elapsed_time:.4f} s')
        self.get_logger().info(f'Pregledano čvorova: {len(examined_nodes)}')
        self.get_logger().info(f'Duljina putanje: {len(path)} čvorova')

        if len(path) == 0:
            self.get_logger().warn('Putanja nije pronađena.')
            self.publish_visualization(examined_nodes, path, discretization_step, animate=False)
            return

        self.get_logger().info('Započinjanje animirane vizualizacije...')
        self.publish_visualization(examined_nodes, path, discretization_step, animate=True, delay=0.01)


    def extract_obstacles_from_map(self, discretization_step):
        obstacles = set()

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                map_y_start = grid_y * discretization_step
                map_x_start = grid_x * discretization_step

                is_obstacle = False

                for my in range(map_y_start, min(map_y_start + discretization_step, self.map_height)):
                    for mx in range(map_x_start, min(map_x_start + discretization_step, self.map_width)):
                        index = my * self.map_width + mx
                        if index < len(self.map_data):
                            occupancy = self.map_data[index]
                            if occupancy > 0:
                                is_obstacle = True
                                break
                    if is_obstacle:
                        break

                if is_obstacle:
                    obstacles.add((grid_x, grid_y))

        return obstacles

    def world_to_grid(self, world_x, world_y, discretization_step):
        pixel_x = int((world_x - self.map_origin_x) / self.map_resolution)
        pixel_y = int((world_y - self.map_origin_y) / self.map_resolution)

        pixel_x = max(0, min(pixel_x, self.map_width - 1))
        pixel_y = max(0, min(pixel_y, self.map_height - 1))

        grid_x = pixel_x // discretization_step
        grid_y = pixel_y // discretization_step

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, discretization_step):
        pixel_x = (grid_x * discretization_step) + (discretization_step // 2)
        pixel_y = (grid_y * discretization_step) + (discretization_step // 2)

        world_x = pixel_x * self.map_resolution + self.map_origin_x
        world_y = pixel_y * self.map_resolution + self.map_origin_y

        return (world_x, world_y)

    def is_valid_grid_position(self, grid_pos):
        x, y = grid_pos
        if x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False
        if (x, y) in self.d_star_lite.obstacles:
            return False
        return True


    def publish_visualization(self, examined_nodes, path, discretization_step,
                              animate=True, delay=0.01):
        """
        Tijekom animacije: svi dosad pretraženi čvorovi (crveni) akumulativno.
        Na kraju: crveni na putanji se obrišu i zamijene zelenima.
        """
        base_array = MarkerArray()
        marker_id = 0

        # Start marker (plavi)
        start_marker = Marker()
        start_marker.header.frame_id = self.map_frame
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = marker_id
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_x
        start_marker.pose.position.y = self.start_y
        start_marker.pose.position.z = 0.25
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = self.discretization * 1.2
        start_marker.scale.y = self.discretization * 1.2
        start_marker.scale.z = self.discretization * 1.2
        start_marker.color.r = 0.0
        start_marker.color.g = 0.0
        start_marker.color.b = 1.0
        start_marker.color.a = 0.9
        base_array.markers.append(start_marker)
        marker_id += 1

        # Goal marker (zuti)
        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = marker_id
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.25
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = self.discretization * 1.2
        goal_marker.scale.y = self.discretization * 1.2
        goal_marker.scale.z = self.discretization * 1.2
        goal_marker.color.r = 1.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.9
        base_array.markers.append(goal_marker)

        if not animate:
            final = MarkerArray()
            final.markers.extend(base_array.markers)

            red_id = 20000
            for gx, gy in examined_nodes:
                wx, wy = self.grid_to_world(gx, gy, discretization_step)
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.id = red_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.1
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = self.discretization * 0.8
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.7
                final.markers.append(m)
                red_id += 1

            green_id = 30000
            for gx, gy in path:
                wx, wy = self.grid_to_world(gx, gy, discretization_step)
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.id = green_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.15
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = self.discretization * 0.9
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 0.8
                final.markers.append(m)
                green_id += 1

            self.marker_publisher.publish(final)
            return

        # 1) crveni (pretrazeni) covorovi
        for i, (gx, gy) in enumerate(examined_nodes):
            frame = MarkerArray()
            frame.markers.extend(base_array.markers)

            for j in range(i + 1):
                pgx, pgy = examined_nodes[j]
                pwx, pwy = self.grid_to_world(pgx, pgy, discretization_step)

                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.id = 20000 + j
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = pwx
                m.pose.position.y = pwy
                m.pose.position.z = 0.1
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = self.discretization * 0.8
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.7
                frame.markers.append(m)

            self.marker_publisher.publish(frame)
            time.sleep(delay)

        # 2) brisanje crvenih i dodavanje zelenih na putanji
        index_of = {pos: idx for idx, pos in enumerate(examined_nodes)}

        delete_array = MarkerArray()
        for gx, gy in path:
            idx = index_of.get((gx, gy), None)
            if idx is None:
                continue
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 20000 + idx
            m.action = Marker.DELETE
            delete_array.markers.append(m)
        self.marker_publisher.publish(delete_array)

        green_array = MarkerArray()
        green_array.markers.extend(base_array.markers)

        green_id = 30000
        for gx, gy in path:
            wx, wy = self.grid_to_world(gx, gy, discretization_step)
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = green_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.discretization * 0.9
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            green_array.markers.append(m)
            green_id += 1

        for idx, (gx, gy) in enumerate(examined_nodes):
            if (gx, gy) in path:
                continue
            wx, wy = self.grid_to_world(gx, gy, discretization_step)
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 20000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.discretization * 0.8
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.7
            green_array.markers.append(m)

        self.marker_publisher.publish(green_array)


def main(args=None):
    rclpy.init(args=args)
    node = DStarLiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

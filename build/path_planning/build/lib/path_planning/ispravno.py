import math
import heapq
from collections import namedtuple

# Jednostavna Node struktura
Node = namedtuple('Node', ['x', 'y'])


class DStarLite:
    """
    D* Lite algoritam za planiranje putanje na gridu
    Pojednostavljena verzija koja radi sa A* pristupom
    """
    
    # Mogućnosti gibanja (8-smjerna povezanost)
    MOTIONS = [
        (1, 0, 1.0),      # desno
        (0, 1, 1.0),      # gore
        (-1, 0, 1.0),     # lijevo
        (0, -1, 1.0),     # dolje
        (1, 1, math.sqrt(2)),   # gore-desno
        (1, -1, math.sqrt(2)),  # dolje-desno
        (-1, 1, math.sqrt(2)),  # gore-lijevo
        (-1, -1, math.sqrt(2))  # dolje-lijevo
    ]
    
    def __init__(self, grid_width, grid_height, obstacle_coords):
        """
        Inicijalizacija D* Lite
        """
        self.width = grid_width
        self.height = grid_height
        self.obstacles = set(obstacle_coords)
        
        # g vrijednosti
        self.g = {}
        
        # f vrijednosti (za A*)
        self.f = {}
        
        # Parent mapa za rekonstrukciju putanje
        self.parent = {}
        
        # Start i goal
        self.start = None
        self.goal = None
        
        # Pregledani čvorovi (za vizualizaciju)
        self.examined_nodes = []
    
    def is_obstacle(self, x, y):
        """Provjeri je li pozicija prepreka"""
        return (x, y) in self.obstacles
    
    def is_valid(self, x, y):
        """Provjeri je li pozicija valjana"""
        return 0 <= x < self.width and 0 <= y < self.height and not self.is_obstacle(x, y)
    
    def get_neighbours(self, x, y):
        """Dohvati sve validne susjedne čvorove"""
        neighbours = []
        for dx, dy, cost in self.MOTIONS:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbours.append((nx, ny, cost))
        return neighbours
    
    def heuristic(self, x1, y1, x2, y2):
        """Heuristička funkcija (Euklid)"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def search(self, start, goal):
        """
        Pronađi putanju od start do goal pomoću A*
        """
        self.start = start
        self.goal = goal
        self.examined_nodes = []
        self.g = {start: 0}
        self.parent = {start: None}
        
        # Priority queue: (f_value, counter, x, y)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0, counter, start[0], start[1]))
        counter += 1
        
        open_set_dict = {start}
        closed_set = set()
        
        while open_set:
            _, _, x, y = heapq.heappop(open_set)
            current = (x, y)
            
            if current in closed_set:
                continue
            
            open_set_dict.discard(current)
            closed_set.add(current)
            self.examined_nodes.append(current)
            
            # Ako smo stigli do goal-a
            if current == self.goal:
                path = self._reconstruct_path()
                return path, self.examined_nodes
            
            # Obradi sve susjede
            for nx, ny, move_cost in self.get_neighbours(x, y):
                neighbour = (nx, ny)
                
                if neighbour in closed_set:
                    continue
                
                # Izračunaj novu g vrijednost
                tentative_g = self.g[current] + move_cost
                
                # Ako smo našli bolu putanju
                if neighbour not in self.g or tentative_g < self.g[neighbour]:
                    self.parent[neighbour] = current
                    self.g[neighbour] = tentative_g
                    
                    # Izračunaj f vrijednost
                    h = self.heuristic(nx, ny, self.goal[0], self.goal[1])
                    f = tentative_g + h
                    self.f[neighbour] = f
                    
                    if neighbour not in open_set_dict:
                        heapq.heappush(open_set, (f, counter, nx, ny))
                        counter += 1
                        open_set_dict.add(neighbour)
        
        # Nema putanje
        return [], self.examined_nodes
    
    def _reconstruct_path(self):
        """Rekonstruiraj putanju od start do goal"""
        path = []
        current = self.goal
        
        while current is not None:
            path.append(current)
            current = self.parent.get(current)
        
        path.reverse()
        return path

######################################################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import time

# Importaj D* Lite algoritam
from .d_star_lite import DStarLite


class DStarLiteNode(Node):
    """
    ROS2 čvor s D* Lite algoritmom za planiranje putanje
    """
    def __init__(self):
        super().__init__('d_star_lite_node')

        self.get_logger().info('Inicijalizacija D* Lite čvora...')

        # Parametri
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/visualization_marker_array')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 18.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('discretization', 0.25)  # 0.25m po čvoru

        # Dohvati parametre
        self.map_topic = self.get_parameter('map_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.discretization = float(self.get_parameter('discretization').value)

        # Varijable za mapu
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_frame = 'map'
        self.map_data = None

        # Diskretizovani grid
        self.grid_width = None
        self.grid_height = None
        self.grid_resolution = self.discretization  # 0.25m

        # D* Lite instanca
        self.d_star_lite = None

        # QoS profil za mapu
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscriber za mapu
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile=map_qos
        )

        # QoS profil za markere
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

        self.get_logger().info(f'Čvor pokrenut, čeka mapu na {self.map_topic}')

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback - kada je mapa primljena
        """
        self.get_logger().info('Mapa primljena!')

        # Spremi info o mapi
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

        # Izračunaj diskretizovani grid
        discretization_step = int(self.discretization / self.map_resolution)
        self.grid_width = (self.map_width + discretization_step - 1) // discretization_step
        self.grid_height = (self.map_height + discretization_step - 1) // discretization_step

        self.get_logger().info(
            f'Diskretizacija: {self.discretization}m ({discretization_step} px)'
        )
        self.get_logger().info(
            f'Diskretizovani grid: {self.grid_width} x {self.grid_height}'
        )

        # Izvuci prepreke iz mape
        obstacle_coords = self.extract_obstacles_from_map(discretization_step)
        self.get_logger().info(f'Pronađeno prepreka: {len(obstacle_coords)}')

        # Kreiraj D* Lite instancu
        self.d_star_lite = DStarLite(
            self.grid_width,
            self.grid_height,
            obstacle_coords
        )

        # Konveriraj start i goal u grid koordinate
        start_grid = self.world_to_grid(self.start_x, self.start_y, discretization_step)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, discretization_step)

        self.get_logger().info(f'Start (world): ({self.start_x}, {self.start_y})')
        self.get_logger().info(f'Start (grid): {start_grid}')
        self.get_logger().info(f'Goal (world): ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Goal (grid): {goal_grid}')

        # Provjeri jesu li validne pozicije
        if not self.is_valid_grid_position(start_grid, discretization_step):
            self.get_logger().error('Start je prepreka ili izvan mape!')
            return

        if not self.is_valid_grid_position(goal_grid, discretization_step):
            self.get_logger().error('Goal je prepreka ili izvan mape!')
            return

        # Izvršiti D* Lite pretragu
        self.get_logger().info('Započinjanje D* Lite pretraživanja...')
        start_time = time.time()

        path, examined_nodes = self.d_star_lite.search(start_grid, goal_grid)

        elapsed_time = time.time() - start_time

        self.get_logger().info(
            f'Pretraživanje gotovo za {elapsed_time:.4f} sekundi'
        )
        self.get_logger().info(
            f'Pregledano čvorova: {len(examined_nodes)}'
        )
        self.get_logger().info(f'Duljina putanje: {len(path)} čvorova')

        # Vizualizacija pretraživanja
        self.publish_visualization(
            examined_nodes, path, discretization_step
        )

    def extract_obstacles_from_map(self, discretization_step):
        """
        Izvuci prepreke iz OccupancyGrid mape
        Ako je bilo koja ćelija u 5x5 bloku zauzeta, čitav blok je prepreka
        """
        obstacles = set()

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                # Pronađi odgovarajući blok u originalnoj mapi
                map_y_start = grid_y * discretization_step
                map_x_start = grid_x * discretization_step

                is_obstacle = False

                # Provjeri sve ćelije u bloku
                for my in range(map_y_start, min(map_y_start + discretization_step, self.map_height)):
                    for mx in range(map_x_start, min(map_x_start + discretization_step, self.map_width)):
                        index = my * self.map_width + mx
                        if index < len(self.map_data):
                            occupancy = self.map_data[index]
                            # Ako je bilo šta zauzeto, čitava ćelija je prepreka
                            if occupancy > 0:
                                is_obstacle = True
                                break
                    if is_obstacle:
                        break

                if is_obstacle:
                    obstacles.add((grid_x, grid_y))

        return obstacles

    def world_to_grid(self, world_x, world_y, discretization_step):
        """
        Pretvori svjetske koordinate u diskretizovane grid koordinate
        """
        # Prvo pretvori u pixel koordinate
        pixel_x = int((world_x - self.map_origin_x) / self.map_resolution)
        pixel_y = int((world_y - self.map_origin_y) / self.map_resolution)

        # Osiguraj da su u granicama
        pixel_x = max(0, min(pixel_x, self.map_width - 1))
        pixel_y = max(0, min(pixel_y, self.map_height - 1))

        # Pretvori u diskretizovane grid koordinate
        grid_x = pixel_x // discretization_step
        grid_y = pixel_y // discretization_step

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, discretization_step):
        """
        Pretvori grid koordinate u svjetske koordinate
        Koristi centar diskretizovane ćelije
        """
        # Pretvori u pixel koordinate (centar bloka)
        pixel_x = (grid_x * discretization_step) + (discretization_step // 2)
        pixel_y = (grid_y * discretization_step) + (discretization_step // 2)

        # Pretvori u svjetske koordinate
        world_x = pixel_x * self.map_resolution + self.map_origin_x
        world_y = pixel_y * self.map_resolution + self.map_origin_y

        return (world_x, world_y)

    def is_valid_grid_position(self, grid_pos, discretization_step):
        """
        Provjeri je li grid pozicija valjana
        """
        x, y = grid_pos

        if x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False

        # Provjeri je li prepreka
        if (x, y) in self.d_star_lite.obstacles:
            return False

        return True

    def publish_visualization(self, examined_nodes, path, discretization_step):
        """
        Objavi markere za vizualizaciju pretraživanja
        - Crveni markeri: pregledani čvorovi
        - Zeleni markeri: putanja
        - Plavi marker: start (na stvarnoj world poziciji)
        - Žuti marker: goal (na stvarnoj world poziciji)
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Crveni markeri za pregledane čvorove
        for grid_x, grid_y in examined_nodes:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, discretization_step)

            marker = Marker()
            marker.header = Header(frame_id=self.map_frame)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(x=world_x, y=world_y, z=0.1)
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.discretization * 0.8
            marker.scale.y = self.discretization * 0.8
            marker.scale.z = self.discretization * 0.8

            # Crvena boja
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

            marker_array.markers.append(marker)
            marker_id += 1

        # Zeleni markeri za putanju
        for grid_x, grid_y in path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, discretization_step)

            marker = Marker()
            marker.header = Header(frame_id=self.map_frame)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(x=world_x, y=world_y, z=0.15)
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.discretization * 0.9
            marker.scale.y = self.discretization * 0.9
            marker.scale.z = self.discretization * 0.9

            # Zelena boja
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

            marker_array.markers.append(marker)
            marker_id += 1

        # PLAVI marker za START - koristi STVARNU world poziciju
        start_marker = Marker()
        start_marker.header = Header(frame_id=self.map_frame)
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = marker_id
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        start_marker.pose.position = Point(x=self.start_x, y=self.start_y, z=0.25)
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale.x = self.discretization * 1.2
        start_marker.scale.y = self.discretization * 1.2
        start_marker.scale.z = self.discretization * 1.2
        
        # PLAVA boja
        start_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
        
        marker_array.markers.append(start_marker)
        marker_id += 1

        # ŽUTI marker za GOAL - koristi STVARNU world poziciju
        goal_marker = Marker()
        goal_marker.header = Header(frame_id=self.map_frame)
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = marker_id
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.25)
        goal_marker.pose.orientation.w = 1.0
        
        goal_marker.scale.x = self.discretization * 1.2
        goal_marker.scale.y = self.discretization * 1.2
        goal_marker.scale.z = self.discretization * 1.2
        
        # ŽUTA boja
        goal_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        
        marker_array.markers.append(goal_marker)
        marker_id += 1

        # Objavi markere
        self.marker_publisher.publish(marker_array)
        self.get_logger().info(
            f'Objavljeno {len(examined_nodes)} pregledanih čvorova, '
            f'{len(path)} čvorova putanje, start (plavi) i goal (žuti) markeri'
        )


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


if __name__ == '__main__':
    main()



###############################################3
import math


class DStarLite:
    """
    D* Lite algoritam za planiranje putanje
    Prema: "D* Lite" (Koenig & Likhachev, 2002)
    """
    
    # Mogućnosti gibanja (8-smjerna povezanost)
    MOTIONS = [
        (1, 0, 1.0),      # desno
        (0, 1, 1.0),      # gore
        (-1, 0, 1.0),     # lijevo
        (0, -1, 1.0),     # dolje
        (1, 1, math.sqrt(2)),   # gore-desno
        (1, -1, math.sqrt(2)),  # dolje-desno
        (-1, 1, math.sqrt(2)),  # gore-lijevo
        (-1, -1, math.sqrt(2))  # dolje-lijevo
    ]
    
    def __init__(self, grid_width, grid_height, obstacle_coords):
        """
        Inicijalizacija D* Lite
        """
        self.width = grid_width
        self.height = grid_height
        self.obstacles = set(obstacle_coords)
        
        # g vrijednost: procjena troška od start do čvora
        self.g = {}
        
        # rhs vrijednost: heuristička procjena
        self.rhs = {}
        
        # Priority queue U - kao lista sortianih ključeva
        self.U = []
        
        # km vrijednost za reprioritetizaciju
        self.km = 0.0
        
        # Start i goal
        self.start = None
        self.goal = None
        
        # Pregledani čvorovi (za vizualizaciju)
        self.examined_nodes = []
        
        # Detektirane nove prepreke
        self.detected_obstacles = set()
    
    def is_obstacle(self, x, y):
        """Provjeri je li pozicija prepreka"""
        return (x, y) in self.obstacles or (x, y) in self.detected_obstacles
    
    def is_valid(self, x, y):
        """Provjeri je li pozicija valjana"""
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                not self.is_obstacle(x, y))
    
    def get_neighbours(self, x, y):
        """Dohvati sve validne susjedne čvorove"""
        neighbours = []
        for dx, dy, cost in self.MOTIONS:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbours.append(((nx, ny), cost))
        return neighbours
    
    def heuristic(self, pos1, pos2):
        """Heuristička funkcija - Chebyshev"""
        x1, y1 = pos1
        x2, y2 = pos2
        # Chebyshev distanca - bolja za 8-smjernu povezanost
        return max(abs(x2 - x1), abs(y2 - y1))

    
    def calculate_key(self, pos):
        """Izračunaj ključ za prioritetni red"""
        g_val = self.g.get(pos, float('inf'))
        rhs_val = self.rhs.get(pos, float('inf'))
        h_val = self.heuristic(self.start, pos)
        
        k1 = min(g_val, rhs_val) + h_val + self.km
        k2 = min(g_val, rhs_val)
        
        return (k1, k2, pos[0], pos[1])
    
    def update_vertex(self, pos):
        """Ažuriraj čvor"""
        # Ako nije cilj, izračunaj rhs vrijednost
        if pos != self.goal:
            min_val = float('inf')
            for neighbour, cost in self.get_neighbours(pos[0], pos[1]):
                val = self.g.get(neighbour, float('inf')) + cost
                min_val = min(min_val, val)
            self.rhs[pos] = min_val
        
        # Ako je već u U, ukloni ga
        self.U = [item for item in self.U if item[2:4] != (pos[0], pos[1])]
        
        # Ako g != rhs, dodaj u U
        g_val = self.g.get(pos, float('inf'))
        rhs_val = self.rhs.get(pos, float('inf'))
        
        if g_val != rhs_val:
            key = self.calculate_key(pos)
            self.U.append(key)
    
    def compute_shortest_path(self):
        """Glavna D* Lite funkcija - Compute_Shortest_Path"""
        iterations = 0
        max_iterations = self.width * self.height * 100
        
        while iterations < max_iterations:
            iterations += 1
            
            # Sortiraj U po ključevima
            self.U.sort(key=lambda x: (x[0], x[1]))
            
            # Ako je U prazan ili je start konzistentan, gotovo
            if not self.U:
                break
            
            # Provjeri je li start konzistentan
            start_g = self.g.get(self.start, float('inf'))
            start_rhs = self.rhs.get(self.start, float('inf'))
            start_key = self.calculate_key(self.start)
            
            if start_g == start_rhs and self.U[0] >= start_key:
                break
            
            # Dohvati čvor sa najmanjim ključem
            kold = self.U[0]
            u_pos = (self.U[0][2], self.U[0][3])
            self.U.pop(0)
            
            # Izračunaj novi ključ
            knew = self.calculate_key(u_pos)
            
            # Ako je ključ promijenjen, ponovno ga dodaj u U
            if (kold[0], kold[1]) < (knew[0], knew[1]):
                self.U.append(knew)
                continue
            
            # Zapamti kao pregledani čvor
            if u_pos not in self.examined_nodes:
                self.examined_nodes.append(u_pos)
            
            # Pronađi sve susjede
            g_val = self.g.get(u_pos, float('inf'))
            rhs_val = self.rhs.get(u_pos, float('inf'))
            
            if g_val > rhs_val:
                # Čvor je Over-consistent - postavi g = rhs
                self.g[u_pos] = rhs_val
                
                # Ažuriraj sve susjede
                for neighbour, cost in self.get_neighbours(u_pos[0], u_pos[1]):
                    if neighbour != self.goal:
                        self.update_vertex(neighbour)
            else:
                # Čvor je Under-consistent - postavi g = inf
                self.g[u_pos] = float('inf')
                
                # Ažuriraj sve susjede i sam čvor
                self.update_vertex(u_pos)
                for neighbour, cost in self.get_neighbours(u_pos[0], u_pos[1]):
                    self.update_vertex(neighbour)
        
        # Provjeri je li start konačno konzistentan
        return self.g.get(self.start, float('inf')) != float('inf')
    
    def extract_path(self):
        """Ekstrahiraj putanju od start do goal koristeći g vrijednosti"""
        if self.g.get(self.start, float('inf')) == float('inf'):
            return []
        
        path = [self.start]
        current = self.start
        max_path_length = self.width * self.height
        steps = 0
        
        while current != self.goal and steps < max_path_length:
            steps += 1
            x, y = current
            
            # Pronađi susjeda sa najmanjom g vrijednosti + troškak prijelaza
            best_neighbour = None
            best_val = float('inf')
            
            for neighbour, cost in self.get_neighbours(x, y):
                g_neighbor = self.g.get(neighbour, float('inf'))
                if g_neighbor < float('inf'):
                    val = g_neighbor + cost
                    if val < best_val:
                        best_val = val
                        best_neighbour = neighbour
            
            if best_neighbour is None:
                # Nema bolje putanje
                return []
            
            # Izbjegni beskonačnu petlju
            if best_neighbour == current:
                return []
            
            path.append(best_neighbour)
            current = best_neighbour
        
        # Provjeri je li putanja valjana
        if current != self.goal:
            return []
        
        return path
    
    def search(self, start, goal):
        """
        Pronađi putanju od start do goal
        
        Returns:
            (path, examined_nodes)
        """
        self.start = start
        self.goal = goal
        self.examined_nodes = []
        self.U = []
        self.g = {}
        self.rhs = {}
        self.km = 0.0
        self.detected_obstacles = set()
        
        # Inicijalizacija: postavi goal na 0, sve ostale na inf
        self.rhs[goal] = 0
        self.g[goal] = float('inf')
        
        # Dodaj goal u priority queue
        key = self.calculate_key(goal)
        self.U.append(key)
        
        # Pronađi putanju
        success = self.compute_shortest_path()
        
        # Ekstrahiraj putanju
        if success:
            path = self.extract_path()
        else:
            path = []
        
        return path, self.examined_nodes

######################### DINAMIČNO
import math


class DStarLite:
    """
    D* Lite algoritam za planiranje putanje
    Prema: "D* Lite" (Koenig & Likhachev, 2002)
    Podržava statičke i dinamičke mape
    """
    
    # Mogućnosti gibanja (8-smjerna povezanost)
    MOTIONS = [
        (1, 0, 1.0),      # desno
        (0, 1, 1.0),      # gore
        (-1, 0, 1.0),     # lijevo
        (0, -1, 1.0),     # dolje
        (1, 1, math.sqrt(2)),   # gore-desno
        (1, -1, math.sqrt(2)),  # dolje-desno
        (-1, 1, math.sqrt(2)),  # gore-lijevo
        (-1, -1, math.sqrt(2))  # dolje-lijevo
    ]
    
    def __init__(self, grid_width, grid_height, obstacle_coords):
        """
        Inicijalizacija D* Lite
        """
        self.width = grid_width
        self.height = grid_height
        
        # Inicijalne prepreke (koje znamo na početku)
        self.initial_obstacles = set(obstacle_coords)
        
        # Sve poznate prepreke (inicijalne + otkrivene)
        self.obstacles = set()
        
        # g vrijednost: trošak od čvora do goal
        self.g = {}
        
        # rhs vrijednost: one-step lookahead trošak
        self.rhs = {}
        
        # Priority queue U - kao lista sortiranih ključeva
        self.U = []
        
        # km vrijednost za reprioritetizaciju
        self.km = 0.0
        
        # Start i goal
        self.start = None
        self.goal = None
        
        # Zadnja pozicija (za km update)
        self.last = None
        
        # Pregledani čvorovi (za vizualizaciju)
        self.examined_nodes = []
        
        # Detektirane nove prepreke tijekom izvršavanja
        self.detected_obstacles = set()
        
        # Flag da li je algoritam inicijaliziran
        self.initialized = False
        
        # Callback funkcija za replanning
        self.replanning_callback = None
    
    def is_obstacle(self, x, y):
        """Provjeri je li pozicija prepreka"""
        return (x, y) in self.obstacles
    
    def is_valid(self, x, y):
        """Provjeri je li pozicija valjana"""
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                not self.is_obstacle(x, y))
    
    def get_neighbours(self, x, y):
        """Dohvati sve validne susjedne čvorove"""
        neighbours = []
        for dx, dy, cost in self.MOTIONS:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):  # ✓ ISPRAVNO - provjerava i prepreke
                neighbours.append(((nx, ny), cost))
        return neighbours
    
    def heuristic(self, pos1, pos2):
        """Heuristička funkcija - Chebyshev"""
        x1, y1 = pos1
        x2, y2 = pos2
        return max(abs(x2 - x1), abs(y2 - y1))
    
    def calculate_key(self, pos):
        """Izračunaj ključ za prioritetni red"""
        g_val = self.g.get(pos, float('inf'))
        rhs_val = self.rhs.get(pos, float('inf'))
        h_val = self.heuristic(self.start, pos)
        
        k1 = min(g_val, rhs_val) + h_val + self.km
        k2 = min(g_val, rhs_val)
        
        return (k1, k2, pos[0], pos[1])
    
    def c(self, pos1, pos2):
        """Trošak prijelaza između dva čvora"""
        if self.is_obstacle(pos2[0], pos2[1]):
            return float('inf')
        
        dx = abs(pos2[0] - pos1[0])
        dy = abs(pos2[1] - pos1[1])
        
        if dx == 1 and dy == 1:
            return math.sqrt(2)
        elif dx + dy == 1:
            return 1.0
        else:
            return float('inf')
    
    def update_vertex(self, pos):
        """Ažuriraj čvor"""
        # Ako nije cilj, izračunaj rhs vrijednost
        if pos != self.goal:
            min_val = float('inf')
            for neighbour, _ in self.get_neighbours(pos[0], pos[1]):
                val = self.g.get(neighbour, float('inf')) + self.c(pos, neighbour)
                min_val = min(min_val, val)
            self.rhs[pos] = min_val
        
        # Ako je već u U, ukloni ga
        self.U = [item for item in self.U if item[2:4] != (pos[0], pos[1])]
        
        # Ako g != rhs, dodaj u U
        g_val = self.g.get(pos, float('inf'))
        rhs_val = self.rhs.get(pos, float('inf'))
        
        if g_val != rhs_val:
            key = self.calculate_key(pos)
            self.U.append(key)
    
    def compute_shortest_path(self):
        """Glavna D* Lite funkcija - Compute_Shortest_Path"""
        iterations = 0
        max_iterations = self.width * self.height * 100
        
        while iterations < max_iterations:
            iterations += 1
            
            # Sortiraj U po ključevima
            self.U.sort(key=lambda x: (x[0], x[1]))
            
            # Ako je U prazan ili je start konzistentan, gotovo
            if not self.U:
                break
            
            # Provjeri je li start konzistentan
            start_g = self.g.get(self.start, float('inf'))
            start_rhs = self.rhs.get(self.start, float('inf'))
            start_key = self.calculate_key(self.start)
            
            if start_g == start_rhs and self.U[0] >= start_key:
                break
            
            # Dohvati čvor sa najmanjim ključem
            kold = self.U[0]
            u_pos = (self.U[0][2], self.U[0][3])
            self.U.pop(0)
            
            # Izračunaj novi ključ
            knew = self.calculate_key(u_pos)
            
            # Ako je ključ promijenjen, ponovno ga dodaj u U
            if (kold[0], kold[1]) < (knew[0], knew[1]):
                self.U.append(knew)
                continue
            
            # Zapamti kao pregledani čvor
            if u_pos not in self.examined_nodes:
                self.examined_nodes.append(u_pos)
            
            # Pronađi sve susjede
            g_val = self.g.get(u_pos, float('inf'))
            rhs_val = self.rhs.get(u_pos, float('inf'))
            
            if g_val > rhs_val:
                # Čvor je Over-consistent - postavi g = rhs
                self.g[u_pos] = rhs_val
                
                # Ažuriraj sve susjede (prethodnike)
                for neighbour, _ in self.get_neighbours(u_pos[0], u_pos[1]):
                    self.update_vertex(neighbour)
            else:
                # Čvor je Under-consistent - postavi g = inf
                self.g[u_pos] = float('inf')
                
                # Ažuriraj sve susjede i sam čvor
                self.update_vertex(u_pos)
                for neighbour, _ in self.get_neighbours(u_pos[0], u_pos[1]):
                    self.update_vertex(neighbour)
        
        # Provjeri je li start konačno konzistentan
        return self.g.get(self.start, float('inf')) != float('inf')
    
    def extract_path(self):
        """Ekstrahiraj putanju od start do goal koristeći g vrijednosti"""
        if self.g.get(self.start, float('inf')) == float('inf'):
            return []
        
        path = [self.start]
        current = self.start
        max_path_length = self.width * self.height
        steps = 0
        
        while current != self.goal and steps < max_path_length:
            steps += 1
            x, y = current
            
            # Pronađi susjeda sa najmanjom g vrijednosti + troškak prijelaza
            best_neighbour = None
            best_val = float('inf')
            
            for neighbour, _ in self.get_neighbours(x, y):
                g_neighbor = self.g.get(neighbour, float('inf'))
                cost = self.c(current, neighbour)
                if g_neighbor < float('inf') and cost < float('inf'):
                    val = g_neighbor + cost
                    if val < best_val:
                        best_val = val
                        best_neighbour = neighbour
            
            if best_neighbour is None:
                # Nema bolje putanje
                return []
            
            # Izbjegni beskonačnu petlju
            if best_neighbour == current:
                return []
            
            path.append(best_neighbour)
            current = best_neighbour
        
        # Provjeri je li putanja valjana
        if current != self.goal:
            return []
        
        return path
    
    def scan_for_obstacles(self, pos):
        """
        Skenira susjedstvo trenutne pozicije i otkriva nove prepreke.
        Vraća listu novootkrivenih prepreka.
        """
        new_obstacles = []
        
        # Skeniraj sve susjede
        for neighbour, _ in self.get_neighbours(pos[0], pos[1]):
            # Ako je susjed u inicijalnim preprekama ali nije u poznatim
            if neighbour in self.initial_obstacles and neighbour not in self.obstacles:
                new_obstacles.append(neighbour)
                self.obstacles.add(neighbour)
                self.detected_obstacles.add(neighbour)
        
        return new_obstacles
    
    def process_obstacle_changes(self, changed_vertices):
        """
        Obradi promjene u preprekama i ažuriraj algoritam.
        changed_vertices: lista (pos, added) gdje je added True za nove, False za uklonjene
        """
        if not changed_vertices:
            return
        
        # Ažuriraj km
        if self.last is not None:
            self.km += self.heuristic(self.last, self.start)
        self.last = self.start
        
        # Pozovi callback ako postoji
        if self.replanning_callback is not None:
            self.replanning_callback(changed_vertices)
        
        # Ažuriraj sve promijenjene čvorove
        for pos, added in changed_vertices:
            if added:
                # Nova prepreka - postavi g i rhs na inf
                self.g[pos] = float('inf')
                self.rhs[pos] = float('inf')
            
            # Ažuriraj sve susjede
            for neighbour, _ in self.get_neighbours(pos[0], pos[1]):
                self.update_vertex(neighbour)
        
        # Replan
        self.compute_shortest_path()
    
    def search(self, start, goal):
        """
        Pronađi putanju od start do goal (statička verzija).
        Koristi se za statičke mape ili inicijalnu pretragu.
        
        Returns:
            (path, examined_nodes)
        """
        self.start = start
        self.goal = goal
        self.examined_nodes = []
        self.U = []
        self.g = {}
        self.rhs = {}
        self.km = 0.0
        self.last = start
        self.detected_obstacles = set()
        self.obstacles = self.initial_obstacles.copy()
        self.initialized = True
        
        # Inicijalizacija: postavi goal na 0, sve ostale na inf
        self.rhs[goal] = 0
        self.g[goal] = float('inf')
        
        # Dodaj goal u priority queue
        key = self.calculate_key(goal)
        self.U.append(key)
        
        # Pronađi putanju
        success = self.compute_shortest_path()
        
        # Ekstrahiraj putanju
        if success:
            path = self.extract_path()
        else:
            path = []
        
        return path, self.examined_nodes
    
    def search_dynamic(self, start, goal, replanning_callback=None):
        """
        Pronađi putanju od start do goal s dinamičkim replanning-om.
        Robot se kreće korak-po-korak i skenira okolinu.
        
        Args:
            start: početna pozicija (x, y)
            goal: ciljna pozicija (x, y)
            replanning_callback: funkcija koja se poziva kada se replan-a
                                 prima listu promijenjenih čvorova
        
        Returns:
            (success, path_taken, all_examined_nodes)
            - success: True ako je cilj dosegnut
            - path_taken: stvarna putanja koju je robot prešao
            - all_examined_nodes: svi pregledani čvorovi
        """
        self.replanning_callback = replanning_callback
        
        # Inicijalizacija
        self.start = start
        self.goal = goal
        self.examined_nodes = []
        self.U = []
        self.g = {}
        self.rhs = {}
        self.km = 0.0
        self.last = start
        self.detected_obstacles = set()
        self.obstacles = set()  # Počinjemo s praznim preprekama
        self.initialized = True
        
        # Inicijalizacija D* Lite
        self.rhs[goal] = 0
        self.g[goal] = float('inf')
        key = self.calculate_key(goal)
        self.U.append(key)
        
        # Inicijalna pretraga
        self.compute_shortest_path()
        
        # Putanja koju robot prelazi
        path_taken = [start]
        current = start
        
        max_steps = self.width * self.height * 2
        step = 0
        
        # Glavna petlja kretanja
        while current != goal and step < max_steps:
            step += 1
            
            # Provjeri postoji li putanja
            if self.g.get(current, float('inf')) == float('inf'):
                # Nema putanje
                return False, path_taken, self.examined_nodes
            
            # Skeniraj okolinu za nove prepreke
            new_obstacles = self.scan_for_obstacles(current)
            
            # Ako su otkrivene nove prepreke, replan
            if new_obstacles:
                changed_vertices = [(obs, True) for obs in new_obstacles]
                self.process_obstacle_changes(changed_vertices)
                
                # Provjeri postoji li još uvijek putanja
                if self.g.get(current, float('inf')) == float('inf'):
                    return False, path_taken, self.examined_nodes
            
            # Odaberi sljedeći korak - čvor sa najmanjom g + c vrijednosti
            best_neighbour = None
            best_val = float('inf')
            
            for neighbour, _ in self.get_neighbours(current[0], current[1]):
                g_neighbor = self.g.get(neighbour, float('inf'))
                cost = self.c(current, neighbour)
                if g_neighbor < float('inf') and cost < float('inf'):
                    val = g_neighbor + cost
                    if val < best_val:
                        best_val = val
                        best_neighbour = neighbour
            
            if best_neighbour is None:
                # Nema valjanog sljedećeg koraka
                return False, path_taken, self.examined_nodes
            
            # Pomakni se na sljedeću poziciju
            current = best_neighbour
            path_taken.append(current)
            self.start = current
        
        # Provjeri je li cilj dosegnut
        success = (current == goal)
        return success, path_taken, self.examined_nodes
    
    def add_obstacle(self, x, y):
        """Dinamički dodaj prepreku"""
        if (x, y) not in self.obstacles:
            self.obstacles.add((x, y))
            self.detected_obstacles.add((x, y))
            if self.initialized:
                self.process_obstacle_changes([((x, y), True)])
    
    def remove_obstacle(self, x, y):
        """Dinamički ukloni prepreku"""
        if (x, y) in self.obstacles:
            self.obstacles.discard((x, y))
            self.detected_obstacles.discard((x, y))
            self.initial_obstacles.discard((x, y))
            if self.initialized:
                self.process_obstacle_changes([((x, y), False)])



#######################FINALNO
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import time

# Importaj D* Lite algoritam
from .d_star_lite import DStarLite


class DStarLiteNode(Node):
    """
    ROS2 čvor s D* Lite algoritmom za planiranje putanje
    """
    def __init__(self):
        super().__init__('d_star_lite_node')

        self.get_logger().info('Inicijalizacija D* Lite čvora...')

        # Parametri
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/visualization_marker_array')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 12.0)
        self.declare_parameter('goal_y', 6.0)
        self.declare_parameter('discretization', 0.25)  # 0.25m po čvoru

        # Dohvati parametre
        self.map_topic = self.get_parameter('map_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.discretization = float(self.get_parameter('discretization').value)

        # Varijable za mapu
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_frame = 'map'
        self.map_data = None

        # Diskretizovani grid
        self.grid_width = None
        self.grid_height = None
        self.grid_resolution = self.discretization  # 0.25m

        # D* Lite instanca
        self.d_star_lite = None

        # QoS profil za mapu
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscriber za mapu
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile=map_qos
        )

        # QoS profil za markere
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

        self.get_logger().info(f'Čvor pokrenut, čeka mapu na {self.map_topic}')

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback - kada je mapa primljena
        """
        self.get_logger().info('Mapa primljena!')

        # Spremi info o mapi
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

        # Izračunaj diskretizovani grid
        discretization_step = int(self.discretization / self.map_resolution)
        self.grid_width = (self.map_width + discretization_step - 1) // discretization_step
        self.grid_height = (self.map_height + discretization_step - 1) // discretization_step

        self.get_logger().info(
            f'Diskretizacija: {self.discretization}m ({discretization_step} px)'
        )
        self.get_logger().info(
            f'Diskretizovani grid: {self.grid_width} x {self.grid_height}'
        )

        # Izvuci prepreke iz mape
        obstacle_coords = self.extract_obstacles_from_map(discretization_step)
        self.get_logger().info(f'Pronađeno prepreka: {len(obstacle_coords)}')

        # Kreiraj D* Lite instancu
        self.d_star_lite = DStarLite(
            self.grid_width,
            self.grid_height,
            obstacle_coords
        )

        # Konveriraj start i goal u grid koordinate
        start_grid = self.world_to_grid(self.start_x, self.start_y, discretization_step)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, discretization_step)

        self.get_logger().info(f'Start (world): ({self.start_x}, {self.start_y})')
        self.get_logger().info(f'Start (grid): {start_grid}')
        self.get_logger().info(f'Goal (world): ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Goal (grid): {goal_grid}')

        # Provjeri jesu li validne pozicije
        if not self.is_valid_grid_position(start_grid, discretization_step):
            self.get_logger().error('Start je prepreka ili izvan mape!')
            return

        if not self.is_valid_grid_position(goal_grid, discretization_step):
            self.get_logger().error('Goal je prepreka ili izvan mape!')
            return

        # Izvršiti D* Lite pretragu
        self.get_logger().info('Započinjanje D* Lite pretraživanja...')
        start_time = time.time()

        path, examined_nodes = self.d_star_lite.search(start_grid, goal_grid)

        elapsed_time = time.time() - start_time

        self.get_logger().info(
            f'Pretraživanje gotovo za {elapsed_time:.4f} sekundi'
        )
        self.get_logger().info(
            f'Pregledano čvorova: {len(examined_nodes)}'
        )
        self.get_logger().info(f'Duljina putanje: {len(path)} čvorova')

        # Vizualizacija pretraživanja
        self.publish_visualization(
            examined_nodes, path, discretization_step
        )

    def extract_obstacles_from_map(self, discretization_step):
        """
        Izvuci prepreke iz OccupancyGrid mape
        Ako je bilo koja ćelija u 5x5 bloku zauzeta, čitav blok je prepreka
        """
        obstacles = set()

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                # Pronađi odgovarajući blok u originalnoj mapi
                map_y_start = grid_y * discretization_step
                map_x_start = grid_x * discretization_step

                is_obstacle = False

                # Provjeri sve ćelije u bloku
                for my in range(map_y_start, min(map_y_start + discretization_step, self.map_height)):
                    for mx in range(map_x_start, min(map_x_start + discretization_step, self.map_width)):
                        index = my * self.map_width + mx
                        if index < len(self.map_data):
                            occupancy = self.map_data[index]
                            # Ako je bilo šta zauzeto, čitava ćelija je prepreka
                            if occupancy > 0:
                                is_obstacle = True
                                break
                    if is_obstacle:
                        break

                if is_obstacle:
                    obstacles.add((grid_x, grid_y))

        return obstacles

    def world_to_grid(self, world_x, world_y, discretization_step):
        """
        Pretvori svjetske koordinate u diskretizovane grid koordinate
        """
        # Prvo pretvori u pixel koordinate
        pixel_x = int((world_x - self.map_origin_x) / self.map_resolution)
        pixel_y = int((world_y - self.map_origin_y) / self.map_resolution)

        # Osiguraj da su u granicama
        pixel_x = max(0, min(pixel_x, self.map_width - 1))
        pixel_y = max(0, min(pixel_y, self.map_height - 1))

        # Pretvori u diskretizovane grid koordinate
        grid_x = pixel_x // discretization_step
        grid_y = pixel_y // discretization_step

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, discretization_step):
        """
        Pretvori grid koordinate u svjetske koordinate
        Koristi centar diskretizovane ćelije
        """
        # Pretvori u pixel koordinate (centar bloka)
        pixel_x = (grid_x * discretization_step) + (discretization_step // 2)
        pixel_y = (grid_y * discretization_step) + (discretization_step // 2)

        # Pretvori u svjetske koordinate
        world_x = pixel_x * self.map_resolution + self.map_origin_x
        world_y = pixel_y * self.map_resolution + self.map_origin_y

        return (world_x, world_y)

    def is_valid_grid_position(self, grid_pos, discretization_step):
        """
        Provjeri je li grid pozicija valjana
        """
        x, y = grid_pos

        if x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False

        # Provjeri je li prepreka
        if (x, y) in self.d_star_lite.obstacles:
            return False

        return True

    def publish_visualization(self, examined_nodes, path, discretization_step):
        """
        Objavi markere za vizualizaciju pretraživanja
        - Crveni markeri: pregledani čvorovi
        - Zeleni markeri: putanja
        - Plavi marker: start (na stvarnoj world poziciji)
        - Žuti marker: goal (na stvarnoj world poziciji)
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Crveni markeri za pregledane čvorove
        for grid_x, grid_y in examined_nodes:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, discretization_step)

            marker = Marker()
            marker.header = Header(frame_id=self.map_frame)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(x=world_x, y=world_y, z=0.1)
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.discretization * 0.8
            marker.scale.y = self.discretization * 0.8
            marker.scale.z = self.discretization * 0.8

            # Crvena boja
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

            marker_array.markers.append(marker)
            marker_id += 1

        # Zeleni markeri za putanju
        for grid_x, grid_y in path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, discretization_step)

            marker = Marker()
            marker.header = Header(frame_id=self.map_frame)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(x=world_x, y=world_y, z=0.15)
            marker.pose.orientation.w = 1.0

            marker.scale.x = self.discretization * 0.9
            marker.scale.y = self.discretization * 0.9
            marker.scale.z = self.discretization * 0.9

            # Zelena boja
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

            marker_array.markers.append(marker)
            marker_id += 1

        # PLAVI marker za START - koristi STVARNU world poziciju
        start_marker = Marker()
        start_marker.header = Header(frame_id=self.map_frame)
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = marker_id
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        start_marker.pose.position = Point(x=self.start_x, y=self.start_y, z=0.25)
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale.x = self.discretization * 1.2
        start_marker.scale.y = self.discretization * 1.2
        start_marker.scale.z = self.discretization * 1.2
        
        # PLAVA boja
        start_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
        
        marker_array.markers.append(start_marker)
        marker_id += 1

        # ŽUTI marker za GOAL - koristi STVARNU world poziciju
        goal_marker = Marker()
        goal_marker.header = Header(frame_id=self.map_frame)
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = marker_id
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position = Point(x=self.goal_x, y=self.goal_y, z=0.25)
        goal_marker.pose.orientation.w = 1.0
        
        goal_marker.scale.x = self.discretization * 1.2
        goal_marker.scale.y = self.discretization * 1.2
        goal_marker.scale.z = self.discretization * 1.2
        
        # ŽUTA boja
        goal_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        
        marker_array.markers.append(goal_marker)
        marker_id += 1

        # Objavi markere
        self.marker_publisher.publish(marker_array)
        self.get_logger().info(
            f'Objavljeno {len(examined_nodes)} pregledanih čvorova, '
            f'{len(path)} čvorova putanje, start (plavi) i goal (žuti) markeri'
        )


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


if __name__ == '__main__':
    main()


########################S ANIMACIJOM
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math
import time

# Importaj D* Lite algoritam
from .d_star_lite import DStarLite


class DStarLiteNode(Node):
    """
    ROS2 čvor s D* Lite algoritmom za planiranje putanje
    """
    def __init__(self):
        super().__init__('d_star_lite_node')

        self.get_logger().info('Inicijalizacija D* Lite čvora...')

        # Parametri
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/visualization_marker_array')
        self.declare_parameter('start_x', 7.0)
        self.declare_parameter('start_y', -7.0)
        self.declare_parameter('goal_x', 18.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('discretization', 0.25)  # 0.25m po čvoru

        # Dohvati parametre
        self.map_topic = self.get_parameter('map_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.discretization = float(self.get_parameter('discretization').value)

        # Varijable za mapu
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_frame = 'map'
        self.map_data = None

        # Diskretizovani grid
        self.grid_width = None
        self.grid_height = None
        self.grid_resolution = self.discretization  # 0.25m

        # D* Lite instanca
        self.d_star_lite = None

        # QoS profil za mapu
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscriber za mapu
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile=map_qos
        )

        # QoS profil za markere
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

        self.get_logger().info(f'Čvor pokrenut, čeka mapu na {self.map_topic}')

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback - kada je mapa primljena
        """
        self.get_logger().info('Mapa primljena!')

        # Spremi info o mapi
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

        # Izračunaj diskretizovani grid
        discretization_step = int(self.discretization / self.map_resolution)
        self.grid_width = (self.map_width + discretization_step - 1) // discretization_step
        self.grid_height = (self.map_height + discretization_step - 1) // discretization_step

        self.get_logger().info(
            f'Diskretizacija: {self.discretization}m ({discretization_step} px)'
        )
        self.get_logger().info(
            f'Diskretizovani grid: {self.grid_width} x {self.grid_height}'
        )

        # Izvuci prepreke iz mape
        obstacle_coords = self.extract_obstacles_from_map(discretization_step)
        self.get_logger().info(f'Pronađeno prepreka: {len(obstacle_coords)}')

        # Kreiraj D* Lite instancu
        self.d_star_lite = DStarLite(
            self.grid_width,
            self.grid_height,
            obstacle_coords
        )

        # Konveriraj start i goal u grid koordinate
        start_grid = self.world_to_grid(self.start_x, self.start_y, discretization_step)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, discretization_step)

        self.get_logger().info(f'Start (world): ({self.start_x}, {self.start_y})')
        self.get_logger().info(f'Start (grid): {start_grid}')
        self.get_logger().info(f'Goal (world): ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Goal (grid): {goal_grid}')

        # Provjeri jesu li validne pozicije
        if not self.is_valid_grid_position(start_grid, discretization_step):
            self.get_logger().error('Start je prepreka ili izvan mape!')
            return

        if not self.is_valid_grid_position(goal_grid, discretization_step):
            self.get_logger().error('Goal je prepreka ili izvan mape!')
            return

        # Izvršiti D* Lite pretragu
        self.get_logger().info('Započinjanje D* Lite pretraživanja...')
        start_time = time.time()

        path, examined_nodes = self.d_star_lite.search(start_grid, goal_grid)

        elapsed_time = time.time() - start_time

        self.get_logger().info(
            f'Pretraživanje gotovo za {elapsed_time:.4f} sekundi'
        )
        self.get_logger().info(
            f'Pregledano čvorova: {len(examined_nodes)}'
        )
        self.get_logger().info(f'Duljina putanje: {len(path)} čvorova')

        # **NOVI DIO: Provjera jesu li pronađena putanja**
        if len(path) == 0:
            self.get_logger().warn('Putanja nije pronađena - prikazivanje bez animacije!')
            # Prikaz samo start/goal + svi pregledani čvorovi (bez zelene putanje)
            self.publish_visualization(examined_nodes, path, discretization_step, animate=False)
            return

        # Vizualizacija pretraživanja (samo ako postoji putanja)
        self.get_logger().info('Započinjanje animirane vizualizacije...')
        self.publish_visualization(examined_nodes, path, discretization_step)

    def extract_obstacles_from_map(self, discretization_step):
        """
        Izvuci prepreke iz OccupancyGrid mape
        Ako je bilo koja ćelija u 5x5 bloku zauzeta, čitav blok je prepreka
        """
        obstacles = set()

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                # Pronađi odgovarajući blok u originalnoj mapi
                map_y_start = grid_y * discretization_step
                map_x_start = grid_x * discretization_step

                is_obstacle = False

                # Provjeri sve ćelije u bloku
                for my in range(map_y_start, min(map_y_start + discretization_step, self.map_height)):
                    for mx in range(map_x_start, min(map_x_start + discretization_step, self.map_width)):
                        index = my * self.map_width + mx
                        if index < len(self.map_data):
                            occupancy = self.map_data[index]
                            # Ako je bilo šta zauzeto, čitava ćelija je prepreka
                            if occupancy > 0:
                                is_obstacle = True
                                break
                    if is_obstacle:
                        break

                if is_obstacle:
                    obstacles.add((grid_x, grid_y))

        return obstacles

    def world_to_grid(self, world_x, world_y, discretization_step):
        """
        Pretvori svjetske koordinate u diskretizovane grid koordinate
        """
        # Prvo pretvori u pixel koordinate
        pixel_x = int((world_x - self.map_origin_x) / self.map_resolution)
        pixel_y = int((world_y - self.map_origin_y) / self.map_resolution)

        # Osiguraj da su u granicama
        pixel_x = max(0, min(pixel_x, self.map_width - 1))
        pixel_y = max(0, min(pixel_y, self.map_height - 1))

        # Pretvori u diskretizovane grid koordinate
        grid_x = pixel_x // discretization_step
        grid_y = pixel_y // discretization_step

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, discretization_step):
        """
        Pretvori grid koordinate u svjetske koordinate
        Koristi centar diskretizovane ćelije
        """
        # Pretvori u pixel koordinate (centar bloka)
        pixel_x = (grid_x * discretization_step) + (discretization_step // 2)
        pixel_y = (grid_y * discretization_step) + (discretization_step // 2)

        # Pretvori u svjetske koordinate
        world_x = pixel_x * self.map_resolution + self.map_origin_x
        world_y = pixel_y * self.map_resolution + self.map_origin_y

        return (world_x, world_y)

    def is_valid_grid_position(self, grid_pos, discretization_step):
        """
        Provjeri je li grid pozicija valjana
        """
        x, y = grid_pos

        if x < 0 or x >= self.grid_width or y < 0 or y >= self.grid_height:
            return False

        # Provjeri je li prepreka
        if (x, y) in self.d_star_lite.obstacles:
            return False

        return True

    
    def publish_visualization(self, examined_nodes, path, discretization_step,
                          animate=True, delay=0.01):
        """
        Vizualizacija:
        - Tijekom animacije: svi dosad pretraženi čvorovi (crveni) se akumuliraju
        - Start (plavi) i goal (žuti) su stalno vidljivi
        - Na kraju: crveni na putanji se obrišu i dodaju se zeleni markeri putanje
        """
        # 0) Start i goal (statično)
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

        # Goal marker (žuti)
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

        # Bez animacije: sve nacrtaj odjednom (start+goal+crveni+zeleni)
        if not animate:
            final = MarkerArray()
            final.markers.extend(base_array.markers)

            # Crveni – svi pregledani
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

            # Zelena putanja
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

        # 1) ANIMACIJA – akumulativni crveni čvorovi
        # svaki pregledani čvor dobije trajni ID: 20000 + i
        for i, (gx, gy) in enumerate(examined_nodes):
            wx, wy = self.grid_to_world(gx, gy, discretization_step)

            frame = MarkerArray()
            frame.markers.extend(base_array.markers)

            # Dodaj sve crvene do sada (akumulativno)
            for j in range(i + 1):
                pgx, pgy = examined_nodes[j]
                pwx, pwy = self.grid_to_world(pgx, pgy, discretization_step)

                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.id = 20000 + j             # stabilan ID po čvoru
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

        # 2) Nakon animacije: obriši crvene na putanji i dodaj zelene

        # mapiraj grid poziciju -> indeks u examined_nodes (radi ID‑ja)
        index_of = {pos: idx for idx, pos in enumerate(examined_nodes)}

        # a) DELETE crvenih na putanji
        delete_array = MarkerArray()
        for gx, gy in path:
            idx = index_of.get((gx, gy), None)
            if idx is None:
                continue
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 20000 + idx       # isti ID kao crveni
            m.action = Marker.DELETE
            delete_array.markers.append(m)

        self.marker_publisher.publish(delete_array)

        # b) ADD zelene putanje
        green_array = MarkerArray()
        green_array.markers.extend(base_array.markers)  # start + goal ostaju

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

        # c) ponovno dodaj sve crvene (osim onih obrisanih na putanji)
        red_id = 20000
        for idx, (gx, gy) in enumerate(examined_nodes):
            if (gx, gy) in path:
                continue  # ovi su obrisani i zamijenjeni zelenim
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


if __name__ == '__main__':
    main()

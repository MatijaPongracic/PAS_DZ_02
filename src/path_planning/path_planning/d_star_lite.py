import math
import heapq
from collections import defaultdict


class PriorityQueue:   
    def __init__(self):
        self.heap = []  # min-heap (k1, k2, x, y)
        self.positions = {}
    
    def Top(self):
        if self.heap:
            k1, k2, x, y = self.heap[0]
            return (k1, k2, x, y)
        return None
    
    def TopKey(self):
        # Vraća samo ključ najmanjeg elementa - vazno za terminaciju
        if self.heap:
            return (self.heap[0][0], self.heap[0][1])
        return (float('inf'), float('inf'))
    
    def Pop(self):
        if not self.heap:
            return None
        k1, k2, x, y = heapq.heappop(self.heap)
        if (x, y) in self.positions:
            del self.positions[(x, y)]
        return (k1, k2, x, y)
    
    def Insert(self, pos, key):
        if pos not in self.positions:
            entry = (key[0], key[1], pos[0], pos[1])
            heapq.heappush(self.heap, entry)
            self.positions[pos] = True
    
    def Update(self, pos, key):
        if pos in self.positions:
            self.Remove(pos)
        self.Insert(pos, key)
    
    def Remove(self, pos):
        if pos in self.positions:
            del self.positions[pos]
            self.heap = [item for item in self.heap 
                        if not (item[2] == pos[0] and item[3] == pos[1])]
            heapq.heapify(self.heap)
    
    def contains(self, pos):
        return pos in self.positions
    
    def empty(self):
        return len(self.heap) == 0


class DStarLite:
    
    # Mogućnosti gibanja (8 smjerova)
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
        self.width = grid_width
        self.height = grid_height
        
        # Prepreke
        self.initial_obstacles = set(obstacle_coords)
        self.obstacles = set()
        
        # Vrijednosti cvorova
        self.g = {}      # trošak od cvora do cilja
        self.rhs = {}    # one step lookahead trošak
        
        # Prioritetni red
        self.U = PriorityQueue()
        
        # km vrijednost za reprioritetizaciju
        self.km = 0.0
        
        # Start i goal
        self.start = None
        self.goal = None
        self.last = None
        
        # Statistika
        self.examined_nodes = []
        self.detected_obstacles = set()
        self.initialized = False
    
    def is_obstacle(self, x, y):
        return (x, y) in self.obstacles
    
    def is_valid(self, x, y):
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                not self.is_obstacle(x, y))
    
    def get_neighbours(self, x, y):
        neighbours = []
        for dx, dy, cost in self.MOTIONS:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbours.append(((nx, ny), cost))
        return neighbours
    
    def get_predecessors(self, x, y):
        return self.get_neighbours(x, y)
    
    def heuristic(self, pos1, pos2):
        # Chebyshev udaljenost - bolja o Manhatten za 8 smjerova
        x1, y1 = pos1
        x2, y2 = pos2
        return max(abs(x2 - x1), abs(y2 - y1))
    
    def key_less(self, k1, k2):
        if k1[0] < k2[0]:
            return True
        if k1[0] == k2[0] and k1[1] < k2[1]:
            return True
        return False
    
    def calculate_key(self, pos):
        g_val = self.g.get(pos, float('inf'))
        rhs_val = self.rhs.get(pos, float('inf'))
        h_val = self.heuristic(self.start, pos)
        
        k1 = min(g_val, rhs_val) + h_val + self.km
        k2 = min(g_val, rhs_val)
        
        return (k1, k2)
    
    def c(self, pos1, pos2):    # Trosak prijelaza
        if self.is_obstacle(pos2[0], pos2[1]):
            return float('inf')
        
        dx = abs(pos2[0] - pos1[0])
        dy = abs(pos2[1] - pos1[1])
        
        if dx == 1 and dy == 1:
            return math.sqrt(2) # za dijagonalne pomake
        elif dx + dy == 1:
            return 1.0  # za gore dolje lijevo desno
        else:
            return float('inf')
    
    def update_vertex(self, u):
        # Ako nije cilj, izračunaj rhs od susjeda
        if u != self.goal:
            min_val = float('inf')
            for successor, _ in self.get_neighbours(u[0], u[1]):
                cost = self.c(u, successor)
                g_succ = self.g.get(successor, float('inf'))
                val = cost + g_succ
                min_val = min(min_val, val)
            self.rhs[u] = min_val
        
        # Ukloni iz prioritetnog reda ako je tamo
        if self.U.contains(u):
            self.U.Remove(u)
        
        # Ako je nekonzistentan, dodaj u red
        g_val = self.g.get(u, float('inf'))
        rhs_val = self.rhs.get(u, float('inf'))
        if g_val != rhs_val:
            key = self.calculate_key(u)
            self.U.Insert(u, key)
    
    def compute_shortest_path(self):
        iterations = 0
        max_iterations = self.width * self.height * 100
        
        while iterations < max_iterations:
            iterations += 1
            
            # Uvjet zaustavljanja
            top_key = self.U.TopKey()
            start_key = self.calculate_key(self.start)
            start_g = self.g.get(self.start, float('inf'))
            start_rhs = self.rhs.get(self.start, float('inf'))
            
            # Ako je red prazan ili je start konzistentan i ima dobar prioritet
            if self.U.empty() or (top_key >= start_key and start_g == start_rhs):
                break
            
            u_entry = self.U.Pop()
            if u_entry is None:
                break
            
            kold = (u_entry[0], u_entry[1])
            u_pos = (u_entry[2], u_entry[3])
            
            # Izračunaj novi ključ za ovaj vertex
            knew = self.calculate_key(u_pos)
            
            # Ako je ključ promijenjen, ponovno ga dodaj u U (heap reordering)
            if self.key_less(kold, knew):
                self.U.Insert(u_pos, knew)
                continue
            
            if u_pos not in self.examined_nodes:
                self.examined_nodes.append(u_pos)
            
            g_val = self.g.get(u_pos, float('inf'))
            rhs_val = self.rhs.get(u_pos, float('inf'))
            
            if g_val > rhs_val:
                # Overconsistent: g = rhs
                self.g[u_pos] = rhs_val
                
                # Azuriraj sve prethodnike
                for pred, _ in self.get_predecessors(u_pos[0], u_pos[1]):
                    self.update_vertex(pred)
            else:
                # Underconsistent: g = inf
                self.g[u_pos] = float('inf')
                
                # Azuriraj vertex i sve prethodike
                self.update_vertex(u_pos)
                for pred, _ in self.get_predecessors(u_pos[0], u_pos[1]):
                    self.update_vertex(pred)
        
        return self.g.get(self.start, float('inf')) != float('inf')
    
    def extract_path(self):
        if self.g.get(self.start, float('inf')) == float('inf'):
            return []
        
        path = [self.start]
        current = self.start
        max_steps = self.width * self.height
        steps = 0
        
        while current != self.goal and steps < max_steps:
            steps += 1
            
            best_successor = None
            best_val = float('inf')
            
            for successor, _ in self.get_neighbours(current[0], current[1]):
                g_succ = self.g.get(successor, float('inf'))
                cost = self.c(current, successor)
                
                if g_succ < float('inf') and cost < float('inf'):
                    val = cost + g_succ
                    if val < best_val:
                        best_val = val
                        best_successor = successor
            
            if best_successor is None:
                return []  # Nema putanje
            
            if best_successor == current:
                return []  # Beskonacna petlja
            
            path.append(best_successor)
            current = best_successor
        
        if current != self.goal:
            return []
        
        return path
    
    def initialize(self):
        self.U = PriorityQueue()
        self.g = {}
        self.rhs = {}
        self.km = 0.0
        self.examined_nodes = []
        self.obstacles = set(self.initial_obstacles)
        
        self.rhs[self.goal] = 0
        self.g[self.goal] = float('inf')
        
        key = self.calculate_key(self.goal)
        self.U.Insert(self.goal, key)
        
        self.initialized = True
    
    def search(self, start, goal):
        self.start = start
        self.goal = goal
        self.last = start
        
        self.initialize()
        
        success = self.compute_shortest_path()
        
        if success:
            path = self.extract_path()
        else:
            path = []
        
        return path, self.examined_nodes
    
    def scan_for_obstacles(self, pos):
        new_obstacles = []
        
        for neighbour, _ in self.get_neighbours(pos[0], pos[1]):
            if neighbour in self.initial_obstacles and neighbour not in self.obstacles:
                new_obstacles.append(neighbour)
                self.obstacles.add(neighbour)
                self.detected_obstacles.add(neighbour)
        
        return new_obstacles
    
    def process_obstacle_changes(self, changed_vertices):
        if not changed_vertices:
            return
        
        if self.last is not None:
            self.km += self.heuristic(self.last, self.start)
        self.last = self.start
        
        for pos, added in changed_vertices:
            if added:
                self.g[pos] = float('inf')
                self.rhs[pos] = float('inf')
            
            for neighbour, _ in self.get_neighbours(pos[0], pos[1]):
                self.update_vertex(neighbour)
        
        # Replanning
        self.compute_shortest_path()
    
    def search_dynamic(self, start, goal):
        self.start = start
        self.goal = goal
        self.last = start
        
        self.initialize()
        self.compute_shortest_path()
        
        path_taken = [start]
        current = start
        
        max_steps = self.width * self.height * 2
        step = 0
        
        while current != goal and step < max_steps:
            step += 1
            
            if self.g.get(current, float('inf')) == float('inf'):
                return False, path_taken, self.examined_nodes
            
            new_obstacles = self.scan_for_obstacles(current)
            if new_obstacles:
                changed_vertices = [(obs, True) for obs in new_obstacles]
                self.process_obstacle_changes(changed_vertices)
                
                if self.g.get(current, float('inf')) == float('inf'):
                    return False, path_taken, self.examined_nodes
            
            best_successor = None
            best_val = float('inf')
            
            for successor, _ in self.get_neighbours(current[0], current[1]):
                g_succ = self.g.get(successor, float('inf'))
                cost = self.c(current, successor)
                if g_succ < float('inf') and cost < float('inf'):
                    val = cost + g_succ
                    if val < best_val:
                        best_val = val
                        best_successor = successor
            
            if best_successor is None:
                return False, path_taken, self.examined_nodes
            
            current = best_successor
            path_taken.append(current)
            self.start = current
        
        success = (current == goal)
        return success, path_taken, self.examined_nodes

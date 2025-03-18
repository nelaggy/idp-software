from .graph import Graph

class Navigator:
    def __init__(self):
        self.node = 1
        self.graph = Graph()
        self.direction = 3
        self.next_node = 1
        self.destination = 1
        self.cnt = 1
        self.path = None

    def set_destination(self, destination: int):
        self.cnt = 1
        self.destination = destination
        paths = self.graph.paths.get(self.node)
        self.path = paths.get(self.destination) if paths is not None else None
        if self.path is None:
            paths = self.graph.paths.get(self.destination)
            self.path = paths.get(self.node) if paths is not None else None
            if self.path is not None:
                print(self.path)
                self.path.reverse()
        if self.path is None:
            raise ValueError(f"Path from {self.node} to {self.destination} not found.")
        self.next_node = self.path[self.cnt]

    def change_direction(self, turn: int):
        self.direction = (self.direction + turn) % 4

    def get_turn(self):
        # 0 - straight ahead, 1 - left, 2 - back, 3 - right, -1 - destination reached
        if self.path is None:
            raise ValueError("Destination not set.")
        self.cnt += 1
        self.node = self.next_node
        self.next_node = self.path[self.cnt] if self.cnt < len(self.path) else self.destination
        if self.node == self.destination:
            return -1
        turn_direction = (self.direction - self.graph.edges[self.node][self.next_node][0]) % 4
        self.direction = self.graph.edges[self.node][self.next_node][0]
        
        return turn_direction
    
    def reset(self):
        self.node = 1
        self.direction = 3
        self.next_node = 1
        self.destination = 1
        self.cnt = 1
        self.path = None

    
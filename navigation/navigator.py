from .graph import Graph

class Navigator:
    def __init__(self):
        self.node = 1
        self.graph = Graph()
        self.direction = 3
        self.next_node = 5
        self.destination = 5
        self.cnt = 0
        self.path = None

    def set_destination(self, destination: int):
        self.cnt = 0
        self.destination = destination
        self.path = self.graph.paths.get((self.node, self.destination))
        if self.path is None:
            raise ValueError(f"Path from {self.node} to {self.destination} not found.")
        self.next_node = self.path[self.cnt]

    def change_direction(self, turn: int):
        self.direction = (self.direction + turn) % 4

    def get_turn(self):
        # 0 - straight ahead, 1 - left, 2 - back, 3 - right, -1 - destination reached
        if self.path is None:
            raise ValueError("Destination not set.")
        if self.next_node == self.destination:
            return -1
        turn_direction = (self.direction - self.graph.edges[self.node][self.next_node][0]) % 4
        self.direction = self.graph.edges[self.node][self.next_node][0]
        self.cnt += 1
        self.next_node = self.path[self.cnt]
        
        return turn_direction

    
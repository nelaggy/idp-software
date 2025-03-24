from .graph import Graph

class Navigator:
    """
    Navigator class for navigation of the robot.
    """
    def __init__(self):
        self.node = 1 # current node
        self.graph = Graph()
        self.direction = 3 # 0 - straight ahead, 1 - left, 2 - back, 3 - right
        self.next_node = 1 # next node
        self.destination = 1 # destination node
        self.cnt = 1 # counter for path
        self.path = None # path from current node to destination node

    def set_destination(self, destination: int):
        """
        Sets the destination and finds path for the robot to navigate to.

        Arguments:
            self
            destination (int): The destination node.
        """
        self.cnt = 1
        self.destination = destination

        # Try to get direct path from current node
        paths = self.graph.paths.get(self.node)
        self.path = paths.get(self.destination) if paths is not None else None
        
        # If no direct path exists, check for reverse path
        if self.path is None:
            paths = self.graph.paths.get(self.destination)
            self.path = paths.get(self.node) if paths is not None else None
            if self.path is not None:
                self.path.reverse()

        # if no path found, raise error
        if self.path is None:
            raise ValueError(f"Path from {self.node} to {self.destination} not found.")
        self.next_node = self.path[self.cnt]

    def change_direction(self, turn: int):
        """
        Changes robot's direction based on the turn

        Arguments:
            turn (int): turn direction
        """
        self.direction = (self.direction + turn) % 4

    def get_turn(self):
        """
        Determines next turn required to navigate towards next node
        Arguments:
            self

        Returns:
            turn direction
        """
        # 0 - straight ahead, 1 - left, 2 - back, 3 - right, -1 - destination reached
        if self.path is None:
            raise ValueError("Destination not set.")
        self.cnt += 1
        self.node = self.next_node
        self.next_node = self.path[self.cnt] if self.cnt < len(self.path) else self.destination
        
        # return -1 if arrived at destination
        if self.node == self.destination:
            return -1
        turn_direction = (self.direction - self.graph.edges[self.node][self.next_node][0]) % 4
        self.direction = self.graph.edges[self.node][self.next_node][0]
        
        return turn_direction
    
    def reset(self):
        """
        Resets Navigator class
        """
        self.node = 1
        self.direction = 3
        self.next_node = 1
        self.destination = 1
        self.cnt = 1
        self.path = None

    
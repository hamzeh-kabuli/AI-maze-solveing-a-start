import tkinter as tk
import heapq
import time
######################################################
# Heuristic Search Problem                           #
# CS7050 Coursework 2023, Prof: Mohamed Ghanem       #
# Student: 22068321, Hamzeh Kabuli                   #
######################################################

# The Node class represents each cell in the maze grid
class Node:
    def __init__(self, x, y, walkable):
        # x, y: Coordinates of the node on the grid
        # walkable: Boolean indicating if the node is traversable
        self.x = x
        self.y = y
        self.walkable = walkable
        # g_cost: Cost from the start node to this node
        self.g_cost = float('inf')
        # h_cost: Heuristic cost from this node to the end node
        self.h_cost = 0
        # f_cost: Total cost (g_cost + h_cost)
        self.f_cost = float('inf')
        # Parent node for reconstructing the path later
        self.parent = None

    # A function to calculate the heuristic based on Manhattan distance
    def calculate_h_cost(self, end_node):
        self.h_cost = abs(self.x - end_node.x) + abs(self.y - end_node.y)
        self.calculate_f_cost()

    # A function to calculate the total cost combining g_cost and h_cost
    def calculate_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost

    # Comparison function for priority queue based on f_cost
    def __lt__(self, other):
        return self.f_cost < other.f_cost

###########################################################################   
# Function to get the walkable neighboring nodes of a given node
def get_neighbors(node, maze_nodes):
    # Potential movements (up, down, left, right)
    directions = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    neighbors = []
    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        # Check if the neighbor is within grid bounds and is walkable (no obstacle)
        if 0 <= x < len(maze_nodes[0]) and 0 <= y < len(maze_nodes) and maze_nodes[y][x].walkable:
            neighbors.append(maze_nodes[y][x])
    return neighbors

###########################################################################
# Function to backtrack from the end node to the start node to find the path
def trace_path(end_node):
    path = []
    current = end_node
    # Follow the chain of parent nodes until the start node
    while current:
        path.append(current)
        current = current.parent
    # The path is built from end to start, so we reverse it
    path.reverse()
    return path

###########################################################################
# The A* search algorithm
def a_star_search(start_node, end_node, update_func, maze_nodes):
    open_list = []
    # Start with the start node in the open list
    heapq.heappush(open_list, (start_node.f_cost, start_node))
    # Nodes already evaluated and not needing to be checked again
    visited_set = set()

    # The cost from the start node to itself is zero
    start_node.g_cost = 0
    # Calculate the heuristic for the start node
    start_node.calculate_h_cost(end_node)


    # Main A* search loop
    while open_list:
        # Pop the node with the lowest f_cost off the open list
        current_node = heapq.heappop(open_list)[1]
        # Visually update the current node (will be marked gray)
        update_func(current_node, "gray")


        # If the end node is reached, reconstruct and return the path
        if current_node == end_node:
            return trace_path(end_node)

        # Add the current node to the closed set
        visited_set.add(current_node)

        # Explore the neighbors of the current node
        for neighbor in get_neighbors(current_node, maze_nodes):
            # Skip neighbors that are already evaluated or not walkable
            if neighbor in visited_set or not neighbor.walkable:
                continue

            # The cost from start to a neighbor is one more than the cost to the current node
            tentative_g_cost = current_node.g_cost + 1
            # If this path to the neighbor is better than any previous one, record it
            if tentative_g_cost < neighbor.g_cost:
                neighbor.parent = current_node
                neighbor.g_cost = tentative_g_cost
                neighbor.calculate_h_cost(end_node)
                update_func(neighbor, "white")
                # If the neighbor is not in the open list, add it
                if neighbor not in open_list:
                    heapq.heappush(open_list, (neighbor.f_cost, neighbor))
    # If the open list is empty but the end node was never reached, no path exists
    return None

###########################################################################
# Function to draw the maze on a Tkinter canvas
def draw_maze(maze_nodes, canvas, cell_width, cell_height):
    for row in maze_nodes:
        for node in row:
            # Determine the rectangle coordinates for each node
            x1 = node.x * cell_width
            y1 = node.y * cell_height
            x2 = x1 + cell_width
            y2 = y1 + cell_height
            # Draw obstacles in red, other nodes in white
            color = 'red' if not node.walkable else 'blue' if (node.x == 0 and node.y == 0) or (node.x == 5 and node.y == 4) else 'white'
            # Draw the cell on the canvas
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='black')
            # Draw the coordinates in each cell
            canvas.create_text((x1 + x2) / 2, y1 + cell_height * 0.2 , text=f'({node.x},{node.y})')

###########################################################################
# The main function where the GUI is created and the A* algorithm is run
def main():
    root = tk.Tk()
    root.title('Solving Maze with A* Search')

    # Dimensions for each cell in the maze
    rows, cols = 5, 6
    cell_width, cell_height = 150, 150
    # Tkinter canvas setup
    canvas = tk.Canvas(root, width=cols*cell_width, height=rows*cell_height)
    canvas.pack()

    # Label for displaying the shortest path
    path_label = tk.Label(root, text="Shortest Path: ")
    path_label.pack()

    # Create the maze grid with Node objects, mark obstacles as not walkable
    maze_nodes = [[Node(j, i, True) for j in range(cols)] for i in range(rows)]
    obstacles = {(0, 1), (2, 1), (2, 3), (3, 1), (3, 4), (4, 4)}
    for obstacle in obstacles:
        maze_nodes[obstacle[0]][obstacle[1]].walkable = False

    # Visualize the initial maze
    draw_maze(maze_nodes, canvas, cell_width, cell_height)

    # Define the start and end nodes for the A* search
    start_node = maze_nodes[0][0]
    end_node = maze_nodes[rows - 1][cols - 1]

    # Function to update the canvas and path label as nodes are processed
    def update_canvas(node, color):
        # Draw the node with the specified color on the canvas
        x1 = node.x * cell_width
        y1 = node.y * cell_height
        x2 = x1 + cell_width
        y2 = y1 + cell_height
        canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='black')
        # Display the node's info
        canvas.create_text((x1 + x2) / 2, y1 + cell_height * 0.2 , text=f'({node.x},{node.y})')
        canvas.create_text((x1 + x2) / 2, y1 + cell_height * 0.5, text=f'G:{node.g_cost}')
        canvas.create_text((x1 + x2) / 2, y1 + cell_height * 0.65, text=f'H:{node.h_cost}')
        canvas.create_text((x1 + x2) / 2, y1 + cell_height * 0.8, text=f'F:{node.f_cost}')
        canvas.update()

        # Only update the path label if the node is part of the path (has a parent)
        if node.parent:
            # Construct the path from the start node to the current node
            current_path = trace_path(node)
            # Update the path label with the nodes' coordinates
            path_label.config(text=f"Shortest Path: {[(n.x, n.y) for n in current_path]}")
            # Force the label to update immediately
            root.update_idletasks()

    # Execute the A* search algorithm
    final_path = a_star_search(start_node, end_node, update_canvas, maze_nodes)

    # Once the path is found, redraw the final path in green
    if final_path:
        for node in final_path:
            update_canvas(node, "green")
            # Short delay to visualize the final path drawing
            time.sleep(0.1)

    # Start the Tkinter event loop
    root.mainloop()
if __name__ == "__main__":
    main()
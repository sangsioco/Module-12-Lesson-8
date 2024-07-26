import heapq

# Task 1: Define graph representation
'''class Graph: __init__ initialize empty vertices dictionary'''
'''add_vertices: add vertices in the graph if it doesn't alreafy exist'''
'''add_edge: add edge between two vertices'''
'''get neighbor: returns given neighbors of a given vertices with edge'''

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertice(self, vertice):
        if vertice not in self.vertices:
            self.vertices[vertice] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}
# Task 2: Dijkstra's Algorithm

def dijkstra(graph, start):
    distances = {vertex: float('infinity') for vertex in graph.vertices}
    distances[start] = 0
    priority_queue = [(0, start)]
    shortest_path_tree = {}

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                shortest_path_tree[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, shortest_path_tree

# Task 3: Testing
if __name__ == "__main__":
    graph = Graph()
    graph.add_vertice('A')
    graph.add_vertice('B')
    graph.add_vertice('C')
    graph.add_vertice('D')
    graph.add_edge('A', 'B', 5)
    graph.add_edge('B', 'C', 3)
    graph.add_edge('A', 'C', 10)
    graph.add_edge('A', 'D', 7)
    graph.add_edge('D', 'C', 2)

    distances, shortest_path_tree = dijkstra(graph, 'A')
    print("Shortest distances:", distances)
    print("Shortest path tree:", shortest_path_tree)

# Task 4: Analyze
'''Time complexity: O(n + m log m) --> n = number of edges and m = number of vertices'''
'''Space complexity: O(m + n) '''
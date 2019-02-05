from collections import deque, namedtuple


#Global variables, definition of infinite and arc
Path = namedtuple('Path', 'start, end, cost')
inf = float('inf')

def make_path(start, end, cost=1):
  return Path(start, end, cost)

def create_graph_from_file(filename, vertex_filename):
    graph = Graph([], 0)
    file = open(filename, 'r')
    while True:
        line = file.readline()
        if line == 'EOF':
            break
        args = line.split()
        graph.add_edge(int(args[0]), int(args[1]), int(args[2]), False)
    file_vertex = open(vertex_filename, 'r')
    graph.no_vertex = int(file_vertex.readline())
    return graph

class Graph:
    def __init__(self, edges, no_vertex):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_path(*edge) for edge in edges]
        self.no_vertex = no_vertex

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Arc {} {} already exists'.format(n1, n2))

        self.edges.append(Path(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Path(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        prev_vert = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            cv = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(cv)
            if distances[cv] == inf:
                break
            for neighbour, cost in self.neighbours[cv]:
                alt_route = distances[cv] + cost
                if alt_route < distances[neighbour]:
                    distances[neighbour] = alt_route
                    prev_vert[neighbour] = cv

        path, cv = deque(), dest
        while prev_vert[cv] is not None:
            path.appendleft((cv, distances[cv]))
            cv = prev_vert[cv]
        if path:
            path.appendleft((cv, distances[cv]))
        return path


'''
    def dijkstra_dial(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        max_weight = 10

        Dist = namedtuple('Dist', 'first second')
        # Initialize all distances as infinite (inf)
        dist = list(Dist(first=inf, second=0))

        B = [list()]*(self.no_vertex*max_weight+1)
        B[0].append(source)
        dist[source].first = 0

        idx = 0
        while True:
            while len(B[idx]) == 0 and idx < max_weight*self.no_vertex:
                idx = idx + 1
            if idx == max_weight*self.no_vertex:
                break

            u = B[idx].pop()
            for i in self.edges:
                v = i.first
                weight = i.cost

                du = dist[u].first
                dv = dist[v].cost

                if dv > du + weight:
                    if dv != inf:
                        B[dv].remove(dist[v].second)
                    dist[v].first = du + weight
                    dv = dist[v].first

                    B[dv].push_front(v)

                    dist[v].second = B[dv].begin()
'''

node = input("Ingrese el nodo fuente\n")
graph = create_graph_from_file('arcos.txt', 'nodos.txt')
for i in range(1, graph.no_vertex):
    f = open("salida_"+str(i)+".txt", "w")
    path = graph.dijkstra(int(node), i)
    f.write(str(i)+"\n")
    for line in path:
        f.write(str(line[0])+" "+str(line[1])+"\n")



import java.util.List;
public class Main {
    public static void main(String[] args) {
        Vertex<String> vertex1 = new Vertex<>("K");
        Vertex<String> vertex2 = new Vertex<>("L");
        Vertex<String> vertex3 = new Vertex<>("M");
        Vertex<String> vertex4 = new Vertex<>("N");
        Vertex<String> vertex5 = new Vertex<>("O");

        // Create weighted graph
        WeightedGraph<String> graph = new WeightedGraph<>();
        graph.addVertex(vertex1);
        graph.addVertex(vertex2);
        graph.addVertex(vertex3);
        graph.addVertex(vertex4);
        graph.addVertex(vertex5);

        // Add edges
        graph.addEdge(vertex1, vertex2, 1.0);
        graph.addEdge(vertex1, vertex3, 2.0);
        graph.addEdge(vertex2, vertex4, 3.0);
        graph.addEdge(vertex3, vertex4, 4.0);
        graph.addEdge(vertex3, vertex5, 5.0);
        graph.addEdge(vertex4, vertex5, 6.0);

        // Perform breadth-first search
        Search<String> BFS = new BreadthFirstSearch<>(graph);
        List<String> BFSPath = BFS.findPath(vertex1, vertex4);
        System.out.println("Breadth-First Search Path: " + BFSPath);
        ((BreadthFirstSearch<String>) BFS).printBFS(vertex1);

        // Perform Dijkstra's algorithm
        Search<String> dijkstra = new DijkstraSearch<>(graph);
        List<String> dijkstraPath = dijkstra.findPath(vertex1, vertex4);
        System.out.println("Dijkstra's Algorithm Path: " + dijkstraPath);
        ((DijkstraSearch<String>) dijkstra).printDijkstra(vertex1);

        // Test other methods
        System.out.println("Vertex K Degree: " + vertex1.getDegree());
        System.out.println("Vertex L Degree: " + vertex2.getDegree());
        System.out.println("Vertex M Degree: " + vertex3.getDegree());
        System.out.println("Vertex N Degree: " + vertex4.getDegree());
        System.out.println("Vertex O Degree: " + vertex5.getDegree());

        System.out.println("Edges in the Graph:");
        for (WeightedGraph<String>.Edge<String> edge : graph.getEdges()) {
            System.out.println("Source: " + edge.getSource().getData());
            System.out.println("Destination: " + edge.getDestination().getData());
            System.out.println("Weight: " + edge.getWeight());
            System.out.println();
        }

        System.out.println("Graph Information:");
        graph.graphInfo();
    }
}
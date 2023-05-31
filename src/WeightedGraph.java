import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WeightedGraph<V> {
    private Map<Vertex<V>, List<Edge<V>>> list;

    public WeightedGraph() {
        list = new HashMap<>();
    }
    public class Edge<V> {
        private Vertex<V> source;
        private Vertex<V> destination;
        private double weight;
        public Edge(Vertex<V> source, Vertex<V> destination, double weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
        }
        public Vertex<V> getSource() {
            return source;
        }
        public Vertex<V> getDestination() {
            return destination;
        }
        public double getWeight() {
            return weight;
        }
    }
    public void addVertex(Vertex<V> vertex) {
        list.put(vertex, new ArrayList<>());
    }
    public void validate(Vertex<V> vertex) {
        if (!list.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex " + vertex + " is out of the range");
        }
    }
    public void addEdge(Vertex<V> source, Vertex<V> destination, double weight) {
        validate(source);
        validate(destination);
        List<Edge<V>> edges = list.get(source);
        edges.add(new Edge<V>(source, destination, weight));
        list.put(source, edges);
    }
    public List<Vertex<V>> getVertices() {
        return new ArrayList<>(list.keySet());
    }
    public List<Edge<V>> getEdges() {
        List<Edge<V>> allEdges = new ArrayList<>();
        for (List<Edge<V>> edge : list.values()) {
            allEdges.addAll(edge);
        }
        return allEdges;
    }
    public List<Edge<V>> getEdge(Vertex<V> vertex) {
        validate(vertex);
        return list.get(vertex);
    }
    public void graphInfo() {
        // Print vertices
        System.out.println("Vertices:");
        for (Vertex<V> vertex : list.keySet()) {
            System.out.println(vertex.getData());
        }
        System.out.println();

        // Print edges with weights
        System.out.println("Edges with Weights:");
        for (List<Edge<V>> edges : list.values()) {
            for (Edge<V> edge : edges) {
                System.out.println("Source: " + edge.getSource().getData());
                System.out.println("Destination: " + edge.getDestination().getData());
                System.out.println("Weight: " + edge.getWeight());
                System.out.println();
            }
        }
    }
}

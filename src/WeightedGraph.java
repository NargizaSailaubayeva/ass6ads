import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
/**
 * WeightedGraph class represents a weighted graph data structure.
 * @param <V> the type of data stored in the vertices of the graph.
 */
public class WeightedGraph<V> {
    private Map<Vertex<V>, List<Edge<V>>> list;
    /**
     *  Constructs a new WeightedGraph object.
     */
    public WeightedGraph() {
        list = new HashMap<>();
    }
    /**
     *  Edge class represents an edge in the weighted graph.
     *  @param <V> the type of data stored in the vertices of the graph.
     */
    public class Edge<V> {
        private Vertex<V> source;
        private Vertex<V> destination;
        private double weight;
        /**
         *  Constructs a new Edge object.
         *  @param source the source vertex of the edge.
         *  @param destination the destination vertex of the edge.
         *  @param weight the weight of the edge.
         */
        public Edge(Vertex<V> source, Vertex<V> destination, double weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
        }
        /**
         *  Returns the source vertex of the edge.
         *  @return the source vertex.
         */
        public Vertex<V> getSource() {
            return source;
        }
        /**
         * Returns the destination vertex of the edge.
         * @return the destination vertex.
         */
        public Vertex<V> getDestination() {
            return destination;
        }
        /**
         *  Returns the weight of the edge.
         *  @return the weight of the edge.
         */
        public double getWeight() {
            return weight;
        }
    }
    /**
     *  Adds a vertex to the graph.
     *  @param vertex the vertex to be added.
     */
    public void addVertex(Vertex<V> vertex) {
        list.put(vertex, new ArrayList<>());
    }
    /**
     *  Validates if the given vertex is present in the graph.
     *  @param vertex the vertex to be validated.
     *  @throws IllegalArgumentException if the vertex is not present in the graph.
     */
    public void validate(Vertex<V> vertex) {
        if (!list.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex " + vertex + " is out of the range");
        }
    }
    /**
     *  Adds an edge between the source and destination vertices with the given weight.
     *  @param source the source vertex of the edge.
     *  @param destination the destination vertex of the edge.
     *  @param weight the weight of the edge.
     */
    public void addEdge(Vertex<V> source, Vertex<V> destination, double weight) {
        validate(source);
        validate(destination);
        List<Edge<V>> edges = list.get(source);
        edges.add(new Edge<V>(source, destination, weight));
        list.put(source, edges);
    }
    /**
     *  Returns a list of all vertices in the graph.
     *  @return a list of all vertices.
     */
    public List<Vertex<V>> getVertices() {
        return new ArrayList<>(list.keySet());
    }
    /**
     *  Returns a list of all edges in the graph.
     *  @return a list of all edges.
     */
    public List<Edge<V>> getEdges() {
        List<Edge<V>> allEdges = new ArrayList<>();
        for (List<Edge<V>> edge : list.values()) {
            allEdges.addAll(edge);
        }
        return allEdges;
    }
    /**
     * Returns a list of edges connected to the specified vertex.
     * @param vertex the vertex to get the edges from.
     * @return a list of edges connected to the vertex.
     */
    public List<Edge<V>> getEdge(Vertex<V> vertex) {
        validate(vertex);
        return list.get(vertex);
    }
    /**
     *  Prints the information about the graph, including vertices and edges with weights.
    */
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

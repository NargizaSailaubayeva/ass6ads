import java.util.HashMap;
import java.util.Map;
/**
 * Vertex class represents a vertex in a graph.
 * @param <V> the type of data stored in the vertex.
 */
public class Vertex <V> {
    private V data;
    private Map<Vertex<V>, Double> adjVertex;
    /**
     *  Constructs a new Vertex object with the given data.
     *  @param data the data to be stored in the vertex.
     */
    public Vertex(V data) {
        this.data = data;
        this.adjVertex = new HashMap<>();
    }
    /**
     *  Returns the data stored in the vertex.
     *  @return the data stored in the vertex.
     */
    public V getData() {
        return data;
    }
    /**
     *  Adds an adjacent vertex with the specified weight to the current vertex.
     *  @param destination the adjacent vertex to be added.
     *  @param weight the weight of the edge connecting the current vertex and the adjacent vertex.
     */
    public void addAdjacentVertex(Vertex<V> destination, double weight) {
        adjVertex.put(destination, weight);
    }
    /**
     * Returns a map of all adjacent vertices and their corresponding weights.
     * @return a map of adjacent vertices and weights.
     */
    public Map<Vertex<V>, Double> getAdjacentVertices() {
        return adjVertex;
    }
    /**
     * Validates if the given vertex is adjacent to the current vertex.
     * @param vertex the vertex to be validated.
     * @throws IllegalArgumentException if the vertex is not adjacent to the current vertex.
     */
    private void validateVertex(Vertex<V> vertex) {
        if (!adjVertex.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex " + vertex + " is out of the range");
        }
    }
    /**
     *  Removes the specified adjacent vertex from the current vertex.
     *  @param vertex the adjacent vertex to be removed.
     */
    public void removeAdjacentVertex(Vertex<V> vertex) {
        validateVertex(vertex);
        adjVertex.remove(vertex);
    }
    /**
     *  Returns the weight of the edge between the current vertex and the specified adjacent vertex.
     *  @param vertex the adjacent vertex.
     *  @return the weight of the edge.
     */
    public double getWeight(Vertex<V> vertex) {
        validateVertex(vertex);
        return adjVertex.get(vertex);
    }
    /**
     *  Checks if the current vertex has an adjacent vertex with the specified vertex.
     *  @param vertex the vertex to check.
     *  @return true if the current vertex has an adjacent vertex with the specified vertex, false otherwise.
     */
    public boolean containsAdjacentVertex(Vertex<V> vertex) {
        return adjVertex.containsKey(vertex);
    }
    /**
     *  Clears all adjacent vertices of the current vertex.
     */
    public void clearAdjacentVertices() {
        adjVertex.clear();
    }
    /**
     *  Returns the degree of the current vertex, which is the number of adjacent vertices.
     *  @return the degree of the vertex.
     */
    public int getDegree() {
        return adjVertex.size();
    }
}

import java.util.HashMap;
import java.util.Map;
public class Vertex <V> {
    private V data;
    private Map<Vertex<V>, Double> adjVertex;

    public Vertex(V data) {
        this.data = data;
        this.adjVertex = new HashMap<>();
    }
    public V getData() {
        return data;
    }
    public void addAdjacentVertex(Vertex<V> destination, double weight) {
        adjVertex.put(destination, weight);
    }
    public Map<Vertex<V>, Double> getAdjacentVertices() {
        return adjVertex;
    }
    private void validateVertex(Vertex<V> vertex) {
        if (!adjVertex.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex " + vertex + " is out of the range");
        }
    }
    public void removeAdjacentVertex(Vertex<V> vertex) {
        validateVertex(vertex);
        adjVertex.remove(vertex);
    }
    public double getWeight(Vertex<V> vertex) {
        validateVertex(vertex);
        return adjVertex.get(vertex);
    }
    public boolean containsAdjacentVertex(Vertex<V> vertex) {
        return adjVertex.containsKey(vertex);
    }
}

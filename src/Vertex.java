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
}

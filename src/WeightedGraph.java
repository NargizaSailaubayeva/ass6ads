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

}

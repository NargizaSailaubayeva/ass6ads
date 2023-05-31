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
    }
}

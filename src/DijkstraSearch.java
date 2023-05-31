import java.util.*;

public class DijkstraSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    public DijkstraSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        return null;
    }
}

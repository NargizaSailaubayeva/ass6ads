import java.util.List;

public class BreadthFirstSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    public BreadthFirstSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        return null;
    }
}

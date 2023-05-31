import java.util.*;

public class DijkstraSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    public DijkstraSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }
    private static class DijkstraNode<V> implements Comparable<DijkstraNode<V>> {
        private Vertex<V> vertex;
        private double distance;

        public DijkstraNode(Vertex<V> vertex, double distance) {
            this.vertex = vertex;
            this.distance = distance;
        }

        public Vertex<V> getVertex() {
            return vertex;
        }

        public double getDistance() {
            return distance;
        }

        @Override
        public int compareTo(DijkstraNode<V> other) {
            return Double.compare(distance, other.distance);
        }
    }
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        return null;
    }
}

import java.util.*;

public class BreadthFirstSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    public BreadthFirstSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }
    public List<Vertex<V>> breadthFirstSearch(Vertex<V> startVertex) {
        List<Vertex<V>> visited = new ArrayList<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        visited.add(startVertex);
        queue.add(startVertex);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);

            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                if (!visited.contains(neighborVertex)) {
                    visited.add(neighborVertex);
                    queue.add(neighborVertex);
                }
            }
        }

        return visited;
    }
    public void printBFS(Vertex<V> startVertex) {
        Set<Vertex<V>> visited = new HashSet<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        visited.add(startVertex);
        queue.offer(startVertex);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            System.out.print(currentVertex.getData() + " ");

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> adjacentVertex = edge.getDestination();
                if (!visited.contains(adjacentVertex)) {
                    visited.add(adjacentVertex);
                    queue.offer(adjacentVertex);
                }
            }
        }

        System.out.println();
    }
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        return null;
    }

}

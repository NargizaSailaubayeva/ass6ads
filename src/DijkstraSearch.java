import java.util.*;

public class DijkstraSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    /**
     * Constructs a DijkstraSearch object with the specified weighted graph.
     * @param graph the weighted graph to perform Dijkstra's algorithm on.
     */
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
    /**
     * Performs Dijkstra's algorithm starting from the specified startVertex and returns the map of distances to each vertex.
     * @param startVertex the vertex to start Dijkstra's algorithm from.
     * @return the map of distances to each vertex from the startVertex.
     */
    public Map<Vertex<V>, Double> dijkstraSearch(Vertex<V> startVertex) {
        Map<Vertex<V>, Double> distances = new HashMap<>();
        PriorityQueue<DijkstraNode<V>> priorityQueue = new PriorityQueue<>();

        // Initialize distances to infinity for all vertices except the start vertex
        for (Vertex<V> vertex : graph.getVertices()) {
            if (vertex.equals(startVertex)) {
                distances.put(vertex, 0.0);
            } else {
                distances.put(vertex, Double.POSITIVE_INFINITY);
            }
        }

        priorityQueue.offer(new DijkstraNode<>(startVertex, 0.0));

        while (!priorityQueue.isEmpty()) {
            DijkstraNode<V> currentNode = priorityQueue.poll();
            Vertex<V> currentVertex = currentNode.getVertex();
            double currentDistance = currentNode.getDistance();

            // Skip if the current distance is greater than the known distance
            if (currentDistance > distances.get(currentVertex)) {
                continue;
            }

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                double edgeWeight = edge.getWeight();
                double distanceThroughCurrent = currentDistance + edgeWeight;

                // Update the distance if it is shorter than the current known distance
                if (distanceThroughCurrent < distances.get(neighborVertex)) {
                    distances.put(neighborVertex, distanceThroughCurrent);
                    priorityQueue.offer(new DijkstraNode<>(neighborVertex, distanceThroughCurrent));
                }
            }
        }

        return distances;
    }
    /**
     * Prints the results of Dijkstra's algorithm starting from the specified startVertex.
     * @param startVertex the vertex to start Dijkstra's algorithm from.
     */
    public void printDijkstra(Vertex<V> startVertex) {
        Map<Vertex<V>, Double> distances = dijkstraSearch(startVertex);

        System.out.println("Dijkstra's Algorithm Results:");
        for (Vertex<V> vertex : distances.keySet()) {
            Double distance = distances.get(vertex);
            String distanceString = (distance == Double.POSITIVE_INFINITY) ? "Infinity" : String.valueOf(distance);
            System.out.println("Vertex: " + vertex.getData() + ", Distance: " + distanceString);
        }
    }

    /**
     * @param parentMap A Map that maps each vertex to its parent vertex in the shortest path.
     * @param destination  The destination vertex for which the path needs to be built.
     * @return  it returns the path list containing the vertices in the path from the source to the destination.
     */
    private List<V> buildPath(Map<Vertex<V>, Vertex<V>> parentMap, Vertex<V> destination) {
        List<V> path = new ArrayList<>();
        Vertex<V> currentVertex = destination;

        while (currentVertex != null) {
            path.add(0, currentVertex.getData());
            currentVertex = parentMap.get(currentVertex);
        }

        return path;
    }

    /**
     *
     * @param source the source vertex.
     * @param destination the destination vertex.
     * @return
     */
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        Map<Vertex<V>, Double> distances = new HashMap<>();
        Map<Vertex<V>, Vertex<V>> parentMap = new HashMap<>();
        PriorityQueue<DijkstraNode<V>> priorityQueue = new PriorityQueue<>();

        // Initialize distances to infinity for all vertices except the source vertex
        for (Vertex<V> vertex : graph.getVertices()) {
            if (vertex.equals(source)) {
                distances.put(vertex, 0.0);
            } else {
                distances.put(vertex, Double.POSITIVE_INFINITY);
            }
        }

        priorityQueue.offer(new DijkstraNode<>(source, 0.0));

        while (!priorityQueue.isEmpty()) {
            DijkstraNode<V> currentNode = priorityQueue.poll();
            Vertex<V> currentVertex = currentNode.getVertex();
            double currentDistance = currentNode.getDistance();

            // Skip if the current distance is greater than the known distance
            if (currentDistance > distances.get(currentVertex)) {
                continue;
            }

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                double edgeWeight = edge.getWeight();
                double distanceThroughCurrent = currentDistance + edgeWeight;

                // Update the distance if it is shorter than the current known distance
                if (distanceThroughCurrent < distances.get(neighborVertex)) {
                    distances.put(neighborVertex, distanceThroughCurrent);
                    parentMap.put(neighborVertex, currentVertex);
                    priorityQueue.offer(new DijkstraNode<>(neighborVertex, distanceThroughCurrent));
                }
            }
        }

        return buildPath(parentMap, destination);
    }
}

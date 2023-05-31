# Laboratory 6 Algorithms and Data Structures
![image](https://github.com/NargizaSailaubayeva/ass6ads/assets/125569063/1b6adc48-6b83-4c43-bc65-aed7de74c113)
___
## Description
+ A **graph** is a data structure that stores connected data. It is a type of non-linear data structure that is used to store data in the form of nodes and edges.
+ **Vertex** is a node that holds a single item from the collection 
+ **Edge** is a connection between two vertices
+ A **path** in a graph is a sequence of vertices connected by edges 
+ **Dijkstra’s algorithm** solves the single-source shortest-paths problem in edge-weighted digraphs with nonnegative weights
![g](https://static.javatpoint.com/core/images/java-graph10.png)
![v](https://mathinsight.org/media/image/image/small_undirected_network_labeled.png)
___
class `Vertex`
```java
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
```
___
class ` WeightedGraph` 
```java
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
/**
 * WeightedGraph class represents a weighted graph data structure.
 * @param <V> the type of data stored in the vertices of the graph.
 */
public class WeightedGraph<V> {
    private Map<Vertex<V>, List<Edge<V>>> list;
    /**
     *  Constructs a new WeightedGraph object.
     */
    public WeightedGraph() {
        list = new HashMap<>();
    }
    /**
     *  Edge class represents an edge in the weighted graph.
     *  @param <V> the type of data stored in the vertices of the graph.
     */
    public class Edge<V> {
        private Vertex<V> source;
        private Vertex<V> destination;
        private double weight;
        /**
         *  Constructs a new Edge object.
         *  @param source the source vertex of the edge.
         *  @param destination the destination vertex of the edge.
         *  @param weight the weight of the edge.
         */
        public Edge(Vertex<V> source, Vertex<V> destination, double weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
        }
        /**
         *  Returns the source vertex of the edge.
         *  @return the source vertex.
         */
        public Vertex<V> getSource() {
            return source;
        }
        /**
         * Returns the destination vertex of the edge.
         * @return the destination vertex.
         */
        public Vertex<V> getDestination() {
            return destination;
        }
        /**
         *  Returns the weight of the edge.
         *  @return the weight of the edge.
         */
        public double getWeight() {
            return weight;
        }
    }
    /**
     *  Adds a vertex to the graph.
     *  @param vertex the vertex to be added.
     */
    public void addVertex(Vertex<V> vertex) {
        list.put(vertex, new ArrayList<>());
    }
    /**
     *  Validates if the given vertex is present in the graph.
     *  @param vertex the vertex to be validated.
     *  @throws IllegalArgumentException if the vertex is not present in the graph.
     */
    public void validate(Vertex<V> vertex) {
        if (!list.containsKey(vertex)) {
            throw new IllegalArgumentException("Vertex " + vertex + " is out of the range");
        }
    }
    /**
     *  Adds an edge between the source and destination vertices with the given weight.
     *  @param source the source vertex of the edge.
     *  @param destination the destination vertex of the edge.
     *  @param weight the weight of the edge.
     */
    public void addEdge(Vertex<V> source, Vertex<V> destination, double weight) {
        validate(source);
        validate(destination);
        List<Edge<V>> edges = list.get(source);
        edges.add(new Edge<V>(source, destination, weight));
        list.put(source, edges);
    }
    /**
     *  Returns a list of all vertices in the graph.
     *  @return a list of all vertices.
     */
    public List<Vertex<V>> getVertices() {
        return new ArrayList<>(list.keySet());
    }
    /**
     *  Returns a list of all edges in the graph.
     *  @return a list of all edges.
     */
    public List<Edge<V>> getEdges() {
        List<Edge<V>> allEdges = new ArrayList<>();
        for (List<Edge<V>> edge : list.values()) {
            allEdges.addAll(edge);
        }
        return allEdges;
    }
    /**
     * Returns a list of edges connected to the specified vertex.
     * @param vertex the vertex to get the edges from.
     * @return a list of edges connected to the vertex.
     */
    public List<Edge<V>> getEdge(Vertex<V> vertex) {
        validate(vertex);
        return list.get(vertex);
    }
    /**
     *  Prints the information about the graph, including vertices and edges with weights.
    */
    public void graphInfo() {
        // Print vertices
        System.out.println("Vertices:");
        for (Vertex<V> vertex : list.keySet()) {
            System.out.println(vertex.getData());
        }
        System.out.println();

        // Print edges with weights
        System.out.println("Edges with Weights:");
        for (List<Edge<V>> edges : list.values()) {
            for (Edge<V> edge : edges) {
                System.out.println("Source: " + edge.getSource().getData());
                System.out.println("Destination: " + edge.getDestination().getData());
                System.out.println("Weight: " + edge.getWeight());
                System.out.println();
            }
        }
    }
}
```
___
`Search`
```java
import java.util.List;
/**
 *  Search interface defines the contract for searching a path between two vertices in a graph.
 *  @param <V> the type of data stored in the vertices of the graph.
 */
public interface Search <V> {
    /**
     * Finds a path between the source vertex and the destination vertex in a graph.
     * @param source the source vertex.
     * @param destination the destination vertex.
     * @return a list of vertices representing the path from the source to the destination.
    */
    List<V> findPath(Vertex<V> source, Vertex<V> destination);
}
```
___
`BreadthFirstSearch`
```java
import java.util.*;

public class BreadthFirstSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;
    /**
     * Constructs a BreadthFirstSearch object with the specified weighted graph.
     * @param graph the weighted graph to perform breadth-first search on.
     */
    public BreadthFirstSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }
    /**
     * Performs breadth-first search starting from the specified startVertex and returns the list of visited vertices.
     * @param startVertex the vertex to start the breadth-first search from.
     * @return the list of visited vertices in breadth-first order.
     */
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
    /**
     * Performs breadth-first search starting from the specified startVertex and prints the vertices in breadth-first order.
     * @param startVertex the vertex to start the breadth-first search from.
     */
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
        List<Vertex<V>> path = new ArrayList<>();
        Queue<Vertex<V>> queue = new LinkedList<>();
        Map<Vertex<V>, Vertex<V>> parentMap = new HashMap<>();

        queue.offer(source);
        parentMap.put(source, null);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            if (currentVertex.equals(destination)) {
                // Destination vertex found, reconstruct the path
                path.add(destination);
                Vertex<V> parent = parentMap.get(destination);
                while (parent != null) {
                    path.add(0, parent);
                    parent = parentMap.get(parent);
                }
                break;
            }

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                if (!parentMap.containsKey(neighborVertex)) {
                    queue.offer(neighborVertex);
                    parentMap.put(neighborVertex, currentVertex);
                }
            }
        }

        return (List<V>) path;
    }
}
```
___
`DijkstraSearch`
```java
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
```
___
`Main`
```java
import java.util.List;
public class Main {
    public static void main(String[] args) {
        // Create vertices
        Vertex<String> vertex1 = new Vertex<>("K");
        Vertex<String> vertex2 = new Vertex<>("L");
        Vertex<String> vertex3 = new Vertex<>("M");
        Vertex<String> vertex4 = new Vertex<>("N");
        Vertex<String> vertex5 = new Vertex<>("O");

        // Create weighted graph
        WeightedGraph<String> graph = new WeightedGraph<>();
        graph.addVertex(vertex1);
        graph.addVertex(vertex2);
        graph.addVertex(vertex3);
        graph.addVertex(vertex4);
        graph.addVertex(vertex5);

        // Add edges
        graph.addEdge(vertex1, vertex2, 1.0);
        graph.addEdge(vertex1, vertex3, 2.0);
        graph.addEdge(vertex2, vertex4, 3.0);
        graph.addEdge(vertex3, vertex4, 4.0);
        graph.addEdge(vertex3, vertex5, 5.0);
        graph.addEdge(vertex4, vertex5, 6.0);

        // Perform breadth-first search
        Search<String> BFS = new BreadthFirstSearch<>(graph);
        List<String> BFSPath = BFS.findPath(vertex1, vertex4);
        System.out.println("Breadth-First Search Path: " + BFSPath);
        ((BreadthFirstSearch<String>) BFS).printBFS(vertex1);

        // Perform Dijkstra's algorithm
        Search<String> dijkstra = new DijkstraSearch<>(graph);
        List<String> dijkstraPath = dijkstra.findPath(vertex1, vertex4);
        System.out.println("Dijkstra's Algorithm Path: " + dijkstraPath);
        ((DijkstraSearch<String>) dijkstra).printDijkstra(vertex1);

        // Test other methods
        System.out.println("Vertex K Degree: " + vertex1.getDegree());
        System.out.println("Vertex L Degree: " + vertex2.getDegree());
        System.out.println("Vertex M Degree: " + vertex3.getDegree());
        System.out.println("Vertex N Degree: " + vertex4.getDegree());
        System.out.println("Vertex O Degree: " + vertex5.getDegree());

        System.out.println("Edges in the Graph:");
        for (WeightedGraph<String>.Edge<String> edge : graph.getEdges()) {
            System.out.println("Source: " + edge.getSource().getData());
            System.out.println("Destination: " + edge.getDestination().getData());
            System.out.println("Weight: " + edge.getWeight());
            System.out.println();
        }

        System.out.println("Graph Information:");
        graph.graphInfo();
    }
}
```
___
## Installation
IntelliJ IDEA 2022.3.1
## Contributing
***Bug reports and\or pull requests are*** 
![welcome](https://media.tenor.com/oqJo9GcbfjYAAAAi/welcome-images-server.gif)

![thanks](https://cliply.co/wp-content/uploads/2021/08/472108170_THANK_YOU_STICKER_400px.gif)

![bye](https://img1.picmix.com/output/stamp/normal/8/5/7/7/2417758_c18a0.gif)

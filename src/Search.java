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

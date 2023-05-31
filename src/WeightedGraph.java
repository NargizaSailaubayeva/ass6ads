public class WeightedGraph <V> {
    private Map<Vertex<V>, List<Edge<V>>> list;
    public WeightedGraph() {
        list = new HashMap<>();
    }

}

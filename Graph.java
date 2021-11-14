import java.util.*;

public class Graph {
    static ArrayList<Integer> bfs(int V,ArrayList<ArrayList<Integer>> adj){
        ArrayList<Integer> bfs = new ArrayList<Integer>();
        Queue<Integer> q = new LinkedList<>();
        boolean vis[] = new boolean[V+1];
        for (int i = 1;i<=V; i++){
            if (vis[i] == false){
                q.add(i);
                vis[i] = true;
                while (!q.isEmpty()){
                    Integer node = q.poll();
                    bfs.add(node);
                    for (Integer it : adj.get(node)){
                        if (vis[it]==false){      
                            q.add(it);
                            vis[it] = true;
                        }
                    }
                }
            }
        }
        return bfs;
    }

    static void addEdge(ArrayList<ArrayList<Integer> > adj,int u, int v){
        adj.get(u).add(v);
        adj.get(v).add(u);
    }
    public static void main(String[] args) {
        ArrayList<ArrayList<Integer>> adj = new ArrayList<ArrayList<Integer>>();
        for (int i = 0;i<=7;i++)adj.add(new ArrayList<Integer>());

        addEdge(adj,1,2);
        addEdge(adj,2,1);
        addEdge(adj,2,3);
        addEdge(adj,2,7);
        addEdge(adj,3,2);
        addEdge(adj,3,7);
        addEdge(adj,3,5);
        addEdge(adj,4,6);
        addEdge(adj,5,3);
        addEdge(adj,5,7);
        addEdge(adj,6,4);
        addEdge(adj,7,2);
        addEdge(adj,2,5);
        
        ArrayList<Integer> bfs = bfs(7,adj);
        System.out.println("bfs for given graph is :\t"+bfs);
    }    
}

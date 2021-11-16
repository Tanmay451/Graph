import java.util.*;
class Node{
    int current, parent;
    public Node(int current, int parent){
        this.current = current;
        this.parent = parent;
    }
} 
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

    static void dfsHelper(int node, ArrayList<ArrayList<Integer>> adj, boolean vis[], ArrayList<Integer> dfs){
        dfs.add(node);
        vis[node] = true;
        for (Integer it: adj.get(node)){
            if (!vis[it]){
                dfsHelper(it, adj, vis, dfs);
            }
        }
    }

    static ArrayList<Integer>dfs(int V, ArrayList<ArrayList<Integer>> adj){
        ArrayList<Integer> dfs = new ArrayList<>();
        boolean vis[] = new boolean[V+1];
        for (int i = 1; i<V;i++){
            if (!vis[i]){
                dfsHelper(i,adj,vis,dfs);
            }
        }
        return dfs;
    }

    static void addEdge(ArrayList<ArrayList<Integer> > adj,int u, int v){
        adj.get(u).add(v);
        adj.get(v).add(u);
    }

    static void addEdgeDirected(ArrayList<ArrayList<Integer> > adj,int u, int v){
        adj.get(u).add(v);
    }

    static boolean isCycleHelper(int i, boolean vis[], ArrayList<ArrayList<Integer>> adg){
        Queue<Node> q = new LinkedList<>();
        q.add(new Node(i,-1));
        while(!q.isEmpty()){
            int current = q.peek().current;
            int parent = q.peek().parent;
            q.remove();
            for (Integer it : adg.get(current)){
                if (!vis[it]){
                    q.add(new Node(it,current));
                    vis[it] = true;
                } else if (it != parent){
                    return true;
                }
            }
        }
        return false;
    }

    static boolean isCycle(int V, ArrayList<ArrayList<Integer>> adj){
        boolean vis[] = new boolean[V+1];
        for (int i = 1; i<=V;i++){
            if (!vis[i]){
                if (isCycleHelper(i,vis, adj)) return true;
            }
        }
        return false;
    }

    static boolean isCycleDFSHelper(int i, int parent, boolean vis[], ArrayList<ArrayList<Integer>> adj){
        vis[i] = true;
        for (Integer it : adj.get(i)){
            if (!vis[it]){
                if(isCycleDFSHelper(it, i, vis, adj)){
                    return true;
                }
            }else if (it != parent){
                return true;
            }
        }
        return false;
    }

    static boolean isCycleDFS(int V, ArrayList<ArrayList<Integer>> adj){
        boolean vis[] = new boolean[V+1];
        for(int i = 1;i<=V;i++){
            if (!vis[i]){
                if (isCycleDFSHelper(i,-1,vis,adj)) return true;
            }
        }
        return false;
    }

    static boolean isBipartiteHelper(int i, int vis[], ArrayList<ArrayList<Integer>> adj){
        vis[i] = 0;
        Queue<Integer> q = new LinkedList<>();
        q.add(i);
        while(!q.isEmpty()){
            Integer node = q.poll();
            for (Integer it : adj.get(node)){
                if (vis[it] == -1){
                    q.add(it);
                    vis[it] = 1-vis[node];
                }else if (vis[it] == vis[node]){
                    return false;
                }
            }
        }

        return true;
    }

    static boolean isBipartite(int V, ArrayList<ArrayList<Integer>> adj){
        int vis[] = new int[V+1];
        for (int i = 0; i<=V;i++){
            vis[i] = -1;
        }
        for (int i = 1; i<= V;i++){
            if (vis[i] == -1){
                if (!isBipartiteHelper(i, vis, adj)) return false;
            }
        }
        return true;
    }

    static boolean isBipartiteDFSHelper(int i, int vis[], ArrayList<ArrayList<Integer>> adj, int c){
        vis[i] = 1-c;
        for (Integer it: adj.get(i)){
            if (vis[it] == -1){
                if (!isBipartiteDFSHelper(it, vis, adj, vis[i])) return false;
            } else if (vis[it] == vis[i]){
                return false;
            }
        }
        return true;
    }

    static boolean isBipartiteDFS(int V, ArrayList<ArrayList<Integer>> adj){
        int vis[] = new int[V+1];
        for (int i = 1; i<=V; i++){
            vis[i] = -1;
        }

        for (int i = 1; i<=V;i++){
            if (vis[i] == -1){
                if (!isBipartiteDFSHelper(i, vis, adj, 1)) return false;
            }
        }
        return true;
    }

    static boolean isCycleDirectedHelper(int i, boolean vis[], boolean dfsVis[], ArrayList<ArrayList<Integer>> adj){
        vis[i] = true;
        dfsVis[i] = true;
        for(Integer it : adj.get(i)){
            if (!vis[it]){
                if (isCycleDirectedHelper(it, vis, dfsVis, adj)){
                    return true;
                } 
            }   else if(dfsVis[it]){
                    return true;
            }
        }
        dfsVis[i] = false;
        return false;
    }

    static boolean isCycleDirected(int V, ArrayList<ArrayList<Integer>> adj){
        boolean vis[] = new boolean[V+1];
        boolean dfsVis[] = new boolean[V+1];
        for (int i = 1; i<= V; i++){
            if(!vis[i]){
                if (isCycleDirectedHelper(i,vis,dfsVis,adj)){
                    return true;
                }
            }
        }
        return false;
    }

    static void topologicalSortHelper(int i, boolean vis[], Stack<Integer> st, ArrayList<ArrayList<Integer>> adj){
        vis[i] = true;
        for (Integer it : adj.get(i)){
            if (!vis[it]){
                topologicalSortHelper(it, vis, st, adj);
            }
        }
        st.push(i);
    }

    static ArrayList<Integer> topologicalSort(int V, ArrayList<ArrayList<Integer>> adj){
        Stack<Integer> st = new  Stack<>();
        boolean vis[] = new boolean[V+1];
        for (int i = 1; i<=V;i++){
            if (!vis[i]){
                topologicalSortHelper(i,vis,st,adj);
            }
        }

        ArrayList<Integer> topologicalSort = new ArrayList<Integer>();
        while(!st.empty()){
            topologicalSort.add(st.pop());
        }
        return topologicalSort;
    }

    static ArrayList<Integer> topologicalSortKahn(int V, ArrayList<ArrayList<Integer>> adj){
        ArrayList<Integer> result = new ArrayList<>();
        int degree[] = new int[V+1];
        for (int i = 1;i<=V;i++){
            for (Integer it: adj.get(i)){
                degree[it]++;
            }
        }

        Queue<Integer> q = new LinkedList<>();
        for (int i = 1; i<V+1; i++){
            if (degree[i] == 0){
                q.add(i);
            }
        }

        while(!q.isEmpty()){
            Integer temp = q.poll();
            result.add(temp);
            for (Integer it : adj.get(temp)){
                degree[it]--;
                if (degree[it] == 0){
                    q.add(it);
                }
            }
        }
        return result; 
    }

    static boolean isCycleKahn(int V, ArrayList<ArrayList<Integer>> adj){
        ArrayList<Integer> result = new ArrayList<>();
        int degree[] = new int[V+1];
        for (int i = 1;i<=V;i++){
            for (Integer it: adj.get(i)){
                degree[it]++;
            }
        }

        Queue<Integer> q = new LinkedList<>();
        for (int i = 1; i<V+1; i++){
            if (degree[i] == 0){
                q.add(i);
            }
        }
        int c = 0;
        while(!q.isEmpty()){
            Integer temp = q.poll();
            result.add(temp);
            for (Integer it : adj.get(temp)){
                degree[it]--;
                if (degree[it] == 0){
                    q.add(it);
                }
            }
            c++;
        }

        if (c == V){
            return false;
        }
        return true;
    }
    public static void main(String[] args) {
        ArrayList<ArrayList<Integer>> adj = new ArrayList<ArrayList<Integer>>();
        for (int i = 0;i<=7;i++)adj.add(new ArrayList<Integer>());

        addEdge(adj,1,2);
        addEdge(adj,2,3);
        addEdge(adj,2,7);
        addEdge(adj,3,5);
        addEdge(adj,4,6);
        addEdge(adj,5,7);
        // addEdge(adj,3,7);


        // System.out.println(adj);
        ArrayList<Integer> bfs = bfs(7,adj);
        System.out.println("BFS for given graph is :\t"+bfs);

        ArrayList<Integer> dfs = dfs(7,adj);
        System.out.println("DFS for given graph is :\t"+dfs);

        boolean isCycle = isCycle(7, adj);
        System.out.println("Detecting circle in given graph :\t"+isCycle);

        boolean isCycleDFS = isCycleDFS(7, adj);
        System.out.println("Detecting circle using DFS in given graph :\t"+isCycleDFS);

        boolean isBipartite = isBipartite(7, adj);
        System.out.println("Detecting bipartite using BFS in given graph :\t"+isBipartite);

        boolean isBipartiteDFS = isBipartiteDFS(7, adj);
        System.out.println("Detecting bipartite using BFS in given graph :\t"+isBipartiteDFS);

        ArrayList<ArrayList<Integer>> adjDirected = new ArrayList<ArrayList<Integer>>();
        for (int i = 0;i<=5;i++)adjDirected.add(new ArrayList<Integer>());

        addEdgeDirected(adjDirected,1,2);
        addEdgeDirected(adjDirected,2,3);
        addEdgeDirected(adjDirected,2,4);
        addEdgeDirected(adjDirected,3,5);
        addEdgeDirected(adjDirected,4,5);

        // System.out.println(adjDirected);
        
        boolean isCycleDirected = isCycleDirected(5,adjDirected);
        System.out.println("Circle in given directed graph :\t"+isCycleDirected);

        ArrayList<Integer> topologicalSort = topologicalSort(5,adjDirected);
        System.out.println("TopologicalSort for given directed graph :\t"+topologicalSort);

        ArrayList<Integer> topologicalSortKahn = topologicalSortKahn(5,adjDirected);
        System.out.println("TopologicalSort using Kahn's algo:\t"+topologicalSortKahn);

        boolean isCycleKahn = isCycleKahn(5,adjDirected);
        System.out.println("Cycle detection using Kahn's algo:\t"+isCycleKahn);
    }    
}

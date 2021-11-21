import java.util.*;

class Node{
    int current, parent;
    public Node(int current, int parent){
        this.current = current;
        this.parent = parent;
    }
}

class Edge implements Comparator<Edge>{
    int u,v,w;
    Edge(int u, int v, int w){
        this.u = u;
        this.v = v;
        this.w = w;
    }
    Edge(){}
    @Override
    public int compare(Edge e1, Edge e2){
        if (e1.w < e2.w){
            return -1;
        } if (e1.w > e2.w){
            return 1;
        } 
        return 0;
    }
}

class Pair implements Comparator<Pair>{
    int v;
    int weight;

    public Pair(int v, int weight){
        this.v = v;
        this.weight = weight;
    }

    Pair(){}

    @Override
    public int compare(Pair n1, Pair n2){
        if (n1.v<n2.v) return -1;
        if (n1.v>n2.v) return 1;
        return 0;
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

    static ArrayList<Integer> shortestPath(int V, ArrayList<ArrayList<Integer>> adj, int source){
        ArrayList<Integer> distance = new ArrayList<>();
        for (int i = 0; i<=V; i++){
            distance.add(10000000);
        }

        distance.set(source,0);
        Queue<Integer> q = new LinkedList<>();
        q.add(source);

        while(!q.isEmpty()){
            int temp = q.poll();
            for (Integer it: adj.get(temp)){
                if (distance.get(it) > distance.get(temp)+1){
                    distance.set(it, distance.get(temp)+1);
                    q.add(it);
                }
            }
        }

        return distance;
    }

    static void topoSortHelper(int i, boolean vis[], ArrayList<ArrayList<Pair>> adj, Stack<Integer> st){
        vis[i] = true;
        for (Pair it : adj.get(i)){
            if(!vis[it.v]){
                topoSortHelper(it.v, vis, adj, st);
            }
        }
        st.push(i);
    }
    
    static Stack<Integer> topoSort(int V, ArrayList<ArrayList<Pair>> adj){
        boolean vis[] = new boolean[V];

        Stack<Integer> st = new Stack<>();

        for (int i = 0; i<V; i++){
            if (!vis[i]){
                topoSortHelper(i, vis, adj,st);
            }
        }

        return st;
    }

    static ArrayList<Integer> shortedPathWithWeight(int V, ArrayList<ArrayList<Pair>> adj){
        ArrayList<Integer> distance = new ArrayList<>();
        for (int i = 0; i< V; i++){
            distance.add(Integer.MAX_VALUE);
        }

        Stack<Integer> topoSort = topoSort(V,adj);

        distance.set(0, 0);

        while(!topoSort.isEmpty()){
            Integer idx = topoSort.pop();
            if (distance.get(idx) != Integer.MAX_VALUE){
                for (Pair it : adj.get(idx)){
                    Integer temp = distance.get(it.v);
                    if (temp > distance.get(idx)+it.weight){
                        distance.set(it.v,distance.get(idx)+it.weight);
                    }
                }
            }
        }

        return distance;
    }

    static ArrayList<Integer> shortestPathForUndirectedGraph(int V, ArrayList<ArrayList<Pair>> adj){
        ArrayList<Integer> dis = new ArrayList<Integer>();
        for (int i = 0;i<V;i++){
            dis.add(Integer.MAX_VALUE);
        }

        PriorityQueue<Pair> pq = new PriorityQueue<Pair>(new Pair());
        pq.add(new Pair(0,0));
        dis.set(0, 0);

        while(!pq.isEmpty()){
            Pair idx = pq.poll();

            for (Pair it : adj.get(idx.v)){
                if (dis.get(it.v) > dis.get(idx.v)+it.weight){
                    dis.set(it.v, dis.get(idx.v)+it.weight);
                    pq.add(it);
                }
            }
        }
        return dis;
    } 
    
    static ArrayList<Integer> spaningTree(int V, ArrayList<ArrayList<Pair>> adj){
        ArrayList<Integer> parent = new ArrayList<Integer>();
        ArrayList<Integer> edge = new ArrayList<Integer>();
        ArrayList<Boolean> mst = new ArrayList<Boolean>();
        for (int i = 0; i<V;i++){
            parent.add(-1);
            edge.add(Integer.MAX_VALUE);
            mst.add(false);
        }
        
        edge.set(0, 0);

        for (int i = 0; i<V-1;i++){
            int tempIdx = 0;
            int tempMinEdge = Integer.MAX_VALUE;

            for (int j = 0;j<V;j++){
                if (edge.get(j) < tempMinEdge && !mst.get(j)){
                    tempIdx = j;
                    tempMinEdge = edge.get(j);
                }
            }

            mst.set(tempIdx, true);

            for (Pair it : adj.get(tempIdx)){
                if (!mst.get(it.v) && edge.get(it.v) > it.weight){
                    edge.set(it.v, it.weight);
                    parent.set(it.v, tempIdx);
                }
            }

        }

        return parent;
    }
    static boolean isContain(ArrayList<Edge> eList, Edge e){
        for (Edge eTemp : eList){
            if (eTemp.u == e.u && eTemp.v == e.v){
                return true;
            }
        }
        return false;
    }

    static int findParent(int parent[], int i){
        if (parent[i] == i){
            return i;
        }
        
        return parent[i] = findParent(parent, parent[i]);
    }

    static void union(int parent[], int rank[], int u, int v){
        int parentU = findParent(parent, u);
        int parentV = findParent(parent, v);

        if (rank[parentU] > rank[parentV]){
            parent[parentV] = parentU;
        } else if (rank[parentV] > rank[parentU]){
            parent[parentU] = parentV;
        } else {
            parent[parentV] = parentU;
            rank[parentU]++;
        }
    }
    static ArrayList<ArrayList<Integer>> spaningTreeUsingKruskal(int V, ArrayList<ArrayList<Pair>> adj){
        ArrayList<ArrayList<Integer>> parent = new ArrayList<ArrayList<Integer>>();
        
        int rank[] = new int[V];
        int par[] = new int[V];

        for (int i = 0; i< V; i++){
            rank[i] = 0;
            par[i] = i;
        }

        ArrayList<Edge> e = new ArrayList<>();

        for (int i = 0; i<V;i++){
            for (Pair p : adj.get(i)){
                boolean isContain = isContain(e,new Edge(p.v,i, p.weight));
                if (!isContain){    
                    e.add(new Edge(i, p.v, p.weight));
                }
            }
        }
        
        Collections.sort(e,new Edge());
        for (Edge tempE : e){
            int parentU = findParent(par, tempE.u);
            int parentV = findParent(par, tempE.v);
            if (parentU != parentV){
                union(par, rank, tempE.u, tempE.v);
                ArrayList<Integer> temp = new ArrayList<>();
                temp.add(tempE.u);
                temp.add(tempE.v);
                parent.add(temp);
            }
        }



        return parent;
    }

    static void dfsToCheckBridges(int node,int parent,ArrayList<ArrayList<Integer>> adj, int dis[], int low[], boolean vis[], int time, ArrayList<ArrayList<Integer>> bridge){
        vis[node] = true;
        dis[node] = low[node] = time;
        time++;
        for (Integer it : adj.get(node)){
            if (it == parent) continue;
            if (!vis[it]){
                dfsToCheckBridges(it, node, adj, dis, low, vis, time, bridge);
                low[node] = Math.min(low[node], low[it]);
                if (low[it] > dis[node]){
                    ArrayList<Integer> temp = new ArrayList<>();
                    temp.add(node);
                    temp.add(it);
                    bridge.add(temp);
                } 
            }else{
                low[node] = Math.min(low[node], low[it]);
            }
        }
    }

    static ArrayList<ArrayList<Integer>> GraphBridges(int V, ArrayList<ArrayList<Integer>> adj){
        ArrayList<ArrayList<Integer>> bridge = new ArrayList<ArrayList<Integer>>();
        boolean vis[] = new boolean[V];
        int dis[] = new int[V];
        int low[] = new int[V];

        for (int i = 0; i<V;i++){
            vis[i] = false;
            dis[i] = low[i] = -1;
        }

        for (int i = 0; i<V;i++){
            if (!vis[i]){
                dfsToCheckBridges(i,-1,adj,dis,low,vis,0,bridge);
            }
        }
        return bridge;
    } 

    static void dfsToCheckArticulationPoint(int node,int parent,ArrayList<ArrayList<Integer>> adj, int dis[], int low[], boolean vis[], int time, boolean articulationPoint[]){
        vis[node] = true;
        dis[node] = low[node] = time;
        time++;
        for (Integer it : adj.get(node)){
            if (it == parent) continue;
            if (!vis[it]){
                dfsToCheckArticulationPoint(it, node, adj, dis, low, vis, time, articulationPoint);
                low[node] = Math.min(low[node], low[it]);
                if (low[it] >= dis[node] && parent != -1){
                    articulationPoint[node] = true;
                } 
            }else{
                low[node] = Math.min(low[node], low[it]);
            }
        }
    }

    static boolean[] GraphArticulation(int V, ArrayList<ArrayList<Integer>> adj){
        boolean articulationPoint[] = new boolean[V];
        boolean vis[] = new boolean[V];
        int dis[] = new int[V];
        int low[] = new int[V];

        for (int i = 0; i<V;i++){
            vis[i] = articulationPoint[i] = false;
            dis[i] = low[i] = -1;
        }

        for (int i = 0; i<V;i++){
            if (!vis[i]){
                dfsToCheckArticulationPoint(i,-1,adj,dis,low,vis,0,articulationPoint);
            }
        }
        return articulationPoint;
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

        ArrayList<Integer> shortestPath = shortestPath(7,adj,1);
        System.out.println("ShortestPath using BFS in given graph :\t"+shortestPath);


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

        ArrayList<ArrayList<Pair>> adjWithWeight = new ArrayList<ArrayList<Pair>>();
        for (int i = 0;i<=5;i++)adjWithWeight.add(new ArrayList<Pair>());

        adjWithWeight.get(0).add(new Pair(1,2));
        adjWithWeight.get(1).add(new Pair(0,2));
        adjWithWeight.get(0).add(new Pair(4,1));
        adjWithWeight.get(4).add(new Pair(0,1));
        adjWithWeight.get(1).add(new Pair(2,3));
        adjWithWeight.get(2).add(new Pair(1,3));
        adjWithWeight.get(4).add(new Pair(2,2));
        adjWithWeight.get(2).add(new Pair(4,2));
        adjWithWeight.get(4).add(new Pair(5,4));
        adjWithWeight.get(5).add(new Pair(4,4));
        adjWithWeight.get(5).add(new Pair(3,1));
        adjWithWeight.get(3).add(new Pair(5,1));
        adjWithWeight.get(2).add(new Pair(3,6));
        adjWithWeight.get(3).add(new Pair(2,6));

        ArrayList<Integer> shortestPathForUndirectedGraph = shortestPathForUndirectedGraph(6,adjWithWeight);
        System.out.println("Shortest path for undirected graph using dijkstra algo:\t"+shortestPathForUndirectedGraph);

        ArrayList<Integer> spaningTree = spaningTree(6,adjWithWeight);
        System.out.println("Spaning tree for graph using prims algo:\t"+spaningTree);

        ArrayList<ArrayList<Integer>> spaningTreeUsingKruskal = spaningTreeUsingKruskal(6,adjWithWeight);
        System.out.println("Spaning tree for graph using kruskal algo:\t"+spaningTreeUsingKruskal);

        

        ArrayList<ArrayList<Pair>> adjDirectedWithWeight = new ArrayList<ArrayList<Pair>>();
        for (int i = 0;i<=5;i++)adjDirectedWithWeight.add(new ArrayList<Pair>());

        adjDirectedWithWeight.get(0).add(new Pair(1,2));
        adjDirectedWithWeight.get(0).add(new Pair(4,1));
        adjDirectedWithWeight.get(1).add(new Pair(2,3));
        adjDirectedWithWeight.get(4).add(new Pair(2,2));
        adjDirectedWithWeight.get(4).add(new Pair(5,4));
        adjDirectedWithWeight.get(5).add(new Pair(3,1));
        adjDirectedWithWeight.get(2).add(new Pair(3,6));
        
        ArrayList<Integer> shortedPathWithWeight = shortedPathWithWeight(6,adjDirectedWithWeight);
        System.out.println("Shorted path for directed graph with weight:\t"+shortedPathWithWeight);


        int n = 12;
        ArrayList<ArrayList<Integer> > a = new ArrayList<ArrayList<Integer> >();
		
		for (int i = 0; i < n; i++) 
			a.add(new ArrayList<Integer>());
			
		a.get(0).add(1);
		a.get(0).add(3);
        a.get(1).add(0);
        a.get(1).add(2);
		a.get(2).add(1);
		a.get(2).add(3);
		a.get(3).add(0);
		a.get(3).add(2);
		a.get(3).add(4);
		a.get(4).add(3);
		a.get(4).add(5);
		a.get(5).add(4);
		a.get(5).add(6);
		a.get(5).add(8);
		a.get(6).add(5);
		a.get(6).add(7);
		a.get(7).add(6);
		a.get(7).add(8);
		a.get(7).add(9);
        a.get(8).add(5);
		a.get(8).add(7);
		a.get(9).add(7);
		a.get(9).add(10);
		a.get(9).add(11);
        a.get(10).add(9);
		a.get(10).add(11);
        a.get(11).add(9);
		a.get(11).add(11);

        ArrayList<ArrayList<Integer>> GraphBridges = GraphBridges(12,a);
        System.out.println("Bridges in graph are:\t"+GraphBridges);

        boolean GraphArticulation[] = GraphArticulation(12,a);
        System.out.println("Articulation point in graph are:\t"+Arrays.toString(GraphArticulation));
    }    
}

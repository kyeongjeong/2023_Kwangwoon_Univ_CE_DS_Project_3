#include "GraphMethod.h"

bool BFS(Graph* graph, int vertex) //perform BFS 
{
    int size = graph->getSize();
    if(vertex >= size){ //if entered vertex not exist in graph
        return false;
    }

    ofstream* fout = graph->getStream();
    (*fout)<<"======== BFS ========"<<endl;
    (*fout)<<"startvertex: "<<vertex<<endl;
    bool* visited = new bool[size];
    fill(visited, visited+size, false); //fill all by false
    visited[vertex] = true;
    (*fout)<<vertex;

    queue<int> que;

    que.push(vertex);
    while(!(que.empty())){ //until que is empty
        vertex = que.front();
        que.pop();

        map<int, int> m;
        graph->getAdjacentEdges(vertex, &m); //get all vertax's adjacent eages

        for(auto it = m.begin(); it!=m.end(); it++){
            if(!visited[it->first]){ //if it->first vertex is not visited
                que.push(it->first); //push to queue
                visited[it->first] = true; //check visit
                (*fout)<<" -> "<<it->first;
            }
        }
    }
    (*fout)<<endl;
	(*fout)<<"======================="<<endl<<endl;

    delete[] visited;
    return true;
}

bool DFS(Graph* graph, int vertex)
{
    int size = graph->getSize();
    if(vertex >= size){ //if entered vertex not exist in graph
        return false;
    }

    ofstream* fout = graph->getStream();
    (*fout)<<"======== DFS ========"<<endl;
    (*fout)<<"startvertex: "<<vertex<<endl;

    bool* visited = new bool[size];
    fill(visited, visited+size, false); //fill by false
    visited[vertex] = true;
    (*fout)<<vertex;

    stack<int> stk;
    bool findNode = false;

    stk.push(vertex);
    while(!(stk.empty())){ //until stack is empty
        vertex = stk.top();

        map<int, int> m;
        graph->getAdjacentEdges(vertex, &m); //get vertex's adjacent edges

        for(auto it = m.begin(); it!=m.end(); it++){
            if(!visited[it->first]){ //vertex's adjacent vertex is not visited vertex
                stk.push(it->first);
                visited[it->first] = true; //check visit
                (*fout)<<" -> "<<it->first;
                findNode = true;
                break;
            }
        }
        if(findNode == false){ //dosen't have not visited adjacent vertex
            stk.pop();
        }

        findNode = false;
    }
    (*fout)<<endl;
    (*fout)<<"======================="<<endl<<endl;

    delete[] visited;
    return true;
}

bool DFS_R(Graph* graph, vector<bool>* visit, int vertex)
{
    ofstream* fout = graph->getStream();

    visit[vertex][0] = true;
    map<int, int> m;
    graph->getAdjacentEdges(vertex, &m); //get adjacent edges

    for(auto it = m.begin(); it!=m.end(); it++){
        if(!visit[it->first][0]){ //if not visited
            (*fout)<<" -> "<<it->first;
            DFS_R(graph, visit, it->first); //recursive call DFS_R
        }
    }
}

bool Kruskal(Graph* graph)
{ 
    ofstream* fout = graph->getStream();    //get ofstream to print log.txt
    int cost = 0;
    vector< pair<int, pair<int, int> > > edge_w_weight;

    int size = graph->getSize(); //get all edges
    for(int i=0; i<size; i++){
        map<int, int> m;
        graph->getAdjacentEdges(i, &m);
        for(auto it = m.begin(); it!=m.end(); it++){
            if(i<it->first)
                edge_w_weight.push_back(make_pair(it->second, make_pair(i, it->first))); //save all edges to edge_w_weight
        }
    }

    int edgeSize = edge_w_weight.size(); //all edge's count

    quickSort(&edge_w_weight, 0, (edgeSize-1)); //all edge sorting by weight

    int * parent = new int[size];
    for(int i=0; i<size; i++){
        parent[i] = -1; //parent initialize
    }

    vector< pair<int, pair<int, int> > > T; //save mst's edge
    while((T.size() < (size-1)) && (!edge_w_weight.empty())){ //if complete
        pair<int, pair<int, int> > edge = edge_w_weight[0]; //select minimum weight edge
        edge_w_weight.erase(edge_w_weight.begin());

        int vertexFrom = edge.second.first;
        int vertexTo = edge.second.second;

        if(find_p(parent, vertexTo) != find_p(parent, vertexFrom)){ //find vertex's root
            T.push_back(edge);
            union_p(parent, find_p(parent, vertexFrom), find_p(parent, vertexTo)); //union set
            cost += edge.first; //calculate cost
        }
    }

    if(T.size() < (size-1)){ //not spanning tree
        delete[] parent;
        return false;
    }

    vector< pair<int, pair<int, int> > > print_T;
    for(auto it = T.begin(); it!=T.end(); it++){ //insert inverse to print
        print_T.push_back(make_pair(it->first, make_pair(it->second.first, it->second.second)));
        print_T.push_back(make_pair(it->first, make_pair(it->second.second, it->second.first)));
    }


    (*fout)<<"======== Kruskal ========"<<endl;
    //sort for print
    quickSort(&print_T, 0, (print_T.size()-1));
    for(int i=0; i<size; i++){
        (*fout)<<"["<<i<<"] "; //all vertex in ascending order
        for(auto it = print_T.begin(); it!=print_T.end(); it++){
            if(it->second.first == i){ //corressponding vertex
                (*fout)<<it->second.second<<"("<<it->first<<") "; //weight print
            }
        }
        (*fout)<<endl;
    }

    (*fout)<<"cost: "<<cost<<endl;
	(*fout)<<"======================="<<endl<<endl;

    delete[] parent;
    return true;
}

bool Dijkstra(Graph* graph, int vertex)
{
    int size = graph->getSize(); //get graph vertex count
    if(vertex >= size){ //if entered vertex not exist in graph
        return false;
    }

    ofstream* fout = graph->getStream();    //get ofstream to print log.txt

    int ** length = new int*[size];
    for(int i=0; i<size; i++){ //dynamic memory allocation
        length[i] = new int[size];
        for(int j=0; j<size; j++){
            length[i][j] = MAX; //initialize in MAX
        }
    }

    for(int i=0; i<size; i++){ //all vertex
        map<int, int> m;
        graph->getAdjacentEdges(i, &m);
        for(auto it = m.begin(); it!=m.end(); it++){ //save all vertex pair's weight
            if(it->second < 0){ //if nagative weight, end dijkstra
                for(int i=0; i<size; i++){ //memory release
                    delete[] length[i];
                }
                delete[] length;
                return false;
            }
            length[i][it->first] = it->second;
        }
    }

    //dynamic memory allocation
    bool* s = new bool[size];
    int * dist = new int[size];
    int * prev = new int[size];

    for(int i=0; i<size; i++){ //setting
        s[i] = false;
        if(i == vertex) dist[i] = 0;
        else dist[i] = length[vertex][i]; //setting dist by weight from startvertex to i
        if(dist[i]==MAX) //setting prev to -1 or startvertex
            prev[i] = -1;
        else {
            if(i == vertex) prev[i] = -1;
            else prev[i] = vertex;
        }
    }
    s[vertex] = true; //set startvertex
    dist[vertex] = 0;

    for(int i=0; i<size; i++){ //repeat update
        int u = -1;
        int min_d = MAX;
        bool check = false;

        for(int j=0; j<size; j++){ //get not visited vertex that smallest weight from visited vertex
            if(s[j] == false){ //j is not visited
                if(dist[j] < min_d){
                    min_d = dist[j];
                    u = j;
                    check = true;
                }
            }
        }

        if(check == false){
            continue; //all possible vertex is linked
        }

        s[u] = true; //check visit u
        map<int, int> m;
        if(u >= 0)
            graph->getAdjacentEdges(u, &m); //get u's adjacent edge

        for(auto it = m.begin(); it!=m.end(); it++){
            if(s[it->first] == false){ //it->first vertex is not visited
                if(dist[it->first] > dist[u]+length[u][it->first]){ //if new route is short, update dist and prev
                    dist[it->first] = dist[u]+length[u][it->first];
                    prev[it->first] = u;
                }
            }
        }
    }

    (*fout)<<"======== Dijkstra ========"<<endl;
    (*fout)<<"startvertex: "<<vertex<<endl;
    for(int i=0; i<size; i++){
        if(i == vertex)
            continue;

        stack<int> route; //to inverse route print

        //print
        (*fout)<<"["<<i<<"] ";
        if(dist[i] == MAX){ //this vertex is not linked
            (*fout)<<"x";
            (*fout)<<endl;
            continue;
        }
        else{
            int path = i;
            while(path>=0){ //find path
                route.push(path);
                path = prev[path];
            }
        }

        int v;
        while(route.size() !=1){ //to not print -> at end
            v = route.top();
            route.pop();

            (*fout)<<v<<" -> ";
        }

        v = route.top();
        route.pop();
        (*fout)<<v<<" ("<<dist[i]<<")";
        (*fout)<<endl;
    }

    (*fout)<<"======================="<<endl<<endl;

    //Release all memory with dynamic allocation
    delete[] s;
    delete[] dist;
    delete[] prev;
    for(int i=0; i<size; i++){
        delete[] length[i];
    }
    delete[] length;

    return true;
}

bool Bellmanford(Graph* graph, int s_vertex, int e_vertex)
{
    ofstream* fout = graph->getStream();    //get ofstream to print log.txt
    int size = graph->getSize();
    if(s_vertex >= size){ //if entered vertex not exist in graph
        return false;
    }
    if(e_vertex >= size){ //if entered vertex not exist in graph
        return false;
    }

    int * dist = new int[size];
    int * prev = new int[size];
    int ** length = new int*[size];
    for(int i=0; i<size; i++){ //dynamic memory allocation
        length[i] = new int[size];
        for(int j=0; j<size; j++){
            length[i][j] = MAX;
        }
    }

    for(int i=0; i<size; i++){
        map<int, int> m;
        graph->getAdjacentEdges(i, &m);
        for(auto it = m.begin(); it!=m.end(); it++){ //setting all pair weight to length
            length[i][it->first] = it->second;
        }
    }

    for(int i=0; i<size; i++){
        dist[i] = length[s_vertex][i]; //setting dist by weight from startvertex to i
        if(dist[i]==MAX)
            prev[i] = -1;
        else{  
            if(i == s_vertex) prev[i] = -1; 
            else prev[i] = s_vertex; //if start to i path is exist, prev setting
        }
    }
    dist[s_vertex] = 0;

    for(int i=2; i<size; i++){
        for(int j=0; j<size; j++){
            if(dist[j] != MAX){ //already visited vertex
                map<int, int> m;
                graph->getAdjacentEdges(j, &m); //get all adjacent edge
                for(auto it = m.begin(); it!=m.end(); it++){ //update short path
                    if(dist[j] == MAX || length[j][it->first]==MAX) continue;
                    if(dist[it->first] > dist[j]+length[j][it->first]){
                        dist[it->first] = dist[j]+length[j][it->first];
                        prev[it->first] = j; //update prev
                    }
                }
            }
        }
    }

    for(int i=2; i<size; i++){ //to find negative weight cycle
        for(int j=0; j<size; j++){
            if(dist[j] != MAX){ 
                map<int, int> m;
                graph->getAdjacentEdges(j, &m);
                for(auto it = m.begin(); it!=m.end(); it++){
                    if(dist[it->first] > (dist[j]+length[j][it->first])){ //new update mean nagative weight cycle exist
                        delete[] dist; //memory release
                        delete[] prev;
                        for(int i=0; i<size; i++){ 
                            delete[] length[i];
                        }
                        delete[] length;

                        return false; //cannot bellmanford
                    }
                }
            }
        }
    }

    (*fout)<<"======== Bellman-Ford ========"<<endl;
    stack<int> route;
    int p_vertex = e_vertex;
    while(prev[p_vertex] >= 0){ //except for start vertex, get path
        route.push(p_vertex);
        p_vertex = prev[p_vertex];
    }
    route.push(s_vertex); //add start vertex to path

    if(route.size() == 1){ //if start to end route is not exist
        route.pop();

        //all memory release
        delete[] dist;
        delete[] prev;
        for(int i=0; i<size; i++){
            delete[] length[i];
        }
        delete[] length;

        (*fout)<<"x"<<endl;
        (*fout)<<"======================="<<endl<<endl;

        return true;        
    }

    while(route.size() !=1){ //print path
        p_vertex = route.top();
        route.pop();

        (*fout)<<p_vertex<<" -> ";
    }

    p_vertex = route.top();
    route.pop();
    (*fout)<<p_vertex<<endl;
    (*fout)<<"cost: "<<dist[e_vertex]<<endl; //print cost
	(*fout)<<"======================="<<endl<<endl;

    delete[] dist; //all memory release
    delete[] prev;
    for(int i=0; i<size; i++){
        delete[] length[i];
    }
    delete[] length;

    return true;
}

bool FLOYD(Graph* graph)
{
    ofstream* fout = graph->getStream();    //get ofstream to print log.txt
    int size = graph->getSize();
    int ** A = new int*[size]; //memory allocation
    for(int i=0; i<size; i++){
        A[i] = new int[size];
        for(int j=0; j<size; j++){
            A[i][j] = MAX; //setting initial by MAX
        }
    }

    for(int i=0; i<size; i++){ //setting A by all pair vertex
        map<int, int> m;
        graph->getAdjacentEdges(i, &m);
        for(auto it = m.begin(); it!=m.end(); it++){
            A[i][it->first] = it->second; //pair's weight
        }
    }

    for(int k = 0; k<size; k++){
        for(int i = 0; i<size; i++){
            for(int j = 0; j<size; j++){ //Calculating the Number of All Cases
                if(A[i][k] == MAX || A[k][j]==MAX) continue;
                A[i][j] = min(A[i][j], A[i][k]+A[k][j]); //save to A small cost weight from i to j
            }
        }
    }

    for(int i=0; i<size; i++){  //if that graph has nagative cost cycle
        if(A[i][i] < 0){ //If the weight that comes back to oneself is negative, nagative cycle is exist
            for(int j=0; j<size; j++){
                delete[] A[j];
            }
            delete[] A;
            return false; //return floyd fail
        }
    }

    (*fout)<<"======== FLOYD ========"<<endl;
    (*fout)<<'\t';
	for(int i=0; i<size; i++)
	{
		(*fout)<<"["<<i<<"]"<<'\t'; //frame print
	}
	(*fout)<<endl;

	for(int i=0; i<size; i++)
	{
		(*fout)<<"["<<i<<"]";
		for(int j=0; j<size && (*fout)<<'\t'; j++)
		{
			if(i == j)
                A[i][j] = 0; //if the path to oneself, cost is 0
            if(A[i][j] == MAX) //if path is not exist, print x
                (*fout)<<"x";
            else   
                (*fout)<<A[i][j]; //each cost print
		}
		(*fout)<<endl;
	}
	(*fout)<<"======================="<<endl<<endl;

    for(int i=0; i<size; i++){
        delete[] A[i];
    }
    delete[] A;

    return true;
}

void quickSort(vector<pair<int, pair<int, int> > >* edge_w_weigth, int low, int high){ //quick sort
    if(low < high){
        if(high-low+1 <= 6) //if segment is smaller than 6, change insertion sort
            insertSort(edge_w_weigth, low, high);
        else{
            int pivot = partition(edge_w_weigth, low, high); //split two part
            quickSort(edge_w_weigth, low, pivot-1); //recursive quick sort
            quickSort(edge_w_weigth, pivot+1, high);
        }
    }
}

int partition(vector<pair<int, pair<int, int> > >* edge_w_weigth, int low, int high){ //in quick sort, split two part
    int i=low;
    int j=high + 1;
    int pivot = (*edge_w_weigth)[low].first;
    do{
        do i++; while((*edge_w_weigth)[i].first < pivot);
        do j--; while((*edge_w_weigth)[j].first > pivot);
        if(i<j) swap((*edge_w_weigth)[i], (*edge_w_weigth)[j]); //to index j and i's value change
    } while(i<j);
    swap((*edge_w_weigth)[low], (*edge_w_weigth)[j]); //change pivot

    return j; //middle index
}

void insertSort(vector<pair<int, pair<int, int> > >* edge_w_weigth, int low, int high){ //insertion sort
    for(int i=low+1; i<high+1; i++){ //from low, to high
        pair<int, pair<int, int> > temp = (*edge_w_weigth)[i];
        int c = i-1;
        for(; low<=c; c--){
            if(temp.first < (*edge_w_weigth)[c].first){
                (*edge_w_weigth)[c+1] = (*edge_w_weigth)[c]; //push back
            }
            else break;
        }
        (*edge_w_weigth)[c+1] = temp; //fill the correct location
    }

}

void union_p(int* parent, int i, int j){ //union function with weight rule
    int temp = parent[i] + parent[j];
    if(parent[i] > parent[j]){ //i has fewer node
        parent[i] = j;
        parent[j] = temp;
    }
    else{ //j has fewer node than i or equal
        parent[j] = i;
        parent[i] = temp;
    }
}

int find_p(int* parent, int i){ //simple find
    int root = i;
    for(; parent[root] >= 0; root = parent[root]); //return i's root

    return root;
}

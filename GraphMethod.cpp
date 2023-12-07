#include "GraphMethod.h"
#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <utility>
#include <algorithm>
#include <climits>

using namespace std;

bool BFS(Graph* graph, char option, int vertex)
{
    ofstream fout;
    fout.open("log.txt", ios::app); 
    if(!fout)
        return false;

    int size = graph->getSize(); 
    bool isVisit[size + 1] = {0, }; 

    fout << "======== BFS ========" << endl;
	if (option == 'Y') 
		fout << "Directed Graph BFS result" << endl;
	else
		fout << "Unirected Graph BFS result" << endl;
	fout << "startvertex: " << vertex << endl;

    queue<int> bfs;
    bfs.push(vertex);
    isVisit[vertex] = true;
    fout << vertex;

    while(bfs.empty() == false) 
    {
        int num = bfs.front();
        bfs.pop();
        if(num != vertex)
            fout <<  " -> " << num;

        map<int, int>* m = new map<int, int>;
        if(option == 'Y')
            graph->getAdjacentEdgesDirect(num, m);
        else 
            graph->getAdjacentEdges(num, m);
        
        map<int, int>::iterator it;
        for(it = m->begin(); it != m->end(); it++)  {
            int newNum = it->first;
            if(isVisit[newNum] == false) {
                bfs.push(newNum);
                isVisit[newNum] = true;
            }
        }
    }
    fout << endl;
    fout << "=====================" << endl << endl;
    return true;
}

bool DFS(Graph* graph, char option, int vertex)
{
    ofstream fout;
    fout.open("log.txt", ios::app); //set output textfile
    if(!fout)
        return false;
    
    fout << "======== DFS ========" << endl;
	if (option == 'Y') 
		fout << "Directed Graph DFS result" << endl;
	else
		fout << "Unirected Graph DFS result" << endl;
	fout << "startvertex: " << vertex << endl;
   
    int size = graph->getSize();
    bool isVisit[size] = {0, };
    int start = vertex;
    fout << start;

    stack<int> dfs;
    dfs.push(start);
    isVisit[start] = true;

    while(dfs.empty() == false)
    {
        int num = dfs.top();
        dfs.pop();

        map<int, int>* m = new map<int, int>;
        if(option == 'Y')
            graph->getAdjacentEdgesDirect(num, m);
        else 
            graph->getAdjacentEdges(num, m);
        map<int, int>::iterator it;

        for(it = m->begin(); it != m->end(); it++) {
            int newNum = it->first;

            if(isVisit[newNum] == false) {
                fout <<  " -> " << newNum;
                isVisit[newNum] = true;

                dfs.push(num);
                dfs.push(newNum);
                break;
            }
        }
    }
    fout << endl;
    fout << "=====================" << endl << endl;
    return true;
}

bool Kruskal(Graph* graph)
{
    ofstream fout;
    fout.open("log.txt", ios::app); 
    if(!fout)
        return false;
    
    int size = graph->getSize();
    vector<pair<int, pair<int, int>>> weightEdge, mst;
    vector<int> parent(size + 1, -1);
    vector<int> rank(size + 1, 0);
    map<int, int>* m = new map<int, int>;
    map<int, int>::iterator it;

    for(int i = 1; i <= size+1; i++) {
        m->clear();
        graph->getAdjacentEdges(i, m);
        for(it = m->begin(); it != m->end(); it++) {
            if(i < it->first)
                weightEdge.push_back(make_pair(it->second, make_pair(i, it->first)));
        }
    }
    quicksort(weightEdge, 0, weightEdge.size() - 1, 6);

    int cost = 0;
    for (const auto& edge : weightEdge) {

        int weight = edge.first;
        int from = edge.second.first;
        int to = edge.second.second;

        int parentFrom = findRoot(parent, from);
        int parentTo = findRoot(parent, to);

        if (parentFrom != parentTo) {
            
            mst.push_back(edge);
            Union(parent, rank, parentFrom, parentTo);
            cost += weight;
        }
    }

    if(mst.size() < size - 1) 
        return false;
    
    vector< pair<int, pair<int, int> > > printEdge;
    for(auto it2 = mst.begin(); it2 != mst.end(); it2++) {
        printEdge.push_back(make_pair(it2->second.second, make_pair(it2->second.first, it2->first)));
        printEdge.push_back(make_pair(it2->second.first, make_pair(it2->second.second, it2->first)));
    }
    
    quicksort(printEdge, 0, printEdge.size() - 1, 6);
    fout << "====== Kruskal ======" << endl;
    for(int i = 1; i <= size; i++) {
        fout << "[" << i << "]\t";
        for(auto it2 = printEdge.begin(); it2 != printEdge.end(); it2++) {
            if(it2->second.first == i) 
                fout << it2->first << "(" << it2->second.second << ") ";
        }
        fout << endl;
    }
    fout << "cost: " << cost << endl;
    fout << "=====================" << endl << endl;
    return true;
}

bool Dijkstra(Graph* graph, char option, int vertex)
{
    ofstream fout;
    fout.open("log.txt", ios::app);
    if(!fout)
        return false;

    int size = graph->getSize();
    vector<int> distance(size + 1, INT_MAX);
    vector<bool> visited(size + 1, false);
    vector<int> path(size + 1, -1);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> dij;
    distance[vertex] = 0;
    dij.push({0, vertex});

    while(!dij.empty()) {

        int dist = dij.top().first;
        int curVertex = dij.top().second;
        dij.pop();

        if(visited[curVertex])
            continue;

        map<int, int>* m = new map<int, int>;
        if(option == 'Y')
            graph->getAdjacentEdgesDirect(curVertex, m);
        else 
            graph->getAdjacentEdges(curVertex, m);

        for (auto it = m->begin(); it != m->end(); ++it) {
            
            int nextVertex = it->first;
            int edgeWeight = it->second;

            if(edgeWeight < 0)
                return false;

            if ((distance[curVertex] != INT_MAX) && (distance[nextVertex] > distance[curVertex]+edgeWeight)) {
                distance[nextVertex] = distance[curVertex] + edgeWeight;
                dij.push({distance[nextVertex], nextVertex});
                path[nextVertex] = curVertex;
            }
        }
        visited[curVertex] = true;
    }

    fout << "====== Dijkstra ======" << endl;
    if (option == 'Y') 
		fout << "Directed Graph Dijkstra result" << endl;
	else
		fout << "Unirected Graph Dijkstra result" << endl;
	fout << "startvertex: " << vertex << endl;
    for(int i = 1; i <= size; i++) {
        
        if(i != vertex) {

            stack<int> printPath;       
            fout << "[" << i << "] ";
            if(distance[i] == INT_MAX) {
               fout << "x" << endl;
               continue;
            }
            else {

                int curNode = i;
                while (curNode != -1) {
                    printPath.push(curNode);
                    curNode = path[curNode];
                }

                while (!printPath.empty()) {
                    fout << printPath.top();
                    printPath.pop();
                    if (!printPath.empty())
                        fout << " -> ";
                }
                fout << " (" << distance[i] << ")" << endl;                
            }
        }
    }
    fout << "=====================" << endl << endl;
    return true;
}

bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex) 
{
    ofstream fout;
    fout.open("log.txt", ios::app);
    if(!fout)
        return false;

    int size = graph->getSize();
}

bool FLOYD(Graph* graph, char option)
{
    ofstream fout;
    fout.open("log.txt", ios::app);
    if(!fout)
        return false;

    int size = graph->getSize();
    int dist[size+1][size+1];
    for (int i = 0; i <= size; ++i) {
        for (int j = 0; j <= size; ++j)
            dist[i][j] = INT_MAX; 
    }

    for(int i = 1; i <= size; i++) {

        map<int, int>* m = new map<int, int>;
        if(option == 'Y')
            graph->getAdjacentEdgesDirect(i, m);
        else 
            graph->getAdjacentEdges(i, m);
        
        for(auto it = m->begin(); it!=m->end(); it++)
            dist[i][it->first] = it->second;
    }

    for (int k = 1; k <= size; k++) {
        for (int i = 1; i <= size; i++) {
            for (int j = 1; j <= size; j++) {
                if ((dist[i][k] != INT_MAX) && (dist[k][j] != INT_MAX) && (dist[i][j] > dist[i][k] + dist[k][j])) 
                    dist[i][j] = dist[i][k] + dist[k][j];
            }
        }
    }

    for(int i = 1; i <= size; i++) {
        if(dist[i][i] < 0) 
            return false; 
        else
            dist[i][i] = 0;
    }
    
    fout << "======== FLOYD ========" << endl;
    if (option == 'Y') 
		fout << "Directed Graph FLOYD result" << endl;
	else
		fout << "Unirected Graph FLOYD result" << endl;
    fout << '\t';

	for(int i = 1; i <= size; i++)
		fout << "[" << i << "]" << '\t';
	fout << endl;

	for(int i = 1; i <= size; i++) {

		fout << "[" << i << "] ";
		for(int j = 1; j <= size && fout << '\t'; j++) {
            if(dist[i][j] == INT_MAX)
                fout << "x";
			else
                fout << dist[i][j];
        }
		fout << endl;
	}
	fout << "=======================" << endl << endl;
    return true;
}

bool KWANGWOON(Graph* graph, int vertex) {

}

void quicksort(vector<pair<int, pair<int, int>>>& arr, int low, int high, int segment_size) {
    
    if (low < high) {      
        if (high - low + 1 <= segment_size) {
            insertionSort(arr, low, high);
        } 
        else {
            int pivot = partition(arr, low, high);
            quicksort(arr, low, pivot - 1, segment_size);
            quicksort(arr, pivot + 1, high, segment_size);
        }
    }
}

int partition(vector<pair<int, pair<int, int>>>& arr, int low, int high) {

    pair<int, pair<int, int>> pivot = arr[(low + high) / 2];
    int i = low - 1;
    int j = high + 1;
    while (true) {
        do {
            ++i;
        } while (arr[i] < pivot);

        do {
            --j;
        } while (arr[j] > pivot);

        if (i >= j) {
            return j;
        }
        swap(arr[i], arr[j]);
    }
}

void insertionSort(vector<pair<int, pair<int, int>>>& arr, int low, int high) {
    
    for (int i = low + 1; i <= high; ++i) {    
        pair<int, pair<int, int>> key = arr[i];
        int j = i - 1;
        while (j >= low && arr[j] > key) {
            arr[j + 1] = arr[j];
            --j;
        }
        arr[j + 1] = key;
    }
}

int findRoot(vector<int>& parent, int i) {
    
    if (parent[i] == -1)
        return i;
    return findRoot(parent, parent[i]);
}

void Union(vector<int>& parent, vector<int>& rank, int x, int y) {
    
    int xRoot = findRoot(parent, x);
    int yRoot = findRoot(parent, y);

    if (rank[xRoot] < rank[yRoot])
        parent[xRoot] = yRoot;
    else if (rank[xRoot] > rank[yRoot])
        parent[yRoot] = xRoot;
    else {
        parent[yRoot] = xRoot;
        rank[xRoot]++;
    }
}
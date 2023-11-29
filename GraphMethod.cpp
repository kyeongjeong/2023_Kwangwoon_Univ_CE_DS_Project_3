#include <iostream>
#include <vector>
#include "GraphMethod.h"
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <utility>

using namespace std;

bool BFS(Graph* graph, char option, int vertex)
{
    ofstream fout;
    fout.open("log.txt", ios::app); 
    if(!fout)
        return false;

    int size = graph->getSize(); 
    bool isVisit[size + 1] = {0, }; 

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
    return true;
}

bool DFS(Graph* graph, char option, int vertex)
{
    ofstream fout;
    fout.open("log.txt", ios::app); //set output textfile
    
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
    return true;
}

bool Kruskal(Graph* graph)
{
   
}

bool Dijkstra(Graph* graph, char option, int vertex)
{
   
}

bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex) 
{
   
}

bool FLOYD(Graph* graph, char option)
{
   
}

bool KWANGWOON(Graph* graph, int vertex) {

}
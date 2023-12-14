#ifndef _GRAPHMETHOD_H_
#define _GRAPHMETHOD_H_

#include "ListGraph.h"
#include "MatrixGraph.h"

bool BFS(Graph* graph, char option, int vertex);     
bool DFS(Graph* graph, char option,  int vertex);     
bool KWANGWOON(Graph* graph, int vertex);  
bool Kruskal(Graph* graph);
bool Dijkstra(Graph* graph, char option, int vertex);    //Dijkstra
bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex); //Bellman - Ford
bool FLOYD(Graph* graph, char option);   //FLoyd

void quicksort(vector<pair<int, pair<int, int>>>& arr, int low, int high, int segment_size);
int partition(vector<pair<int, pair<int, int>>>& arr, int low, int high);
void insertionSort(vector<pair<int, pair<int, int>>>& arr, int low, int high);
int findRoot(vector<int>& parent, int i);
void Union(vector<int>& parent, vector<int>& rank, int x, int y);
int init(int node, int start, int end, vector<int> &_arr, vector<int> &_seg);
void update(int node, int start, int end, int target, int diff_value, vector<int>& _arr, vector<int>& _seg);
int sum(int node, int start, int end, int left, int right, vector<int>& _arr, vector<int>& _seg);

#endif

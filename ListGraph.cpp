#include "ListGraph.h"
#include <iostream>
#include <utility>

ListGraph::ListGraph(bool type, int size) : Graph(type, size)
{
    m_Type = type;
	m_List = new map<int, int>[size + 1];
}

ListGraph::~ListGraph()	
{
	
}

void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m)	 //Definition of getAdjacentEdges(No Direction == Undirected)
{
	map<int, int>::iterator it = m_List[vertex].begin(); //iterator

	for(int i = 0;  i < m_Size; i++) {
		it = m_List[i].begin();
		for(; it != m_List[i].end(); it++) {
			if(it->first == vertex)
				m->insert(pair<int, int>(i, it->second));

			if(i == vertex)
				m->insert(pair<int, int>(it->first, it->second));
		}
	}
}

void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)	//Definition of getAdjacentEdges(Directed graph)
{
	
}

void ListGraph::insertEdge(int from, int to, int weight) //Definition of insertEdge
{
	m_List[from].insert(pair<int, int>(to, weight));
}

bool ListGraph::printGraph(ofstream *fout)	//Definition of print Graph
{
	*fout << "========PRINT========" << endl;
	
	for(int i = 1; i <= m_Size; i++) {
		
		*fout << "[" << i << "]";

		for(auto it_ = m_List[i].begin(); it_ != m_List[i].end() && *fout << " -> "; it_++)
			*fout << "("<<it_->first << "," << it_->second<<")";
		*fout << endl;
	}
	*fout << "=====================" << endl << endl;
	return true;
}
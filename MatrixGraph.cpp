#include "MatrixGraph.h"
#include <iostream>
#include <vector>
#include <string>

MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size)
{
    m_Mat = new int*[size + 1];
	for(int i = 1; i < size + 1; i++) {
		m_Mat[i] = new int[size + 1];
		memset(m_Mat[i], 0, sizeof(int)*(size + 1));
	}
}

MatrixGraph::~MatrixGraph()
{

}

void MatrixGraph::getAdjacentEdges(int vertex, map<int, int>* m)
{	

}

void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)
{
	
}

void MatrixGraph::insertEdge(int from, int to, int weight)	
{
	m_Mat[from][to] = weight;
}

bool MatrixGraph::printGraph(ofstream *fout)	
{
	*fout << "========PRINT========" << endl;
	*fout << '\t';

	for(int i = 1; i <= m_Size; i++)
		*fout << "[" << i << "]" << '\t';
	*fout << endl;

	for(int i = 1; i <= m_Size; i++) {

		*fout << "[" << i << "] ";

		for(int j = 1; j <= m_Size && *fout << '\t'; j++)
			*fout << m_Mat[i][j];
		*fout << endl;
	}
	*fout << "=====================" << endl << endl;
	return true;
}

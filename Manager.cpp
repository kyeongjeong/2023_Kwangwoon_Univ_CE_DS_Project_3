#include "Manager.h"
#include "GraphMethod.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

Manager::Manager()	
{
	graph = nullptr;	
	fout.open("log.txt", ios::app);
	load = 0;	//Anything is not loaded
}

Manager::~Manager()
{
	if(load)	//if graph is loaded, delete graph
		delete graph;	
	if(fout.is_open())	//if fout is opened, close file
		fout.close();	//close log.txt File
}

void Manager::run(const char* command_txt){
	ifstream fin;	//Command File Input File Stream
	fin.open(command_txt, ios_base::in);//File open with read mode
		
	if(!fin) { //If command File cannot be read, Print error
		fout<<"command file open error"<<endl;
		return;	//Return
	}

	while (!fin.eof()) {

		string cmd, line;
		getline(fin, line);
		if (!line.empty() && line.back() == '\r') // if you encounter '/r', ignore it
			line.pop_back();

		vector<string> tokens;
		char* token = strtok(&line[0], " "); 

		while (token != NULL) { // read in units of '\t' separation
			tokens.push_back(token);
			token = strtok(NULL, " ");
		}
		cmd = tokens[0];
		
		if (cmd == "LOAD") { // if command is LOAD

			string filename;
			if(tokens.size() == 2) {
				filename = tokens[1];

				if(!LOAD(filename))
					printErrorCode(100);
			}
			else
				printErrorCode(100);
		}

		else if (cmd == "PRINT") { // if command is PRINT
			if(tokens.size() != 1 || !PRINT())
				printErrorCode(200);
		}

		else if (cmd == "BFS") { // if command is BFS

			char option;
			int vertex;
			if(tokens.size() == 3) {

				option = tokens[1][0];
				vertex = stoi(tokens[2]);
				if(!mBFS(option, vertex))
					printErrorCode(300);
			}
			else
				printErrorCode(300);
		}

		else if (cmd == "DFS") { // if command is DFS

			char option;
			int vertex;
			if(tokens.size() == 3) {

				option = tokens[1][0];
				vertex = stoi(tokens[2]);
				if(!mDFS(option, vertex))
					printErrorCode(400);
			}
			else
				printErrorCode(400);
		}

		else if(cmd == "KWANGWOON") { // if command is KWANGWOON
			if(tokens.size() != 1 || !mKwoonWoon(1))
				printErrorCode(500);
		}

		else if (cmd == "KRUSKAL") { // if command is KRUSKAL
			if(tokens.size() != 1 || !mKRUSKAL())
				printErrorCode(600);
		}

		else if (cmd == "DIJKSTRA") { // if command is DIJKSTRA
			char option;
			int vertex;
			if(tokens.size() == 3) {

				option = tokens[1][0];
				vertex = stoi(tokens[2]);
				if(!mDIJKSTRA(option, vertex))
					printErrorCode(700);
			}
			else
				printErrorCode(700);
		}

		else if(cmd == "BELLMANFORD") { // if command is BELLMANFORD

			char option;
			int s_vertex, e_vertex;
			if(tokens.size() == 4) {
				
				option = tokens[1][0];
				s_vertex = stoi(tokens[2]);
				e_vertex = stoi(tokens[3]);
				if(!mBELLMANFORD(option, s_vertex, e_vertex))
					printErrorCode(800);
			}
			else
				printErrorCode(800);
		}

		else if (cmd == "FLOYD") { // if command is FLOYD
			char option;
			if(tokens.size() == 2) {

				option = tokens[1][0];
				if(!mFLOYD(option))
					printErrorCode(900);
			}
			else
				printErrorCode(900);
		}

		else if (cmd == "EXIT") { // if command is EXIT	
			fout << "========EXIT========" << endl;
			fout << "Success" << endl;
			fout << "====================" << endl << endl;
			break;
		}

		else 
			printErrorCode(1000);
	}
	fin.close();
	return;
}

bool Manager::LOAD(string filename)
{
	ifstream fgraph;
	fgraph.open(filename);
	if(!fgraph)
		return false;

	if(graph != nullptr) {
		delete graph;
		graph = nullptr;
	}

	int from, to, weight, size;
	char type, *token;
	string line;

	fgraph >> type >> size;
	getline(fgraph, line);

	if (type == 'L') {

		graph = new ListGraph(type, size);
		while (!fgraph.eof()) {
			
			vector<string> tokens;
			getline(fgraph, line);
			token = strtok(&line[0], " "); 

			while (token != NULL) {
				tokens.push_back(token);
				token = strtok(NULL, " ");
			}

			if (tokens.size() == 1) {
				from = stoi(tokens[0]);
			}
			else if (tokens.size() == 2) {
				to = stoi(tokens[0]);
				weight = stoi(tokens[1]);
				graph->insertEdge(from, to, weight);
			}
			else 
				return false;
		}
	}
	else if (type == 'M') {

		graph = new MatrixGraph(type, size);
		from = 0;
		while (!fgraph.eof()) {

			for(to = 1; to <= size; to++) {
				int num;
				fgraph >> num;
				if(num != 0)
					graph->insertEdge(from + 1, to, num);
			}
			from++;
		}
	}
	else 
		return false;

	fout << "========LOAD========" << endl;
	fout << "Success" << endl;
	fout << "====================" << endl << endl;
	return true;
}

bool Manager::PRINT()	
{
	if(graph == nullptr) // Check if the graph is null
		return false;
	
	graph->printGraph(&fout);
	return true;
}

bool Manager::mBFS(char option, int vertex)	
{
	if((graph == nullptr) || (vertex < 1) || (vertex > graph->getSize())) // Check if the graph is null or the vertex is out of bounds
		return false;
    if(option != 'Y' && option != 'N') // Check if the option is valid
        return false;

	return BFS(graph, option, vertex);
}

bool Manager::mDFS(char option, int vertex)	
{
	if((graph == nullptr) || (vertex < 1) || (vertex > graph->getSize())) // Check if the graph is null or the vertex is out of bounds
		return false;
    if(option != 'Y' && option != 'N') // Check if the option is valid
        return false;

	return DFS(graph, option, vertex);
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	if((graph == nullptr) || (vertex < 1) || (vertex > graph->getSize())) // Check if the graph is null or the vertex is out of bounds
		return false;
    if(option != 'Y' && option != 'N') // Check if the option is valid
        return false;

	return Dijkstra(graph, option, vertex);
}

bool Manager::mKRUSKAL()
{
 	if(graph == nullptr) // Check if the graph is null
		return false;
	
	return Kruskal(graph);
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	// Check if the graph is null or the vertex is out of bounds
	if((graph == nullptr) || (s_vertex < 1) || (s_vertex > graph->getSize()) || (e_vertex < 1) || (e_vertex > graph->getSize()))
		return false;
	if(option != 'Y' && option != 'N') // Check if the option is valid
        return false;

	return Bellmanford(graph, option, s_vertex, e_vertex);
}

bool Manager::mFLOYD(char option)
{
	if(graph == nullptr) // Check if the graph is null
		return false;
	if(option != 'Y' && option != 'N') // Check if the option is valid
        return false;

	return FLOYD(graph, option);
}

bool Manager::mKwoonWoon(int vertex) {
	
	if(graph == nullptr) // Check if the graph is null
		return false;

	return KWANGWOON(graph, 1);
}

void Manager::printErrorCode(int n)
{
	fout<<"========ERROR======="<<endl;
	fout<<n<<endl;
	fout<<"===================="<<endl << endl;
}

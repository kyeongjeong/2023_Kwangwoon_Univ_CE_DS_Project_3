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
		
		if (cmd == "LOAD") {

			string filename;
			if(tokens.size() == 2) {
				filename = tokens[1];

				if(!LOAD(filename))
					printErrorCode(100);
			}
			else
				printErrorCode(100);
		}

		else if (cmd == "PRINT") {
			if(tokens.size() != 1 || !PRINT())
				printErrorCode(200);
		}

		else if (cmd == "BFS") {

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

		else if (cmd == "DFS") {

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
		else {
			printErrorCode(1000);
			break;
		}
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
	if(graph == nullptr) 
		return false;
	
	graph->printGraph(&fout);
	return true;
}

bool Manager::mBFS(char option, int vertex)	
{
	if((graph == nullptr) || (vertex < 1)) 
		return false;
    if(option != 'Y' && option != 'N')
        return false;
	
	fout << "========BFS========" << endl;
	if (option == 'Y') 
		fout << "Directed Graph BFS result" << endl;
	else
		fout << "Unirected Graph BFS result" << endl;
	fout << "startvertex: " << vertex << endl;
	
	BFS(graph, option, vertex);
	fout << "===================" << endl << endl;
	return true;
}

bool Manager::mDFS(char option, int vertex)	
{
	if((graph == nullptr) || (vertex < 1)) 
		return false;
    if(option != 'Y' && option != 'N')
        return false;
	
	fout << "========DFS========" << endl;
	if (option == 'Y') 
		fout << "Directed Graph DFS result" << endl;
	else
		fout << "Unirected Graph DFS result" << endl;
	fout << "startvertex: " << vertex << endl;
	
	DFS(graph, option, vertex);
	fout << "===================" << endl << endl;
	return true;
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	
}

bool Manager::mKRUSKAL()
{
 	
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	
}

bool Manager::mFLOYD(char option)
{
	
}

bool Manager::mKwoonWoon(int vertex) {
	
}

void Manager::printErrorCode(int n)
{
	fout<<"========ERROR======="<<endl;
	fout<<n<<endl;
	fout<<"===================="<<endl << endl;
}



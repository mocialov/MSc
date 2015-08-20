#include "graph.h"
 
// Adds an edge to an undirected graph
void addEdge(struct Graph* graph, int src, int dest, float weight)
{
    // Add an edge from src to dest.  A new node is added to the adjacency
    // list of src.  The node is added at the begining
    struct AdjListNode* newNode = newAdjListNode(dest, weight);
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;
 
    // Since graph is undirected, add an edge from dest to src also
    newNode = newAdjListNode(src, weight);
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;
}


// A utility function to print the adjacenncy list representation of graph
void printGraph(struct Graph* graph)
{
    int v;
    for (v = 0; v < graph->V; ++v)
    {
        struct AdjListNode* pCrawl = graph->array[v].head;
        printf("\n Adjacency list of vertex %d\n head ", v);
        while (pCrawl)
        {
            printf("-> %d (%f)", pCrawl->dest, pCrawl->weight);
            pCrawl = pCrawl->next;
        }
        printf("\n");
    }
}
 
// Driver program to test above functions
int main()
{
	// create the graph given in above fugure
	int V = 5;
	struct Graph* graph = createGraph(V);
   	addEdge(graph, 0, 1, -1);
   	addEdge(graph, 0, 4, -1);
   	addEdge(graph, 1, 2, -1);
   	addEdge(graph, 1, 3, -1);
   	addEdge(graph, 1, 4, -1);
   	addEdge(graph, 2, 3, -1);
   	addEdge(graph, 3, 4, -1);

	// print the adjacency list representation of the above graph
   	printGraph(graph);
	
   	return 0;
}
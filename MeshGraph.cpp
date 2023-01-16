#include "MeshGraph.h"
#include "BinaryHeap.h"

// For printing
#include <fstream>
#include <iostream>
#include <sstream>


MeshGraph::MeshGraph(const std::vector<Double3>& vertexPositions,
                     const std::vector<IdPair>& edges)
{   
    int v1size = vertexPositions.size();
    edgeCount = edges.size();
    adjList.resize(v1size);
    vertices.resize(v1size);
    for (int i = 0; i < v1size; i++)
    {
        Vertex newVertex;
        newVertex.id = i;
        newVertex.position3D = vertexPositions[i];
        vertices[i] = newVertex;
    }

    for (int i = 0; i < edgeCount ; i++)
    {
        adjList[edges[i].vertexId0].push_back(&vertices[edges[i].vertexId1]);
        adjList[edges[i].vertexId1].push_back(&vertices[edges[i].vertexId0]);
    }
}

double MeshGraph::AverageDistanceBetweenVertices() const
{

    if(edgeCount == 0){
        return 0;
    }
    int size = adjList.size();
    double sum = 0;
    for (int i = 0; i < size; i++)
    {   
        std::list<Vertex* >::const_iterator iterator = adjList[i].begin();
        while (iterator != adjList[i].end())
        {
            sum += vertices[i].position3D.Distance( (*iterator)->position3D, vertices[i].position3D);
            iterator++;
        }
        
    }
    sum = sum/(2*edgeCount);
    return sum;
    
}

double MeshGraph::AverageEdgePerVertex() const
{
    return (double)TotalEdgeCount()/TotalVertexCount();
}

int MeshGraph::TotalVertexCount() const
{
    return vertices.size();
}

int MeshGraph::TotalEdgeCount() const
{
    return edgeCount;
}

int MeshGraph::VertexEdgeCount(int vertexId) const
{

    if(vertexId >= vertices.size()) return -1 ;

    int edgeNumber = 0;


    for (std::list<Vertex* >::const_iterator iterator = adjList[vertexId].begin() ; iterator != adjList[vertexId].end(); iterator++){
        edgeNumber ++;
    }
   
    return edgeNumber;
}

void MeshGraph::ImmediateNeighbours(std::vector<int>& outVertexIds,
                                    int vertexId) const
{
    if(vertexId >= vertices.size()) return;

    for (std::list<Vertex* >::const_iterator iterator = adjList[vertexId].begin() ; iterator != adjList[vertexId].end(); iterator++){
        outVertexIds.push_back((*iterator)->id);
    }
}

void MeshGraph::PaintInBetweenVertex(std::vector<Color>& outputColorAllVertex,
                                     int vertexIdFrom, int vertexIdTo,
                                     const Color& color) const
{   
    int verticesSize = vertices.size();
    if( vertexIdFrom > verticesSize || vertexIdTo > verticesSize || 
        vertexIdFrom < 0            || vertexIdTo < 0){
        outputColorAllVertex.resize(0);
        return;
    }
    outputColorAllVertex.resize(verticesSize);

    std::vector<double> distances;
    std::vector<int> prev;
    BinaryHeap pqueue;

    distances.resize(verticesSize);
    prev.resize(verticesSize);

    for (int i = 0; i < verticesSize; i++)
    {
        distances[i]  = INFINITY;
        prev[i] = -1;
        pqueue.Add(i, INFINITY);
    }
    distances[vertexIdFrom] = 0;
    pqueue.ChangePriority(vertexIdFrom, 0);
    int poppedID;
    double poppedWeight;

    while (pqueue.HeapSize() > 0)
    {
        pqueue.PopHeap(poppedID, poppedWeight);

        std::vector<int> neighbors;
        ImmediateNeighbours(neighbors, poppedID);

        for (int i = 0; i < neighbors.size(); i++)
        {   
            int neighborID = neighbors[i];
            double newDistance = distances[poppedID] + Double3::Distance(vertices[poppedID].position3D, vertices[neighborID].position3D );

            if(distances[neighborID] > newDistance){
                
                pqueue.ChangePriority(neighborID, newDistance);
                prev[neighborID] = poppedID;
                distances[neighborID] = newDistance;
            }
        }
    }
    int trackVertex = vertexIdTo;

    while(trackVertex != -1){

        outputColorAllVertex[trackVertex] = color;
        trackVertex = prev[trackVertex];
    }


}

void MeshGraph::PaintInRangeGeodesic(std::vector<Color>& outputColorAllVertex,
                                    int vertexId, const Color& color,
                                    int maxDepth, FilterType type,
                                    double alpha) const
{
    int verticesSize = vertices.size();
    if( vertexId > verticesSize || vertexId < 0){
        outputColorAllVertex.resize(0);
        return;
    }
    
    std::vector<double> distances;
    distances.resize(verticesSize);
    std::vector<bool> marked;
    marked.resize(verticesSize); 
    std::vector<int> depths;
    depths.resize(verticesSize);

    int outID;
    double outWeight;
    int strictWeight = 0 ;
    int depth = 0;

    outputColorAllVertex.resize(verticesSize);
    
    for (int i = 0; i < verticesSize; i++)
    {   
        marked[i]  = false;
        distances[i] = 0;
    }
    
    BinaryHeap queue;
    queue.Add(vertexId, strictWeight);
    strictWeight++;
    marked[vertexId] = true;
    depths[vertexId] = depth;
    
    while (queue.HeapSize() > 0)
    {
        queue.PopHeap(outID, outWeight);
        
        std::vector<int> neighbors;
        ImmediateNeighbours(neighbors, outID);

        int neighbourSize = neighbors.size();

                                            //sorting:
                                            bool swapped = true;
                                            while(swapped)
                                            {
                                                swapped = false;
                                                for (int i = 0; i <  neighbourSize-1; i++)
                                                {
                                                    if (neighbors[i]>neighbors[i+1] )
                                                    {
                                                        neighbors[i] += neighbors[i+1];
                                                        neighbors[i+1] = neighbors[i] - neighbors[i+1];
                                                        neighbors[i] -=neighbors[i+1];
                                                        swapped = true;
                                                    }
                                                }   
                                            }           
                                            //sorted.
        
        

        for (int i = 0; i < neighbourSize; i++)
        {   
            int neighborID = neighbors[i];

            if(marked[neighborID]) continue;

            double distance = distances[outID] + Double3::Distance(vertices[outID].position3D, vertices[neighborID].position3D );
            queue.Add(neighborID, strictWeight);
            strictWeight++;
            marked[neighborID] = true;
            distances[neighborID] = distance;
            depths[neighborID] = depths[outID]+1;
        }
    }

    for (int i = 0; i < verticesSize; i++)
    {       

        if ( depths[i] <= maxDepth){
            
            if(type == FILTER_GAUSSIAN){
                double filterValue = exp(-((distances[i]*distances[i])/(alpha*alpha)));
                outputColorAllVertex[i].r = (unsigned char)(color.r * filterValue);
                outputColorAllVertex[i].g = (unsigned char)(color.g * filterValue);
                outputColorAllVertex[i].b = (unsigned char)(color.b * filterValue);
            }
            else if(type == FILTER_BOX){
                double filterValue = (distances[i]< alpha && distances[i] > -alpha) ? 1 : 0;
                outputColorAllVertex[i].r = (unsigned char)(color.r * filterValue);
                outputColorAllVertex[i].g = (unsigned char)(color.g * filterValue);
                outputColorAllVertex[i].b = (unsigned char)(color.b * filterValue);
            }
        }   
    }

}

void MeshGraph::PaintInRangeEuclidian(std::vector<Color>& outputColorAllVertex,
                                      int vertexId, const Color& color,
                                      int maxDepth, FilterType type,
                                      double alpha) const
{
    int verticesSize = vertices.size();
    if( vertexId > verticesSize || vertexId < 0){
        outputColorAllVertex.resize(0);
        return;
    }
    std::vector<double> distances;
    distances.resize(verticesSize);
    std::vector<bool> marked;
    marked.resize(verticesSize); 
    std::vector<int> depths;
    depths.resize(verticesSize);

    int outID;
    double outWeight;
    int strictWeight = 0 ;
    int depth = 0;

    outputColorAllVertex.resize(verticesSize);
    
    for (int i = 0; i < verticesSize; i++)
    {   
        marked[i]  = false;
    }
    
    BinaryHeap queue;
    queue.Add(vertexId, strictWeight);
    strictWeight++;
    marked[vertexId] = true;
    depths[vertexId] = depth;
    
    while (queue.HeapSize() > 0)
    {
        queue.PopHeap(outID, outWeight);
        
        std::vector<int> neighbors;
        ImmediateNeighbours(neighbors, outID);

        int neighbourSize = neighbors.size();

                                            //sorting:
                                            bool swapped = true;
                                            while(swapped)
                                            {
                                                swapped = false;
                                                for (int i = 0; i <  neighbourSize-1; i++)
                                                {
                                                    if (neighbors[i]>neighbors[i+1] )
                                                    {
                                                        neighbors[i] += neighbors[i+1];
                                                        neighbors[i+1] = neighbors[i] - neighbors[i+1];
                                                        neighbors[i] -=neighbors[i+1];
                                                        swapped = true;
                                                    }
                                                }   
                                            }           
                                            //sorted.
        
        

        for (int i = 0; i < neighbourSize; i++)
        {   
            int neighborID = neighbors[i];

            if(marked[neighborID]) continue;

            double distance = Double3::Distance(vertices[vertexId].position3D, vertices[neighborID].position3D );
            queue.Add(neighborID, strictWeight);
            strictWeight++;
            marked[neighborID] = true;
            distances[neighborID] = distance;
            depths[neighborID] = depths[outID]+1;
        }
    }

    for (int i = 0; i < verticesSize; i++)
    {       

        if ( depths[i] <= maxDepth){
            
            if(type == FILTER_GAUSSIAN){
                double filterValue = exp(-((distances[i]*distances[i])/(alpha*alpha)));
                outputColorAllVertex[i].r = (unsigned char)(color.r * filterValue);
                outputColorAllVertex[i].g = (unsigned char)(color.g * filterValue);
                outputColorAllVertex[i].b = (unsigned char)(color.b * filterValue);
            }
            else if(type == FILTER_BOX){
                double filterValue = (distances[i]< alpha && distances[i] > -alpha) ? 1 : 0;
                outputColorAllVertex[i].r = (unsigned char)(color.r * filterValue);
                outputColorAllVertex[i].g = (unsigned char)(color.g * filterValue);
                outputColorAllVertex[i].b = (unsigned char)(color.b * filterValue);
            }
        }   
    }

}

void MeshGraph::WriteColorToFile(const std::vector<Color>& colors,
                                 const std::string& fileName)
{
    // IMPLEMENTED
    std::stringstream s;
    for(int i = 0; i < static_cast<int>(colors.size()); i++)
    {
        int r = static_cast<int>(colors[i].r);
        int g = static_cast<int>(colors[i].g);
        int b = static_cast<int>(colors[i].b);

        s << r << ", " << g << ", " << b << "\n";
    }
    std::ofstream f(fileName.c_str());
    f << s.str();
}

void MeshGraph::PrintColorToStdOut(const std::vector<Color>& colors)
{
    // IMPLEMENTED
    for(int i = 0; i < static_cast<int>(colors.size()); i++)
    {
        std::cout << static_cast<int>(colors[i].r) << ", "
                  << static_cast<int>(colors[i].g) << ", "
                  << static_cast<int>(colors[i].b) << "\n";
    }
}


// ---------------- HELPER -----------------


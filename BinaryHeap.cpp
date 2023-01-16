#include "BinaryHeap.h"

BinaryHeap::BinaryHeap()
{
    // TODO: or not
    elements.resize(1);
    size = 0;
}


bool BinaryHeap::Add(int uniqueId, double weight)
{
    // TODO:

    for(int i = 1; i <= size; i++){
        
        if(elements[i].uniqueId == uniqueId){
            return false;
        }
    }



    int hole = ++size;

    if (size == elements.size()) {
      elements.resize( size * 2 );
    }

    

    while(hole > 1 && (elements[hole/2].weight >= weight)) {
        elements[hole] = elements[hole/2];
        hole = hole/2;
    }

    HeapElement newNode;
    newNode.uniqueId= uniqueId;
    newNode.weight = weight;

    elements[hole] = newNode;

    return true;
}

bool BinaryHeap::PopHeap(int& outUniqueId, double& outWeight)
{
    // 
    if(size == 0) return false;

    // take the Data
    outUniqueId = elements[1].uniqueId;
    outWeight = elements[1].weight;
    // swap first and last element
    elements[1] = elements[size];
    // pop last element
    elements[size].uniqueId = 0;
    elements[size].weight = 0;
    size--;
    bubbleDown(1);

    return true;

}

bool BinaryHeap::ChangePriority(int uniqueId, double newWeight)
{
    int exists = 0;
    for(int i = 1 ; i <= size ; i++ ){

        if(elements[i].uniqueId == uniqueId){
            elements[i].weight = newWeight;
            exists = 1;
            break;
        }
    }

    if( !exists ) return false;

    for(int i = size/2; i > 0; i-- ){
        bubbleDown(i);
    }

    return true;
}

int BinaryHeap::HeapSize() const
{
    // TODO:
    return size;
}

void BinaryHeap::bubbleDown(int pos) 
{   
    int child; 
    HeapElement temp = elements[pos];
    
    for( ;pos <= size/2; pos = child){

        child= pos* 2;
        if(child != size && elements[child + 1].weight < elements[child].weight){
            child++;
        }
        if(elements[child].weight < temp.weight){
            elements[pos] = elements[child];
        }
        else break;
    }
    elements[pos] = temp;
}

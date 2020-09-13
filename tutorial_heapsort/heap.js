/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    var eIntIdx = heap.length;
    var prntIdx = Math.floor((eIntIdx - 1 ) / 2);
    heap.push(new_element);
    var heaped = (eIntIdx <= 0) || (heap[prntIdx] <= heap[eIntIdx]);

    while(!heaped){
        var tmp = heap[prntIdx];
        heap[prntIdx] = heap[eIntIdx];
        heap[eIntIdx] = tmp;

        eIntIdx = prntIdx;
        prntIdx = Math.floor((eIntIdx - 1)/2);
        heaped = (eIntIdx <= 0) ||(heap[prntIdx] <= heap[eIntIdx]);
    }
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    var temp;
    temp = heap[0];
    heap[0] = heap[heap.length - 1];
    heap[heap.length - 1] = temp;
    heap.pop();
    var inde = 0;
    
    while(2 * inde + 1< heap.length){
        var j = 2 * inde + 1;
        if(j + 1 >= heap.length){
            if(heap[j] < heap[inde]){
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
            }
            break;
        }
        else{
            if((heap[j] > heap[j + 1]) && (heap[j] < heap[inde])){
                j++
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if((heap[j] > heap[j + 1]) && (heap[j + 1] < heap[inde])){
                j++;
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if((heap[j + 1] < heap[inde])){
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if(heap[j] < heap[inde]){
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else{
                break;
            }  
        }
    }
    
    return temp;
    


    // STENCIL: implement your min binary heap extract operation
}
minheaper.extract = minheap_extract;
// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object







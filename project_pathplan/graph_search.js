/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
        }
    }
    G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].distance = 0;
    G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].priority = Math.sqrt((G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].x)
     * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].x) 
     + (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].y) 
     * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].y));
     G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps].queued = true;
    pq_insert(visit_queue, G[(q_init[0] + 2)/eps][(q_init[1] + 2)/eps]);
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    var temp = pq_extract(visit_queue);
    draw_2D_configuration([temp.x, temp.y], "visited");
    G[temp.i][temp.j].visited = true;
    if((q_goal[0] == (Math.round(temp.x/eps) *eps)) && (q_goal[1] == (Math.round(temp.y/eps)*eps))){
        drawHighlightedPathGraph(temp);
        search_iterate = false;
        return "succeeded";
    }
    else{
        //up
        if(!testCollision([G[temp.i - 1][temp.j].x, G[temp.i - 1][temp.j].y]) 
        && !G[temp.i - 1][temp.j].visited){
            if(!G[temp.i - 1][temp.j].queued){
                G[temp.i - 1][temp.j].parent = temp;
                G[temp.i - 1][temp.j].distance = temp.distance + eps;
                G[temp.i - 1][temp.j].priority = Math.sqrt((G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i - 1][temp.j].x)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i - 1][temp.j].x) 
                + (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i - 1][temp.j].y)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i - 1][temp.j].y)) + G[temp.i - 1][temp.j].distance;
                pq_insert(visit_queue, G[temp.i - 1][temp.j]);
                G[temp.i - 1][temp.j].queued = true;
                draw_2D_configuration([temp.x - eps, temp.y], "queued");
            }
        }
        if(!testCollision([G[temp.i][temp.j + 1].x, G[temp.i][temp.j + 1].y]) 
        && !G[temp.i][temp.j + 1].visited){
            if(!G[temp.i][temp.j + 1].queued){
                G[temp.i][temp.j + 1].parent = temp;
                G[temp.i][temp.j + 1].distance = temp.distance + eps;
                G[temp.i][temp.j + 1].priority = Math.sqrt((G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i][temp.j + 1].x)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i][temp.j + 1].x) 
                + (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i][temp.j + 1].y)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i][temp.j + 1].y)) + G[temp.i][temp.j + 1].distance;
                pq_insert(visit_queue, G[temp.i][temp.j + 1]);
                G[temp.i][temp.j + 1].queued = true;
                draw_2D_configuration([temp.x, temp.y + eps], "queued");
            }
        }
        if(!testCollision([G[temp.i + 1][temp.j].x, G[temp.i + 1][temp.j].y]) 
        && !G[temp.i + 1][temp.j].visited){
            if(!G[temp.i + 1][temp.j].queued){
                G[temp.i + 1][temp.j].parent = temp;
                G[temp.i + 1][temp.j].distance = temp.distance + eps;
                G[temp.i + 1][temp.j].priority = Math.sqrt((G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i + 1][temp.j].x)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i + 1][temp.j].x) 
                + (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i + 1][temp.j].y)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i + 1][temp.j].y)) + G[temp.i + 1][temp.j].distance;
                pq_insert(visit_queue, G[temp.i + 1][temp.j]);
                G[temp.i + 1][temp.j].queued = true;
                draw_2D_configuration([temp.x + eps, temp.y], "queued");
            }
        }
        if(!testCollision([G[temp.i][temp.j - 1].x, G[temp.i][temp.j - 1].y]) 
        && !G[temp.i][temp.j - 1].visited){
            if(!G[temp.i][temp.j - 1].queued){
                G[temp.i][temp.j - 1].parent = temp;
                G[temp.i][temp.j - 1].distance = temp.distance + eps;
                G[temp.i][temp.j - 1].priority = Math.sqrt((G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i][temp.j - 1].x)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].x - G[temp.i][temp.j - 1].x) 
                + (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i][temp.j - 1].y)
                * (G[(q_goal[0] + 2)/eps][(q_goal[1] + 2)/eps].y - G[temp.i][temp.j - 1].y)) + G[temp.i][temp.j - 1].distance;
                pq_insert(visit_queue, G[temp.i][temp.j - 1]);
                G[temp.i][temp.j - 1].queued = true;
                draw_2D_configuration([temp.x, temp.y - eps], "queued");
            }
        }
        if(visit_queue.length == 0) return "failed";
        return "iterating";
    }
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.
function pq_insert(heap, new_element) {
    var eIntIdx = heap.length;
    var prntIdx = Math.floor((eIntIdx - 1 ) / 2);
    heap.push(new_element);
    var heaped = (eIntIdx <= 0) || (heap[prntIdx].priority <= heap[eIntIdx]).priority;

    while(!heaped){
        var tmp = heap[prntIdx];
        heap[prntIdx] = heap[eIntIdx];
        heap[eIntIdx] = tmp;

        eIntIdx = prntIdx;
        prntIdx = Math.floor((eIntIdx - 1)/2);
        heaped = (eIntIdx <= 0) ||(heap[prntIdx].priority <= heap[eIntIdx].priority);
    }
    // STENCIL: implement your min binary heap insert operation
}

function pq_extract(heap) {
    var temp;
    temp = heap[0];
    heap[0] = heap[heap.length - 1];
    heap[heap.length - 1] = temp;
    heap.pop();
    var inde = 0;
    
    while(2 * inde + 1< heap.length){
        var j = 2 * inde + 1;
        if(j + 1 >= heap.length){
            if(heap[j].priority < heap[inde].priority){
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
            }
            break;
        }
        else{
            if((heap[j].priority > heap[j + 1].priority) && (heap[j].priority < heap[inde].priority)){
                j++
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if((heap[j].priority > heap[j + 1].priority) && (heap[j + 1].priority < heap[inde].priority)){
                j++;
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if((heap[j + 1].priority < heap[inde].priority)){
                var temp1 = heap[inde];
                heap[inde] = heap[j];
                heap[j] = temp1;
                inde = j;
            }
            else if(heap[j].priority < heap[inde].priority){
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

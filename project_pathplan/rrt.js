/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

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

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    var q_rand = randomConfig();
    if(extendRRT(T_a, q_rand) != "Trapped"){
        if(connectRRT(T_b, T_a.vertices[T_a.newest].vertex) == "reached"){
            var path = Path();
            drawHighlightedPath(path);
            search_iterate = false;
            return "succeeded";
        }
    }
    var temp = T_a;
    T_a = T_b;
    T_b = temp;
    return "extended";

}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

function randomConfig(){
    var q_rand = [Math.random() * 9 - 2, Math.random() * 9 - 2];
    return q_rand;
}

function extendRRT(tree, q){
    var q_near = findNearestNeighbor(q, tree);// index
    var norm = Math.sqrt((tree.vertices[q_near].vertex[0] - q[0]) * (tree.vertices[q_near].vertex[0] - q[0]) +
        (tree.vertices[q_near].vertex[1] - q[1]) * (tree.vertices[q_near].vertex[1] - q[1]));
    var q_new = [(q[0] - tree.vertices[q_near].vertex[0])/norm * eps + tree.vertices[q_near].vertex[0],
    (q[1] - tree.vertices[q_near].vertex[1])/norm * eps + tree.vertices[q_near].vertex[1]]//[,]
    if(newConfig(q_new) == "not"){
        insertTreeVertex(tree, q_new);
        insertTreeEdge(tree,q_near,tree.newest);
        if(Math.sqrt((q[0]-q_new[0]) * (q[0]-q_new[0]) + (q[1]-q_new[1]) * (q[1]-q_new[1])) < eps){
            return "reached";
        }
        else{
            return "advanced";
        }
    }
    return "Trapped";
}

function findNearestNeighbor(q, tree){
    var min_vertex = 0;
    var min_dist = 200000000;
    for(var i = 0; i < tree.vertices.length; i++){
        var dist = Math.sqrt((tree.vertices[i].vertex[0] - q[0]) * (tree.vertices[i].vertex[0] - q[0]) +
        (tree.vertices[i].vertex[1] - q[1]) * (tree.vertices[i].vertex[1] - q[1]));
        if(dist < min_dist){
            min_dist = dist;
            min_vertex = i;
        }
    }
    return min_vertex;
}

function newConfig(q_new){
    if(testCollision(q_new)){
        return "Trapped";
    }
    else{
        return "not";
    }
}

function connectRRT(tree, q_new){
    var S = "advanced";
    while(S == "advanced"){
        S = extendRRT(tree, q_new);
    }
    return S;
}

function Path(){
    var T_a_path = [];
    var T_a_touched = [T_a.vertices[0]];
    T_a_path = dfsPath(T_a, T_a.vertices[T_a.newest], T_a.vertices[0], T_a_touched);
    var T_b_path = [];
    var T_b_touched = [T_b.vertices[0]];
    T_b_path = dfsPath(T_b, T_b.vertices[T_b.newest], T_b.vertices[0], T_b_touched);
    for(var i = T_b_path.length - 1; i >= 0; i--){
        T_a_path.push(T_b_path[i]);
    }
    return T_a_path;
}

function dfsPath(tree, q_goal, q_start, stack){
    var q = stack[0];
    var visited = [];
    while(q != q_goal){
        visited.push(q);
        stack.pop();
        for(var i = 0; i < q.edges.length; i ++){
            var exist = false;
            for(var j = 0; j < visited.length; j ++){
                if(q.edges[i] == visited[j]){
                    exist = true;
                }
            }
            if(exist == false){
                stack.push(q.edges[i]);
                q.edges[i].parent = q;
            }
        }
        q = stack[stack.length - 1];
    }
    var q = q_goal;
    var path = [];
    while(q != q_start){
        path.unshift(q);
        q = q.parent
    }
    path.unshift(q_start);
    return path;
 
    
}
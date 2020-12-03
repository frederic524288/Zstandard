//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [[],[],[],[]];
    mat[0] = m1[0].slice(0);
    mat[1] = m1[1].slice(0);
    mat[2] = m1[2].slice(0);
    mat[3] = m1[3].slice(0);
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
    //returns 2D array that is the result of m1*m2
    var m3 = [];
    for(var i = 0; i < m1.length; i++){
        var temp = [];
        for(var j = 0; j < m2[0].length; j++){
            temp.push(0);
        }
        m3.push(temp);
    }

    for(var i = 0; i < m1.length; i++){
        for(var j = 0; j < m2[0].length; j++){
            for(var c = 0; c < m1[0].length; c++){
                m3[i][j] = m3[i][j] + m1[i][c] * m2[c][j]
            }
        }
    }
    return m3;
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1*m2
    var mh = [];
    for(var i = 0; i < m[0].length; i++){
        var temp = [];
        for(var j = 0; j < m.length; j++){
            temp.push(0);
        }
        mh.push(temp);
    }

    for(var i = 0; i < mh.length; i++){
        for(var j = 0; j < mh[0].length; j++){
            mh[i][j] = m[j][i];
        }
    }
    return mh;
}

function matrix_pseudoinverse(m) {
    //returns pseudoinverse of matrix m
    if(m.length > m[0].length){
        var temp = numeric.inv(matrix_multiply(matrix_transpose(m), m));
        return matrix_multiply(temp, matrix_transpose(m));
    }
    else{
        var temp = numeric.inv(matrix_multiply(m, matrix_transpose(m)));
        return matrix_multiply(matrix_transpose(m), temp);
    }
}

//function matrix_invert_affine(m) {
     // returns 2D array that is the invert affine of 4-by-4 matrix m

//}

function vector_normalize(v) {
    // returns normalized vector for v
    if(v[0][0] || (v[0][0] == 0)){
        var norms = 0;
        for(var i = 0 ; i < v.length; i++){
            norms = norms + v[i][0] * v[i][0];
        }
        var norm = Math.sqrt(norms);
        for(var i = 0 ; i < v.length; i++){
            v[i][0] = v[i][0] / norm;
        }
        return v;
    }
    var norms = 0;
    for(var i = 0 ; i < v.length; i++){
        norms = norms + v[i] * v[i];
    }
    var norm = Math.sqrt(norms);
    for(var i = 0 ; i < v.length; i++){
        v[i] = v[i] / norm;
    }
    return v;
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions
    var c = [0,0,0];
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
    return c;
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    var temp = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
    return temp;
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    var temp = [[1,0,0,tx],[0,1,0,ty],[0,0,1,tz],[0,0,0,1]];
    return temp;
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var temp = [[1,0,0,0],[0,Math.cos(angle),-Math.sin(angle),0],
                [0,Math.sin(angle),Math.cos(angle),0],[0,0,0,1]];
    return temp;
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var temp = [[Math.cos(angle),0,Math.sin(angle),0],[0,1,0,0],
                [-Math.sin(angle),0,Math.cos(angle),0],[0,0,0,1]];
    return temp;
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var temp = [[Math.cos(angle),-Math.sin(angle),0,0],
                [Math.sin(angle),Math.cos(angle),0,0],
                [0,0,1,0],[0,0,0,1]];
    return temp;
}
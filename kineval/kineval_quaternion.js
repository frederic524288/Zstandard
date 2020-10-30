//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    //returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
     var q = {};
     q.a = Math.cos(angle/2);
     q.b = axis[0] * Math.sin(angle/2);
     q.c = axis[1] * Math.sin(angle/2);
     q.d = axis[2] * Math.sin(angle/2);
    return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
      //returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    var temp = Math.sqrt(q1.a * q1.a + q1.b * q1.b + q1.c * q1.c + q1.d * q1.d);
    q.a = q1.a / temp;
    q.b = q1.b / temp;
    q.c = q1.c / temp;
    q.d = q1.d / temp;
    return q;
}

kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d;
    q.b = q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c;
    q.c = q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b;
    q.d = q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a;
    return q;
}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix
    var temp = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]];
    temp[0][0] = 1 - 2 * (q.c * q.c + q.d * q.d);
    temp[0][1] = 2 * (q.b * q.c - q.a * q.d);
    temp[0][2] = 2 * (q.a * q.c + q.b * q.d);
    temp[1][0] = 2 * (q.b * q.c + q.a *q.d); 
    temp[1][1] = 1 - 2 * (q.b * q.b + q.d * q.d);
    temp[1][2] = 2 * (q.c * q.d - q.a * q.b);
    temp[2][0] = 2 * (q.b * q.d - q.a * q.c);
    temp[2][1] = 2 * (q.a * q.b + q.c * q.d);
    temp[2][2] = 1 - 2 * (q.b *q.b + q.c * q.c);
    return temp;
}
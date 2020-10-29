
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    var translational = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
    var xR = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var yR = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    var zR = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    robot.links[robot.base].xform = matrix_multiply(matrix_multiply(matrix_multiply(translational, xR),yR),zR);
    kineval.buildFKTransforms(robot.base);
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    kineval.buildFKTransforms = function buildFKTransforms(part){
        if(!robot.links[part].children) return
        for(var i =0; i < robot.links[part].children.length; i++){
            var translational = generate_translation_matrix(
                robot.joints[robot.links[part].children[i]].origin.xyz[0], 
                robot.joints[robot.links[part].children[i]].origin.xyz[1], 
                robot.joints[robot.links[part].children[i]].origin.xyz[2]);
            var xR = generate_rotation_matrix_X(robot.joints[robot.links[part].children[i]].origin.rpy[0]);
            var yR = generate_rotation_matrix_Y(robot.joints[robot.links[part].children[i]].origin.rpy[1]);
            var zR = generate_rotation_matrix_Z(robot.joints[robot.links[part].children[i]].origin.rpy[2]);

            robot.joints[robot.links[part].children[i]].xform = matrix_multiply(robot.links[part].xform,
                matrix_multiply(matrix_multiply(matrix_multiply(translational, xR),yR),zR));
            robot.links[robot.joints[robot.links[part].children[i]].child].xform = 
                robot.joints[robot.links[part].children[i]].xform;
            kineval.buildFKTransforms(robot.joints[robot.links[part].children[i]].child);
        }
    }

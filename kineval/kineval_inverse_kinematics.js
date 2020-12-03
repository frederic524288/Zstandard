
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}


kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length
    var endeff = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    robot.dx = [[0],[0],[0],[0],[0],[0]];
    robot.dx[0][0] = endeffector_target_world.position[0][0] - endeff[0][0];
    robot.dx[1][0] = endeffector_target_world.position[1][0] - endeff[1][0];
    robot.dx[2][0] = endeffector_target_world.position[2][0] - endeff[2][0];

    var part = endeffector_joint;
    robot.jacobian = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
    for(var i = 3; i >= 0; i--){
        var origin = [[0], 
                        [0],
                        [0],[1]];
        var axis = [[robot.joints[part].axis[0]], 
                    [robot.joints[part].axis[1]], 
                    [robot.joints[part].axis[2]], [1]];
        var worigin = matrix_multiply(robot.joints[part].xform, origin);
        var waxis = matrix_multiply(robot.joints[part].xform, axis);
        var ri = [0,0,0];
        ri[0] = endeff[0][0] - worigin[0][0];
        ri[1] = endeff[1][0] - worigin[1][0];
        ri[2] = endeff[2][0] - worigin[2][0];
        var wi = [0,0,0];
        wi[0] = waxis[0][0] - worigin[0][0];
        wi[1] = waxis[1][0] - worigin[1][0];
        wi[2] = waxis[2][0] - worigin[2][0];

        var Jvi = vector_cross(wi,ri);
        robot.jacobian[0][i] = Jvi[0];
        robot.jacobian[1][i] = Jvi[1];
        robot.jacobian[2][i] = Jvi[2];
        robot.jacobian[3][i] = wi[0];
        robot.jacobian[4][i] = wi[1];
        robot.jacobian[5][i] = wi[2];

        part = robot.links[robot.joints[part].parent].parent;
    }
    var jacoinver = matrix_pseudoinverse(robot.jacobian);
    robot.dq = matrix_multiply(jacoinver, robot.dx);
    part = endeffector_joint;
    for(var i = 3; i >= 0; i--){
        robot.joints[part].control = robot.dq[i][0] * kineval.params.ik_steplength;
        part = robot.links[robot.joints[part].parent].parent;
    }


}




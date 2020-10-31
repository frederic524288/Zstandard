
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/
var check = function(){
    if(condition){
        // run when condition is met
    }
    else {
        setTimeout(check, 1000); // check again in a second
    }
}


kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 
    var error = 0;
    for(var x in robot.joints){
        error += (kineval.params.setpoint_target[x] - robot.joints[x].angle);
    }
    if((error < 0.1) && (error > -0.1)){
        if(kineval.params.dance_pose_index != 5){
            for(var x in kineval.setpoints[kineval.params.dance_sequence_index[kineval.params.dance_pose_index]]){
                kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_sequence_index[kineval.params.dance_pose_index]][x];
            }
            kineval.params.dance_pose_index += 1;
        }
        else{
            for(var x in kineval.setpoints[kineval.params.dance_sequence_index[0]]){
                kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_sequence_index[0]][x];
            }
            kineval.params.dance_pose_index = 1;
        }
    }
    // STENCIL: implement FSM to cycle through dance pose setpoints
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for(x in robot.joints){
        robot.joints[x].control = (kineval.params.setpoint_target[x] - robot.joints[x].angle) * robot.joints[x].servo.p_gain;
    }
}



function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {
    // STENCIL: a correct Euler integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        theta_dotdot = pendulum_acceleration(pendulum, gravity);
        pendulum.angle = pendulum.angle + dt * pendulum.angle_dot;
        pendulum.angle_dot = pendulum.angle_dot + dt * theta_dotdot;
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
    }
    else if (numerical_integrator === "velocity verlet") {
    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        a_now = pendulum_acceleration(pendulum, gravity);
        pendulum.angle = pendulum.angle + pendulum.angle_dot * dt + a_now * dt * dt / 2;
        a_later = pendulum_acceleration(pendulum, gravity);
        pendulum.angle_dot = pendulum.angle_dot + (a_now + a_later)/2 * dt;
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 

    return -gravity / pendulum.length * Math.sin(pendulum.angle) 
    + PID(pendulum,accumulated_error, dt)[0].control / pendulum.mass/ (pendulum.length * pendulum.length);
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp: 290, kd:250, ki:0};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error 
    var error = pendulum.desired - pendulum.angle;
    accumulated_error = accumulated_error + error;
    pre_error = pendulum.desired - pendulum.angle_previous;
    pendulum.control += pendulum.servo.kp * error + pendulum.servo.ki * accumulated_error 
    + pendulum.servo.kp * (error - pre_error) / dt;

    return [pendulum, accumulated_error];
}

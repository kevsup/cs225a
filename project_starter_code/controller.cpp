// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include "project_constants.h"

#include <iostream>
#include <string>
#include <chrono>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

using namespace std;
using namespace Eigen;


#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
//int state = POSORI_CONTROLLER;

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

void moveTruck(VectorXd q_desired, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, double drive_time);
void moveArm(VectorXd xd, Matrix3d &Rd, VectorXd &qd, VectorXd &command_torques, Sai2Model::Sai2Model* &robot);
void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N, const MatrixXd& task_jacobian, Sai2Model::Sai2Model* &robot);
void openBoxStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q);
void grabMailStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q);
void placeMailStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q);
bool moveGripperToParcel(Vector3d &xd, Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, VectorXd &initial_q, double vel_threshold);
bool moveGripperToBox(Vector3d &xd, Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, VectorXd &initial_q, double vel_threshold, bool gripped, bool vertical=false);


States state = WAIT_FOR_BOX;
MailStates mail_state = MOVE_OVERHEAD;
PlaceStates place_state = FRONT;
BoxStates box_state = MOVE_IN_FRONT;

// constants
const int TRUCK_JTS = 3;    // number of truck joints   
const int ARM_JTS = 7;      // number of arm joints
const int GRIP_JTS = 2;     // number of gripper joints
const double MAX_TRUCK_VEL = 2;
const double MAX_ARM_VEL = 1;

const string CONTROL_LINK= "link7";
const Vector3d CONTROL_POINT = Vector3d(0.0,0.0,0.07);

    /*
std::string JOINT_TORQUES_SENSED_KEY;
*/

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {
    // start redis client
    auto redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    VectorXd initial_q = robot->_q;
    robot->updateModel();

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);

/*
    // pose task
    auto posori_task = new Sai2Primitives::PosOriTask(robot, CONTROL_LINK, CONTROL_POINT);


#ifdef USING_OTG
    posori_task->_use_interpolation_flag = true;
#else
    posori_task->_use_velocity_saturation_flag = true;
#endif
    
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_kp_pos = 200.0;
    posori_task->_kv_pos = 20.0;
    posori_task->_kp_ori = 200.0;
    posori_task->_kv_ori = 20.0;
*/

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
    joint_task->_use_interpolation_flag = true;
    cout << "using OTG" << endl;
#else
    joint_task->_use_velocity_saturation_flag = true;
    cout << "not using OTG" << endl;
#endif

    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->setDynamicDecouplingFull();
    N_prec.setIdentity();
    joint_task->updateTaskModel(N_prec);
    joint_task->_saturation_velocity(0) = 100;
    joint_task->_kp = 100.0;
    joint_task->_kv = 20.0;

    VectorXd q_desired = initial_q;
    q_desired(0) = 6;

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000); 
    double start_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;

    double drive_time_init = start_time;
    double grip_time_init;

    VectorXd detection_vector(1);
    detection_vector.setZero();
    redis_client.setEigenMatrixJSON(DETECTION_STATE, detection_vector);

    //intialize counter
    int counter = 0;

    while (runloop) {
        // wait for next scheduled loop
        //timer.waitForNextLoop();

        // read simulation state
        fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

        if (fSimulationLoopDone) {

            double time = timer.elapsedTime() - start_time;

            // read robot state from redis
            robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
            robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

            // update model
            robot->updateModel();
            switch (state) {
                case WAIT_FOR_BOX:
                {
                    /*
                    // maybe use pre-made joint space controller for this, but need to remove USING_OTG
                    joint_task->updateTaskModel(N_prec);
                    joint_task->_desired_position = q_desired;
                    joint_task->computeTorques(joint_task_torques);
                    command_torques = joint_task_torques;
                    //cout << "step_des_pos = " << joint_task->_step_desired_position << endl;
                    */

                    // Action: driving
                    q_desired = initial_q;
                    q_desired(0) = 6;
                    moveTruck(q_desired, command_torques, robot, time - drive_time_init);

                    // Trigger: camera detects mailbox aka arrives at desired position
                    if ((robot->_q - q_desired).norm() < 1 
                                    && robot->_dq.norm() < 0.001) {
                        cout << "Truck has arrived!!!!" << endl;
                        // get coordinates of mailbox using camera
                        state = SCAN_FOR_BOX;
                    }
                    break;
                }
                case SCAN_FOR_BOX:
                {
                    state = OPEN_BOX;
                    // detection_vector = redis_client.getEigenMatrixJSON(DETECTION_STATE);
                    // if (detection_vector(0) == 1){
                    //     cout << "Next: open box!!!!" << endl;
                    // 	state = OPEN_BOX;
                    // } else {
                    // 	// move end effector
                    //     double angle = -45 * M_PI / 180;
                    //     Matrix3d Rd;
                    //     Rd << -cos(angle), -sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, -1;
                    //     Matrix3d rot;
                    //     rot << 0, 1, 0, 0, 0, -1, -1, 0, 0;
                    //     Rd = rot * Rd;
                    //     Vector3d xd = Vector3d(5.8, 0.35, 0.66);
                    //     VectorXd qd = q_desired;
                    //     qd(TRUCK_JTS) += M_PI;
                    //     moveArm(xd, Rd, qd, command_torques, robot);
                    // }
                    break;
                }
                case OPEN_BOX:
                {
                    //openBoxStateMachine(robot, q_desired, command_torques, grip_time_init,  time, initial_q);
                    state = GRAB_MAIL;
                    break;
                }
                case GRAB_MAIL:
                {
                    grabMailStateMachine(robot, q_desired, command_torques, grip_time_init, time, initial_q);
                    break;
                }
                case PLACE_MAIL:
                {
                    placeMailStateMachine(robot, q_desired, command_torques, grip_time_init, time, initial_q);
                    break;
                }
                case CLOSE_BOX:
                {

                    state = RETRACT_ARM;
                    break;
                }
                case RETRACT_ARM:
                {
                    // placeholder code. Keeps robot from looping through states
                    q_desired << q_desired.head(TRUCK_JTS), initial_q.segment(TRUCK_JTS, ARM_JTS + GRIP_JTS);
                    moveTruck(q_desired, command_torques, robot, time);

                    drive_time_init = time;
                    //state = WAIT_FOR_BOX;
                    break;
                }
                default:
                {
                    //shouldn't be stateless
                    cout << "I am stateless :(" << endl;
                }
            }

    /*  
            if(state == JOINT_CONTROLLER)
            {
                // update task model and set hierarchy
                N_prec.setIdentity();
                joint_task->updateTaskModel(N_prec);

                // compute torques
                joint_task->computeTorques(joint_task_torques);

                command_torques = joint_task_torques;

                if( (robot->_q - q_init_desired).norm() < 0.15 )
                {
                    posori_task->reInitializeTask();
                    posori_task->_desired_position += Vector3d(-0.1,0.1,0.1);
                    posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

                    joint_task->reInitializeTask();
                    joint_task->_kp = 0;

                    state = POSORI_CONTROLLER;
                }
            }

            else if(state == POSORI_CONTROLLER)
            {
                // update task model and set hierarchy
                N_prec.setIdentity();
                posori_task->updateTaskModel(N_prec);
                N_prec = posori_task->_N;
                joint_task->updateTaskModel(N_prec);

                // compute torques
                posori_task->computeTorques(posori_task_torques);
                joint_task->computeTorques(joint_task_torques);

                //command_torques = posori_task_torques + joint_task_torques;
                command_torques = posori_task_torques;
                for (int i = 1; i < 11; i++) command_torques(i) = 0;
                cout << command_torques << endl;
            }
    */

            // send to redis
            redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
            VectorXd state_vector(1);
            state_vector << (state == PLACE_MAIL && place_state == BACKOUT ? place_state : state);
            redis_client.setEigenMatrixJSON(ROBOT_STATE, state_vector);

            // ask for next simulation loop
            fSimulationLoopDone = false;
            redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
            controller_counter++;
        }

        // controller loop is done
        fControllerLoopDone = true;
        redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

    }

    command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // controller loop is turned off
    fControllerLoopDone = false;
    redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}

void openBoxStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q) {

    switch (box_state) {
        case MOVE_IN_FRONT:
        {
            Vector3d xd(6.26-0.13, 1 - 0.575 - 0.3, -1.32 + 1.84 + 0.21);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 1, false, true)) {
                box_state = MOVE_TO_HANDLE;
                q_desired = robot->_q;
                cout << "Next: drop down" << endl;
            }
            break;
        }
        case MOVE_TO_HANDLE:
        {
            Vector3d xd(6.26-0.13, 1 - 0.575 - 0.15, -1.32 + 1.84 + 0.21);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 0.01, false, true)) {
                box_state = GRIP_HANDLE;
                q_desired = robot->_q;
                grip_time_init = time;
                cout << "Next: grip parcel" << endl;
            }
            break;
        }
        case GRIP_HANDLE:
        {
            // basic joint space control w/ high gains
            double kp = 900;
            double kv = 60;

            q_desired(10) = 0;
            q_desired(11) = 0;
            command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq);  
           
            // grip for a fixed amount of time
            if (time - grip_time_init > 1) {
                box_state = PAIN;
                q_desired = robot->_q;
                cout << "completed gripping" << endl;
            }
            break;
        }
        case PAIN:
        {
            Vector3d xd(6.26-0.13, 1 - 0.575 - 0.15 -0.01, -1.32 + 1.84 + 0.21);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 0.01, true, true)) {
                //box_state = GRIP_HANDLE;
                q_desired = robot->_q;
                grip_time_init = time;
                cout << "Next: grip parcel" << endl;
            }

            break;
        }
        default:
        {
            //shouldn't be stateless
            cout << "Mail is stateless :(" << endl;
        }
    }
}


void grabMailStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q) {
    switch (mail_state) {
        case MOVE_OVERHEAD:
        {
            Vector3d xd = Vector3d(5.6, 0, 0.8);
            if (moveGripperToParcel(xd, robot, q_desired, command_torques, initial_q, 1)) {
                mail_state = DROP_DOWN;
                q_desired = robot->_q;
                cout << "Next: drop down" << endl;
            }
            break;
        }
        case DROP_DOWN:
        {
            Vector3d xd = Vector3d(5.4, 0, 0.62);
            if (moveGripperToParcel(xd, robot, q_desired, command_torques, initial_q, 0.01)) {
                mail_state = GRIP_PARCEL;
                q_desired = robot->_q;
                grip_time_init = time;
                cout << "Next: grip parcel" << endl;
            }
            break;
        }
        case GRIP_PARCEL:
        {
            // basic joint space control w/ high gains
            double kp = 900;
            double kv = 60;

            q_desired(10) = 0;
            q_desired(11) = 0;
            command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq);  

            /* 
            Matrix3d R;
            robot->rotation(R, "link7");
            cout << "R = " << R << endl;
            Vector3d x;
            robot->position(x, "link7", CONTROL_POINT);
            cout << "x = " << x << endl;
            */
           
            // grip for a fixed amount of time
            if (time - grip_time_init > 1) {
                mail_state = MOVE_OVERHEAD;
                state = PLACE_MAIL;
                q_desired = robot->_q;
                cout << "completed gripping" << endl;
            }
            break;
        }
        default:
        {
            //shouldn't be stateless
            cout << "Mail is stateless :(" << endl;
        }
    }
}


void placeMailStateMachine(Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, double &grip_time_init, double time, VectorXd &initial_q) {
    switch (place_state) {
        case FRONT:
        {
            Vector3d xd = Vector3d(6.15, -0.1, 0.66);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 1, true)) {
                place_state = INSIDE;
                q_desired = robot->_q;
                cout << "Next: inside" << endl;
            }
            break;
        }
        case INSIDE:
        {
            Vector3d xd = Vector3d(6.15, 0.35, 0.66);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 0.01, true)) {
                place_state = RELEASE;
                q_desired = robot->_q;
                grip_time_init = time;
                cout << "Next: release" << endl;
            }
            break;
        }
        case RELEASE:
        {
            double kp = 100;
            double kv = 20;

            q_desired(10) = initial_q(10);
            q_desired(11) = initial_q(11);
            command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq);  
           
            // release for a fixed amount of time
            if (time - grip_time_init > 1) {
                place_state = BACKOUT;
                state = PLACE_MAIL;
                q_desired = robot->_q;
                cout << "Next: backout" << endl;
            }
            break;
        }
        case BACKOUT:
        {
            Vector3d xd = Vector3d(6.15, -0.1, 0.7);
            if (moveGripperToBox(xd, robot, q_desired, command_torques, initial_q, 0.01, false)) {
                place_state = FRONT;
                state = CLOSE_BOX;
                q_desired = robot->_q;
                cout << "Next: close box" << endl;
            }
            break;
        }
        default:
        {
            //shouldn't be stateless
            cout << "Placing mail is stateless :(" << endl;
        }
    }
}

// joint space control with velocity saturation
void moveTruck(VectorXd q_desired, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, double drive_time) {
    double kp = 100;
    double kv = 20;
    VectorXd b(robot->dof());
    robot->coriolisForce(b);
    double qd_mid = MAX_TRUCK_VEL * drive_time;
    if (q_desired(0) > qd_mid) {
        q_desired(0) = qd_mid;
    }

    // note: simviz.cpp already adds gravity vector
    command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + b;  
}

// Rd is direction cosines
void moveArm(VectorXd xd, Matrix3d &Rd, VectorXd &qd, VectorXd &command_torques, Sai2Model::Sai2Model* &robot) {
    double kp = 100;    // for operation space control
    double kv = 20;
    double kpj = 200;   // for posture / joint space control
    double kvj = 50;
    double kpg = 2500;   // for gripper
    double kvg = 100;

    Vector3d x, x_dot;
    robot->position(x, CONTROL_LINK, CONTROL_POINT);
    robot->linearVelocity(x_dot, CONTROL_LINK, CONTROL_POINT);
    MatrixXd J;
    robot->J_0(J, CONTROL_LINK, CONTROL_POINT);

    // chop off truck joints
    MatrixXd J_bot = J.block(0, TRUCK_JTS,6, ARM_JTS);

    MatrixXd J_bar, Lambda, N;
    operationalSpaceMatrices(Lambda, J_bar, N, J_bot, robot);
    Matrix3d R;
    robot->rotation(R, CONTROL_LINK);
    
    Vector3d d_phi = Vector3d(0,0,0);
    for (int i = 0; i < 3; i++) {
        d_phi -= 0.5 * R.col(i).cross(Rd.col(i));
    }

    Vector3d omega;
    robot->angularVelocity(omega, CONTROL_LINK);

    // velocity saturation
    Vector3d x_dot_d = kp / kv * (xd - x);
    double param = MAX_ARM_VEL / x_dot_d.norm();
    double v_sat = abs(param) > 1 ? param / abs(param) : param;
    VectorXd Fv = -kv *(x_dot - v_sat * x_dot_d);

    //VectorXd Fv = kp * (xd - x) - kv * x_dot;
    VectorXd Fw = kp * (-d_phi) - kv * omega;
    VectorXd F(Fv.size() + Fw.size());
    F << Fv, Fw; 
    F = Lambda * F;
    MatrixXd M_truck = robot->_M.block(0,0, TRUCK_JTS, TRUCK_JTS);
    MatrixXd M_arm = robot->_M.block(TRUCK_JTS, TRUCK_JTS, ARM_JTS, ARM_JTS);
    int num_truck_arm_joints = TRUCK_JTS + ARM_JTS;
    MatrixXd M_gripper = robot->_M.block(num_truck_arm_joints, num_truck_arm_joints, GRIP_JTS, GRIP_JTS);

    command_torques << 
        // keep truck stationary
        M_truck * (-kpj * (robot->_q.head(TRUCK_JTS) - qd.head(TRUCK_JTS)) - kvj * robot->_dq.head(TRUCK_JTS)),
        // arm: operation space
        J_bot.transpose() * F 
        // arm: nullspace posture control
        + N.transpose() * M_arm * (-kpj * (robot->_q.segment(TRUCK_JTS, ARM_JTS) - qd.segment(TRUCK_JTS, ARM_JTS)) 
                                                  - kvj * robot->_dq.segment(TRUCK_JTS, ARM_JTS)),           
        // gripper control
        M_gripper * (-kpg * (robot->_q.tail(GRIP_JTS) - qd.tail(GRIP_JTS)) - kvg * robot->_dq.tail(GRIP_JTS));

    // note: simviz.cpp already adds gravity vector
}


void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N,
                                    const MatrixXd& task_jacobian, Sai2Model::Sai2Model* &robot)
{
    MatrixXd N_prec = MatrixXd::Identity(ARM_JTS, ARM_JTS);

    // resize matrices
    int k = task_jacobian.rows();
    Lambda.setZero(k,k);
    Jbar.setZero(ARM_JTS,k);
    N.setIdentity(ARM_JTS, ARM_JTS);

    MatrixXd _M_inv = robot->_M_inv.block(TRUCK_JTS, TRUCK_JTS, ARM_JTS, ARM_JTS);

    // Compute the matrices
    MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
    Lambda = inv_inertia.llt().solve(MatrixXd::Identity(k,k));
    Jbar = _M_inv*task_jacobian.transpose()*Lambda;
    MatrixXd Ni = MatrixXd::Identity(ARM_JTS, ARM_JTS);
    N = MatrixXd::Identity(ARM_JTS, ARM_JTS) - Jbar*task_jacobian;
    N = N*N_prec;
}

bool moveGripperToParcel(Vector3d &xd, Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, VectorXd &initial_q, double vel_threshold) {
    double angle = -45 * M_PI / 180;
    Matrix3d Rd;
    Rd << -cos(angle), -sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, -1;
    q_desired << q_desired.head(TRUCK_JTS), initial_q.segment(TRUCK_JTS, ARM_JTS + GRIP_JTS);
    q_desired(TRUCK_JTS) += M_PI;
    moveArm(xd, Rd, q_desired, command_torques, robot);
    Vector3d x;
    robot->position(x, CONTROL_LINK, CONTROL_POINT);
    return (x - xd).norm() < 0.01 && robot->_dq.norm() < vel_threshold;
} 

bool moveGripperToBox(Vector3d &xd, Sai2Model::Sai2Model* &robot, VectorXd &q_desired, VectorXd &command_torques, VectorXd &initial_q, double vel_threshold, bool gripped, bool vertical) {
    double angle = -45 * M_PI / 180;
    Matrix3d Rd;
    Rd << -cos(angle), -sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, -1;
    Matrix3d rot;
    if (vertical) {
        rot << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    } else {
        rot << 0, 1, 0, 0, 0, -1, -1, 0, 0;
    }
    Rd = rot * Rd;
    if (gripped) {
        q_desired << q_desired.head(TRUCK_JTS), initial_q.segment(TRUCK_JTS, ARM_JTS), 0, 0;
    } else {
        q_desired << q_desired.head(TRUCK_JTS), initial_q.segment(TRUCK_JTS, ARM_JTS + GRIP_JTS);
    }
    moveArm(xd, Rd, q_desired, command_torques, robot);
    Vector3d x;
    robot->position(x, CONTROL_LINK, CONTROL_POINT);
    return (x - xd).norm() < 0.01 && robot->_dq.norm() < vel_threshold;
}

//------------------------------------------------------------------------------

double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
  assert(x == "false" || x == "true");
  return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
  return b ? "true" : "false";
} 

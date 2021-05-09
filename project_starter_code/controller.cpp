// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <chrono> // pls

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
//int state = POSORI_CONTROLLER;

void moveTruck(VectorXd q_desired, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, double drive_time);
void moveArm(VectorXd xd, Matrix3d Rd, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, Vector3d pos_in_link);
void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N, const MatrixXd& task_jacobian, Sai2Model::Sai2Model* &robot);

typedef enum {INIT, WAIT_FOR_BOX, SCAN_FOR_BOX, OPEN_BOX, GRAB_MAIL, PLACE_MAIL, CLOSE_BOX, RETRACT_ARM} States;

States state = WAIT_FOR_BOX;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";

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

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0.0,0.0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

    Vector3d p_desired = Vector3d(6, 0, 0);
    posori_task->_desired_position = p_desired;

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

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired << 0.0, 0.0, 0.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

    double drive_time_init = start_time;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();
        switch (state) {
            case WAIT_FOR_BOX:
            {
                // Action: driving
                VectorXd q_desired = initial_q;
                q_desired(0) = 6;
                moveTruck(q_desired, command_torques, robot, time - drive_time_init);

                /*
                // maybe use pre-made joint space controller for this
				joint_task->reInitializeTask();
                N_prec.setIdentity();
			    joint_task->updateTaskModel(N_prec);
	            joint_task->_desired_position = q_desired;
                joint_task->computeTorques(joint_task_torques);
                command_torques = joint_task_torques;
                */

                // Trigger: camera detects mailbox aka arrives at desired position
                //if ((robot->_q - q_desired).norm() < 1) {
                if ((robot->_q - q_desired).norm() < 1 
                                && robot->_dq.norm() < 0.01) {
                    cout << "Truck has arrived!!!!" << endl;
                    // get coordinates of mailbox using camera
                    state = SCAN_FOR_BOX;
                }
                break;
            }
            case SCAN_FOR_BOX:
            {

                cout << "Next: open box!!!!" << endl;
                state = OPEN_BOX;
                break;
            }
            case OPEN_BOX:
            {
                Vector3d xd = Vector3d(6, 0.5, 0.8);
                Matrix3d Rd;
                Rd << cos(M_PI/3),0,sin(M_PI/3),0,1,0,-sin(M_PI/3),0,cos(M_PI/3);
                moveArm(xd, Rd, command_torques, robot, control_point);
                Vector3d x;
                robot->position(x, "link7", control_point);
                if ((x - xd).norm() < 0.1) {
                    state = GRAB_MAIL;
                    cout << "Next: grab mail!!!!" << endl;
                }
                break;
            }
            case GRAB_MAIL:
            {

                state = PLACE_MAIL;
                break;
            }
            case PLACE_MAIL:
            {

                state = CLOSE_BOX;
                break;
            }
            case CLOSE_BOX:
            {

                state = RETRACT_ARM;
                break;
            }
            case RETRACT_ARM:
            {
                
                drive_time_init = time;
                state = WAIT_FOR_BOX;
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

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

double sat(double param) {
    if (abs(param) > 1 && param > 0) {
        return 1;
    } else if (abs(param) > 1) {
        return -1;
    } else {
        return param;
    }
}

void moveTruck(VectorXd q_desired, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, double drive_time) {
    double kp = 100;
    double kv = 20;
    VectorXd g(robot->dof());
    robot->gravityVector(g);
    VectorXd b(robot->dof());
    robot->coriolisForce(b);
    double V_max = 3;
    double qd_mid = V_max * drive_time;
    if (q_desired(0) > qd_mid) {
        q_desired(0) = qd_mid;
    }
    command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + b + g;  
}

// Rd is direction cosines
void moveArm(VectorXd xd, Matrix3d Rd, VectorXd &command_torques, Sai2Model::Sai2Model* &robot, Vector3d pos_in_link) {
    string link_name = "link7";
    Vector3d x, x_dot;
    robot->position(x, link_name, pos_in_link);
    robot->linearVelocity(x_dot, link_name, pos_in_link);
    MatrixXd J;
    robot->J_0(J, link_name, pos_in_link);
    // chop off truck joints
    MatrixXd J_bot = J.block(0,3,6,7);

    MatrixXd J_bar, Lambda, N;
    operationalSpaceMatrices(Lambda, J_bar, N, J_bot, robot);
    VectorXd g(robot->dof());
    robot->gravityVector(g);
    Matrix3d R;
    robot->rotation(R, link_name);
    
    Vector3d d_phi = Vector3d(0,0,0);
    for (int i = 0; i < 3; i++) {
        d_phi -= 0.5 * R.col(i).cross(Rd.col(i));
    }

    Vector3d omega;
    robot->angularVelocity(omega, link_name);
    double kp = 100;
    double kv = 20;
    double kvj = 14;

    VectorXd Fv = kp * (xd - x) - kv * x_dot;
    VectorXd Fw = kp * (-d_phi) - kv * omega;
    VectorXd F(Fv.size() + Fw.size());
    F << Fv, Fw; 
    F = Lambda * F;
    command_torques << 0,0,0,J_bot.transpose() * F - N.transpose() * kvj * robot->_dq;
    command_torques += g;
}


void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N,
                                    const MatrixXd& task_jacobian, Sai2Model::Sai2Model* &robot)
{
    MatrixXd N_prec = MatrixXd::Identity(7,7);

	auto t1 = chrono::high_resolution_clock::now();
	auto t2 = chrono::high_resolution_clock::now();
	double duration = 0;

	t1 = chrono::high_resolution_clock::now();

	// resize matrices
	int k = task_jacobian.rows();
	Lambda.setZero(k,k);
	Jbar.setZero(7,k);
	N.setIdentity(7,7);

    MatrixXd _M_inv = robot->_M_inv.block(3,3,7,7);

	// Compute the matrices
	MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
	Lambda = inv_inertia.llt().solve(MatrixXd::Identity(k,k));
	Jbar = _M_inv*task_jacobian.transpose()*Lambda;
	MatrixXd Ni = MatrixXd::Identity(7,7);
	N = MatrixXd::Identity(7,7) - Jbar*task_jacobian;
	N = N*N_prec;
}

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"
#include <random>  // used for white-noise generation
#include <iostream>
#include <string>
#include "force_sensor/ForceSensorSim.h"  // references src folder in sai2-common directory 
#include "force_sensor/ForceSensorDisplay.h"
#include <signal.h>
#include "project_constants.h"

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}
bool fSimulationLoopDone = false;
bool fControllerLoopDone = true; // initialize as true for first loop

using namespace std;
using namespace Eigen;

// redis keys:
// - write:
const std::string EE_FORCE_KEY = "sai2::cs225a::sensor::force";
const std::string EE_MOMENT_KEY = "sai2::cs225a::sensor::moment";
const string ee_link_name = "link7";

RedisClient redis_client;

// force sensor
ForceSensorSim* force_sensor;

// display widget for forces at end effector
ForceSensorDisplay* force_display;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Sai2Model::Sai2Model* mailbox, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);
//void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);
// void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback boolean check for objects in camera FOV
bool cameraFOV(Vector3d object_pos, Vector3d camera_pos, Matrix3d camera_ori, double radius, double fov_angle);

// helper function for cameraFOV
bool compareSigns(double a, double b);

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

// flag for controlling mailbox lid
bool freezeLid = false;

int main() {
    cout << "Loading URDF world model file: " << world_file << endl;

    // start redis client
    redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    // robot->updateModel();

    // load robot objects
    auto letter = new Sai2Model::Sai2Model(letter_file, false);
    auto mailbox = new Sai2Model::Sai2Model(mailbox_file, false);
    // letter->updateModel();

    // load simulation world
    auto sim = new Simulation::Sai2Simulation(world_file, false);
    sim->setCollisionRestitution(0.0);
    //sim->setCoeffFrictionStatic(0.6);
    //sim->setCoeffFrictionDynamic(0.5);
    sim->setCoeffFrictionStatic(10);
    sim->setCoeffFrictionDynamic(10);

	// initialize force sensor: needs Sai2Simulation sim interface type
	force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);

    // read joint positions, velocities, update model
    sim->getJointPositions(robot_name, robot->_q);
    sim->getJointVelocities(robot_name, robot->_dq);
    robot->updateKinematics();

    sim->getJointPositions("letter", letter->_q);
    sim->getJointVelocities("letter", letter->_dq);
    letter->updateKinematics();

    sim->getJointPositions("mailbox", mailbox->_q);
    sim->getJointVelocities("mailbox", mailbox->_dq);
    mailbox->updateKinematics();

    /*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);
    glfwSetMouseButtonCallback(window, mouseClick);

    // init click force widget 
    auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
    ui_force_widget->setEnable(false);

    // cache variables
    double last_cursorx, last_cursory;

    // initialize glew
    glewInitialize();

    redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
	redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

    fSimulationRunning = true;
    thread sim_thread(simulation, robot, letter, mailbox, sim, ui_force_widget);
    // thread sim_thread(simulation, robot, sim, ui_force_widget);
    
    // while window is open:
    while (!glfwWindowShouldClose(window) && fSimulationRunning)
    {
        // update graphics. this automatically waits for the correct amount of time
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot_name, robot);
        graphics->updateGraphics("letter", letter);
        graphics->updateGraphics("mailbox", mailbox);
        force_display->update();
        graphics->render(camera_name, width, height);

        // swap buffers
        glfwSwapBuffers(window);

        // wait until all GL commands are completed
        glFinish();

        // check for any OpenGL errors
        GLenum err;
        err = glGetError();
        assert(err == GL_NO_ERROR);

        // poll for events
        glfwPollEvents();

        // move scene camera as required
        // graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        Eigen::Vector3d cam_depth_axis;
        cam_depth_axis = camera_lookat - camera_pos;
        cam_depth_axis.normalize();
        Eigen::Vector3d cam_up_axis;
        // cam_up_axis = camera_vertical;
        // cam_up_axis.normalize();
        cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
        Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
        cam_roll_axis.normalize();
        Eigen::Vector3d cam_lookat_axis = camera_lookat;
        cam_lookat_axis.normalize();
        if (fTransXp) {
            camera_pos = camera_pos + 0.05*cam_roll_axis;
            camera_lookat = camera_lookat + 0.05*cam_roll_axis;
        }
        if (fTransXn) {
            camera_pos = camera_pos - 0.05*cam_roll_axis;
            camera_lookat = camera_lookat - 0.05*cam_roll_axis;
        }
        if (fTransYp) {
            // camera_pos = camera_pos + 0.05*cam_lookat_axis;
            camera_pos = camera_pos + 0.05*cam_up_axis;
            camera_lookat = camera_lookat + 0.05*cam_up_axis;
        }
        if (fTransYn) {
            // camera_pos = camera_pos - 0.05*cam_lookat_axis;
            camera_pos = camera_pos - 0.05*cam_up_axis;
            camera_lookat = camera_lookat - 0.05*cam_up_axis;
        }
        if (fTransZp) {
            camera_pos = camera_pos + 0.1*cam_depth_axis;
            camera_lookat = camera_lookat + 0.1*cam_depth_axis;
        }       
        if (fTransZn) {
            camera_pos = camera_pos - 0.1*cam_depth_axis;
            camera_lookat = camera_lookat - 0.1*cam_depth_axis;
        }
        if (fRotPanTilt) {
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            //TODO: might need to re-scale from screen units to physical units
            double compass = 0.006*(cursorx - last_cursorx);
            double azimuth = 0.006*(cursory - last_cursory);
            double radius = (camera_pos - camera_lookat).norm();
            Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
            camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
            Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
            camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
        }
        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
        glfwGetCursorPos(window, &last_cursorx, &last_cursory);

        ui_force_widget->setEnable(fRobotLinkSelect);
        if (fRobotLinkSelect)
        {
            double cursorx, cursory;
            int wwidth_scr, wheight_scr;
            int wwidth_pix, wheight_pix;
            std::string ret_link_name;
            Eigen::Vector3d ret_pos;

            // get current cursor position
            glfwGetCursorPos(window, &cursorx, &cursory);

            glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
            glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

            int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
            int viewy = floor(cursory / wheight_scr * wheight_pix);

            if (cursorx > 0 && cursory > 0)
            {
                ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
                //TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
                // then drag the mouse over a link to start applying a force to it.
            }
        }
    }

    // stop simulation
    fSimulationRunning = false;
    fSimulationLoopDone = false;
    redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
    sim_thread.join();

    // destroy context
    glfwSetWindowShouldClose(window,GL_TRUE);
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

    return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Sai2Model::Sai2Model* mailbox, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
//void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
// void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000); 
    double last_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;

    // init variables
    VectorXd g(dof);
    Eigen::Vector3d ui_force;
    ui_force.setZero();
    Eigen::VectorXd ui_force_command_torques;
    ui_force_command_torques.setZero();
    VectorXd state_vector(1);
    state_vector.setZero();
    redis_client.setEigenMatrixJSON(ROBOT_STATE, state_vector);
    bool mailGripped = false;
    bool mailPlaced = false;
    Vector3d camera_pos, mailbox_pos;  // init camera detection variables 
    Matrix3d camera_ori;
    bool detect;

    // manual object offset since the offset in world.urdf file since positionInWorld() doesn't account for this 
    Vector3d mailbox_offset;
    mailbox_offset << 5.9, 1.68, 0.76;
    Vector3d robot_offset;
    robot_offset << 0.0, 0.3, 0;  

    const std::string true_message = "Detected";
    const std::string false_message = "Not Detected";
    // setup redis client data container for pipeset (batch write)
    std::vector<std::pair<std::string, std::string>> redis_data(2);

    // sensed forces and moments from sensor
    Eigen::Vector3d sensed_force;
    Eigen::Vector3d sensed_moment;

    // Update lid position 
    mailbox->_q(0) = 0;
    sim->setJointPositions("mailbox", mailbox->_q);
    mailbox->updateModel();



    while (fSimulationRunning) {
        if (fControllerLoopDone || fRobotLinkSelect) {
            if (fControllerLoopDone) {
                // read arm torques from redis and apply to simulated robot
                try {
				    command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
                } catch (...) {
                    cout << "caught redis exception" << endl;
                    break;
                }
			}
			else {
				command_torques.setZero();
			}

            // get gravity torques
            robot->gravityVector(g);

            ui_force_widget->getUIForce(ui_force);
            ui_force_widget->getUIJointTorques(ui_force_command_torques);

            if (fRobotLinkSelect)
                sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + g);
            else
                sim->setJointTorques(robot_name, command_torques + g);

            // integrate forward
            double curr_time = timer.elapsedTime();
            double loop_dt = curr_time - last_time; 
            sim->integrate(0.001);

            // read joint positions, velocities, update model
            sim->getJointPositions(robot_name, robot->_q);
            sim->getJointVelocities(robot_name, robot->_dq);
            robot->updateModel();

            // update force sensor readings
            force_sensor->update(sim);
            force_sensor->getForceLocalFrame(sensed_force);  // refer to ForceSensorSim.h in sai2-common/src/force_sensor (can also get wrt global frame)
            force_sensor->getMomentLocalFrame(sensed_moment);

            //std::cout << "Sensed Force: " << 1000 * sensed_force.transpose() << "Sensed Moment: " << 1000 * sensed_moment.transpose() << std::endl;

            try {
                state_vector = redis_client.getEigenMatrixJSON(ROBOT_STATE);
            } catch (...) {
                cout << "caught redis exception" << endl;
                break;
            }
            if (state_vector(0) == SCAN_FOR_BOX){

                // // Update lid position 
                // // sim->getJointPositions("mailbox", mailbox->_q);
                // // mailbox->_q(0) = -1.57;
                // // sim->setJointPositions("mailbox", mailbox->_q);
                // // mailbox->updateModel();

               
                // // query object position and ee pos/ori for camera detection 
                // mailbox->positionInWorld(mailbox_pos, "link0");
                // robot->positionInWorld(camera_pos, "link7");
                // robot->rotationInWorld(camera_ori, "link7");  // local to world frame 


                // // add position offset in world.urdf file since positionInWorld() doesn't account for this 
                // mailbox_pos += mailbox_offset;
                // camera_pos += robot_offset;  // camera position/orientation is set to the panda's last link

                // // object camera detect 
                // detect = cameraFOV(mailbox_pos, camera_pos, camera_ori, 2.0, M_PI);
                // if (detect == true) {
                //     /*mailbox_pos(0) += dist(generator);  // add white noise 
                //     mailbox_pos(1) += dist(generator);
                //     mailbox_pos(2) += dist(generator);*/
                //     VectorXd detection_vector(1);
                //     detection_vector(0) = 1;
                //     redis_client.setEigenMatrixJSON(DETECTION_STATE, detection_vector);
                //     redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, true_message);
                //     redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(mailbox_pos));
                // }
                // else {
                //     redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, false_message);
                //     redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(Vector3d::Zero()));
                // }
            } else if(!mailGripped && state_vector(0) != PLACE_MAIL) {
                letter->_q(1) = -robot->_q(0);
                sim->setJointPositions("letter", letter->_q);
                VectorXd letter_vel(letter->dof());
                letter_vel.setZero();
                sim->setJointVelocities("letter", letter_vel);
            } else if (state_vector(0) == BACKOUT) {
                // hack for now: prevent the gripper from dragging the letter with friction
                VectorXd letter_vel(letter->dof());
                letter_vel.setZero();
                sim->setJointVelocities("letter", letter_vel);
            } else {
                sim->getJointPositions("letter", letter->_q);
                sim->getJointVelocities("letter", letter->_dq);
                mailGripped = true;
            }
            letter->updateModel();


            // if (state_vector(0) == OPEN_BOX || state_vector(0) == CLOSE_BOX) {
            //     sim->getJointPositions("mailbox", mailbox->_q);
            //     sim->getJointVelocities("mailbox", mailbox->_dq);
            //     mailbox->updateModel();
            // } else if (state_vector(0) == PLACE_MAIL || state_vector(0) == GRAB_MAIL) {
            //     mailbox->_q(0) = -1.57;
            //     sim->setJointPositions("mailbox", mailbox->_q);
            //     VectorXd lid_vel(mailbox->dof());
            //     lid_vel.setZero();
            //     sim->setJointVelocities("mailbox", lid_vel);
            //     mailbox->updateModel();
            // } else {
            //     mailbox->_q(0) = 0;
            //     sim->setJointPositions("mailbox", mailbox->_q);
            //     VectorXd lid_vel(mailbox->dof());
            //     lid_vel.setZero();
            //     sim->setJointVelocities("mailbox", lid_vel);
            //     mailbox->updateModel(); 
            // }

            if (state_vector(0) == WAIT_FOR_BOX) {
                mailbox->_q(0) = 0;
                sim->setJointPositions("mailbox", mailbox->_q);
                VectorXd lid_vel(mailbox->dof());
                lid_vel.setZero();
                sim->setJointVelocities("mailbox", lid_vel);
            } else if (state_vector(0) == OPEN_BACKOUT) {
                mailbox->_q(0) = -M_PI/2;
                sim->setJointPositions("mailbox", mailbox->_q);
                VectorXd lid_vel(mailbox->dof());
                lid_vel.setZero();
                sim->setJointVelocities("mailbox", lid_vel);
            } else {
                if (state_vector(0) == RETRACT_ARM || state_vector(0) == CLOSE_BOX && mailbox->_q(0) > 0) {
                    freezeLid = true;
                }
                if (freezeLid) {
                    mailbox->_q(0) = 0;
                    sim->setJointPositions("mailbox", mailbox->_q);
                    VectorXd lid_vel(mailbox->dof());
                    lid_vel.setZero();
                    sim->setJointVelocities("mailbox", lid_vel);
                } else {
                    sim->getJointPositions("mailbox", mailbox->_q);
                    sim->getJointVelocities("mailbox", mailbox->_dq);
                }
            }
            mailbox->updateModel();

            

            // simulation loop is done
	        fSimulationLoopDone = true;

	        // ask for next control loop
	        fControllerLoopDone = false;

            try {
                // write new robot state to redis
                redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
                redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
                redis_client.setEigenMatrixJSON(LETTER_JOINT_ANGLES_KEY, letter->_q);
                redis_client.setEigenMatrixJSON(LETTER_JOINT_VELOCITIES_KEY, letter->_dq);
                redis_client.setEigenMatrixJSON(EE_FORCE_KEY, sensed_force);
                redis_client.setEigenMatrixJSON(EE_MOMENT_KEY, sensed_moment);

                redis_data.at(0) = std::pair<string, string>(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
                redis_data.at(1) = std::pair<string, string>(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone)); // ask for next control loop
                redis_client.pipeset(redis_data);
            } catch (...) {
                cout << "caught redis exception" << endl;
                break;
            }

            //update last time
            last_time = curr_time;
        }

        // read controller state
        try {
            fControllerLoopDone = string_to_bool(redis_client.get(CONTROLLER_LOOP_DONE_KEY));
        } catch (...) {
            cout << "caught redis exception" << endl;
            break;
        }
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
    cerr << "GLFW Error: " << description << endl;
    exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
    bool ret = false;
    #ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) {
        cout << "Failed to initialize GLEW library" << endl;
        cout << glewGetErrorString(ret) << endl;
        glfwTerminate();
    } else {
        ret = true;
    }
    #endif
    return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            fSimulationRunning = false;
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            fTransXp = set;
            break;
        case GLFW_KEY_LEFT:
            fTransXn = set;
            break;
        case GLFW_KEY_UP:
            fTransYp = set;
            break;
        case GLFW_KEY_DOWN:
            fTransYn = set;
            break;
        case GLFW_KEY_A:
            fTransZp = set;
            break;
        case GLFW_KEY_Z:
            fTransZn = set;
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    bool set = (action != GLFW_RELEASE);
    //TODO: mouse interaction with robot
    switch (button) {
        // left click pans and tilts
        case GLFW_MOUSE_BUTTON_LEFT:
            fRotPanTilt = set;
            // NOTE: the code below is recommended but doesn't work well
            // if (fRotPanTilt) {
            //  // lock cursor
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            // } else {
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            // }
            break;
        // if right click: don't handle. this is for menu selection
        case GLFW_MOUSE_BUTTON_RIGHT:
            fRobotLinkSelect = set;
            break;
        // if middle click: don't handle. doesn't work well on laptops
        case GLFW_MOUSE_BUTTON_MIDDLE:
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------

bool cameraFOV(Vector3d object_pos, Vector3d camera_pos, Matrix3d camera_ori, double radius, double fov_angle) {
    // init
    Vector3d a, b, c, d;
    // Vector3d normal = camera_ori.col(2);  // normal vector in world frame 

    // local camera frame vertex coordinates 
    Vector3d v1, v2, v3; 
    v1 << 0, -radius*tan(fov_angle), radius;
    v2 << radius*tan(fov_angle)*cos(M_PI/6), radius*tan(fov_angle)*sin(M_PI/6), radius;
    v3 << -radius*tan(fov_angle)*cos(M_PI/6), radius*tan(fov_angle)*sin(M_PI/6), radius;

    // world frame vertex coordinates centered at the object 
    a = camera_pos - object_pos;
    b = camera_pos + camera_ori*v1 - object_pos;
    c = camera_pos + camera_ori*v2 - object_pos;
    d = camera_pos + camera_ori*v3 - object_pos;

    // calculate if object position is inside tetrahedron 
    vector<double> B(4);
    B.at(0) = ( -1*(b(0)*c(1)*d(2) - b(0)*c(2)*d(1) - b(1)*c(0)*d(2) + b(1)*c(2)*d(0) + b(2)*c(0)*d(1) - b(2)*c(1)*d(0)) );
    B.at(1) = ( a(0)*c(1)*d(2) - a(0)*c(2)*d(1) - a(1)*c(0)*d(2) + a(1)*c(2)*d(0) + a(2)*c(0)*d(1) - a(2)*c(1)*d(0) );
    B.at(2) = ( -1*(a(0)*b(1)*d(2) - a(0)*b(2)*d(1) - a(1)*b(0)*d(2) + a(1)*b(2)*d(0) + a(2)*b(0)*d(1) - a(2)*b(1)*d(0)) );
    B.at(3) = ( a(0)*b(1)*c(2) - a(0)*b(2)*c(1) - a(1)*b(0)*c(2) + a(1)*b(2)*c(0) + a(2)*b(0)*c(1) - a(2)*b(1)*c(0) );
    double detM = B.at(0) + B.at(1) + B.at(2) + B.at(3);

    // sign check
    bool test;
    for (int i = 0; i < B.size(); ++i) {
        test = compareSigns(detM, B.at(i));
        if (test == false) {
            return false;
        }
    }
    return true;
}

//------------------------------------------------------------------------------

bool compareSigns(double a, double b) {
    if (a > 0 && b > 0) {
        return true;
    }
    else if (a < 0 && b < 0) {
        return true;
    }
    else {
        return false;
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

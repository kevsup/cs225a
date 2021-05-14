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

using namespace std;
using namespace Eigen;

// redis keys:
// - write:
// const std::string EE_FORCE_KEY = "sai2::cs225a::sensor::force";
// const std::string EE_MOMENT_KEY = "sai2::cs225a::sensor::moment";

RedisClient redis_client;

// force sensor
// ForceSensorSim* force_sensor;

// display widget for forces at end effector
// ForceSensorDisplay* force_display;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);
// void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

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
    // letter->updateModel();

    // load simulation world
    auto sim = new Simulation::Sai2Simulation(world_file, false);
    sim->setCollisionRestitution(0.0);
    //sim->setCoeffFrictionStatic(0.6);
    //sim->setCoeffFrictionDynamic(0.5);
    sim->setCoeffFrictionStatic(10);
    sim->setCoeffFrictionDynamic(10);

    // read joint positions, velocities, update model
    sim->getJointPositions(robot_name, robot->_q);
    sim->getJointVelocities(robot_name, robot->_dq);
    robot->updateKinematics();

    sim->getJointPositions("letter", letter->_q);
    sim->getJointVelocities("letter", letter->_dq);
    letter->updateKinematics();

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

    fSimulationRunning = true;
    thread sim_thread(simulation, robot, letter, sim, ui_force_widget);
    // thread sim_thread(simulation, robot, sim, ui_force_widget);
    
    // while window is open:
    while (!glfwWindowShouldClose(window) && fSimulationRunning)
    {
        // update graphics. this automatically waits for the correct amount of time
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot_name, robot);
        graphics->updateGraphics("letter", letter);
        // force_display->update();
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
    sim_thread.join();

    // destroy context
    glfwSetWindowShouldClose(window,GL_TRUE);
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

    return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* letter, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {
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

    // sensed forces and moments from sensor
    // Eigen::Vector3d sensed_force;
 //    Eigen::Vector3d sensed_moment;

    while (fSimulationRunning) {
        fTimerDidSleep = timer.waitForNextLoop();

        // get gravity torques
        robot->gravityVector(g);

        // read arm torques from redis and apply to simulated robot
        command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
        
        ui_force_widget->getUIForce(ui_force);
        ui_force_widget->getUIJointTorques(ui_force_command_torques);

        if (fRobotLinkSelect)
            sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + g);
        else
            sim->setJointTorques(robot_name, command_torques + g);

        // integrate forward
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time; 
        sim->integrate(loop_dt);

        // read joint positions, velocities, update model
        sim->getJointPositions(robot_name, robot->_q);
        sim->getJointVelocities(robot_name, robot->_dq);
        robot->updateModel();

        state_vector = redis_client.getEigenMatrixJSON(ROBOT_STATE);
        if (!mailGripped && state_vector(0) != PLACE_MAIL) {
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

        // write new robot state to redis
        redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
        redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
        redis_client.setEigenMatrixJSON(LETTER_JOINT_ANGLES_KEY, letter->_q);
        redis_client.setEigenMatrixJSON(LETTER_JOINT_VELOCITIES_KEY, letter->_dq);
        // redis_client.setEigenMatrixJSON(EE_FORCE_KEY, sensed_force);
        // redis_client.setEigenMatrixJSON(EE_MOMENT_KEY, sensed_moment);

        //update last time
        last_time = curr_time;
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


#include <string>
#include <vector>

using namespace std;

typedef enum {INIT, WAIT_FOR_BOX, SCAN_FOR_BOX, OPEN_BOX, GRAB_MAIL, PLACE_MAIL, CLOSE_BOX, RETRACT_ARM} States;
typedef enum {MOVE_OVERHEAD, DROP_DOWN, GRIP_PARCEL} MailStates;
typedef enum {FRONT, INSIDE, RELEASE, BACKOUT} PlaceStates;
typedef enum {MOVE_IN_FRONT, MOVE_TO_HANDLE, GRIP_HANDLE, PAIN, UNGRIP_HANDLE, OPEN_BACKOUT} BoxStates;
typedef enum {MOVE_IN_FRONT_CLOSE, MOVE_TO_HANDLE_CLOSE, GRIP_HANDLE_CLOSE, PAIN_CLOSE, UNGRIP_HANDLE_CLOSE, CLOSE_BACKOUT} CloseStates;

const int BACKOUT_MESSAGE_ENCODING = 142857;
const int OPEN_BACKOUT_MESSAGE_ENCODING = 285714;
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string LETTER_JOINT_ANGLES_KEY = "sai2::cs225a::letter::sensors::q";
const std::string LETTER_JOINT_VELOCITIES_KEY = "sai2::cs225a::letter::sensors::dq";
const std::string MAILBOX_JOINT_ANGLES_KEY = "sai2::cs225a::mailbox::sensors::q";
const std::string MAILBOX_JOINT_VELOCITIES_KEY = "sai2::cs225a::mailbox::sensors::dq";
const std::string ROBOT_STATE = "sai2::cs225a::project::arm::state";
const std::string DETECTION_STATE = "sai2::cs225a::project::arm::detection";
const std::string CAMERA_DETECT_KEY = "sai2::cs225a::camera::detect";
const std::string CAMERA_OBJ_POS_KEY = "sai2::cs225a::camera::obj_pos";
const std::string SIMULATION_LOOP_DONE_KEY = "cs225a::simulation::done";
const std::string CONTROLLER_LOOP_DONE_KEY = "cs225a::controller::done";
const std::string CAMERA_TRACK_KEY = "cs225a::camera::track";

const string world_file = "./resources/world_mailbot.urdf";
const string robot_file = "./resources/mmp_panda.urdf";
const string robot_name = "mmp_panda";
const string camera_name = "camera_fixed";
const string letter_file = "./resources/letter.urdf";
const vector<string> box_files{"./resources/box1.urdf", "./resources/box2.urdf", "./resources/box3.urdf"};
const string mailbox_file = "./resources/mailbox.urdf";

const double LETTER_GAP = 0.35;
const double HOUSE_OFFSET = 6;

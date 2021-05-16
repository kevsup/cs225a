#include <string>

typedef enum {INIT, WAIT_FOR_BOX, SCAN_FOR_BOX, OPEN_BOX, GRAB_MAIL, PLACE_MAIL, CLOSE_BOX, RETRACT_ARM} States;
typedef enum {MOVE_OVERHEAD, DROP_DOWN, GRIP_PARCEL} MailStates;
typedef enum {FRONT, INSIDE, RELEASE, BACKOUT} PlaceStates;

const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string LETTER_JOINT_ANGLES_KEY = "sai2::cs225a::letter::sensors::q";
const std::string LETTER_JOINT_VELOCITIES_KEY = "sai2::cs225a::letter::sensors::dq";
const std::string ROBOT_STATE = "sai2::cs225a::project::arm::state";
const std::string CAMERA_DETECT_KEY = "sai2::cs225a::camera::detect";
const std::string CAMERA_OBJ_POS_KEY = "sai2::cs225a::camera::obj_pos";

const string world_file = "./resources/world_mailbot.urdf";
const string robot_file = "./resources/mmp_panda.urdf";
const string robot_name = "mmp_panda";
const string camera_name = "camera_fixed";
const string letter_file = "./resources/letter.urdf";
const string mailbox_file = "./resources/mailbox.urdf";

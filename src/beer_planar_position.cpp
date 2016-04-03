// for planar position experiments with ins gamepad
// subscribe to topic "ins_gamepad"
// setup service client to "/gazebo/set_model_state"

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <ins_gamepad_interaction/ins_gamepad.h>

// global variables
ins_gamepad_interaction::ins_gamepad gamepad_data;
bool gamepad_data_updated = false;  // flag for new data
float px = 0;  // position of the beer
float py = 0;

// callback for topic "ins_gamepad"
void gamepadCallback(const ins_gamepad_interaction::ins_gamepad& msg_holder) {
    gamepad_data = msg_holder;
    gamepad_data_updated = true;
}

float absolute(float u) {
    if (u >= 0) return u;
    else return -u;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "beer_planar_position");  // name of the node
    ros::NodeHandle nh;  // node handle
    // subscribe to topic "ins_gamepad"
    ros::Subscriber gamepad_subscriber = nh.subscribe("ins_gamepad", 1, gamepadCallback);

    // initialize a service client to set model state
    ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>(
        "/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state_srv_msg;

    // prepare some service request data
    set_model_state_srv_msg.request.model_state.model_name = "beer";
    set_model_state_srv_msg.request.model_state.pose.position.z = 0.0;  // on the floor
    set_model_state_srv_msg.request.model_state.pose.orientation.x = 0;
    set_model_state_srv_msg.request.model_state.pose.orientation.y = 0;


    // the loop
    float squaresum;
    double time_last, time_d;  // last time stamp and duration
    bool time_initialized = false;  // flag to initialize time
    while (ros::ok()) {
        if (gamepad_data_updated) {
            if (gamepad_data.button_g == true) {
                // press green button, reset beer to original location, in case vel drift
                // this button sometimes doesn't work well
                ros::Duration(1.0).sleep();  // avoid multiple detection
                px = 0.0; py = 0.0;  // reset accumulating position
                set_model_state_srv_msg.request.model_state.pose.position.x = 0.0;
                set_model_state_srv_msg.request.model_state.pose.position.y = 0.0;
                set_model_state_srv_msg.request.model_state.pose.position.z = 0.0;
                set_model_state_srv_msg.request.model_state.pose.orientation.w = 1.0;
                set_model_state_srv_msg.request.model_state.pose.orientation.x = 0.0;
                set_model_state_srv_msg.request.model_state.pose.orientation.y = 0.0;
                set_model_state_srv_msg.request.model_state.pose.orientation.z = 0.0;
                set_model_state_client.call(set_model_state_srv_msg);
                time_initialized = false;  // also to reset time initialization stamp
            }
            // flow control base on initializatio of integration time
            if (!time_initialized) {
                time_last =ros::Time::now().toSec();
                time_initialized = true;
            }
            else {
                // the integration for relative 2D position
                // get time duration
                double time_now = ros::Time::now().toSec();
                time_d = time_now - time_last;  // the duration
                time_last = time_now;
                // there are occisional bad data in the quaternion to be deleted
                squaresum = pow(gamepad_data.q0_klm, 2) + pow(gamepad_data.q1_klm, 2)
                    + pow(gamepad_data.q2_klm, 2) + pow(gamepad_data.q3_klm, 2);
                if  (absolute(squaresum - 1) < 0.2) {
                    // some quaternion don't even have square sum of 1
                    // set the orientation around vertical axis
                    set_model_state_srv_msg.request.model_state.pose.orientation.w = gamepad_data.q0_klm;
                    set_model_state_srv_msg.request.model_state.pose.orientation.z =
                        sqrt(1 - pow(gamepad_data.q0_klm, 2));
                    // calculate the 2D position
                    double psi = acos(gamepad_data.q0_klm);
                    px = px + (gamepad_data.vx * cos(psi) - gamepad_data.vy * sin(psi)) * time_d;
                    py = py + (gamepad_data.vx * sin(psi) + gamepad_data.vy * cos(psi)) * time_d;
                    set_model_state_srv_msg.request.model_state.pose.position.x = px;
                    set_model_state_srv_msg.request.model_state.pose.position.y = py;
                    set_model_state_client.call(set_model_state_srv_msg);
                    ROS_INFO_STREAM("px: " << px << "; py: " << py);
                    // sometimes px and py goes to nan, why?
                }
            }
            gamepad_data_updated = false;  // reset the data update flag
        }
        ros::spinOnce();
    }
}



// for visualization of 3D orientation data from ins gamepad
// subscribe to topic "ins_gamepad"
// setup service client to "/gazebo/set_model_state"

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <ins_gamepad_interaction/ins_gamepad.h>

// global variables
ins_gamepad_interaction::ins_gamepad gamepad_data;
bool gamepad_data_updated = false;  // flag for new data

// callback for topic "ins_gamepad"
void gamepadCallback(const ins_gamepad_interaction::ins_gamepad& msg_holder) {
    gamepad_data = msg_holder;
    gamepad_data_updated = true;
}

// why the abs() function doesn't work here agian?
float absolute(float u) {
    if (u >= 0) return u;
    else return -u;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drill_3D_orientation");  // name of the node
    ros::NodeHandle nh;  // node handle
    // subscribe to topic "ins_gamepad"
    ros::Subscriber gamepad_subscriber = nh.subscribe("ins_gamepad", 1, gamepadCallback);

    // initialize a service client to set model state
    ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>(
        "/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state_srv_msg;

    // prepare some service request data
    set_model_state_srv_msg.request.model_state.model_name = "drill";
    set_model_state_srv_msg.request.model_state.pose.position.x = 0.0;
    set_model_state_srv_msg.request.model_state.pose.position.y = 0.0;
    set_model_state_srv_msg.request.model_state.pose.position.z = 0.2;  // set to 1m hieight
    
    // the loop
    float squaresum;
    while (ros::ok()) {
        if (gamepad_data_updated) {
            // there are some bad data in the quaternion that make the drill flashes
            // need to delete these data
            squaresum = pow(gamepad_data.q0_klm, 2) + pow(gamepad_data.q1_klm, 2)
                + pow(gamepad_data.q2_klm, 2) + pow(gamepad_data.q3_klm, 2);
            if  (absolute(squaresum - 1) < 0.2) {
                // some quaternion don't even has square sum of 1
                set_model_state_srv_msg.request.model_state.pose.orientation.w = gamepad_data.q0_klm;
                set_model_state_srv_msg.request.model_state.pose.orientation.x = gamepad_data.q1_klm;
                set_model_state_srv_msg.request.model_state.pose.orientation.y = gamepad_data.q2_klm;
                set_model_state_srv_msg.request.model_state.pose.orientation.z = gamepad_data.q3_klm;
                set_model_state_client.call(set_model_state_srv_msg);
            }
            gamepad_data_updated = false;
        }
        ros::spinOnce();
    }
}



#include <ros/ros.h>
#include <cmath>

#include <s8_common_node/Node.h>
#include <s8_utils/math.h>
#include <s8_go_straight_controller/go_straight_controller_node.h>
#include <s8_pid/PIDController.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <s8_pose/pose_node.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_go_straight_controller/GoStraightAction.h>

#define HZ                              25

#define TOPIC_POSE                  s8::pose_node::TOPIC_POSE_SIMPLE

using namespace s8;
using namespace s8::go_straight_controller_node;
using namespace s8::pid;
using namespace s8::utils::math;

class GoStraight : public Node {
    ros::Subscriber pose_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_go_straight_controller::GoStraightAction> go_straight_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    PIDController align_pid;

    bool aligning, recieved_goal;
    int robot_rotation;
    int goal_roation;

    bool going;
    double linear_speed;
    double v, w;

public:
    GoStraight(int hz) : align_pid(hz), v(0.0), w(0.0), aligning(false), going(false), stop_action(ACTION_STOP, true), go_straight_action(nh, ACTION_GO_STRAIGHT, boost::bind(&GoStraight::action_execute_go_straight_callback, this, _1), false) {
        pose_subscriber = nh.subscribe<geometry_msgs::Pose2D>(TOPIC_POSE, 1, &GoStraight::pose_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);
        
        go_straight_action.registerPreemptCallback(boost::bind(&GoStraight::go_straight_cancel_callback, this));
        go_straight_action.start();

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

    void update() {
        if(!going) {
            return;
        }
        //Do controller stuff:

        publish();
    }

    void stop() {
        ROS_INFO("Stopping...");
        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            ROS_INFO("Stop action finished. %s", state.toString().c_str());
        } else {
            ROS_WARN("Stop action timed out.");
        }
    }
    
private:
    void go_straight_cancel_callback() {
        stop_going_straight();
        ROS_INFO("Cancelling go straight action...");
    }

    void stop_going_straight() {
        aligning = false;
        going = false;
        v = 0.0;
        w = 0.0;
    }

    void action_execute_go_straight_callback(const s8_go_straight_controller::FollowWallGoalConstPtr & goal) {
        
    }

    void publish() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        // Check for too high angular speed
        if (w > 1.5 || w < -1.5)  {
            //ROS_WARN("Possibly bad IR sensor value");
            w = 0;  
        }

        twist.angular.z = w;
        twist_publisher.publish(twist);
    }

    void init_params() {
        
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    GoStraight go_straight(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        go_straight.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

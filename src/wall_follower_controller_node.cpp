#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <s8_msgs/IRDistances.h>

#include <s8_common_node/Node.h>

#define NODE_NAME                       "s8_wall_follower_controller_node"
#define HZ                              10

#define TOPIC_TWIST                     "/s8/twist"
#define TOPIC_IR_DISTANCES              "/s8/ir_distances"

#define PARAM_KP_NAME                   "kp"
#define PARAM_KP_DEFAULT                6.0
#define PARAM_KD_NAME                   "kd"
#define PARAM_KD_DEFAULT                6.0
#define PARAM_KP_NEAR_NAME              "kp_near"
#define PARAM_KP_NEAR_DEFAULT           1.0
#define PARAM_KP_FAR_NAME               "kp_far"
#define PARAM_KP_FAR_DEFAULT            0.2
#define PARAM_DISTANCE_NAME             "distance"
#define PARAM_DISTANCE_DEFAULT          0.1

#define IR_INVALID_VALUE                -1.0

class WallFollower : public s8::Node {
    ros::Subscriber ir_distances_subscriber;
    ros::Publisher twist_publisher;

    double kp_near;
    double kp_far;
    double kp;
    double kd;
    double prev_error;
    double distance;

    double left_front;
    double left_back;
    double right_front;
    double right_back;

    double v;
    double w;
    double w_temp;

public:
    WallFollower() : v(0.0), w(0.0), left_front(IR_INVALID_VALUE), left_back(IR_INVALID_VALUE), right_front(IR_INVALID_VALUE), right_back(IR_INVALID_VALUE), prev_error(0.0) {
        init_params();
        print_params();

        ir_distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1000, &WallFollower::ir_distances_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1000);
    }

    void update() {
        if(false && is_ir_valid_value(left_front) && is_ir_valid_value(left_back)) {
            //Do left wall following

            ROS_INFO("Following left wall... back: %.2lf, front: %.2lf", left_back, left_front);
        } else if(is_ir_valid_value(right_front) && is_ir_valid_value(right_back)) {
            //Do right wall following

            double diff = right_front - right_back;
            double average = (right_back + right_front) / 2 ;

            w = -kp * diff;
            w += -kd * (diff-prev_error) / 2;
            prev_error = diff;

            if (average-distance < 0) {
                // if too close to the wall, turn left fast
                w += kp_near * (distance - average);
            } else {
                //if too far away to the wall, turn right slowly
                w_temp = -kp_far * (distance - average);

                if(w_temp < -0.2) {
                    w_temp = -0.2;
                }

                w += w_temp;
            }

            v = 0.3;

            ROS_INFO("Following right wall... back: %.2lf, front: %.2lf, diff %lf, w: %.2lf", right_back, right_front, diff, w);
        } else {
            //Stop.
            v = 0.0;
            w = 0.0;
            ROS_INFO("No walls in range. left_back: %.2lf, left_front: %.2lf, right_back: %.2lf, right_front: %.2lf", left_back, left_front, right_back, right_front);
        }

        publish();
    }
    
private:
    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        left_front = ir_distances->left_front;
        left_back = ir_distances->left_back;
        right_front = ir_distances->right_front;
        right_back = ir_distances->right_back;
    }

    bool is_ir_valid_value(double value) {
        return value > 0.0;
    }

    void publish() {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;

        twist_publisher.publish(twist);
    }

    void init_params() {
        add_param(PARAM_KP_NAME, kp, PARAM_KP_DEFAULT);
        add_param(PARAM_KD_NAME, kd, PARAM_KD_DEFAULT);
        add_param(PARAM_KP_NEAR_NAME, kp_near, PARAM_KP_NEAR_DEFAULT);
        add_param(PARAM_KP_FAR_NAME, kp_far, PARAM_KP_FAR_DEFAULT);
        add_param(PARAM_DISTANCE_NAME, distance, PARAM_DISTANCE_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    WallFollower wall_follower;

    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        wall_follower.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

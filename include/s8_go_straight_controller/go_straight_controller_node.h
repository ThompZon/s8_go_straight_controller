#ifndef __GO_STRAIGHT_CONTROLLER_NODE_H
#define __GO_STRAIGHT_CONTROLLER_NODE_H

#include <string>
#include <s8_motor_controller/motor_controller_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>

namespace s8 {
    namespace go_straight_controller_node {
        const std::string NODE_NAME =                       "s8_go_straight_controller_node";

        const std::string TOPIC_IR_DISTANCES =              s8::ir_sensors_node::TOPIC_IR_DISTANCES;
        const std::string TOPIC_TWIST =                     s8::motor_controller_node::TOPIC_TWIST;
        const std::string ACTION_STOP =                     s8::motor_controller_node::ACTION_STOP;
        const std::string ACTION_GO_STRAIGHT =              "/s8/go_straight";

        enum GoStraightFinishedReason {
            TIMEOUT,
            FOUND_WALL,
            PREEMPTED
        };

        enum Direction {
            EAST = s8::,
            NORTH = 90,
            WEST = 180,
            SOUTH = 270
            LEFT = s8::motor_controller_node::RotationDirection::LEFT,
            RIGHT = s8::motor_controller_node::RotationDirection::RIGHT
        };

        std::string to_string(WallToFollow wall_to_follow) {
            switch(wall_to_follow) {
                case WallToFollow::LEFT: return "LEFT";
                case WallToFollow::RIGHT: return "RIGHT";
            }
            return "UNKNOWN";
        }
    }
}

#endif

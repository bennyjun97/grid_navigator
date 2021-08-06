# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import atexit
import math
import numpy as np


class Navigator(Node):

    def __init__(self):
        super().__init__('minicoco_main')
        # parameter server 사용하기
        self.path_thres = 100
        self.marker_scope_upp = 5  # -12 (rack) or 5 (flat)
        self.marker_scope_low = 9  # -8 (rack) or 9 (flat)
        self.dist_to_marker = 0.52  # 0.73 (rack) or 0.52 (flat)
        self.turning_90 = math.pi/2*1.05
        self.forward_speed = 0.07
        self.turning_speed = 0.3
        self.p_control_coeff = 0.003
        self.p_control_thres = 10  # 30 (rack) or 10 (flat)
        self.standby = True
        self.move_index = 0
        self.moves = []
        # location and orientation of the robot
        self.max_row = 3
        self.max_col = 3
        self.row = 1
        self.col = 1
        self.orientation = 1  # 1234 = NWSE
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'pathway_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos)

    def listener_callback(self, msg):
        # go back to standby mode if movement is all done
        if self.standby is False and self.move_index >= len(self.moves):
            self.move_index = 0
            self.standby = True
            # print('task complete, waiting for new task')
            self.get_logger().info('task complete, waiting for new task')
        # if standby, input from user to plan movement
        if self.standby:
            column_id = input('Enter movement or the target column ID: ')
            self.moves, valid = self.get_moves(column_id)
            if valid is False:
                print('error: invalid input')
                return
            self.standby = False
            self.move_index = 0
        err = msg.data[0]
        marker_err = msg.data[1]
        # if marker is in the scope, go front for certain distance
        # if it is time to turn, turn 90 degrees
        if marker_err > self.marker_scope_upp and marker_err < self.marker_scope_low:
            self.go_and_turn(self.moves[self.move_index])
            self.move_index += 1
        # stop if no path is found
        elif err == 1000:
            # print('unable to detect pathway')
            self.get_logger().info('unable to detect pathway')
            self.stop_going()
        # follow pathway using p controller
        else:
            self.follow_pathway(err)

    # index 0 is skipping marker, 1 is turning ccw, 2 is turning cw
    def go_and_turn(self, command):
        # print('marker is in scope')
        self.get_logger().info('marker is in scope')
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.publisher_.publish(twist)
        # print('stepping on the marker...')
        self.get_logger().info('stepping on the marker...')
        time.sleep(self.dist_to_marker/twist.linear.x)
        twist.linear.x = 0.0
        if command == 1 or command == 2:
            twist.angular.z = self.turning_speed
            if command == 2:
                twist.angular.z *= -1
            self.publisher_.publish(twist)
            # print('turning 90 degrees...')
            self.get_logger().info('turning 90 degrees...')
            time.sleep(self.turning_90/np.abs(twist.angular.z))
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.2)
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        # print('following path again')
        self.get_logger().info('following pathway again')

    # stop the robot
    def stop_going(self):
        twist = Twist()
        self.publisher_.publish(twist)
        # print('robot stopped')
        self.get_logger().info('robot stopped')

    # p control to follow the pathway
    def follow_pathway(self, err):
        twist = Twist()
        twist.linear.x = self.forward_speed
        if err > self.path_thres:
            err = self.path_thres
        if err < -1*self.path_thres:
            err = -1*self.path_thres
        if np.abs(err) > self.p_control_thres:
            if err > 0:
                twist.angular.z = -1*self.p_control_coeff*(err-self.p_control_thres)
            else:
                twist.angular.z = -1*self.p_control_coeff*(err+self.p_control_thres)
        self.publisher_.publish(twist)

    # given id, return moves, as an array of 0, 1, and 2.
    def get_moves(self, id):
        # READ CAREFULLY
        # column counts from right to left, row counts from bottom to top
        # origin is bottom right intersection (marker)
        # id is RRCC (row and column)
        # ex) 0304 represents 3rd row and 4th column
        moves = []
        valid = self.check_valid(id)
        if valid:
            if id[0] == '#':
                ver_move = int(id[1:3])
                hor_move = int(id[3:5])-1
                if hor_move == 0:
                    hor_move = 1
                ver_hor = [ver_move, hor_move]
                for i in ver_hor:
                    for j in range(i):
                        if j != i-1:
                            moves += [0]
                        else:
                            moves += [1]
                # double the moves to close the loop
                moves += moves
                # print('moves:', moves)
            else:
                for i in id:
                    moves += [int(i)]
        return moves, valid

    # check validity of the user's input
    def check_valid(self, id):
        if id[0] == '#':
            try:
                int(id[1:3])
                int(id[3:5])
            except BaseException:
                return False
            if len(id) != 5:
                print('ID is too long')
                return False
            if not (self.col == 1 and self.row == 1 and self.orientation == 1):
                print('robot is not in the original location and orientation')
                return False
        else:
            # copy location and orientation
            col = self.col
            row = self.row
            ori = self.orientation
            for i in id:
                if i not in ['0', '1', '2']:
                    print('for movement, number should be 0, 1, or 2')
                    return False
                if ori == 1:
                    row += 1
                elif ori == 2:
                    col += 1
                elif ori == 3:
                    row -= 1
                elif ori == 4:
                    col -= 1
                if col > self.max_col or col < 1 or row > self.max_row or row < 1:
                    print('robot will crash', col, row, ori)
                    return False
                else:
                    pass
                    # print('ok', col, row, ori)
                turn = int(i)
                if turn == 2:
                    turn = -1
                # change orientation
                ori += turn
                if ori == 5:
                    ori = 1
                if ori == 0:
                    ori = 4
            if ori == 1:
                row += 1
            elif ori == 2:
                col += 1
            elif ori == 3:
                row -= 1
            elif ori == 4:
                col -= 1
            if col > self.max_col or col < 1 or row > self.max_row or row < 1:
                print('robot will crash', col, row, ori)
                return False
        return True


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()

    atexit.register(Navigator.stop_going, navigator)

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

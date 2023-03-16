#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

import baxter_interface
import moveit_commander


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

        # To let spwan_chessboard know to spawn next piece
        self._publish_move = rospy.Publisher("spawn_next", Empty, queue_size=5)



    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def pnp_callback(piece_place_loc, pnp):

    # Shift Z coords to convert from rviz to gazebo
    piece_place_loc.position.z = piece_place_loc.position.z - 0.93

    # Match orientation for gripper to be overhead and parallel to chess piece
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    piece_place_loc.orientation = overhead_orientation

    print("Starting Picking")
    pnp.pick(Pose(position=Point(x=0.6, y=0.6, z=0.78-0.93),orientation=overhead_orientation))
    print("Starting Placing")
    pnp.place(piece_place_loc)
    print('Placing Occured')
    
    # Placing has complete, ping spawn_chessboard to start next loop/spawn next piece ready for pnp
    pnp._publish_move.publish(Empty())

def play_chess(data, args):

    print("\nPlaying chess")
    pnp = args[0]
    overhead_orientation = args[1]
    positions = rospy.get_param('piece_target_position_map')

    # row, col of r0, r7, k2, K7, R0, R7
    pick_list = ['20', '70', '31', '77', '07', '56']
    # row, col to move the pieces to
    place_list = ['31', '71', '30', '75', '06', '55']

    pick_poses = list()
    place_poses = list()

    # loop through pick an place lists and get coordinates of each grid
    for pick in pick_list:
        coord = positions[pick]
        pick_poses.append(Pose(position=Point(x = coord[0], y = coord[1], z = coord[2]), orientation=overhead_orientation))
    
    for place in place_list:
        coord = positions[place]
        place_poses.append(Pose(position=Point(x = coord[0], y = coord[1], z = coord[2]), orientation=overhead_orientation))
    
    # perform 6 moves
    for i in range(6):
        if rospy.is_shutdown():
            break
        print("\nPicking from %s and placing at %s" % (str(pick_list[i]),str(place_list[i])))
        pnp.pick(pick_poses[i])
        pnp.place(place_poses[i])


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters

    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

    starting_pose = Pose(position=Point(x=0.7, y=0.135, z=0.35),orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Move to the desired starting angles
    pnp.move_to_start(starting_pose)

    # Subscribing to spawn_chessboard to place the piece in the correct position and invoke callback
    spawn_chessboard_subscriber = rospy.Subscriber("spawn_chessboard", Pose, pnp_callback, pnp)

    # Subscribing to play_chess to let baxter start playing chess
    play_chess_subscriber = rospy.Subscriber("play_chess", Empty, play_chess, (pnp, overhead_orientation))

    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())

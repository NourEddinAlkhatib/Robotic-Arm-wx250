import time
import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx250", moving_time=1.5, accel_time=0.75)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set initial arm and gripper pose
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.25)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.go_to_sleep_pose()
#    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
#    while True
    success, clusters = pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=True)
    # pick up all the objects and drop them in a virtual basket in front of the robot
#    if success == True:
    for cluster in reversed(clusters):
            x, y, z = cluster["position"]
            if z>=0.12:
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z+0.05, pitch=0)
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z-0.075, pitch=0)
                bot.gripper.close()
                time.sleep(0.5)
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.015, z=z-0.075, pitch=0, roll=1.5)
                bot.arm.set_ee_pose_components(x=0.45, y=0, z=0.35, pitch=0, roll=1.5)
                bot.arm.set_ee_pose_components(x=0.3, y=0.25, z=0.35, pitch=0, roll=1.5)
                bot.arm.set_ee_pose_components(x=0.3, y=0.25, z=0.35, pitch=0, roll=0)
                bot.gripper.open()
                bot.arm.go_to_home_pose()
                bot.arm.go_to_sleep_pose()
                pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=True)
            elif 0.12>z and z>=0.04:
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z+0.05, pitch=0)
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z-0.05, pitch=0)
                bot.gripper.close()
                time.sleep(0.5)
                bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.015, z=z+0.1, pitch=0)
                bot.arm.go_to_home_pose()
                bot.arm.set_ee_pose_components(x=0.3, y=0.25, z=0.35, pitch=0)
                bot.gripper.open()
                bot.arm.go_to_home_pose()
                bot.arm.go_to_sleep_pose()
                pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=True)
            elif z<0.04:
                bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.1, pitch=1.5)
                bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.03, pitch=1.5)
                bot.gripper.close()
                time.sleep(0.5)
                bot.arm.set_ee_pose_components(x=0.25, y=0, z=0.31, pitch=1.5)
                bot.arm.set_ee_pose_components(x=0.25, y=0.25, z=0.31, pitch=1.5)
                bot.gripper.open()
                bot.arm.set_ee_pose_components(x=0.25, y=0.25, z=0.31, pitch=0)
                bot.arm.go_to_home_pose()
                bot.arm.go_to_sleep_pose()
                pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=True)
            else:
                bot.arm.go_to_sleep_pose()
    bot.arm.go_to_sleep_pose()
#    else:
#        break
#        sys.exit()

if __name__=='__main__':
    main()

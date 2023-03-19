import time
import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# This program scans the space after the arm tries to pick each objects one time, then tries to pick them up again.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx250'
# Then change to this directory and type 'python3 version2.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx250", moving_time=1.5, accel_time=0.75)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set gripper pose
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(x=0.4, y=-0.25, z=0.25)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.go_to_sleep_pose()

    # get the cluster positions
    # sort them from min to max 'x' position w.r.t. the 'wx250/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
    while success == True:
        success, clusters = pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
        if success == True:
    # pick up all the objects and drop them in a virtual basket next to the robot
            for cluster in clusters:
                    x, y, z = cluster["position"]
                    if z>=0.12:
                        bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z+0.05, pitch=0)
                        bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.02, z=z-0.1, pitch=0)
                        bot.gripper.close()
                        time.sleep(1)
                        bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.015, z=z-0.075, pitch=0, roll=1.5)
                        bot.arm.set_ee_pose_components(x=0.45, y=0, z=0.40, pitch=0, roll=1.5)
                        bot.arm.set_ee_pose_components(x=0.3, y=0.35, z=0.40, pitch=0, roll=1.5)
                        bot.arm.set_ee_pose_components(x=0.3, y=0.35, z=0.40, pitch=0, roll=0)
                        bot.gripper.open()
                        bot.arm.go_to_home_pose()
                        bot.arm.go_to_sleep_pose()
                        pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
                        break
                    elif 0.12>z and z>=0.05:
                        bot.arm.set_ee_pose_components(x=x-0.04, y=y+0.01, z=z+0.05, pitch=0)
                        bot.arm.set_ee_pose_components(x=x-0.04, y=y+0.01, z=z-0.005, pitch=0)
                        bot.gripper.close()
                        time.sleep(1)
                        bot.arm.set_ee_pose_components(x=x-0.03, y=y+0.015, z=z+0.1, pitch=0)
                        bot.arm.go_to_home_pose()
                        bot.arm.set_ee_pose_components(x=0.3, y=0.35, z=0.4, pitch=0)
                        bot.gripper.open()
                        bot.arm.go_to_home_pose()
                        bot.arm.go_to_sleep_pose()
                        pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
                        break
                    elif z<0.05:
                        bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.1, pitch=1.5)
                        bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.08, pitch=1.5)
                        bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.04, pitch=1.5, roll=1.5)
                        bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.04, pitch=1.5, roll=-1.5)
                        bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.04, pitch=1.5)
                        bot.gripper.close()
                        time.sleep(1)
                        bot.arm.set_ee_pose_components(x=0.25, y=0, z=0.35, pitch=0, roll=1.5)
                        bot.arm.set_ee_pose_components(x=0.3, y=0.35, z=0.4, pitch=0, roll=1.5)
                        bot.arm.set_ee_pose_components(x=0.3, y=0.35, z=0.4, pitch=0, roll=0)
                        bot.gripper.open()
                        bot.arm.go_to_home_pose()
                        bot.arm.go_to_sleep_pose()
                        pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
                        break
                    else:
                        bot.arm.go_to_sleep_pose()
                        break
            bot.arm.go_to_sleep_pose()
        else:
            break

if __name__=='__main__':
    main()

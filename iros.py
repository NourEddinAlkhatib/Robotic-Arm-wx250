import time
import sys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# This program scans the space after the arm tries to pick three objects one time.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx250'
# Then change to this directory and type 'python3 final_code.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx250", moving_time=2, accel_time=1)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set gripper pose
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(x=0.3, y=0.1, z=0.2)
    time.sleep(1)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, y=-0.1, z=0.2)
    

    # get the cluster positions
    # sort them from min to max 'x' position w.r.t. the 'wx250/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
    while success == True:
        success, clusters = pcl.get_cluster_positions(ref_frame="wx250/base_link", sort_axis="x", reverse=False)
        if success == True:
            # pick up all the objects and drop them in a virtual basket next to the robot
            for cluster in clusters:
                x, y, z = cluster["position"]
                bot.arm.set_ee_pose_components(x=x, y=y+0.026, z=z+0.1, pitch=1.5)
                bot.gripper.open()
                bot.arm.set_ee_pose_components(x=x, y=y+0.03, z=z+0.02, pitch=1.5)
                bot.gripper.close()
                time.sleep(1)
                bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.1, pitch=1.5)
                bot.arm.set_ee_pose_components(x=0.35, y=-0.045, z=0.095, pitch=1.5)
                bot.gripper.open()
                bot.arm.set_ee_pose_components(x=0.4, y=-0.04, z=0.2, pitch=1.5)
                bot.arm.go_to_sleep_pose()
        else:
            break

if __name__=='__main__':
    main()

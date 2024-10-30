import rospy
from std_msgs.msg import String
import subprocess


def reset_rovio(msg):
    if msg.data == "reset_rovio":
        print("Reset ROVIO received")
        subprocess.call("killall rovio_node", shell=True)
        subprocess.Popen(["roslaunch", "rovio", "rovio_davis_cvpr_live.launch"])



if __name__=="__main__":
    rospy.init_node('listener', anonymous=True)
    subscriber =  rospy.Subscriber('/evo/remote_key', String, reset_rovio)
    rospy.spin()
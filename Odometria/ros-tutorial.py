#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import telnetlib
#host = "192.168.0.61"
#port = "10002"
#tn = telnetlib.Telnet("192.168.0.61", "10002")
#tn2 =  telnetlib.Telnet("192.168.0.61", "10001")
def talker():
     #pub = rospy.Publisher('gpsdata', String, queue_size=10)
     pub2 = rospy.Publisher('imudata', String, queue_size=10)
     rospy.init_node('gps_node', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     #tn = telnetlib.Telnet("192.168.0.61", "10002")
     tn2 =  telnetlib.Telnet("192.168.0.61", "10001")
     while not rospy.is_shutdown():
         print "aqui1"
         tn2.close()
         tn2.open("192.168.0.61", "10001")
         #line = tn.read_until("\n")
         line2 = tn2.read_until("\n",1)
         #rospy.loginfo(line)
         #rospy.loginfo(line2)
         print "aqui2"


         #print(line)
         print(line2)
         #pub.publish(line)
         pub2.publish(line2)
         rate.sleep()

if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass


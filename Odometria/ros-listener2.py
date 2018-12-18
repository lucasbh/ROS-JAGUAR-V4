import rospy
import serial
import pynmea2
from std_msgs.msg import String
#from micropyGPS import MicropyGPS


def callback(data):
      msg = pynmea2.parse(data.data)
      print (msg)
      streamreader = pynmea2.NMEAStreamReader (msg)      
      #print (msg.lat)
      
      rospy.loginfo(rospy.get_caller_id() + " Latitude %s %s %s", msg.latitude, msg.latitude_minutes, msg.latitude_seconds)
				
      rospy.loginfo(rospy.get_caller_id() + " Longitude %s %s %s", msg.longitude, msg.longitude_minutes, msg.longitude_seconds)


        
def listener():
 
      rospy.init_node('gps_sl', anonymous=True)
 
      rospy.Subscriber("gpsdata", String, callback)
  
      #spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
 
def calculo_distancia():

	R = 6371000   # metres
	φ1 = lat1.toRadians()
	φ2 = lat2.toRadians()
	Δφ = (lat2-lat1).toRadians();
	Δλ = (lon2-lon1).toRadians();

	a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
		Math.cos(φ1) * Math.cos(φ2) *
		Math.sin(Δλ/2) * Math.sin(Δλ/2);
	c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

	d = R * c;


if __name__ == '__main__':
     listener()

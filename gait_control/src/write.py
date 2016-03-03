import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag','w')

try:
    str = String()
    str.data = 'foo'

    i1 = Int32(data=42)
    i2 = Int32(data=43)

    bag.write('chatter', i1 )
    bag.write('chatter', i1 )
    bag.write('num', str)
    
finally:
    bag.close()

import rosbag
bag = rosbag.Bag('test.bag')
i=1
for topic, msg, t in bag.read_messages(topics=['chatter','num']):
    print msg,i
    i=i+1
bag.close()
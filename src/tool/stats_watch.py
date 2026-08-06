#!/usr/bin/env python3
import rospy, time
from rosgraph_msgs.msg import TopicStatistics

stats = {}

def cb(msg: TopicStatistics):
    if msg.node_sub != "/driver_model_manager_node":
        return
    s = stats.setdefault(msg.topic, {"pub":"", "deliv":0, "drop":0,
                                     "p_mean":0.0, "p_max":0.0,
                                     "age_mean":0.0, "age_max":0.0,
                                     "traffic":0})
    s["pub"]      = msg.node_pub
    s["deliv"]    = msg.delivered_msgs
    s["drop"]     = msg.dropped_msgs
    s["traffic"]  = msg.traffic
    s["p_mean"]   = msg.period_mean.to_sec()
    s["p_max"]    = msg.period_max.to_sec()
    s["age_mean"] = msg.stamp_age_mean.to_sec()
    s["age_max"]  = msg.stamp_age_max.to_sec()

def main():
    rospy.init_node("stats_watch")
    rospy.Subscriber("/statistics", TopicStatistics, cb, queue_size=100)
    rate = rospy.Rate(0.5)  # every 2s
    while not rospy.is_shutdown():
        print("\033[2J\033[H", end="")  # clear screen
        print("topic                                deliv  drop   Hz(avg)  T_max(ms)  age_mean(ms)  age_max(ms)    pub")
        for t, s in sorted(stats.items()):
            hz = (1.0/s["p_mean"]) if s["p_mean"] > 0 else 0.0
            print("{:34s} {:6d} {:5d} {:8.2f} {:10.1f} {:13.1f} {:12.1f} {}".format(
                t[:34], s["deliv"], s["drop"], hz, s["p_max"]*1000.0,
                s["age_mean"]*1000.0, s["age_max"]*1000.0, s["pub"]))
        rate.sleep()

if __name__ == "__main__":
    main()

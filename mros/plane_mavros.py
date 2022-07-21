#!/usr/bin/env python3

from sqlalchemy import false, true
import rospy
import json
import os
from pymavlink import mavutil
from mavros import command
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped

class example_mavros:

    mission_started=0
    starting_time=0
    last_wp=0
    def __init__(self):
        self.setup()
        self.main()
        self.last_wp
        self.starting_time
        self.mission_started = 0
    
    def setup(self):
        rospy.Subscriber('mavros/mission/reached', WaypointReached, self.WP_callback)
        rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.WPL_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.LP_callback)

    def WP_callback(self,msg):
        if(self.mission_started):
            time = msg.header.stamp.secs
            elapsed_time = time-self.starting_time
            self.starting_time = time
            if(elapsed_time!=0):
                rospy.loginfo("MISSION Waypoint #%s reached. Elapsed time: %d s", msg.wp_seq, elapsed_time)
        elif(msg.wp_seq==1):
            rospy.loginfo("Mission Starting")
            self.starting_time = msg.header.stamp.secs
            self.mission_started = true
            self.last_wp = msg.wp_seq
            self.mission_started=1

        self.last_wp = msg.wp_seq
#                                                                                       Callbacks
    def WPL_callback(self,msg):
        self.mission_wp = msg
    
    def LP_callback(self,msg):
        self.pose = msg.pose
#                                                                                     ^ Callbacks
    def read_mission(self,mission_filename):
        wps = []
        with open(mission_filename, 'r') as f:
            for waypoint in self.read_plan_file(f):
                wps.append(waypoint)
                rospy.logdebug(waypoint)

        # set first item to current
        if wps:
            wps[0].is_current = True
        return wps


    def read_plan_file(self,f):
        """file_name(TextIOWrapper)"""
        d = json.load(f)
        if 'mission' in d:
            d = d['mission']

        if 'items' in d:
            for wp in d['items']:
                yield Waypoint(
                    is_current=False,
                    frame=int(wp['frame']),
                    command=int(wp['command']),
                    param1=float('nan'
                                if wp['params'][0] is None else wp['params'][0]),
                    param2=float('nan'
                                if wp['params'][1] is None else wp['params'][1]),
                    param3=float('nan'
                                if wp['params'][2] is None else wp['params'][2]),
                    param4=float('nan'
                                if wp['params'][3] is None else wp['params'][3]),
                    x_lat=float(wp['params'][4]),
                    y_long=float(wp['params'][5]),
                    z_alt=float(wp['params'][6]),
                    autocontinue=bool(wp['autoContinue']))
        else:
            raise IOError("no mission items")

    def send_wps(self,waypoints, timeout):
            """waypoints, timeout(int): seconds"""
            rospy.loginfo("sending mission waypoints")



            wp_push_srv = rospy.ServiceProxy('mavros/mission/push',WaypointPush)
            if self.mission_wp.waypoints:
                rospy.loginfo("FCU already has mission waypoints")

            loop_freq = 1  # Hz
            rate = rospy.Rate(loop_freq)
            wps_sent = False
            wps_verified = False
            for i in range(timeout * loop_freq):
                if not wps_sent:
                    try:
                        res = wp_push_srv(start_index=0, waypoints=waypoints)
                        wps_sent = res.success
                        if wps_sent:
                            rospy.loginfo("waypoints successfully transferred")
                    except rospy.ServiceException as e:
                        rospy.logerr(e)
                else:
                    if len(waypoints) == len(self.mission_wp.waypoints):
                        rospy.loginfo("number of waypoints transferred: {0}".
                                    format(len(waypoints)))
                        wps_verified = True

                if wps_sent and wps_verified:
                    rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                                format(i / loop_freq, timeout))
                    break

                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    rospy.logerr(e)

            if(wps_sent and wps_verified):
                print("mission could not be transferred and verified | timeout(seconds): {0}".format(timeout))

    def mission_run(self):
        # if len(sys.argv) < 2:
        #     rospy.logerr("Error!!\ncorrect typing is python3 %s mission_file"%sys.argv[0])
        #     return

        # mission_file_name = sys.argv[1]
        # mission_file = os.path.dirname(
        #         os.path.realpath(__file__)) + "/missions/" + mission_file_name
        # print("reading mission {0}".format(mission_file))
        # rospy.loginfo("reading mission {0}".format(mission_file))
        mission_file = "/home/aag/Masaüstü/kodlar/benim_mavros_kodlarım/missions/baskabaska.plan"

        try:
            wps = self.read_mission(mission_file)
            num0fwp=len(wps)
        except IOError as xcpt:
            rospy.logerr(xcpt)

        self.send_wps(wps,30)
        apm = apmFlightMode()
        # apm.setModeGuided()
        # apm.setArm()
        # print(self.pose)
        # apm.setTakeoff()
        # while(self.pose.position.z<1):
        #     pass
        # # now = rospy.get_rostime()
        # # while(rospy.get_rostime().secs-now.secs<10):
        # #     pass
        apm.setModeAuto()
        rospy.Subscriber('mavros/mission/reached', WaypointReached, self.WP_callback)

    def main(self):
        rospy.init_node('mission_node_aag', anonymous=True)
        self.mission_run()
        rospy.spin()

class apmFlightMode:
    def __init__(self):
        self.setup()

    def setup(self):

        self.altitude = Altitude()
        #                                                                            wait for services
        try:        
            rospy.loginfo("Waiting for Services...")
            rospy.wait_for_service('mavros/cmd/takeoff',timeout=None)
            rospy.wait_for_service('mavros/cmd/arming',timeout=None)
            rospy.wait_for_service('mavros/set_mode',timeout=None)
            rospy.loginfo("SUCCES: Services are active")
        except:
            rospy.logerr("ERROR:Cannot connecting to services!!!")
            rospy.loginfo("ERROR:Cannot connecting to services!!!")
        #                                                                            Message subscribe
        #self.state=State()
        #self.state_sub = rospy.Subscriber('mavros/state', State,self.state_callback)
        self.altitude = Altitude()
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.state_callback)
        #                                                                            Services
        self.takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff',mavros_msgs.srv.CommandTOL)
        self.armService = rospy.ServiceProxy('mavros/cmd/arming',mavros_msgs.srv.CommandBool)
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))


    def setTakeoff(self):
        self.takeoffService(altitude = 2.5)
        rospy.loginfo("Takeoff")

    def setArm(self):
        self.armService(True)
        rospy.loginfo("Armed")

    def setModeGuided(self):
        self.flightModeService(custom_mode='GUIDED')
        rospy.loginfo("GUIDE MODE:")

    def setModeAuto(self):
        self.flightModeService(custom_mode='AUTO')
        rospy.loginfo("AUTO MODE:")
    
    def setModeLand(self):
        self.flightModeService(custom_mode='LAND')
        rospy.loginfo("LAND MODE:")


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                self.state.mode = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.flightModeService(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

        if(mode_set==false):
            rospy.logerr("failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".format(mode, old_mode, timeout))


if __name__ == '__main__':
    try:
        aurora = example_mavros()
    except rospy.ROSInterruptException:
        pass











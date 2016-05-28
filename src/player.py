#!/usr/bin/python

import os
import rospy
import numpy as np
from scipy.interpolate import interp1d
from sensor_msgs.msg import JointState

class KinematicsAnimator(object):
    def __init__(self):
        freq = rospy.get_param('~freq', 50)
        rospy.loginfo('Publishing frequency = ' + str(freq))

        fname = rospy.get_param('~file', None)
        if fname is None:
            rospy.logerr("Must provide private param '~file'")
            rospy.signal_shutdown('No CSV file provided')
        else:
            # Find out the absolute path if the input path is relative
            if not os.path.isabs(fname):
                fname =  os.path.abspath(fname)
            rospy.loginfo('CSV file absolute path = ' + fname)

        self.rate = rospy.get_param('~rate', 1.0)

        rospy.loginfo('Time scaling = ' + str(self.rate))

        # load data from csv file
        with open(fname, 'r') as f:
            fline = f.readline()
        self.names = fline.rstrip('\r\n').split(',')
        self.names = self.names[1:]
        self.dat = np.loadtxt(fname, skiprows=1, delimiter=',')
        self._fint = interp1d(1 / float(self.rate) * self.dat[:, 0],
                              self.dat[:, 1:], kind='linear', axis=0)
        self.base_time = rospy.Time.now()

        # create a joint_states publisher
        self.state_pub = rospy.Publisher('joint_states', JointState,
                latch=True, queue_size=3)

        # create a timer
        self.pbtimer = rospy.Timer(rospy.Duration(1 / float(freq)),
                                   self.timercb)
        return

    def timercb(self, time_dat):
        t = (rospy.Time.now() - self.base_time).to_sec()
        try:
            q = self._fint(t)
        except ValueError:
            # rospy.loginfo(str(t)+" q = " + str(q))
            q = self._fint.y[-1]
            if t - self._fint.x[-1] > 2.0:
                rospy.loginfo('Animation complete! Replaying it.')
                self.base_time = rospy.Time.now()

        '''
        ; Provide all the joint names
        ; We are assuming here that the input csv file doesn't have head and finger information
        '''
        joint_names = ['head_pan', 'l_gripper_l_finger_joint',
                       'l_gripper_r_finger_joint',
                       'r_gripper_l_finger_joint',
                       'r_gripper_r_finger_joint'] + self.names

        # Setting the head and finger joints to 0
        joint_position = [0] * 5 + q.tolist()
        js = JointState(name=joint_names, position=joint_position)
        js.header.stamp = rospy.Time.now()
        self.state_pub.publish(js)
        return

def main():
    rospy.init_node('animating_csv_files', log_level=rospy.INFO)

    try:
        animator = KinematicsAnimator()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


if __name__ == '__main__':
    main()

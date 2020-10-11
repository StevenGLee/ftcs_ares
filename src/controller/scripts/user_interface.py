#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import random
import rospy
from PySide2 import QtCore, QtWidgets, QtGui
from PySide2.QtUiTools import QUiLoader
from ftcs_control_msg.msg import Formation
import os

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()
        self.ui = QUiLoader().load('param_adjust.ui')
        self.ui.publish_topic.clicked.connect(self.publish)
    def publish(self):
        formation1 = Formation()
        formation1.position.x=self.ui.pos_x_1.value()
        formation1.position.y=self.ui.pos_y_1.value()
        formation1.scale.x=self.ui.scp_x_1.value()
        formation1.scale.y=self.ui.scp_y_1.value()
        formation1.theta=self.ui.theta_1.value()
        formation1.clockwise_rotation=self.ui.rho_1.value()
        formation1.speed=self.ui.omega_1.value()
        formation_publisher_1.publish(formation1)

        formation2 = Formation()
        formation2.position.x=self.ui.pos_x_2.value()
        formation2.position.y=self.ui.pos_y_2.value()
        formation2.scale.x=self.ui.scp_x_2.value()
        formation2.scale.y=self.ui.scp_y_2.value()
        formation2.theta=self.ui.theta_2.value()
        formation2.clockwise_rotation=self.ui.rho_2.value()
        formation2.speed=self.ui.omega_2.value()
        formation_publisher_2.publish(formation2)

        formation3 = Formation()
        formation3.position.x=self.ui.pos_x_3.value()
        formation3.position.y=self.ui.pos_y_3.value()
        formation3.scale.x=self.ui.scp_x_3.value()
        formation3.scale.y=self.ui.scp_y_3.value()
        formation3.theta=self.ui.theta_3.value()
        formation3.clockwise_rotation=self.ui.rho_3.value()
        formation3.speed=self.ui.omega_3.value()
        formation_publisher_3.publish(formation3)

if __name__ == "__main__":

    try:
        print(os.getcwd())
        rospy.init_node('user_interface', anonymous=False)

        formation_publisher_1 = rospy.Publisher('/robot1/formation', Formation, queue_size=10)
        formation_publisher_2 = rospy.Publisher('/robot2/formation', Formation, queue_size=10)
        formation_publisher_3 = rospy.Publisher('/robot3/formation', Formation, queue_size=10)

        app = QtWidgets.QApplication([])

        widget = MyWidget()
        widget.ui.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException as r:
        rospy.loginfo("user_interface node terminated.") 
        rospy.loginfo(r) 
    
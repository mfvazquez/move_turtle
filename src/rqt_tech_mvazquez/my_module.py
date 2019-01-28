import os
import rospy
import rospkg
import thread
import actionlib

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from tech_mvazquez.msg import * # Goal
from std_srvs.srv import *


WARNING_MESSAGE = 'Only numbers!'
SPEED_PARAM = '/turtle/speed'
PAUSE_SERVICE = '/tech_mvazquez/pause'
RESUME_SERVICE = '/tech_mvazquez/resume'
ACTION_SERVER_MOVE = '/tech_mvazquez/move_action_server'
ACTION_SERVER_TELEPORT = '/tech_mvazquez/teleport_action_server'


def is_number(text):
    """ Return true if text can be cast to a float """
    try:
        float(text)
        return True
    except ValueError:
        return False


class UI(Plugin):

    def __init__(self, context):
        super(UI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('UI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        self._widget.setWindowTitle('Reach Goal')
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('tech_mvazquez'), 'resource', 'form.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('UI')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.pushButton_go.clicked[bool].connect(self.go_button_action)
        self._widget.pushButton_set_speed.clicked[bool].connect(self.set_speed_button)
        self._widget.pushButton_pause.clicked[bool].connect(self.pause_button)
        self._widget.pushButton_resume.clicked[bool].connect(self.resume_button)
        self._widget.pushButton_cancel.clicked[bool].connect(self.cancel_button)
        self._widget.pushButton_teleport.clicked[bool].connect(self.teleport_button)

        self._widget.progressBar_goal.setValue(0)

        # Services
        self.pause_service = rospy.ServiceProxy(PAUSE_SERVICE, Empty)
        self.resume_service = rospy.ServiceProxy(RESUME_SERVICE, Empty)

        # Actions
        self.move_client = actionlib.SimpleActionClient(ACTION_SERVER_MOVE, ReachGoalAction)
        self.move_client.wait_for_server()

        self.teleport_client = actionlib.SimpleActionClient(ACTION_SERVER_TELEPORT, ReachGoalAction)
        self.teleport_client.wait_for_server()

        self.action_active = False


    def parse_text_box(self, text_box):
        """ Check if the text_box has a number. If there is not a number
        it change the text to a constant warning messange """
        value_text = text_box.text()
        if not is_number(value_text):
            text_box.setText(WARNING_MESSAGE)
            return (False, None)
        return (True, float(value_text))


    def go_button_action(self):
        """ Action of the go_button. It send a goal to the actionlib server move """
        if self.action_active:
            return

        (data_ok, goal) = self.check_goal()
        if not data_ok:
            return

        self.action_active = True

        self.move_client.send_goal(goal, done_cb=self.refresh_status ,feedback_cb=self.show_feedback)


    def refresh_status(self, terminal_status, result):
        """ This method is called when an action is finished  """
        self.action_active = False


    def show_feedback(self, feedback):
        """ Show the feedback throw the label_status text label """
        self._widget.progressBar_goal.setValue(feedback.distance_progress)
        self._widget.label_status.setText(feedback.status)


    def cancel_button(self):
        """ Action of the button cancel. It cancel the goal
        sended to the actionlib server move """
        self._widget.label_status.setText('canceled')
        self.move_client.cancel_all_goals()


    def set_speed_button(self):
        """ set the speed of the turtle changing the parameter SPEED_PARAM """
        (data_ok, speed) = self.parse_text_box(self._widget.lineEdit_speed)
        if data_ok:
            thread.start_new_thread(rospy.set_param, (SPEED_PARAM, speed))


    def pause_button(self):
        """ Set the turtle on pause calling the service pause_service """
        thread.start_new_thread(self.pause_service, ())

    def resume_button(self):
        """ Resume the pause calling the service resume_service """
        thread.start_new_thread(self.resume_service, ())

    def teleport_button(self):
        """ Send a goal to the teleport actionlib teleport server """
        if self.action_active:
            return

        (data_ok, goal) = self.check_goal()
        if not data_ok:
            return
        self.teleport_client.send_goal(goal)


    def check_goal(self):
        """ Check if the goal input is correct
        Post: If the goal is correct return True and a goal, if
        is incorrect it change the incorrect value of the text box
        with a warning message and return False and None instead a goal """

        data_ok = True
        (feedback, x) = self.parse_text_box(self._widget.lineEdit_x)
        data_ok = data_ok and feedback
        (feedback, y) = self.parse_text_box(self._widget.lineEdit_y)
        data_ok = data_ok and feedback
        (feedback, theta) = self.parse_text_box(self._widget.lineEdit_theta)
        data_ok = data_ok and feedback
        if not data_ok:
            return (False, None)

        goal = ReachGoalGoal()
        goal.x = x
        goal.y = y
        goal.theta = theta

        return (True, goal)

#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import threading
import time
import datetime
import sys
import roslib 
import rospy
import tf.transformations
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs
from std_msgs.msg import String
from std_srvs.srv import Empty
from databasehandler import DatabaseHandler
from geometry_msgs.msg import PoseWithCovarianceStamped
from cyborg_navigation.msg import NavigationGoToAction
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, StateMachineResult, StateMachineFeedback, EmotionalState, EmotionalFeedback, SystemState

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.3"
__all__ = []

class NavigationServer():
	"""NavigationServer"""
	
	client_base_feedback = StateMachineFeedback()
	client_base_result = StateMachineResult()
	next_location = None
	command_location = None
	current_location = None
	reason = ""

	map_name = "ntnu2.map"
	base_canceled = False
	base_succeded = False

	current_x = 0.0
	current_y = 0.0
	current_emotion = "neutral"

	planing_timeout = 60 # (s)
	moving_timeout = 1000 # (s)
	taking_timeout = 60 # (s)


	def __init__(self, database_file=""):
		self.scheduler_rate = rospy.Rate(1) # (hz)
		self.server_rate = rospy.Rate(0.5) # (hz)

		self.server_planing = actionlib.SimpleActionServer(rospy.get_name() + "/planing", StateMachineAction, execute_cb=self.server_planing_callback, auto_start = False)
		self.server_moving = actionlib.SimpleActionServer(rospy.get_name() + "/moving", StateMachineAction, execute_cb=self.server_moving_callback, auto_start = False)
		self.server_talking = actionlib.SimpleActionServer(rospy.get_name() + "/talking", StateMachineAction, execute_cb=self.server_talking_callback, auto_start = False)
		self.server_go_to = actionlib.SimpleActionServer(rospy.get_name() + "/go_to", NavigationGoToAction, execute_cb=self.server_go_to_callback, auto_start = False)
		self.server_planing.start()
		self.server_moving.start()
		self.server_talking.start()
		self.server_go_to.start()
		self.client_base = actionlib.SimpleActionClient("/rosarnl_node/move_base", MoveBaseAction)
		self.emotion_publisher = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)
		self.event_publisher = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
		self.speech_publisher = rospy.Publisher("/cyborg_text_to_speech/text_to_speech", String, queue_size=100)
		self.database_handler = DatabaseHandler(filename=database_file)
		self.location_subscriber = rospy.Subscriber("/rosarnl_node/amcl_pose", PoseWithCovarianceStamped, self.location_callback)
		self.emotion_subscriber = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotion_callback, queue_size=100)
		self.text_subscriber = rospy.Subscriber("/text_from_speech", String, self.text_callback, queue_size=100)
		self.scheduler_thread = threading.Thread(target=self.scheduler)
		self.scheduler_thread.daemon = True # Thread terminates when main thread terminates
		self.scheduler_thread.start()
		rospy.loginfo("NavigationServer: Activated.")


	# Updates the current position when the position subscriber receives data
	def location_callback(self, data):
		self.current_x = data.pose.pose.position.x
		self.current_y = data.pose.pose.position.y


	# Updates the current emotion when the emotion subscriber recives data from the controller (emotion system)
	def emotion_callback(self, data):
		self.current_emotion = data.to_emotional_state


	# Thread, updating current location name based on current possition, checks for ongoing events and current position compared to the event, if ongoing event is an other location it publish a navigation_schedulaer event for the state machine
	def scheduler(self): # Threaded
		while (not rospy.is_shutdown()):
			self.current_location = self.database_handler.find_location(robot_map_name=self.map_name, location_x=self.current_x, location_y=self.current_y)
			current_location_name = self.current_location.location_name if self.current_location != None else ""
			event = self.database_handler.search_ongoing_events(robot_map_name=self.map_name, current_date=datetime.datetime.now())
			if event != None:
				if event.location_name != current_location_name:
					self.event_publisher.publish("navigation_schedualer")
			self.scheduler_rate.sleep()


	# Called once when the robot base (ROSARNL) goal completes
	def client_base_done_callback(self, state, result):
		if self.is_controlling_base:
			if (state == 3): # Succeded aka arived at location
				self.base_succeded = True
			elif (state == 2): # Happens when we cancel goals aka preemted
				self.base_canceled = True
			elif (state == 1): # Canceled?
				self.base_canceled = True
			self.client_base_state = state
			self.client_base_result = result
			rospy.logdebug("NavigationServer: Base has completed its execution with " + str(state) + " and result " + str(result) + ".")


	# Called once when the robot base (ROSARNL) goal becomes active
	def client_base_active_callback(self):
		rospy.logdebug("NavigationServer: Base goal has gone active.")
		self.is_controlling_base = True
		

	# Called every time feedback is received for the goal for the robot base (ROSARNL)
	def client_base_feedback_callback(self, feedback):
		rospy.logdebug("NavigationServer: Received feedback from base - " + str(feedback) + ".")
		self.client_base_feedback = feedback


	# Called when the controller (state machine) sets the navigation_planing state as active
	def server_planing_callback(self, goal):
		rospy.logdebug("NavigationServer: Executing planing state.")
		time.sleep(2) # Let roscore update connections

		# Select what to do based on event
		self.next_location = None
		if goal.event == "navigation_schedualer":
			if self.current_emotion == "angry":
				self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.map_name, crowded=False)
				self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
				self.change_state(event="navigation_start_moving")
			else:
				self.next_location = self.database_handler.search_ongoing_events(robot_map_name=self.map_name, current_date=datetime.datetime.now())
				self.send_emotion(pleasure=0, arousal=0, dominance=-0.1)
				self.change_state(event="navigation_start_moving")

		elif goal.event == "navigation_emotional":
			if self.current_emotion in ["angry", "sad", "fear", "inhibited"]:
				self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.map_name, crowded=False)
				self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
				self.change_state(event="navigation_start_moving")
			elif self.current_emotion in ["happy", "loved", "dignified", "neutral", "elated"]:
				self.next_location = self.database_handler.search_for_crowded_locations(robot_map_name=self.map_name, crowded=True)
				self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
				self.change_state(event="navigation_start_moving")
			elif self.current_emotion in ["bored", "curious", "unconcerned"]:
				self.next_location = "wandering"
				self.send_emotion(pleasure=0, arousal=0, dominance=0.1)
				self.change_state(event="navigation_start_wandering")

		elif goal.event == "navigation_command":
			self.next_location = self.command_location
			self.send_emotion(pleasure=0, arousal=0, dominance=-0.2)
			self.change_state(event="navigation_start_moving")


	# Publishes an event and waits for a change of state.
	def change_state(self, event=None):
		if event != None and self.next_location != None:
			self.event_publisher.publish(event)
		else:
			self.server_planing.set_aborted()
			return

		# Wait until state is preemted, or abort if it takes to long time
		started_waiting = time.time() # Prevent eternal looping
		while not rospy.is_shutdown():
			if self.server_planing.is_preempt_requested():
				self.server_planing.set_preempted()
				return
			elif (time.time() - started_waiting > self.planing_timeout): # Prevent eternal looping
				self.server_planing.set_aborted() 
				return
			self.server_rate.sleep()


	# Called when the controller (state machine) sets the navigation_moving state as active
	def server_moving_callback(self, goal):
		rospy.logdebug("NavigationServer: Executing moving state." + " Navigation movinging cmd " + str(self.next_location))
		self.goal = goal
		self.base_canceled = False
		self.base_succeded = False
		self.is_controlling_base = False

		# Select how to move based on event
		if goal.event == "navigation_start_wandering":
			self.start_wandering()
		elif goal.event == "navigation_start_moving" and self.next_location != None:
			self.start_moving()
		else:
			self.server_moving.set_aborted()
			rospy.logdebug("NavigationServer: Received event that cant be handled - " + str(goal.event) + ".")


	# The robot base starts to wander and keeps wandering until it is preemted or no longer in the right mood
	def start_wandering(self):
		# Tell base to start wandering
		rospy.logdebug("NavigationServer: Waiting for base wandering service.")
		rospy.wait_for_service("/rosarnl_node/wander")
		rospy.logdebug("NavigationServer: wandering service available.")
		try:
			baseStartWandering = rospy.ServiceProxy("/rosarnl_node/wander", Empty)
			baseStartWandering()
		except rospy.ServiceException, e:
			rospy.logdebug("NavigationServer: wandering service error - " + str(e))

		started_waiting = time.time() # Prevent eternal looping
		feedback_cycle = time.time() 
		while not rospy.is_shutdown():
			if self.server_moving.is_preempt_requested():
				self.client_base.cancel_all_goals() # HERE - goal on wandering?
				try:
					baseStop = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
					baseStop()
				except rospy.ServiceException, e:
					rospy.logdebug("NavigationServer: stop service error - " + str(e))
				self.server_moving.set_preempted()
				return
			if self.current_emotion not in ["bored", "curious", "unconcerned"]:
				self.client_base.cancel_all_goals() # HERE - goal on wandering?
				try:
					baseStop = rospy.ServiceProxy("/rosarnl_node/stop", Empty)
					baseStop()
				except rospy.ServiceException, e:
					rospy.logdebug("NavigationServer: stop service error - " + str(e))
				self.event_publisher.publish("navigation_wandering_completed")

				# Wait until state is preemted
				started_waiting = time.time() # Prevent eternal looping
				while not rospy.is_shutdown():
					if self.server_moving.is_preempt_requested():
						self.server_moving.set_preempted()
						return
					self.server_rate.sleep()
				return
			if (time.time() - feedback_cycle > 15): # send emotional feedback every 15 secounds
				self.send_emotion(pleasure=0.02, arousal=0.05, dominance=0.02)
				feedback_cycle = time.time()
			self.server_rate.sleep()


	# The robot base moves to the self.next_location
	# Increases PAD values while moving
	# Is preemtable. Ends in Succeded or Aborted
	def start_moving(self):
		rospy.logdebug("NavigationServer: Contacting base with goal.")
		self.send_goal(location=self.next_location)

		# Wait until state is preemted, or abort if it takes to long time
		started_waiting = time.time() # Prevent eternal looping
		feedback_cycle = time.time() 
		while not rospy.is_shutdown():
			if self.base_succeded:
				self.current_location = self.next_location
				self.send_emotion(pleasure=self.next_location.enviorment, arousal=0.00, dominance=0.00)
				server_result = StateMachineResult()
				self.server_moving.set_succeeded(server_result)
				next_location = None
				return
			if self.server_moving.is_preempt_requested():
				self.client_base.cancel_all_goals()
				self.server_moving.set_preempted()
				return
			if self.base_canceled:
				self.server_moving.set_aborted()
				return
			if (time.time() - started_waiting > self.moving_timeout): # Prevent eternal looping
				self.client_base.cancel_all_goals()
				self.server_moving.set_aborted() 
				return
			if (time.time() - feedback_cycle > 15): # Give some feedback for the emotion
				self.send_emotion(pleasure=0.00, arousal=0.03, dominance=0.01)
				feedback_cycle = time.time()
			self.server_rate.sleep()




	# Called when the controller (state machine) sets the navigation_talking state as active
	def server_talking_callback(self, goal):
		rospy.logdebug("NavigationServer: Executing talking state - event was " + str(goal.event) + ".")
		rate = rospy.Rate(.25) # (hz)
		rate.sleep() # Let roscore update list
		if goal.event == "succeded":
			if self.reason == "navigation_direction":
				self.speech_publisher.publish("Human, this is " + self.current_location.location_name)
				self.reason = ""
			else:
				self.find_response(location=self.current_location.location_name, response_type="navigation_response", emotion=self.current_emotion)
			self.server_talking.set_succeeded()
			return

		elif goal.event == "aborted":
			if self.reason == "navigation_direction":
				self.speech_publisher.publish("Human, I am sorry, but I am unable to reach " + self.next_location.location_name)
			else:
				self.speech_publisher.publish("Oh lord, I am stuck " )
			self.server_talking.set_aborted()
			return

		elif goal.event == "navigation_feedback":
			self.find_response(location=self.command_location.location_name, response_type="navigation_response", emotion=self.current_emotion)
			self.event_publisher.publish("navigation_feedback_completed")
			self.server_talking_wait()
			return
		
		elif goal.event == "navigation_information":
			self.speech_publisher.publish("I think I know where that is. Would you like me to show you?")
			# Wait until state is preemted, or abort if it takes to long time
			started_waiting = time.time() # Prevent eternal looping
			while not rospy.is_shutdown():
				if "yes" in self.text:
					self.text = ""
					self.reason = "navigation_direction"
					self.speech_publisher.publish("At once!")
					self.event_publisher.publish("navigation_command")
					self.server_talking_wait()
					return
				if "no" in self.text:
					self.text = ""
					self.speech_publisher.publish("I wont go then...")
					self.event_publisher.publish("navigation_feedback_completed")
					self.server_talking_wait()
					return
				if self.server_talking.is_preempt_requested():
					self.server_talking.set_preempted()
					return
				if (time.time() - started_waiting > self.taking_timeout): # Prevent eternal looping
					self.server_talking.set_aborted() 
					return
				self.server_rate.sleep()

		elif goal.event == "navigation_command":
			if self.current_emotion == "angry":
				self.send_emotion(pleasure=0, arousal=0, dominance=0.2)
				self.command_location = None
				self.speech_publisher.publish("I dont want to go there! Stop telling me what to do human!")
				self.event_publisher.publish("navigation_feedback_completed")
				self.server_talking_wait()
				return
			else:
				self.speech_publisher.publish("You would like me to go to " + self.command_location.location_name + "?")
				# Wait until state is preemted, or abort if it takes to long time
				started_waiting = time.time() # Prevent eternal looping
				while not rospy.is_shutdown():
					if "yes" in self.text:
						self.text = ""
						self.reason = "navigation_direction"
						self.speech_publisher.publish("At once!")
						self.event_publisher.publish("navigation_command")
						self.server_talking_wait()
						return
					if "no" in self.text:
						self.text = ""
						self.speech_publisher.publish("I wont go then...")
						self.event_publisher.publish("navigation_feedback_completed")
						self.server_talking_wait()
						return
					if self.server_talking.is_preempt_requested():
						self.server_talking.set_preempted()
						return
					if (time.time() - started_waiting > self.taking_timeout): # Prevent eternal looping
						self.server_talking.set_aborted() 
						return
					self.server_rate.sleep()

		else:
			self.speech_publisher.publish("I dont understand, what am I doing?")
			self.server_talking.set_aborted()


	# Wait until state is preemted, or abort if it takes to long time
	def server_talking_wait(self):
		# Wait until state is preemted, or abort if it takes to long time
		started_waiting = time.time() # Prevent eternal looping
		while not rospy.is_shutdown():
			if self.server_talking.is_preempt_requested():
				self.server_talking.set_preempted()
				return
			if (time.time() - started_waiting > self.taking_timeout): # Prevent eternal looping
				self.server_talking.set_aborted() 
				return
			self.server_rate.sleep()


	# This can be used by other nodes for moving the cyborg to a known location.
	def server_go_to_callback(self, goal):
		rospy.logdebug("NavigationServer: go to server received a goal - " + str(goal))
		self.next_location = self.database_handler.search_for_location(location_name=goal.location_name)
		if self.next_location != None:
			self.send_goal(location=self.next_location)

			# Wait until state is preemted or succeeded
			while not rospy.is_shutdown():
				if self.base_succeded:
					self.current_location = self.next_location
					server_result = NavigationGoToResult()
					server_result.status = "succeeded"
					self.server_go_to.set_succeeded(server_result)
					return
				if self.server_go_to.is_preempt_requested():
					self.client_base.cancel_all_goals()
					server_result = NavigationGoToResult()
					server_result.status = "preemted"
					self.server_go_to.set_preempted(server_result)
					return
				if self.base_canceled:
					server_result = NavigationGoToResult()
					server_result.status = "aborted"
					self.server_go_to.set_aborted(server_result)
					return
				else:
					server_feedback = NavigationGoToFeedback()
					server_feedback.status = "moving"
					self.server_go_to.publish_feedback(feedback)
				self.server_rate.sleep()
		else:
			rospy.logdebug("NavigationServer: Go to server received a goal with unrecognized name - " + str(goal))
			server_result = NavigationGoToResult()
			server_result.status = "aborted"
			self.server_go_to.set_aborted(server_result)


	# Called when the speech to text publishes new text
	# Searches the text from speech for keywords to see if the Navigation module can act on it, if so, an event is sent to the state machine.
	def text_callback(self, data):
		self.text = data.data
		rospy.logdebug("NavigationServer: Recived text - " + self.text)
		if "go to " in data.data or "move to" in data.data or "go " in data.data:
			locations = self.database_handler.get_all_locations()
			for location in locations:
				if location.location_name in data.data:
					self.command_location = location
					self.event_publisher.publish("navigation_command")
					rospy.logdebug("NavigationServer: Cmd to next location - " + str(location))

		elif "where is " in data.data:
			locations = self.database_handler.get_all_locations()
			for location in locations:
				if location.location_name in data.data:
					self.command_location = location
					self.event_publisher.publish("navigation_information")
					rospy.logdebug("NavigationServer: information about location - " + str(location))

		elif "think of " in data.data or "think about " in data.data:
			locations = self.database_handler.get_all_locations()
			for location in locations:
				if location.location_name in data.data:
					self.command_location = location
					self.event_publisher.publish("navigation_feedback")
					rospy.logdebug("NavigationServer: Opinion about location - " + str(location))


	# Sends the location to the ROSARNL node (Pioneer XL robot base)
	def send_goal(self, location):
		if (self.client_base.wait_for_server(rospy.Duration.from_sec(5.0)) == False ):
			rospy.logwarn("NavigationServer: ERROR - Unable to connect to Pionner XL.")
			self.base_canceled = True
			return
		pose = geometry_msgs.msg.Pose()
		pose.position.x = location.x
		pose.position.y = location.y
		pose.position.z = location.z
		q = tf.transformations.quaternion_from_euler(location.p, location.j, location.r)
		pose.orientation = geometry_msgs.msg.Quaternion(*q)
		goal = MoveBaseGoal()
		goal.target_pose.pose = pose
		goal.target_pose.header.frame_id = location.robot_map_name
		goal.target_pose.header.stamp = rospy.Time.now()
		self.client_base.send_goal(goal, self.client_base_done_callback, self.client_base_active_callback, self.client_base_feedback_callback)
		rospy.logdebug("NavigationServer: Location goal is - " + str(self.next_location))
	

	# Sends the emotional change to the controller
	def send_emotion(self, pleasure, arousal, dominance):
		msg = EmotionalFeedback()
		msg.delta_pleasure = pleasure
		msg.delta_arousal = arousal
		msg.delta_dominance = dominance 
		self.emotion_publisher.publish(msg)


	# Chack the database for responses (aka voice output) for the Cyborg
	def find_response(self, location, response_type, emotion):
		response = self.database_handler.search_for_response(response_type=response_type, emotion=emotion)
		if response != None:
			speech = response.message.replace("LOCATION", location)
			self.speech_publisher.publish(speech)




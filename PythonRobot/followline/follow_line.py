import track_cv as track
import time
import numpy as np
import traceback
import math
import track_conf as tconf
import logging

logging.basicConfig(filename="track.log",level=logging.DEBUG)

class LineTrackingBehavior(object):
	"""Behavior to find and get close to a colored object"""
	def __init__(self, interval=1):
		self.PN = 0
		self.last_turn = 0
		self.last_angle = 0
	
	def find_line(self, side, frame):
		logging.debug (("Finding line", side))
		if side == 0:
			return None, None
			
		for i in xrange(0, tconf.find_turn_attempts):
			self.turn_action(side, tconf.find_turn_step)
			angle, shift = self.get_vector(frame)
			if angle is not None:
				return angle, shift

		return None, None

	def get_vector(self, frame):
		angle, shift = track.handle_pic(frame, True)
		return angle, shift

	def check_shift_turn(self, angle, shift):
		turn_state = 0
		if angle < tconf.turn_angle or angle > 180 - tconf.turn_angle:
			turn_state = np.sign(90 - angle)

		shift_state = 0
		if abs(shift) > tconf.shift_max:
			shift_state = np.sign(shift)
		return turn_state, shift_state

	def get_turn(self, turn_state, shift_state):
		turn_dir = 0
		turn_val = 0
		if shift_state != 0:
			turn_dir = shift_state
			turn_val = tconf.shift_step if shift_state != turn_state else tconf.turn_step
		elif turn_state != 0:
			turn_dir = turn_state
			turn_val = tconf.turn_step
		return turn_dir, turn_val                

	def turn_action(self, r, t):
		turn_cmd = "s0" if r > 0 else "0s"
		ret_cmd = "f0" if r > 0 else "0f"
		turn = "Right" if r > 0 else "Left"
		logging.debug(("Turn", turn, t))
		if turn == "Left":
			return 1
		else:
			return 2

	def follow_way_line(self, frame):
		a, shift = self.get_vector(frame)
		if a is None:
			if self.last_turn != 0:
				a, shift = self.find_line(self.last_turn, frame)
				if a is None:
					return -1, -1, -1, frame
			elif self.last_angle != 0:
				logging.debug(("Looking for line by angle", self.last_angle))
				self.turn_action(np.sign(90 - self.last_angle), tconf.turn_step)
				return -1, -1, -1, frame
			else:
				return -1, -1, -1, frame

		logging.debug(("Angle", a, "Shift", shift))

		turn_state, shift_state = self.check_shift_turn(a, shift)

		turn_dir, turn_val = self.get_turn(turn_state, shift_state)

		final_turn = 0
		final_angle = a
		final_shift = shift
		if turn_dir != 0:
			final_turn = self.turn_action(turn_dir, turn_val)
			self.last_turn = turn_dir
		else:
			self.last_turn = 0
		self.last_angle = a
		
		return final_turn, final_angle, final_shift, frame

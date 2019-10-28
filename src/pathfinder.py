#!usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import tf
import math


# Both START and GOAL are in the "map" reference frame (i.e. x and y represent the distance, in meters, from the origin of "map").
START = [2,1] # The starting point of the robot.
GOAL = [9,1] # The goal point of the robot.

PUBLISH_RATE = 100 # The rate at which the pose_sequence messages are published.
DISPLAY_RATE = 20 # The rate at which the Pose in RVIZ updates. Increase this value to make the Pose arrow in RVIZ move faster.


# Initializes the pathfinder node.
def init_pathfinder():
	rospy.init_node("pathfinder_node")


# Defines the PathFinder class, which finds and prints a path from START to GOAL.
class PathFinder:

	# Constructor for PathFinder.
	def __init__(self, start, goal):

		# Convert both the start and goal from the map frame to the occupancy grid frame.
		self.start_map = start
		self.goal_map = goal

		self.pose_pub = rospy.Publisher("pose_sequence", PoseStamped, queue_size = 0) # Publishes each Pose in the path.
		self.map_sub = rospy.Subscriber("map",  OccupancyGrid, self.get_map, queue_size = 1) # Gets the occupancy grid data.

		self.map_data = None # Instantiates the variable to hold the occupancy grid data.

		self.publish_rate = rospy.Rate(PUBLISH_RATE)
		self.display_rate = rospy.Rate(DISPLAY_RATE)


	# Callback method to get the occupancy grid data.
	def get_map(self, msg):
		self.map_data = msg.data
		self.map_width = msg.info.width 
		self.map_height = msg.info.height 
		self.num_nodes = self.map_width *self.map_height # Sets the total number of nodes in the occupancy grid.
		self.resolution = msg.info.resolution


	# Performs depth-first search with history.
	def dfs(self):

		# Waits until the occupancy grid data is received.
		while self.map_data == None:
			self.publish_rate.sleep()

		self.start_ind = self.map_to_grid_frame(self.start_map)
		self.goal_ind = self.map_to_grid_frame(self.goal_map)

		visited_array = [0] * self.num_nodes # Keeps track of visited nodes. 0 = Not Visited, 1 = Visited. Uses Row-Major indexing.
		self.path_tracker = [-1] * self.num_nodes # Keeps track of the parent node of each node. Uses Row-Major indexing.
		
		# Uses a stack to progress through the DFS.
		stack = []
		stack.append(self.start_ind) # Add the first node, the start point, to the stack.

		# Loop through until either the stack is empty or the goal is found.
		found_goal = False
		while not len(stack) == 0 and not found_goal:
			curr_node = stack.pop() 

			# If the goal is found, stop the while loop.
			if curr_node == self.goal_ind:
				found_goal = True
				print "Found the goal!"

			x = curr_node[0]
			y = curr_node[1]
			curr_rm_ind = self.calc_rm_ind(curr_node) # Calculates Row-Major index of the current node
			visited_array[curr_rm_ind] = 1 # Mark node as visited.
			neighbors = self.get_neighbors(curr_node) # Gets the neighbors of the current node.

			# Loop through each neighbor, and if not visited and not occupied, add to stack.
			for i in range(len(neighbors)):
				neighbor = neighbors[i]
				rm_ind_neigh = self.calc_rm_ind(neighbor)
				if visited_array[rm_ind_neigh] == 0 and self.map_data[rm_ind_neigh] == 0:
					self.path_tracker[rm_ind_neigh] = curr_rm_ind
					stack.append(neighbor)
			

	# Displays the path of the robot in RVIZ.
	def display_path(self):
		print "Displaying the path on RVIZ..."

		# Uses the path_tracker array to reverse-engineer the path. This 'path' array starts from the goal and moves to the start.
		path = []
		curr_node_rm = self.calc_rm_ind(self.goal_ind)
		while not self.path_tracker[curr_node_rm] == -1:
			orientation = self.calc_orientation(curr_node_rm, self.path_tracker[curr_node_rm])
			path.append(self.rm_to_pose_stamped(curr_node_rm, orientation))
			curr_node_rm = self.path_tracker[curr_node_rm]

		path.append(self.rm_to_pose_stamped(curr_node_rm, 0)) # To append the last node in the array, which represents the start.
		
		path.reverse() # Reverses the array so that the order is from start to goal.

		# Publishes the poses.
		while not rospy.is_shutdown():
			for pose in path:
				self.pose_pub.publish(pose)
				self.display_rate.sleep() # Pauses in between each pose so that visualizing is easier in RVIZ.

			self.publish_rate.sleep()


	# Converts a Row-Major index to a PoseStamped object
	def rm_to_pose_stamped(self, ind, orientation):
		row = int(ind/self.map_height)
		col = int(ind%self.map_width)

		# Creates the pose, with proper coordinates and orientation
		pose = Pose()
		pose.position.x = col*self.resolution
		pose.position.y = row*self.resolution
		pose.position.z = 0
		euler = [0, 0, orientation]
		quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation)
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]

		# Creates a PoseStamped from the above Pose
		pose_st = PoseStamped()
		pose_st.header.stamp = rospy.Time.now()
		pose_st.header.frame_id = "map"
		pose_st.pose = pose

		return pose_st


	# Converts the map frame to the occupancy grid frame.
	def map_to_grid_frame(self, map_point):
		return [int(map_point[0]/self.resolution), int(map_point[1]/self.resolution)]


	# Gets the neighbors of the node. Returns the neighbor in reverse-order-of-preference, so that when added to the stack, neighbors are explored in the correct order.
	# Order of Preference: Right, Up, Left, Down
	def get_neighbors(self, node):
		x = node[0]
		y = node[1]
		if x == 0:
			if y == 0:
				return [[x, y+1], [x+1, y]]
			elif y == self.map_height - 1:
				return [[x, y-1], [x+1, y]]
			else:
				return [[x, y-1], [x, y+1], [x+1, y]]
		elif x == self.map_width - 1:
			if y == 0:
				return [[x-1, y], [x, y+1]]
			elif y == self.map_height - 1:
				return [[x, y-1], [x-1, y]]
			else:
				return [[x, y-1], [x-1, y], [x, y+1]]
		else:
			if y == 0:
				return [[x-1, y], [x, y+1], [x+1, y]]
			elif y == self.map_height - 1:
				return [[x, y-1], [x-1, y], [x+1, y]]
			else:
				return [[x, y-1], [x-1, y], [x, y+1], [x+1, y]]


	# Calculates the orientation of the robot when moving from the parent node to the curr node.
	def calc_orientation(self, curr, parent):
		curr_x = int(curr%self.map_width)
		curr_y = int(curr/self.map_height)
		parent_x = int(parent%self.map_width)
		parent_y = int(parent/self.map_height)
		
		if curr_x == parent_x + 1:
			return 0 # Orientation = Right
		elif curr_x == parent_x - 1:
			return math.pi # Orientation = Left
		elif curr_y  == parent_y + 1:
			return math.pi/2 # Orientation = Up
		elif curr_y == parent_y - 1:
			return 3*math.pi/2 # Orientation = Down


	# Calculates the Row-Major index from the x,y coordinates of the node.
	def calc_rm_ind(self, node):
		return int((node[1])*self.map_width) + int(node[0])


# Main method to run the pathfinder. Creates the pathfinder, runs the DFS, and then displays the path.
if __name__ == "__main__":
	init_pathfinder() 
	pathfinder = PathFinder(START, GOAL)
	pathfinder.dfs()
	pathfinder.display_path()
	

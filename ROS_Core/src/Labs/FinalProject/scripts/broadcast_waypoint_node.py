import rospy
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from You_Need_to_Define_the_import_Path_Here import RefPath

rospy.wait_for_service('/routing/plan')
plan_client = rospy.ServiceProxy('/routing/plan', Plan)

x_start = 0 # x coordinate of the start
y_start = 0 # y coordinate of the start

x_goal = 0 # x coordinate of the goal
y_goal = 0 # y coordinate of the goal

plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
plan_response = plan_client(plan_request)

# The following script will generate a reference path in [RefPath](scripts/task2_world/util.py#L65) class, which has been used in your Lab1's ILQR planner
x = []
y = []
width_L = []
width_R = []
speed_limit = []

for waypoint in plan_respond.path.poses:
    x.append(waypoint.pose.position.x)
    y.append(waypoint.pose.position.y)
    width_L.append(waypoint.pose.orientation.x)
    width_R.append(waypoint.pose.orientation.y)
    speed_limit.append(waypoint.pose.orientation.z)
            
centerline = np.array([x, y])

# This is the reference path that we passed to the ILQR planner in Lab1
ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=False)
from math import atan2, sqrt, pow, pi


def distance(current_pose, objective_pose):
    """ Calculate the euclidean distance between current_pose and
    objective_pose.
    Pre: current_pose and objective_pose must have .x and .y parameters """
    x_distance = objective_pose.x - current_pose.x
    y_distance = objective_pose.y - current_pose.y
    linear_distance = sqrt(pow(x_distance,2)+pow(y_distance,2))
    return linear_distance


def normalize_angle(angle):
    """ return the angle normalized between -pi and pi """
    div = round(angle/(2*pi))
    return angle-div*2*pi


def angle_between_points(current_point, objective_point):
    """ return the angle between current_point and objective_point
    Pre: current_point and objective_point must have .x, .y  and .theta
    parameters.
    Post: the angle returned is normalized between -pi and pi """
    x_distance = objective_point.x - current_point.x
    y_distance = objective_point.y - current_point.y
    angular_distance =  atan2(y_distance, x_distance)
    return normalize_angle(angular_distance)


def angle_between_poses(current_pose, objective_pose):
    """ return the angle between two poses. It's like angle_between_points
    but consider the orrientation of current_pose """
    angular_distance = angle_between_points(current_pose, objective_pose) - current_pose.theta
    return normalize_angle(angular_distance)

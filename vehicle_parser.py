from math import dist, sin, cos, pi, sqrt

def is_ahead_of_ev(ev_heading, ev_center_position, obs_center_position):
    cos_theta = cos_theta_calculating(ev_heading, ev_center_position, obs_center_position)
    if 0.5 <= cos_theta and cos_theta <= 1:
        return True
    return False

def is_next_to_ev(ev_heading, ev_center_position, obs_center_position):
    cos_theta = cos_theta_calculating(ev_heading, ev_center_position, obs_center_position)
    if 0 <= cos_theta and cos_theta <= 0.5:
        return True
    return False

def is_blinds_spots(ev_heading, ev_center_position, obs_center_position):
    cos_theta = cos_theta_calculating(ev_heading, ev_center_position, obs_center_position)
    cos_150 = - sqrt(3) / 2
    if cos_150 <= cos_theta and cos_theta <= 0:
        return True
    return False

def is_behind_ev(ev_heading, ev_center_position, obs_center_position):
    cos_theta = cos_theta_calculating(ev_heading, ev_center_position, obs_center_position)
    cos_150 = - sqrt(3) / 2
    if -1 <= cos_theta and cos_theta <= cos_150:
        return True
    return False

def cos_theta_calculating(ev_heading, ev_center_position, obs_center_position):
    ev_heading_vector = ev_direction_vector(ev_heading)
    obs_vector = obstacle_vector(ev_center_position, obs_center_position)
    dot_product = ev_heading_vector[0] * obs_vector[0] + ev_heading_vector[-1] * obs_vector[-1]
    obs_vector_len = dist((0, 0), obs_vector)
    cos_theta = dot_product / obs_vector_len
    return cos_theta

def ev_direction_vector(ev_heading):
    x = cos(ev_heading)
    y = sin(ev_heading)
    return (x, y)

def obstacle_vector(ev_center_position, obs_center_position):
    x = obs_center_position['x'] - ev_center_position['x']
    y = obs_center_position['y'] - ev_center_position['y']
    return (x, y)

class Vehicle():
    def __init__(self, vehicle_param_file):
        self.vehicle = self.parse(vehicle_param_file)

    def parse(self, vehicle_param_file):
        self.front_edge_to_center = 0
        self.back_edge_to_center = 0
        self.left_edge_to_center = 0
        self.right_edge_to_center = 0
        self.length = 0
        self.width = 0
        self.height = 0

        self.min_turn_radius = 0
        self.max_acceleration = 0
        self.max_deceleration = 0
        self.max_steer_angle = 0
        self.max_steer_angle_rate = 0
        self.steer_ratio = 0
        self.wheel_base = 0
        self.wheel_rolling_radius = 0
        self.max_abs_speed_when_stopped = 0
        self.brake_deadzone = 0
        self.throttle_deadzone = 0

        for line in open(vehicle_param_file, 'r'):
            line = line.strip()
            if 'front_edge_to_center:' in line:
                value = float(line[22:])
                self.front_edge_to_center = value
            if 'back_edge_to_center:' in line:
                value = float(line[21:])
                self.back_edge_to_center = value
            if 'left_edge_to_center:' in line:
                value = float(line[21:])
                self.left_edge_to_center = value
            if 'right_edge_to_center:' in line:
                value = float(line[22:])
                self.right_edge_to_center = value
            if 'length:' in line:
                value = float(line[8:])
                self.length = value
            if 'width:' in line:
                value = float(line[7:])
                self.width = value
            if 'height:' in line:
                value = float(line[8:])
                self.height = value
            if 'min_turn_radius:' in line:
                value = float(line[17:])
                self.min_turn_radius = value
            if 'max_acceleration:' in line:
                value = float(line[18:])
                self.max_acceleration = value
            if 'max_deceleration:' in line:
                value = float(line[18:])
                self.max_deceleration = value
            if 'max_steer_angle:' in line:
                value = float(line[17:])
                self.max_steer_angle = value
            if 'max_steer_angle_rate:' in line:
                value = float(line[22:])
                self.max_steer_angle_rate = value
            if 'steer_ratio:' in line:
                value = float(line[13:])
                self.steer_ratio = value
            if 'wheel_base:' in line:
                value = float(line[12:])
                self.wheel_base = value
            if 'wheel_rolling_radius:' in line:
                value = float(line[22:])
                self.wheel_rolling_radius = value
            if 'max_abs_speed_when_stopped:' in line:
                value = float(line[28:])
                self.max_abs_speed_when_stopped = value
            if 'brake_deadzone:' in line:
                value = float(line[16:])
                self.brake_deadzone = value
            if 'throttle_deadzone:' in line:
                value = float(line[19:])
                self.throttle_deadzone = value

    def __str__(self):
        print('front_edge_to_center: {}'.format(self.front_edge_to_center))
        print('back_edge_to_center: {}'.format(self.back_edge_to_center))
        print('left_edge_to_center: {}'.format(self.left_edge_to_center))
        print('right_edge_to_center: {}'.format(self.right_edge_to_center))
        print('length: {}'.format(self.length))
        print('width: {}'.format(self.width))
        print('height: {}'.format(self.height))
        print('min_turn_radius: {}'.format(self.min_turn_radius))
        print('max_acceleration: {}'.format(self.max_acceleration))
        print('max_deceleration: {}'.format(self.max_deceleration))
        print('max_steer_angle: {}'.format(self.max_steer_angle))
        print('max_steer_angle_rate: {}'.format(self.max_steer_angle_rate))
        print('steer_ratio: {}'.format(self.steer_ratio))
        print('wheel_base: {}'.format(self.wheel_base))
        print('wheel_rolling_radius: {}'.format(self.wheel_rolling_radius))
        print('max_abs_speed_when_stopped: {}'.format(self.max_abs_speed_when_stopped))
        print('brake_deadzone: {}'.format(self.brake_deadzone))
        print('throttle_deadzone: {}'.format(self.throttle_deadzone))


if __name__ == '__main__':
    vehicle = Vehicle('vehicles/Lincoln2017 MKZ-vehicle_param.pb.txt')
    # vehicle.__str__()
    ev_heading = 0.5 * pi
    ev_position = {'x': 0, 'y': 0}
    # obs_position = {'x': 1, 'y': 1}
    # print(obstacle_vector(ev_position, obs_position))
    for i in range(8):
        x = cos(pi / 4 * i)
        y = sin(pi / 4 * i)
        obs_position = {'x': x, 'y': y}
        if is_ahead_of_ev(ev_heading, ev_position, obs_position) == True:
            print('{} is ahead of EV'.format(obs_position))
        if is_next_to_ev(ev_heading, ev_position, obs_position) == True:
            print('{} is next to EV'.format(obs_position))
        if is_blinds_spots(ev_heading, ev_position, obs_position) == True:
            print('{} is blinds spot of EV'.format(obs_position))
        if is_behind_ev(ev_heading, ev_position, obs_position) == True:
            print('{} is behind EV'.format(obs_position))
        print()

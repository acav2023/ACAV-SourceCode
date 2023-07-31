from recording_parsing import messages_aligning
from slicing_and_localizing import recording_vectorizing, first_frame_index_of_accident, recording_splitting, \
    vectors_denoising
from vehicle_parser import is_ahead_of_ev, is_next_to_ev, is_blinds_spots, is_behind_ev, Vehicle
from ev_behavior_checking_utils import ahead_priority_checking, next_priority_checking, behind_priority_checking, \
    obs_ignore_decision_checking, obs_follow_decision_checking, \
    obs_yield_decision_checking, obs_overtake_decision_checking, \
    common_speed_checking
from map_parser import Map, is_on_lane, is_in_junction
from vehicle_parser import ev_direction_vector
from math import dist, sqrt

def prechecking(localization_msgs, obstacle_msgs, prediction_msgs, planning_msgs, signal_msgs, map):
    critical_obstacles = critical_obstacles_identifying(localization_msgs, obstacle_msgs)
    cri_obs_list = critical_obstacle_list_obtaining(critical_obstacles)
    ret = {}
    msg_len = len(localization_msgs)
    speed_planning_checking_index = [-1 for i in range(0, msg_len)]
    for cri_obs in cri_obs_list:
        obs_id = cri_obs
        # print('obs_id: {}'.format(obs_id))
        obs_ret = {}
        trajectory_ground_truth = obstacle_trajectory_obtaining(obstacle_msgs, obs_id)
        msg_len = len(localization_msgs)
        priority_checking_ret = [-1 for i in range(0, msg_len)]
        trajectory_checking_ret = [-1 for i in range(0, msg_len)]
        obs_decision_checking_ret = [-1 for i in range(0, msg_len)]
        obs_decision_checking = ['' for i in range(0, msg_len)]
        # speed_planning_checking_ret = [-1 for i in range(0, msg_len)]
        for i in range(msg_len):
            if obs_id in critical_obstacles[i]:
                priority_checking_ret[i], trajectory_checking_ret[i] = prediction_checking(obs_id,
                                    trajectory_ground_truth, localization_msgs[i], prediction_msgs[i], obstacle_msgs[i])
                # obs_decision_checking_ret[i], obs_decision_checking[i] = \
                #     npc_decision_checking(map, obs_id, localization_msgs[i], planning_msgs[i], obstacle_msgs[i])
                obs_decision_checking_ret[i], obs_decision_checking[i] = \
                    planning_checking(obs_id, localization_msgs[i], planning_msgs[i], obstacle_msgs[i], map)
                speed_planning_checking_index[i] = 1
        obs_ret['prediction_priority'] = priority_checking_ret
        obs_ret['prediction_trajectory'] = trajectory_checking_ret
        obs_ret['planning_decision'] = obs_decision_checking_ret
        obs_ret['planning_decision_detailed'] = obs_decision_checking
        obs_ret['planning_speed'] = []
        ret[str(obs_id)] = obs_ret
    speed_planning_checking_ret = [-1 for i in range(0, msg_len)]
    for i in range(msg_len):
        if 1 == speed_planning_checking_index[i]:
            speed_planning_checking_ret[i] = speed_planning_checking(localization_msgs[i], planning_msgs[i], obstacle_msgs[i])
    for cri_obs in cri_obs_list:
        obs_id = cri_obs
        ret[str(obs_id)]['planning_speed'] = speed_planning_checking_ret
    return ret

def rechecking_v0(prim_checking_ret, localization_msgs, obstacle_msgs, planning_msgs, ev):
    msg_len = len(localization_msgs)
    rechecking_ret = [-1 for i in range(0, msg_len)]
    relative_index_reported = [-1 for i in range(0, msg_len)]

    frame_indices = notable_frame_index_obtaining(prim_checking_ret, msg_len)
    # frame_indices = [1 for i in range(0, msg_len)]
    notable_obs_id = list(prim_checking_ret.keys())

    ev_traj = ev_trajectory_obtaining(localization_msgs)
    ev_heading = ev_headings(localization_msgs)
    ev_st_graphs = ev_stgraph_obtaining(planning_msgs)
    # print('ev_traj_len: {}'.format(len(ev_traj)))
    # print(ev_traj)

    obs_info = {}
    for obs_id in notable_obs_id:
        info = obs_info_obtaining(obstacle_msgs, obs_id)
        obs_info[obs_id] = info

    obs_checking_points = {}
    for obs_id in notable_obs_id:
        obs_points = obs_checking_points_obtaining(obstacle_msgs, obs_id)
        obs_checking_points[obs_id] = obs_points


    for i in range(msg_len):
        if 1 == frame_indices[i]:
            obs_info_curr = {}
            for obs_id in notable_obs_id:
                obs_info_curr[obs_id] = obs_info[obs_id][i:]
            obs_check_points_curr = {}
            for obs_id in notable_obs_id:
                obs_check_points_curr[obs_id] = obs_checking_points[obs_id][i:]
            rechecking_ret[i], relative_index_reported[i] = stgraph_checking_v0(notable_obs_id, ev_traj[i:],
                                    ev_heading[i], ev_st_graphs[i], obs_info_curr, obs_check_points_curr, ev)

    # print(frame_indices)
    # print(rechecking_ret)
    critical_frame_counter = 0
    for i in range(msg_len):
        if 1 == rechecking_ret[i]:
            critical_frame_counter += 1

    return rechecking_ret, relative_index_reported, critical_frame_counter

def rechecking_v1(prim_checking_ret, localization_msgs, obstacle_msgs, planning_msgs, ev, first_index=-1):
    msg_len = len(localization_msgs)
    rechecking_ret = []
    pln_checking_ret = [-1 for i in range(0, msg_len)]
    ctl_checking_ret = [0 for i in range(0, msg_len)]
    acc_time_pred = [-1 for i in range(0, msg_len)]

    potential_frame_indices = notable_frame_index_obtaining(prim_checking_ret, msg_len)
    notable_obs_id = list(prim_checking_ret.keys())

    ev_traj_gt = ev_trajectory_obtaining(localization_msgs)

    obs_checking_points = {}
    for obs_id in notable_obs_id:
        obs_points = obs_checking_points_obtaining(obstacle_msgs, obs_id)
        obs_checking_points[obs_id] = obs_points


    # for i in range(msg_len):
    if -1 == first_index:
        first_index = msg_len
    # print(potential_frame_indices)
    for i in range(first_index):
        if 1 == potential_frame_indices[i]:
            obs_check_points_curr = {}
            for obs_id in notable_obs_id:
                obs_check_points_curr[obs_id] = obs_checking_points[obs_id][i:]
            pln_checking_ret[i], ctl_checking_ret[i], acc_time_pred[i] = stgraph_checking_v1(prim_checking_ret, i, notable_obs_id,
                                                    planning_msgs[i], ev_traj_gt[i:], obs_check_points_curr, ev)

    # print(rechecking_ret)
    # print(pln_checking_ret)
    # print(ctl_checking_ret)
    planning_status_prev = pln_checking_ret[0]
    for i in range(1, msg_len):
        if -1 < pln_checking_ret[i] and 4 > pln_checking_ret[i]:
            planning_status_prev = pln_checking_ret[i]
        elif 5 == pln_checking_ret[i] and -1 != planning_status_prev:
            pln_checking_ret[i] = planning_status_prev
        # elif -1 == rechecking_ret[0][i]:
        #     rechecking_ret[0][i] = 0
    # print(pln_checking_ret)
    # print(ctl_checking_ret)
    # print()

    pln_checking_ret = vectors_denoising(pln_checking_ret)
    ctl_checking_ret = vectors_denoising(ctl_checking_ret)
    # print(pln_checking_ret)
    # print(ctl_checking_ret)
    rechecking_ret.append(pln_checking_ret)
    rechecking_ret.append(ctl_checking_ret)

    critical_frame_counter = 0
    for i in range(first_index):
        if -1 != rechecking_ret[0][i] or 1 == rechecking_ret[1][i]:
            critical_frame_counter += 1

    return rechecking_ret, critical_frame_counter, acc_time_pred

def prediction_checking(obs_id, obstacle_trajectory_gt, localization_msg, prediction_msg, obstacle_msg):
    # (COMPLETED) Check priority
    # (COMPLETED) Check trajectory prediction
    perception_obstacle = {}
    if 'perceptionObstacle' in obstacle_msg:
        for obs_perc in obstacle_msg['perceptionObstacle']:
            if obs_id == obs_perc['id']:
                perception_obstacle = obs_perc
    obs_position = obstacle_position_classifying(localization_msg, perception_obstacle)
    priority_checking_ret = obstacle_priority_checking(obs_id, obs_position, prediction_msg)
    # print(obs_position)

    trajectory_checking_ret = 1
    trajectory_len = len(obstacle_trajectory_gt)
    if 'predictionObstacle' in prediction_msg:
        for obs_pred in prediction_msg['predictionObstacle']:
            if obs_id == obs_pred['perceptionObstacle']['id']:
                if 'trajectory' in obs_pred:
                    for j in range(0, trajectory_len):
                        if obs_pred['timestamp'] == obstacle_trajectory_gt[j]['timestamp']:
                            trajectory_checking_ret = predicted_trajectory_checking(obstacle_trajectory_gt[j:],
                                                                obs_pred['trajectory'][0]['trajectoryPoint'])
    return priority_checking_ret, trajectory_checking_ret

def planning_checking(obs_id, localization_msg, planning_msg, obstacle_msg, map):
    # Check the obstacle driving decision
    # Check the speed planning
    decision_checking_ret, decision = npc_decision_checking(map, obs_id, localization_msg, planning_msg, obstacle_msg)
    # speed_checking_ret = speed_planning_checking(localization_msg, planning_msg, obstacle_msg)
    # return decision_checking_ret, decision, speed_checking_ret
    return decision_checking_ret, decision


def notable_frame_index_obtaining(ret, msg_len):
    frame_indices = [0 for i in range(0, msg_len)]
    frame_indices[0] = 1
    # frame_indices[0], frame_indices[1], frame_indices[2] = 1, 1, 1
    for obs in ret:
        obs_checking_ret = ret[obs]
        # obs_priority_ret = obs_checking_ret['prediction_priority']
        obs_trajectory_ret = obs_checking_ret['prediction_trajectory']
        obs_decision_ret = obs_checking_ret['planning_decision']
        obs_decision = obs_checking_ret['planning_decision_detailed']
        obs_speed_ret = obs_checking_ret['planning_speed']
        seg_len = len(obs_speed_ret)
        decision_note = ['' for i in range(0, msg_len)]

        for i in range(1, seg_len):
            vec_curr = (obs_trajectory_ret[i], obs_decision_ret[i], obs_decision[i], obs_speed_ret[i])
            if is_notable(vec_curr):
                frame_indices[i] = 1

    return frame_indices

def stgraph_checking_v0(notable_obs_id, ev_traj, ev_heading, ev_stgraph, obs_info_curr, obs_check_points_curr, ev):
    checking_ret = 0
    rel_index = -1
    stb_len = len(ev_stgraph)
    s_curr = 0

    threshold = sqrt(ev.front_edge_to_center*ev.front_edge_to_center + ev.left_edge_to_center*ev.left_edge_to_center)

    for i in range(1, len(ev_traj)):
        ev_pos_prev = (ev_traj[i - 1]['x'], ev_traj[i - 1]['y'])
        ev_pos_curr = (ev_traj[i]['x'], ev_traj[i]['y'])
        s_curr += dist(ev_pos_prev, ev_pos_curr)
        for obs in notable_obs_id:
            for obs_point in obs_check_points_curr[obs][i]:
                obs_point_curr = (obs_point['x'], obs_point['y'])
                distance_curr = dist(obs_point_curr, ev_pos_curr)
                if distance_curr < threshold:
                    relative_index = round(i*0.08/0.1)
                    if relative_index < (stb_len/2):
                        if ev_stgraph[relative_index][0] > ev_stgraph[stb_len - 1 - relative_index][0]:
                            if s_curr < ev_stgraph[relative_index][0] and s_curr > \
                                    ev_stgraph[stb_len - 1 - relative_index][0]:
                                checking_ret = 1
                                rel_index = i
                                return checking_ret, rel_index
                        else:
                            if s_curr < ev_stgraph[stb_len - 1 - relative_index][0] and s_curr > \
                                    ev_stgraph[relative_index][0]:
                                checking_ret = 1
                                rel_index = i
                                return checking_ret, rel_index
    return checking_ret, rel_index

def stgraph_checking_v1(prim_checking_ret, index_curr, notable_obs_id, planning_msg, ev_traj_gt, obs_check_points_curr, ev):
    threshold = sqrt(
        ev.front_edge_to_center * ev.front_edge_to_center + ev.left_edge_to_center * ev.left_edge_to_center)
    s_curr = 0
    out_of_control = 0
    predicted_time = -1

    ## Check the speed planning and control
    ev_traj_008 = ev_planning_trajectory_obtaining_008(planning_msg, ev_traj_gt[0])
    max_len = min(len(ev_traj_008), len(ev_traj_gt))
    for i in range(max_len):
        ev_pos = (ev_traj_gt[i]['x'], ev_traj_gt[i]['y'])
        ev_pln = (ev_traj_008[i]['x'], ev_traj_008[i]['y'])
        error = dist(ev_pos, ev_pln)
        if error > 0.4:
            out_of_control = 1
            break

    ## Check planning module
    ev_traj_010 = ev_planning_trajectory_obtaining_010(planning_msg, ev_traj_gt[0])
    len_traj_pred = len(ev_traj_010)
    if 0 == len_traj_pred:
        return 5, out_of_control, predicted_time

    ev_pos_prev = (ev_traj_010[0]['x'], ev_traj_010[0]['y'])
    max_len = min(len_traj_pred, len(ev_traj_gt))
    for i in range(max_len):
        ev_pos_curr = (ev_traj_010[i]['x'], ev_traj_010[i]['y'])
        s_curr += dist(ev_pos_prev, ev_pos_curr)
        for obs in notable_obs_id:
            if len(obs_check_points_curr[obs]) > i:
                for obs_point in obs_check_points_curr[obs][i]:
                    obs_point_curr = (obs_point['x'], obs_point['y'])
                    distance_curr = dist(obs_point_curr, ev_pos_curr)
                    if distance_curr < threshold:
                        predicted_time = i
                        if 1 == prim_checking_ret[obs]['prediction_trajectory'][index_curr]:
                            # print(len_traj_pred)
                            return 1, out_of_control, predicted_time
                        else:
                            return 2, out_of_control, predicted_time

    ## Short-signed planning
    if len_traj_pred < 38:
        delta_s = ev_traj_010[-1]['v'] * 0.1
        ev_heading_vector = ev_direction_vector(ev_traj_010[-1]['theta'])
        delta_x = delta_s * ev_heading_vector[0]
        delta_y = delta_s * ev_heading_vector[-1]
        # print(ev_traj_010[-1])
        # print('({}, {})'.format(delta_x, delta_y))
        for i in range(len_traj_pred, len(ev_traj_gt)):
            ev_pos_curr = {}
            x_pstn = ev_traj_010[-1]['x'] + delta_x
            y_pstn = ev_traj_010[-1]['y'] + delta_y
            ev_pos_curr['x'] = x_pstn
            ev_pos_curr['y'] = y_pstn
            ev_traj_010.append(ev_pos_curr)

        for i in range(len(ev_traj_gt)):
            ev_pos_curr = (ev_traj_010[i]['x'], ev_traj_010[i]['y'])
            s_curr += dist(ev_pos_prev, ev_pos_curr)
            for obs in notable_obs_id:
                if len(obs_check_points_curr[obs]) > i:
                    for obs_point in obs_check_points_curr[obs][i]:
                        obs_point_curr = (obs_point['x'], obs_point['y'])
                        distance_curr = dist(obs_point_curr, ev_pos_curr)
                        if distance_curr < threshold:
                            predicted_time = i
                            return 3, out_of_control, predicted_time

    return 0, out_of_control, predicted_time




"""
Functions for all
"""

def critical_obstacles_identifying(localization_msgs, obstacle_msgs):
    critical_obstacles = []
    msg_len = len(localization_msgs)
    for i in range(0, msg_len):
        critical_obs = []
        ev_pos_cur = (localization_msgs[i]['pose']['position']['x'], localization_msgs[i]['pose']['position']['y'],
              localization_msgs[i]['pose']['position']['z'])
        ev_vel_vec_cur = (localization_msgs[i]['pose']['linearVelocity']['x'],
            localization_msgs[i]['pose']['linearVelocity']['y'], localization_msgs[i]['pose']['linearVelocity']['z'])
        ev_vel_cur = dist((0, 0, 0), ev_vel_vec_cur)
        if 'perceptionObstacle' in obstacle_msgs[i]:
            for obs in obstacle_msgs[i]['perceptionObstacle']:
                obs_pos = (obs['position']['x'], obs['position']['y'], obs['position']['z'])
                obs_vel_vec = (obs['velocity']['x'], obs['velocity']['y'], obs['velocity']['z'])
                obs_vel = dist((0, 0, 0), obs_vel_vec)
                critical_radius = max(3 * ev_vel_cur, 3 * obs_vel)
                distance = dist(ev_pos_cur, obs_pos)
                if distance < critical_radius:
                    critical_obs.append(obs['id'])
        critical_obstacles.append(critical_obs)
        # print(critical_obs)
    return critical_obstacles

def critical_obstacle_list_obtaining(critical_obstacles):
    critical_obs_list = []
    lists = []
    for frame in critical_obstacles:
        lists += frame
    critical_obs_list = list(set(lists))
    return critical_obs_list

def obstacle_position_classifying(localization_msg, perception_obstacle):
    """
    Check which area the obstacle @param perception_obstacle is located near EV in the current frame

    :param localization_msg: the localization message in the current frame
    :param perception_obstacle: the given obstacle
    :return: a code string
    """
    ev_heading = 0
    ev_center_position = {'x': 0, 'y': 0}
    obs_center_position = {'x': 0, 'y': 0}
    if 'pose' in localization_msg:
        if 'heading' in localization_msg['pose']:
            ev_heading = localization_msg['pose']['heading']
        if 'position' in localization_msg['pose']:
            ev_center_position['x'] = localization_msg['pose']['position']['x']
            ev_center_position['y'] = localization_msg['pose']['position']['y']
    if 'position' in perception_obstacle:
        obs_center_position['x'] = perception_obstacle['position']['x']
        obs_center_position['y'] = perception_obstacle['position']['y']
    # print('ev_heading: {}, ev_center: {}, obs_center: {}'.format(ev_heading, ev_center_position, obs_center_position))
    if is_ahead_of_ev(ev_heading, ev_center_position, obs_center_position) == True:
        # print('{} is ahead of EV'.format(obs_center_position))
        return 'AHEAD'
    if is_next_to_ev(ev_heading, ev_center_position, obs_center_position) == True:
        # print('{} is next to EV'.format(obs_center_position))
        return 'NEXT'
    if is_blinds_spots(ev_heading, ev_center_position, obs_center_position) == True:
        # print('{} is blinds spot of EV'.format(obs_center_position))
        return 'BLIND'
    if is_behind_ev(ev_heading, ev_center_position, obs_center_position) == True:
        # print('{} is behind EV'.format(obs_center_position))
        return 'BEHIND'

def is_notable(vec):
    for item in vec:
        if type(item) == int and item > 0:
            return True
    return False

def ev_headings(localization_msgs):
    ev_heading = []
    for msg in localization_msgs:
        if 'pose' in msg and 'heading' in msg['pose']:
            ev_heading.append(msg['pose']['heading'])
    return  ev_heading


"""
Functions for @func prediction_checking
"""

def obstacle_priority_checking(obs_id, obs_position, prediction_msg):
    checking_ret = -1
    priority_pred = ''
    subtype_pred = ''
    is_static = False
    if 'predictionObstacle' in prediction_msg:
        for obs_pred in prediction_msg['predictionObstacle']:
            if obs_id == obs_pred['perceptionObstacle']['id']:
                if 'priority' in obs_pred:
                    priority_pred = obs_pred['priority']['priority']
                if 'subType' in obs_pred['perceptionObstacle']:
                    subtype_pred = obs_pred['perceptionObstacle']['subType']
                if 'isStatic' in obs_pred:
                    is_static = obs_pred['isStatic']
    # print('{} {} {}'.format(priority_pred, subtype_pred, is_static))
    if 'AHEAD' == obs_position:
        checking_ret = ahead_priority_checking(priority_pred, subtype_pred)
    elif 'NEXT' == obs_position:
        checking_ret = next_priority_checking(priority_pred, subtype_pred, is_static)
    elif 'BLIND' == obs_position:
        checking_ret = next_priority_checking(priority_pred, subtype_pred, is_static)
    elif 'BEHIND' == obs_position:
        checking_ret = behind_priority_checking(priority_pred, subtype_pred)
    return checking_ret

def obstacle_trajectory_obtaining(obstacle_msgs, obs_id):
    obs_traj = []
    timestamp_prev = 0
    for msg in obstacle_msgs:
        traj = {}
        if 'perceptionObstacle' in msg:
            for obs in msg['perceptionObstacle']:
                if obs_id == obs['id']:
                    if timestamp_prev != obs['timestamp']:
                        traj['timestamp'] = obs['timestamp']
                        traj['position'] = obs['position']
                        # traj['theta'] = obs['theta']
                        # print(obs['theta'])
                        # traj['velocity'] = obs['velocity']
                        obs_traj.append(traj)
                        timestamp_prev = obs['timestamp']
                        # print(traj)
    return obs_traj

def ev_planning_trajectory_obtaining_010(planning_msg, ev_pos_curr=''):
    ev_planning_traj = []
    i = 0
    if 'trajectoryPoint' in planning_msg:
        for tjpt in planning_msg['trajectoryPoint']:
            if abs(tjpt['relativeTime'] - i * 0.1) < 0.05:
                waypoint = {}
                waypoint['x'] = tjpt['pathPoint']['x']
                waypoint['y'] = tjpt['pathPoint']['y']
                waypoint['theta'] = tjpt['pathPoint']['theta']
                waypoint['v'] = tjpt['v']
                waypoint['relativeTime'] = tjpt['relativeTime']
                ev_planning_traj.append(waypoint)
                i += 1
    # relative_time = -1
    # delta_t = planning_msg['header']['timestampSec'] - ev_pos_curr['timestampSec']
    # print('    {}'.format(delta_t))
    # if 'trajectoryPoint' in planning_msg:
    #     for tjpt in planning_msg['trajectoryPoint']:
    #         if tjpt['relativeTime'] - 0 < delta_t:
    #             relative_time = tjpt['relativeTime']
    #             break
    #         else:
    #             planning_msg['trajectoryPoint'].remove(tjpt)
    #
    #     for tjpt in planning_msg['trajectoryPoint']:
    #         if abs(tjpt['relativeTime'] - relative_time) < 0.01:
    #             waypoint = {}
    #             waypoint['x'] = tjpt['pathPoint']['x']
    #             waypoint['y'] = tjpt['pathPoint']['y']
    #             waypoint['theta'] = tjpt['pathPoint']['theta']
    #             waypoint['v'] = tjpt['v']
    #             waypoint['relativeTime'] = tjpt['relativeTime']
    #             ev_planning_traj.append(waypoint)
    #             relative_time += 0.1
    # for item in ev_planning_traj:
    #     print(item['relativeTime'])
    return ev_planning_traj

def ev_planning_trajectory_obtaining_008(planning_msg, ev_pos_curr=''):
    ev_planning_traj = []
    i = 0
    # for tjpt in planning_msg['trajectoryPoint']:
    #     if abs(tjpt['relativeTime'] - i * 0.08) < 0.05:
    #         waypoint = {}
    #         waypoint['x'] = tjpt['pathPoint']['x']
    #         waypoint['y'] = tjpt['pathPoint']['y']
    #         waypoint['theta'] = tjpt['pathPoint']['theta']
    #         waypoint['v'] = tjpt['v']
    #         waypoint['relativeTime'] = tjpt['relativeTime']
    #         ev_planning_traj.append(waypoint)
    #         i += 1
    relative_time = -1
    delta_t = planning_msg['header']['timestampSec'] - ev_pos_curr['timestampSec']
    # print('    {}'.format(delta_t))
    if 'trajectoryPoint' in planning_msg:
        for tjpt in planning_msg['trajectoryPoint']:
            if tjpt['relativeTime'] - 0 < delta_t:
                relative_time = tjpt['relativeTime']
                break
            else:
                planning_msg['trajectoryPoint'].remove(tjpt)

        for tjpt in planning_msg['trajectoryPoint']:
            if abs(tjpt['relativeTime'] - relative_time) < 0.01:
                waypoint = {}
                waypoint['x'] = tjpt['pathPoint']['x']
                waypoint['y'] = tjpt['pathPoint']['y']
                waypoint['theta'] = tjpt['pathPoint']['theta']
                waypoint['v'] = tjpt['v']
                waypoint['relativeTime'] = tjpt['relativeTime']
                ev_planning_traj.append(waypoint)
                relative_time += 0.08
    # for item in ev_planning_traj:
    #     print(item['relativeTime'])
    return ev_planning_traj

def ev_trajectory_obtaining(localization_msgs):
    ev_traj = []
    for msg in localization_msgs:
        waypoint = {}
        if 'pose' in msg and 'position' in msg['pose']:
            waypoint['x'] = msg['pose']['position']['x']
            waypoint['y'] = msg['pose']['position']['y']
            waypoint['z'] = msg['pose']['position']['z']
            waypoint['timestampSec'] = msg['header']['timestampSec']
            ev_traj.append(waypoint)
    return ev_traj

def obs_checking_points_obtaining(obstacle_msgs, obs_id):
    obs_checking_points = []
    for msg in obstacle_msgs:
        obs_points = []
        if 'perceptionObstacle' in msg:
            for obs in msg['perceptionObstacle']:
                if int(obs_id) == obs['id']:
                    traj = obs['position']
                    obs_points.append(traj)
                    polygon_points = []
                    for pp in obs['polygonPoint']:
                        polygon_points.append(pp)
                    if 0 != len(polygon_points):
                        for i in range(1, len(polygon_points)):
                            pp = {}
                            pp1 = polygon_points[i-1]
                            pp2 = polygon_points[i]
                            pp['x'] = (pp1['x'] + pp2['x']) / 2
                            pp['y'] = (pp1['y'] + pp2['y']) / 2
                            pp['z'] = (pp1['z'] + pp2['z']) / 2
                            polygon_points.append(pp)
                        else:
                            pp = {}
                            pp1 = polygon_points[-1]
                            pp2 = polygon_points[0]
                            pp['x'] = (pp1['x'] + pp2['x']) / 2
                            pp['y'] = (pp1['y'] + pp2['y']) / 2
                            pp['z'] = (pp1['z'] + pp2['z']) / 2
                            polygon_points.append(pp)
                    obs_points += polygon_points
        obs_checking_points.append(obs_points)
    return obs_checking_points

def obs_info_obtaining(obstacle_msgs, obs_id):
    obs_info = []
    for msg in obstacle_msgs:
        if 'perceptionObstacle' in msg:
            for obs in msg['perceptionObstacle']:
                if int(obs_id) == obs['id']:
                    info = {}
                    info['position'] = obs['position']
                    info['length'] = obs['length']
                    info['width'] = obs['width']
                    info['theta'] = obs['theta']
                    info['polygonPoint'] = obs['polygonPoint']
                    obs_info.append(info)
    return obs_info

def predicted_trajectory_checking(actual_trajectory_segment, predicted_trajectory):
    segment_length = len(actual_trajectory_segment)
    end_index = 30
    position_error_sum = 0
    if segment_length < 30:
        end_index = segment_length
    for i in range(end_index):
        actual_position = (actual_trajectory_segment[i]['position']['x'], actual_trajectory_segment[i]['position']['y'])
        predicted_position = (predicted_trajectory[i]['pathPoint']['x'], predicted_trajectory[i]['pathPoint']['y'])
        position_error_cur = dist(actual_position, predicted_position)
        position_error_sum += position_error_cur
    position_error_mean = position_error_sum / end_index
    # print('position_error: {}'.format(position_error_mean))
    # TODO: Set a proper threshold here for position prediction error
    if position_error_mean < 0.5:
        return 0
    return 1

"""
Functions for @func planning_checking
"""

def npc_decision_obtaining(obs_id, object_decisions):
    decision = ''
    if 'decision' in object_decisions:
        for obs_deci in object_decisions['decision']:
            if obs_id == obs_deci['perceptionId'] and 'objectDecision' in obs_deci:
                decision = npc_decision_parser(obs_deci['objectDecision'])
    return decision

def npc_decision_parser(object_decision):
    for od in object_decision:
        if 'ignore' in od:
            return 'ignore'
        elif 'stop' in od:
            return 'stop'
        elif 'follow' in od:
            return 'follow'
        elif 'yield' in od:
            return 'yield'
        elif 'overtake' in od:
            return 'overtake'

def npc_decision_checking(map, obs_id, localization_msg, planning_msg, obstacle_msg):
    checking_ret = -1
    current_decision = ''
    obs_type = ''
    obs_position = ''
    perception_obstacle = {}

    if 'decision' in planning_msg and 'objectDecision' in planning_msg['decision']:
        object_decisions = planning_msg['decision']['objectDecision']
        current_decision = npc_decision_obtaining(obs_id, object_decisions)
    if 'perceptionObstacle' in obstacle_msg:
        for obs in obstacle_msg['perceptionObstacle']:
            if obs_id == obs['id'] and 'type' in obs:
                obs_type = obs['type']
                perception_obstacle = obs
    obs_position = obstacle_position_classifying(localization_msg, perception_obstacle)

    if 'ignore' == current_decision:
        checking_ret = obs_ignore_decision_checking(obs_type, localization_msg)
    elif 'follow' == current_decision:
        checking_ret = obs_follow_decision_checking(obs_type, localization_msg)
    # elif 'stop' == current_decision:
    #     checking_ret = obs_stop_decision_checking(obs_type, localization_msg)
    elif 'yield' == current_decision:
        checking_ret = obs_yield_decision_checking(obs_type, obs_position, localization_msg, map)
    elif 'overtake' == current_decision:
        checking_ret = obs_overtake_decision_checking(localization_msg, map)
    if None == current_decision:
        checking_ret = 1
        current_decision = 'none'
    if checking_ret != 1:
        current_decision = ''
    return checking_ret, current_decision

def speed_planning_checking(localization_msg, planning_msg, obstacle_msg):
    checking_ret = common_speed_checking(localization_msg)
    return checking_ret

def ev_stgraph_obtaining(planning_msgs):
    st_boundary = []
    for msg in planning_msgs:
        cur_st = []
        if 'debug' in msg and 'planningData' in msg['debug']:
            if 'stGraph' in msg['debug']['planningData']:
                if 'boundary' in msg['debug']['planningData']['stGraph'][0]:
                    for bnd in msg['debug']['planningData']['stGraph'][0]['boundary']:
                        if 'Generated ST-Boundary' == bnd['name']:
                            for pt in bnd['point']:
                                cur_st.append((pt['s'], pt['t']))
        st_boundary.append(cur_st)
    return st_boundary


def get_npc_feature(prediction_msg, map):
    features = {}
    if 'predictionObstacle' in prediction_msg:
        for obs in prediction_msg['predictionObstacle']:
            obs_feature = {}
            obs_id_curr, type_curr, area_curr, speed_curr, priority_curr, tag_curr, is_static_curr = \
                0, '', 'OFF_LINE', 0,  '', '', False
            if 'perceptionObstacle' in obs and 'id' in obs['perceptionObstacle']:
                obs_id_curr = obs['perceptionObstacle']['id']
            if 'perceptionObstacle' in obs and 'type' in obs['perceptionObstacle']:
                type_curr = obs['perceptionObstacle']['type']
            if 'perceptionObstacle' in obs and 'position' in obs['perceptionObstacle']:
                position_curr = obs['perceptionObstacle']['position']
                obs_pos = (position_curr['x'], position_curr['y'], position_curr['z'])
                if is_on_lane(obs_pos, map):
                    area_curr = 'ON_LANE'
                if is_in_junction(obs_pos, map):
                    area_curr = 'JUNCTION'
            if 'perceptionObstacle' in obs and 'velocity' in obs['perceptionObstacle']:
                speed = obs['perceptionObstacle']['velocity']
                speed_vec = (speed['x'], speed['y'], speed['z'])
                speed_curr = dist(speed_vec, (0, 0, 0))
            if 'priority' in obs and 'priority' in obs['priority']:
                priority_curr = obs['priority']['priority']
            if 'interactiveTag' in obs and 'interactiveTag' in obs['interactiveTag']:
                tag_curr = obs['interactiveTag']['interactiveTag']
            if 'isStatic' in obs:
                is_static_curr = obs['isStatic']
            obs_feature['type'], obs_feature['area'], obs_feature['speed'], obs_feature['priority'], obs_feature['tag'],\
                obs_feature['is_static'] = type_curr, area_curr, speed_curr, priority_curr, tag_curr, is_static_curr
            features[obs_id_curr] = obs_feature
    return features


def predictor_assigning(prediction_msg, map):
    features = get_npc_feature(prediction_msg, map)
    npc_ids = list(features.keys())
    predictors = {}
    for npc in npc_ids:
        type_curr, area_curr, speed_curr, priority_curr, tag_curr, is_static = \
            features[npc]['type'], features[npc]['area'], features[npc]['speed'], features[npc]['priority'], \
            features[npc]['tag'], features[npc]['is_static']
        # predictors[npc] = ''
        predictors_curr = []
        if 'ignore' == priority_curr:
            predictors_curr.append('EMPTY_PREDICTOR')
        elif is_static:
            predictors_curr.append('EMPTY_PREDICTOR')
        else:
            if 'VEHICLE' == type_curr:
                if 'INTERACTION' == tag_curr:
                    predictors_curr.append('EMPTY_PREDICTOR')
                if 'caution' == priority_curr:
                    if 'JUNCTION' == area_curr:
                        predictors_curr.append('INTERACTION_PREDICTOR')
                    elif 'ON_LANE' == area_curr:
                        predictors_curr.append('MOVE_SEQUENCE_PREDICTOR')
                    else:
                        predictors_curr.append('EXTRAPOLATION_PREDICTOR')
                if 'OFF_LANE' == area_curr:
                    predictors_curr.append('FREE_MOVE_PREDICTOR')
                elif 'JUNCTION' == area_curr:
                    predictors_curr.append('LANE_SEQUENCE_PREDICTOR')
                else:
                    predictors_curr.append('LANE_SEQUENCE_PREDICTOR')
            elif 'PEDESTRIAN' == type_curr:
                predictors_curr.append('FREE_MOVE_PREDICTOR')
            elif 'BICYCLE' == type_curr:
                if 'OFF_LANE' == area_curr:
                    predictors_curr.append('FREE_MOVE_PREDICTOR')
                else:
                    predictors_curr.append('LANE_SEQUENCE_PREDICTOR')
            else:
                if 'OFF_LANE' == area_curr:
                    predictors_curr.append('FREE_MOVE_PREDICTOR')
                else:
                    predictors_curr.append('LANE_SEQUENCE_PREDICTOR')
        predictors[npc] = predictors_curr
    return predictors


if __name__ == '__main__':
    original_record_dir = 'record/T-2.record'
    # directory = '/home/sun/sun/AlignedMessagesData/Trace0/'

    map = Map('maps/san_francisco.bin')
    map.parse()

    ev = Vehicle('vehicles/Lincoln2017 MKZ-vehicle_param.pb.txt')

    localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs = \
        messages_aligning(original_record_dir)

    ev_traj_gt = ev_trajectory_obtaining(localization_msgs)

    for j in range(136, 189):
        # if 'trajectoryPoint' in planning_msgs[j]:
        #     tsmp = planning_msgs[j]['trajectoryPoint'][-1]['relativeTime']
        #     print(j-136, tsmp)
        #     for tjpt in planning_msgs[j]['trajectoryPoint']:
        #         print(tjpt['relativeTime'])
        print(j-136)
        traj = ev_planning_trajectory_obtaining_008(planning_msgs[j], ev_traj_gt[j])
        if 'trajectoryPoint' in planning_msgs[j]:
            print(planning_msgs[j]['trajectoryPoint'][-1]['relativeTime'])
            # for tjpt in planning_msgs[j]['trajectoryPoint']:
            #     print('    {}'.format(tjpt['relativeTime']))
        print(traj[-1]['relativeTime'])
        print('  {} {}'.format(len(traj), len(ev_traj_gt[j:])))
        # for tjpt in traj:
        #     print('    {}'.format(tjpt['relativeTime']))
        print()

    # for j in range(136, 189):
    #     traj = ev_planning_trajectory_obtaining_008(planning_msgs[j], ev_traj_gt[j])
    #     ev_traj = ev_traj_gt[j:]
    #     max_len = min(len(traj), len(ev_traj))
    #     for i in range(max_len):
    #         ev_pos = (ev_traj[i]['x'], ev_traj[i]['y'])
    #         ev_pln = (traj[i]['x'], traj[i]['y'])
    #         error = dist(ev_pos, ev_pln)
    #         if error > 0.4:
    #             print('{} {} {}'.format(j-136, i, error))
    #         # print(ev_traj[i])
    #         # print(traj[i])
    #     print()

    # m_vectors, pp_vectors, pl_vectors = recording_vectorizing(localization_msgs, obstacle_msgs, signal_msgs,
    #                                                              prediction_msgs, planning_msgs, map)
    #
    # fi = first_frame_index_of_accident(localization_msgs, obstacle_msgs, ev)
    # print(fi)
    #
    # clips = recording_splitting(m_vectors, pp_vectors, pl_vectors, fi)
    # print(clips)
    #
    # ret = prechecking(localization_msgs, obstacle_msgs, prediction_msgs, planning_msgs, signal_msgs, map)
    # # print(ret)
    #
    # rechecking_ret, cf_counter, pred_time = rechecking_v1(ret, localization_msgs, obstacle_msgs, planning_msgs, ev, fi)
    # print()
    # # print(rechecking_ret[0:clips[0][0]])
    # print(rechecking_ret[clips[0][0]:fi])
    # print(cf_counter)
    # print()
    #
    # notable_obs_id = list(ret.keys())
    # for obs in notable_obs_id:
    #     print(obs)
    #     print(ret[obs]['prediction_priority'][clips[0][0]:fi])
    #     print(ret[obs]['prediction_trajectory'][clips[0][0]:fi])
    #     print(ret[obs]['planning_decision'][clips[0][0]:fi])
    #     print(ret[obs]['planning_decision_detailed'][clips[0][0]:fi])
    #     print()
    # print(ret[notable_obs_id[0]]['planning_speed'][clips[0][0]:fi])
    #
    # for i in range(fi):
    #     if -1 != pred_time[i]:
    #         print('{}: {}'.format(i - clips[0][0], pred_time[i]))

    # _, relat_index, _ = rechecking(ret, localization_msgs, obstacle_msgs, planning_msgs, ev)
    # print(relat_index)
    #
    # error_sum = 0
    # frame_counter = 0
    # for i in range(len(relat_index)):
    #     if -1 != relat_index[i]:
    #         error_sum += abs(i + relat_index[i] - fi)
    #         # print('    frame index: {}    current error: {}'.format(i, abs(i + relat_index[i] - fi)))
    #         frame_counter += 1
    #
    # # print('Total: {} {}    {}'.format(error_sum, frame_counter, error_sum/frame_counter))
    #
    # aindex_pred = accident_prediction(relat_index)
    # print(aindex_pred)
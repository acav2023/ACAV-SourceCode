from slicing_and_localizing import is_near_a_junction, is_near_a_crosswalk, signal_color
from math import dist

def ahead_priority_checking(priority_pred, obstacle_subtype):
    checking_ret = -1
    caution_subtype_list = ['ST_TRUCK', 'ST_BUS', 'ST_CYCLIST', 'ST_MOTORCYCLIST', 'ST_TRICYCLIST', 'ST_PEDESTRIAN']
    normal_subtype_list = ['ST_CAR', 'ST_VAN']
    if obstacle_subtype in caution_subtype_list:
        # if 'CAUTION' == priority_pred:
        #     checking_ret = 0
        # elif 'NORMAL' == priority_pred:
        #     checking_ret = 1
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 3
        if 'IGNORE' == priority_pred:
            checking_ret = 1
    elif obstacle_subtype in normal_subtype_list:
        # if 'CAUTION' == priority_pred or 'NORMAL' == priority_pred:
        #     checking_ret = 0
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 2
        if 'IGNORE' == priority_pred:
            checking_ret = 1
    return checking_ret

def next_priority_checking(priority_pred, obstacle_subtype, is_static):
    checking_ret = -1
    caution_subtype_list = ['ST_TRUCK', 'ST_BUS', 'ST_CYCLIST', 'ST_MOTORCYCLIST', 'ST_TRICYCLIST', 'ST_PEDESTRIAN']
    normal_subtype_list = ['ST_CAR', 'ST_VAN']
    if obstacle_subtype in caution_subtype_list:
        # if 'CAUTION' == priority_pred:
        #     checking_ret = 0
        # elif 'NORMAL' == priority_pred:
        #     checking_ret = 1
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 3
        if 'IGNORE' == priority_pred:
            checking_ret = 1
    elif obstacle_subtype in normal_subtype_list:
        # if 'CAUTION' == priority_pred or 'NORMAL' == priority_pred:
        #     checking_ret = 0
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 2
        if 'IGNORE' == priority_pred:
            checking_ret = 1
    if is_static and (obstacle_subtype in caution_subtype_list or obstacle_subtype in normal_subtype_list):
        # if 'CAUTION' == priority_pred:
        #     checking_ret = 0
        # elif 'NORMAL' == priority_pred:
        #     checking_ret = 1
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 3
        # if 'IGNORE' == priority_pred:
            checking_ret = 1
    return checking_ret

def behind_priority_checking(priority_pred, obstacle_subtype):
    checking_ret = -1
    caution_subtype_list = ['ST_TRUCK', 'ST_BUS', 'ST_CYCLIST', 'ST_MOTORCYCLIST', 'ST_TRICYCLIST', 'ST_PEDESTRIAN',
                            'ST_CAR', 'ST_VAN']
    if obstacle_subtype in caution_subtype_list:
        # if 'CAUTION' == priority_pred:
        #     checking_ret = 0
        # elif 'NORMAL' == priority_pred:
        #     checking_ret = 1
        # elif 'IGNORE' == priority_pred:
        #     checking_ret = 3
        if 'IGNORE' == priority_pred:
            checking_ret = 1
    return checking_ret

def obs_ignore_decision_checking(obs_type, localization_msg):
    return 1

def obs_follow_decision_checking(obs_type, localization_msg):
    return 1

def obs_stop_decision_checking(obs_type, localization_msg):
    if 'VEHICLE' == obs_type:
        return 1
    return -1

def obs_yield_decision_checking(obs_type, obs_position, localization_msg, map):
    if 'PEDESTRIAN' == obs_type or 'BICYCLE' == obs_type:
        return 0
    if is_near_a_junction(localization_msg, map):
        return 0
    if is_near_a_crosswalk(localization_msg, map):
        return 0
    if 'BLIND' == obs_position or 'BEHIND' == obs_position:
        return 1
    return -1

def obs_overtake_decision_checking(localization_msg, map):
    if is_near_a_junction(localization_msg, map):
        return 1
    if is_near_a_crosswalk(localization_msg, map):
        return 1
    return -1

def common_speed_checking(localization_msg):
    if 'pose' in localization_msg :
        if 'linearVelocity' in localization_msg['pose']:
            v_vec = (localization_msg['pose']['linearVelocity']['x'], localization_msg['pose']['linearVelocity']['y'],
                 localization_msg['pose']['linearVelocity']['z'])
            v = dist((0, 0, 0), v_vec)
            if v > 8.33:
                return 1
            elif v < 5.56:
                return 1
    return -1

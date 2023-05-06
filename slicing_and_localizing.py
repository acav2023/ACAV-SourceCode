from map_parser import near_crosswalk, near_junction, near_stop_sign
from math import dist, sqrt

def recording_vectorizing(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs, map):
    """
    Obtain feature vectors of current record, environment feature vectors and EV's behavior feature vectors

    :param localization_msgs: the list of localization messages
    :param obstacle_msgs: the list of obstacle messages
    :param signal_msgs: the list of traffic light messages
    :param prediction_msgs: the list of prediction messages
    :param planning_msgs: the list of planning messages
    :param map: current map
    :return: denoised environment feature vectors and EV's behavior feature vectors
    """
    # env_vectors = map_vectors_extracting(localization_msgs, signal_msgs, map)
    # behav_vectors = ev_behavior_vectors_extracting(localization_msgs, obstacle_msgs, prediction_msgs, planning_msgs)
    # npc_vectors = npcs_feature_extracting(planning_msgs)
    map_vectors = map_vectors_extracting(localization_msgs, signal_msgs, map)
    perc_pred_vectors = perception_prediction_vectors_extracting(localization_msgs, obstacle_msgs, prediction_msgs)
    plan_vectors = planning_feature_extracting(planning_msgs)
    return map_vectors, perc_pred_vectors, plan_vectors

def recording_splitting(map_vectors, pp_vectors, pl_vectors, first_index):
    """
    Splitting the entire recording into a series of segments.

    :param map_vectors: map feature vectors
    :param pp_vectors: perception & prediction feature vectors
    :param pl_vectors: planning feature vectors
    :return: (first_index, last_index)
    """
    vec_len = len(map_vectors)
    clips_1 = vectors_splitting(map_vectors)
    clips_2 = vectors_splitting(pp_vectors)
    clips_3 = vectors_splitting(pl_vectors)

    # voting
    ballot_list = [0 for i in range(vec_len)]
    ballot_list[0], ballot_list[-1] = 2, 2
    votes = clips_1 + clips_2
    for vote in votes:
        index = vote[-1]
        ballot_list[index] += 1
    votes = clips_3
    for vote in votes:
        index = vote[-1]
        ballot_list[index] += 2

    clips_t2 = []
    current_segment_start = 0
    for i in range(1, vec_len):
        if ballot_list[i] >= 2:
            current_segment_end = i
            clips_t2.append((current_segment_start, current_segment_end))
            current_segment_start = i

    # Check & Merge
    clips_filtered = []
    for cl in clips_t2:
        if not map_vector_checking(map_vectors, cl) or not perception_prediction_checking(pp_vectors, cl):
            clips_filtered.append(cl)
        elif cl[0] <= first_index and first_index <= cl[-1]:
            clips_filtered.append(cl)
    # print(clips_filtered)
    clips_final = [clips_filtered[0]]
    for i in range(1, len(clips_filtered)):
        if clips_final[-1][-1] == clips_filtered[i][0]:
            clip_tmp = clips_final.pop()
            clips_final.append((clip_tmp[0], clips_filtered[i][-1]))
        else:
            clips_final.append(clips_filtered[i])

    clip_final = [(-1, -1)]
    for item in clips_final:
        if item[0] < first_index and first_index <= item[-1]:
            clip_final = [item]
            break
    # print(clips_final, first_index, clip_final)

    return clip_final

def map_vector_checking(map_vectors, clip):
    score = 0
    vote = False
    for i in range(clip[0], clip[-1]):
        if set([False, False, False])< set(map_vectors[i]):
            score += 1
    if 0.8 < (score/(clip[-1]-clip[0])):
        vote = True
    return vote

def perception_prediction_checking(pp_vectors, clip):
    score = 0
    vote = False
    for i in range(clip[0], clip[-1]):
        if [False, False, [], []] == pp_vectors[i]:
            score += 1
    if 0.8 < (score/(clip[-1]-clip[0])):
        vote = True
    return vote

def accident_segment_localizing(clips, localization_msgs, obstacle_msgs, ev):
    a_index = first_frame_index_of_accident(localization_msgs, obstacle_msgs, ev)
    start, end = -1, -1
    for c in clips:
        if a_index > c[0] and a_index <= c[-1]:
            start, end = c[0], c[-1]
    return (start, end)

def first_frame_index_of_accident(localization_msgs, obstacle_msgs, ev):
    msg_len = len(localization_msgs)
    for i in range(msg_len):
        if has_accidents_current_frame(localization_msgs[i], obstacle_msgs[i], ev):
            return i
    return -1

def has_accidents_current_frame(localization_msg, obstacle_msg, ev):
    """
    Whether an accident occurred in this frame?

    :param localization_msg: a localization message
    :param obstacle_msg: obstacle_msg
    :return: @true if there was an accident
    """
    threshold = sqrt(
        ev.front_edge_to_center * ev.front_edge_to_center + ev.left_edge_to_center * ev.left_edge_to_center)
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    if 'perceptionObstacle' in obstacle_msg:
        for obs in obstacle_msg['perceptionObstacle']:
            obs_pos = (obs['position']['x'], obs['position']['y'], obs['position']['z'])
            distance = dist(ev_pos, obs_pos)
            # if distance < 3:
            if distance < threshold:
                return True
            for pts in obs['polygonPoint']:
                pt = (pts['x'], pts['y'], pts['z'])
                distance = dist(ev_pos, pt)
                # if distance < 3:
                if distance < threshold:
                    # print(obs['id'])
                    return True
    return False

def is_near_a_crosswalk(localization_msg, map):
    """
    Is EV's current position near a crosswalk?

    :param localization_msg: a localization message
    :param map: current {@class Map} map
    :return: {@code True} if EV is near a crosswalk, {@code False} otherwise
    """
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    return near_crosswalk(ev_pos, map)

def is_near_a_junction(localization_msg, map):
    """
    Is EV's current position near a junction?

    :param localization_msg: a localization message
    :param map: current {@class Map} map
    :return: {@code True} if EV is near a junction, {@code False} otherwise
    """
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    return near_junction(ev_pos, map)

def is_near_a_stop_sign(localization_msg, map):
    """
    Is EV's current position near a stop sign?

    :param localization_msg: a localization message
    :param map: current {@class Map} map
    :return: {@code True} if EV is near a stop sign, {@code False} otherwise
    """
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    return near_stop_sign(ev_pos, map)

def signal_color(signal_msg):
    """
    What color does the EV recognize the signal light is, if the EV detects a signal light?

    :param signal_msg: a signal message
    :return: The color of the signal light if detected, {@code None} otherwise
    """
    if signal_msg['containLights'] == True:
        return signal_msg['trafficLight'][0]['color']
    else:
        return None

def is_approaching_other_traffic_participant(localization_msg, obstacle_msg):
    """
    Is EV approaching other traffic participants?

    :param localization_msg: a localization message
    :param obstacle_msg: obstacle_msg
    :return: {@code True} if EV is approaching other traffic participants, {@code False} otherwise
    """
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    ev_vel_vec = (localization_msg['pose']['linearVelocity']['x'], localization_msg['pose']['linearVelocity']['y'],
                  localization_msg['pose']['linearVelocity']['z'])
    ev_vel = dist((0, 0, 0), ev_vel_vec)
    if 'perceptionObstacle' in obstacle_msg:
        for obs in obstacle_msg['perceptionObstacle']:
            obs_pos = (obs['position']['x'], obs['position']['y'], obs['position']['z'])
            obs_vel_vec = (obs['velocity']['x'], obs['velocity']['y'], obs['velocity']['z'])
            obs_vel = dist((0, 0, 0), obs_vel_vec)
            distance = dist(ev_pos, obs_pos)
            if 0 != ev_vel:
                t = distance / ev_vel
                if t < 3:
                    return True
            if 0 != obs_vel:
                t = distance / obs_vel
                if t < 3:
                    return True
    return False

def is_too_close_to_other_traffic_participant(localization_msg, obstacle_msg):
    """
    Is EV too close to other traffic participants?

    :param localization_msg: a localization message
    :param obstacle_msg: obstacle_msg
    :return: {@code True} if EV is too close to other traffic participants, {@code False} otherwise
    """
    ev_pos = (localization_msg['pose']['position']['x'], localization_msg['pose']['position']['y'],
              localization_msg['pose']['position']['z'])
    current_closest = 999999
    if 'perceptionObstacle' in obstacle_msg:
        for obs in obstacle_msg['perceptionObstacle']:
            obs_pos = (obs['position']['x'], obs['position']['y'], obs['position']['z'])
            distance = dist(ev_pos, obs_pos)
            if distance < current_closest:
                current_closest = distance
    # print(current_closest)
    if current_closest < 5:
        return True
    else:
        return False

def is_rss_safe(planning_msg):
    """
    Is EV rss safe now?

    :param planning_msg: a planning message
    :return: {@code True} if EV is rss safe, {@code False} otherwise
    """
    if 'rssInfo' in planning_msg and 'isRssSafe' in planning_msg['rssInfo']:
        return planning_msg['rssInfo']['isRssSafe']
    return False

def recognized_ODD(planning_msg):
    """
    Which ODD is the recognized by EV?

    :param planning_msg: a planning message
    :return: the recognized ODD (Scenario) type of EV
    """
    if 'debug' in planning_msg and 'scenario' in planning_msg['debug']['planningData'] \
        and 'scenarioType' in planning_msg['debug']['planningData']['scenario']:
        return planning_msg['debug']['planningData']['scenario']['scenarioType']
    return ''

def recognized_ODD_stage(planning_msg):
    """
    Which ODD stage is the recognized by EV?

    :param planning_msg: a planning message
    :return: the recognized ODD (Scenario) stage type of EV
    """
    if 'debug' in planning_msg and 'scenario' in planning_msg['debug']['planningData'] \
        and 'stageType' in planning_msg['debug']['planningData']['scenario']:
        return planning_msg['debug']['planningData']['scenario']['stageType']
    return ''

def current_main_decision(planning_msg):
    """
    Return the current main decision of EV. Does EV decide to stop?

    :param planning_msg: a planning message
    :return: the current main decision of EV
    """
    if 'decision' in planning_msg and 'mainDecision' in planning_msg['decision']:
        if 'stop' in planning_msg['decision']['mainDecision'] and \
                'reasonCode' in planning_msg['decision']['mainDecision']['stop']:
            return planning_msg['decision']['mainDecision']['stop']['reasonCode']
        if 'estop' in planning_msg['decision']['mainDecision'] and \
                'reasonCode' in planning_msg['decision']['mainDecision']['estop']:
            return planning_msg['decision']['mainDecision']['estop']['reasonCode']
    return ""

def caution_type_obstacles(prediction_msg):
    """
    Which obstacle is marked as CAUTION type by EV?

    :param prediction_msg: a prediction message
    :return: a list of obstacles marked as CAUTION type
    """
    cautions = []
    if 'predictionObstacle' in prediction_msg:
        for obs in prediction_msg['predictionObstacle']:
            if 'priority' in obs and 'priority' in obs['priority']:
                if obs['priority']['priority'] == 'CAUTION':
                    cautions.append(obs['perceptionObstacle']['id'])
    return cautions

def ignore_type_obstacles(prediction_msg):
    """
    Which obstacle is marked as IGNORE type by EV?

    :param prediction_msg: a prediction message
    :return: a list of obstacles marked as IGNORE type
    """
    ignores = []
    if 'predictionObstacle' in prediction_msg:
        for obs in prediction_msg['predictionObstacle']:
            if 'priority' in obs and 'priority' in obs['priority']:
                if obs['priority']['priority'] == 'IGNORE':
                    ignores.append(obs['perceptionObstacle']['id'])
    return ignores

def current_object_decision(planning_msg):
    """
    Return the current EV's decision for each object respectively.

    :param planning_msg: a planning message
    :return: the current EV's decision for each object respectively
    """
    object_decisions = {}
    if 'decision' in planning_msg and 'objectDecision' in planning_msg['decision'] and \
            'decision' in planning_msg['decision']['objectDecision']:
        for decision in planning_msg['decision']['objectDecision']['decision']:
            obs_id = decision['id']
            obs_decision = ""
            if 'ignore' in decision['objectDecision'][0]:
                obs_decision = 'ignore'
            elif 'stop' in decision['objectDecision'][0]:
                obs_decision = 'stop'
            elif 'follow' in decision['objectDecision'][0]:
                obs_decision = 'follow'
            elif 'yield' in decision['objectDecision'][0]:
                obs_decision = 'yield'
            elif 'overtake' in decision['objectDecision'][0]:
                obs_decision = 'overtake'
            elif 'nudge' in decision['objectDecision'][0]:
                obs_decision = 'nudge'
            elif 'avoid' in decision['objectDecision'][0]:
                obs_decision = 'avoid'
            elif 'sidePass' in decision['objectDecision'][0]:
                obs_decision = 'sidePass'
            object_decisions[obs_id] = obs_decision
    return object_decisions

"""
DEPRECATED! :(

Denoise vectors, by the next two functions: 
    @func  vectors_denoising(original_vectors, window_size=5)
    @func  major_vectors_extracting(vectors)
"""

def vectors_denoising(original_vectors, window_size=3):
    """
    DEPRECATED!
    Smooth the noises in the list of vectors using a sliding window

    :param original_vectors: a list of vectors with noises
    :param window_size: length of the sliding window
    :return: denoised vectors
    """
    denoised_vectors = []
    if window_size > len(original_vectors):
        for i in range(len(original_vectors)):
            denoised_vectors.append(original_vectors[0])
        return denoised_vectors
    for i in range(len(original_vectors) - window_size):
        current_window = original_vectors[i: i+window_size]
        major_vectors = major_vectors_extracting(current_window)
        denoised_vectors.append(major_vectors)
    for i in range(len(original_vectors) - window_size, len(original_vectors)):
        denoised_vectors.append(denoised_vectors[-1])
    return denoised_vectors

def major_vectors_extracting(vectors):
    """
    DEPRECATED!
    Obtain the major vector of vectors in current sliding window

    :param vectors: vectors in current sliding window
    :return: major vector
    """
    unique_vectors = []
    for vec in vectors:
        flag = False
        for i in range(len(unique_vectors)):
            if unique_vectors[i][0] == vec:
                unique_vectors[i][1] += 1
                flag = True
                break
        if not flag:
            unique_vectors.append([vec, 1])

    max_count = -1
    major_vector = None
    for i in range(len(unique_vectors)):
        if unique_vectors[i][1] > max_count:
            max_count = unique_vectors[i][1]
            major_vector = unique_vectors[i][0]

    return major_vector

def map_vectors_extracting(localization_msgs, signal_msgs, map):
    """
    Extract feature vectors of the environment

    :param localization_msgs: a list of localization messages
    :param signal_msgs: a list of signal messages
    :param map: current map
    :return: a list of feature vectors
    """
    feature_vectors = []
    msg_len = len(localization_msgs)
    for i in range(0, msg_len):
        feature_vector = []
        feature_vector.append(is_near_a_crosswalk(localization_msgs[i], map))
        feature_vector.append(is_near_a_junction(localization_msgs[i], map))
        feature_vector.append(is_near_a_stop_sign(localization_msgs[i], map))
        feature_vector.append(signal_color(signal_msgs[i]))
        feature_vectors.append(feature_vector)
    return feature_vectors

def perception_prediction_vectors_extracting(localization_msgs, obstacle_msgs, prediction_msgs):
    """
    Extract feature vectors of the EV's behavior

    :param localization_msgs: a list of localization messages
    :param obstacle_msgs: a list of obstacle messages
    :param prediction_msgs: a list of prediction messages
    :return: a list of feature vectors
    """
    feature_vectors = []
    msg_len = len(localization_msgs)
    for i in range(0, msg_len):
        feature_vector = []
        feature_vector.append(is_approaching_other_traffic_participant(localization_msgs[i], obstacle_msgs[i]))
        feature_vector.append(is_too_close_to_other_traffic_participant(localization_msgs[i], obstacle_msgs[i]))
        feature_vector.append(caution_type_obstacles(prediction_msgs[i]))
        feature_vector.append(ignore_type_obstacles(prediction_msgs[i]))
        feature_vectors.append(feature_vector)
    return feature_vectors

def planning_feature_extracting(planning_msgs):
    feature_vectors = []
    msg_len = len(planning_msgs)
    for i in range(0, msg_len):
        feature_vector = []
        feature_vector.append(is_rss_safe(planning_msgs[i]))
        feature_vector.append(recognized_ODD(planning_msgs[i]))
        feature_vector.append(recognized_ODD_stage(planning_msgs[i]))
        feature_vector.append(current_main_decision(planning_msgs[i]))
        feature_vector.append(current_object_decision(planning_msgs[i]))
        feature_vectors.append(feature_vector)
    return feature_vectors

def vectors_splitting(vectors):
    """
    Split the vectors to get original clips

    :param vectors: feature vectors
    :return: original clips
    """
    clips = []
    current_segment_start = 0
    current_segment_end = 0
    vec_len = len(vectors)
    for i in range(1, vec_len):
        current_segment_end = i
        if vectors[i] != vectors[i - 1]:
            clips.append((current_segment_start, current_segment_end))
            current_segment_start = i

    clips.append((current_segment_start, current_segment_end))
    return clips

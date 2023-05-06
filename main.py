import argparse
import datetime
from map_parser import Map
from vehicle_parser import Vehicle
from recording_parsing import messages_aligning, record_rewrite
from slicing_and_localizing import recording_vectorizing, first_frame_index_of_accident, recording_splitting
from field_localizing import prechecking, rechecking_v1


def ev_status_decoder(status_code):
    ev_status = ''
    if -1 == status_code:
        ev_status = '    No <Planning> Faults Discovered: ' \
                    'EV keeps safe distance from NPCs, or EV has almost stopped\n'
    elif 1 == status_code:
        ev_status = '    In <Planning> Module and <Prediction> Module: ' \
                    'Wrong identification of drivable area, caused by wrong trajectory prediction\n'
    elif 2 == status_code:
        ev_status = '    In <Planning> Module: ' \
                    'Wrong speed planning, although identifying drivable area correctly\n'
    elif 3 == status_code:
        ev_status = '    In <Planning> Module: ' \
                    'Short-sighted planning, less than 2 seconds\n'
    # elif 4 == status_code:
    #     ev_status = '    In <Control> Module: ' \
    #                 'Collision-free planning, but out of control\n'
    elif 0 == status_code:
        ev_status = '    No <Planning> Faults Discovered: ' \
                    'Collision-free long-term speed planning\n'
    elif 5 == status_code:
        ev_status = '    In <Planning> Module: ' \
                    'EV reuses the previous planning, which sometimes could be improper\n'
    return ev_status

def npc_status_info(start, end, prechecking_ret):
    npc_info = ''
    notable_obs_id = list(prechecking_ret.keys())
    for obs in notable_obs_id:
        count_pp = prechecking_ret[obs]['prediction_priority'][start:end].count(1)
        if count_pp > 2 or ((end - start) <= 3 and count_pp > 0):
            npc_info += '        For NPC "{}": Wrong priority prediction\n'.format(obs)
        count_pt = prechecking_ret[obs]['prediction_trajectory'][start:end].count(1)
        if count_pt > 2 or ((end - start) <= 3 and count_pt > 0):
            npc_info += '        For NPC "{}": Wrong trajectory prediction\n'.format(obs)
        decision_set = set(prechecking_ret[obs]['planning_decision_detailed'][start:end])
        for item in decision_set:
            if '' != item:
                count_pd = prechecking_ret[obs]['planning_decision_detailed'][start:end].count(item)
                if count_pd > 2 or ((end - start) <= 3 and count_pd > 0):
                    npc_info += '        For NPC "{}": Improper "{}" decision\n'.format(obs, item)
                    analysis = decision_analyzer(item)
                    npc_info += '            ' + analysis
        # for item in prechecking_ret[obs]['prediction_priority'][start:end]:
        #     if 1 == item:
        #         npc_info += '        For NPC "{}": Wrong priority prediction\n'.format(obs)
        #         break
        # for item in prechecking_ret[obs]['prediction_trajectory'][start:end]:
        #     if 1 == item:
        #         npc_info += '        For NPC "{}": Wrong trajectory prediction\n'.format(obs)
        #         break
        # for item in prechecking_ret[obs]['planning_decision_detailed'][start:end]:
        #     if '' != item:
        #         npc_info += '        For NPC "{}": Improper "{}" decision\n'.format(obs, item)
        #         analysis = decision_analyzer(item)
        #         npc_info += '            ' + analysis
        #         break
    if 0 != len(notable_obs_id):
        for item in prechecking_ret[notable_obs_id[0]]['planning_speed'][start:end]:
            if 1 == item:
                npc_info += '        Improper speed planning, too fast or too slow\n'
                break
    return npc_info

def decision_analyzer(decision):
    analysis = ''
    if 'ignore' == decision:
        analysis = 'Unsafe, EV is approaching the NPC now\n'
    elif 'follow' == decision:
        analysis = 'Unsafe, EV is approaching the NPC now\n'
    elif 'yield' == decision:
        analysis = 'Unsafe, it may be unexpected to other drivers\n'
    elif 'overtake' == decision:
        analysis = 'Illegal and unsafe, EV is on the junction now\n'
    elif 'none' == decision:
        analysis = 'Unsafe, EV is approaching the NPC now\n'
    return analysis

def is_noteworthy_v0(segment, accident_frame):
    is_critical = '(Safety-critical!)'
    start = segment[0]
    end = segment[1]
    code = segment[-1][0]
    out_of_control = segment[-1][-1]
    first_second = accident_frame - 13 * 3
    second_second = accident_frame - 13 * 2
    third_second = accident_frame - 13 * 1
    # print(first_second, second_second, third_second)
    if has_intersection(start, end, first_second, second_second):
        if 1 == code or 2 == code or 3 == code:
            return is_critical
    if has_intersection(start, end, second_second, third_second):
        if 2 == code or 3 == code or 1 == out_of_control:
            return is_critical
    if has_intersection(start, end, third_second, accident_frame):
        if 1 == out_of_control:
            return is_critical
    return ''

def is_planning_noteworthy(remediation_list):
    ret_list = []
    flag_0 = False
    r_min = 99999
    index_min = -99
    for i in range(len(remediation_list)):
        if 2 > remediation_list[i] and -99998 < remediation_list[i]:
            ret_list.append('      ** Critical <planning> events!**\n')
            flag_0 = True
        else:
            ret_list.append('')
        if r_min > remediation_list[i] and remediation_list[i] > 1:
            r_min = remediation_list[i]
            index_min = i
    if not flag_0 and -99 != index_min:
        ret_list[index_min] = '      ** Critical <planning> events! **\n'
    return ret_list

def has_remediation(segment, accident_frame, acc_time_pred):
    start = segment[0]
    end = segment[1]
    causal_event = segment[-1][0]
    if 0 < causal_event and causal_event < 4:
        return time_comparing(accident_frame, start, end, acc_time_pred)
    return -99999

def is_control_noteworthy(segment, accident_frame):
    end = segment[1]
    out_of_control = segment[-1][-1]
    if 1 == out_of_control:
        if 38 > accident_frame - end:
            return '      ** Critical <control> events! **\n'
    return ''

def is_out_of_control(seg_start, seg_end, ctl_checking_ret):
    for i in range(seg_start, seg_end):
        if 1 == ctl_checking_ret[i]:
            return 1
    return 0


def out_of_control_decoder(status_code):
    ev_status = ''
    if 1 == status_code:
        ev_status = '    In <Control> Module: ' \
                'EV is skidding sometimes\n'
    return ev_status

def has_intersection(start_target, end_target, start_reference, end_reference):
    if 1 == end_target - start_target:
        return False
    length1 = end_target - start_target
    length2 = end_reference - start_reference
    length_ = 0
    if start_target <= start_reference:
        length_ = min(end_reference, end_target) - start_reference
    else:
        length_ = min(end_reference, end_target) - start_target
    if length_ / length1 > 0.4 or length_ / length2 > 0.4:
        return True
    return False

def time_comparing(accident_frame, start, end, acc_time_pred):
    delta_start = 0
    delta_end = 0
    for i in range(start, end):
        if -1 != acc_time_pred[i]:
            delta_start = i + acc_time_pred[i] - accident_frame
            break
    for i in range(end-1, -1, -1):
        if -1 != acc_time_pred[i]:
            delta_end = i + acc_time_pred[i] - accident_frame
            break
    # print(delta_end - delta_start)
    return (delta_end - delta_start)

def storyteller(prechecking_ret, rechecking_ret, report_dir, acc_time_pred):
    scenario = []
    seg_start = 0
    seg_end = -1
    accident_frame = len(rechecking_ret[0])
    for i in range(1, accident_frame):
        prev = rechecking_ret[0][i-1]
        curr = rechecking_ret[0][i]
        if prev != curr:
            seg_end = i
            is_oc = is_out_of_control(seg_start, seg_end, rechecking_ret[1])
            scenario.append((seg_start, seg_end, [prev, is_oc]))
            seg_start = i
    is_oc = is_out_of_control(seg_start, len(rechecking_ret[0]), rechecking_ret[1])
    scenario.append((seg_start, len(rechecking_ret[0]), [rechecking_ret[0][-1], is_oc]))
    # print(scenario)
    print(' Please check "{}" for the report file. :)'.format(report_dir))
    print('==============================================================================')
    # print('The final report is also as following: \n')
    is_control_critical_list = []
    ev_status_list = []
    npc_info_list = []
    out_of_control_list = []
    frame_range_list = []
    remediation_list = []
    for item in scenario:
        is_control_critical = is_control_noteworthy(item, accident_frame)
        is_control_critical_list.append(is_control_critical)
        ev_status = ev_status_decoder(item[-1][0])
        ev_status_list.append(ev_status)
        npc_info = npc_status_info(item[0], item[1], prechecking_ret)
        npc_info_list.append(npc_info)
        out_of_control = out_of_control_decoder(item[-1][-1])
        out_of_control_list.append(out_of_control)
        frame_range = 'Frame #{}-{}: \n'.format(item[0], (item[1] - 1))
        frame_range_list.append(frame_range)
        remediation_list.append(has_remediation(item, accident_frame, acc_time_pred))
    is_planning_critical_list = is_planning_noteworthy(remediation_list)
    with open(report_dir, 'w') as report_file:
        for i in range(len(scenario)):
            is_critical = is_planning_critical_list[i] + is_control_critical_list[i]
            frame_range_list[i] += is_critical
            description = frame_range_list[i] + ev_status_list[i] + npc_info_list[i] + out_of_control_list[i]
            # print(description)
            report_file.write(description + '\n')
        accident_info = 'Frame #{}:\n    Accident!\n'.format(accident_frame)
        # print(accident_info)
        report_file.write(accident_info)
    print()

def main(original_record_dir, map_params_dir, vehicle_params_dir, reduced_segment_dir, report_dir):
    map = Map(map_params_dir)
    map.parse()
    ev = Vehicle(vehicle_params_dir)
    print('==============================================================================')
    print(' Accident driving recording "{}"'.format(original_record_dir))

    localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs = \
        messages_aligning(original_record_dir)
    m_vectors, pp_vectors, pl_vectors = recording_vectorizing(localization_msgs, obstacle_msgs, signal_msgs,
                                                              prediction_msgs, planning_msgs, map)
    fi = first_frame_index_of_accident(localization_msgs, obstacle_msgs, ev)
    if -1 == fi:
        print(' Did not detect any accidents.')
        print('==============================================================================')
        return
    clips = recording_splitting(m_vectors, pp_vectors, pl_vectors, fi)
    # print(clips, fi)
    if clips[0][0] < fi and fi <= clips[0][-1]:
        start = clips[0][0]
        end = fi
        loc_msgs = localization_msgs[start:end]
        obs_msgs = obstacle_msgs[start:end]
        pre_msgs = prediction_msgs[start:end]
        pla_msgs = planning_msgs[start:end]
        sig_msgs = signal_msgs[start:end]
        # record_rewrite(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs,
        #                reduced_segment_dir, start, end)
        ret = prechecking(loc_msgs, obs_msgs, pre_msgs, pla_msgs, sig_msgs, map)
        rechecking_ret, cf_counter, acc_time_pred = rechecking_v1(ret, loc_msgs, obs_msgs, pla_msgs, ev)
        storyteller(ret, rechecking_ret, report_dir, acc_time_pred)
    else:
        print(' {} {} {}'.format(clips[0][0], fi, clips[0][-1]))
        print('==============================================================================')
        return

if __name__ == '__main__':
    now = datetime.datetime.now()
    time_string = str(now)
    parser = argparse.ArgumentParser(description='ACAV is a framework for Automatic Causality analysis of AV accidents. ')
    parser.add_argument('--map', '-m', default='san_francisco.bin', type=str,
                        help='the directory of the map file')
    parser.add_argument('--vehicle', '-v', default='Lincoln2017 MKZ-vehicle_param.pb.txt', type=str,
                        help='the directory of the parameter file of EV')
    parser.add_argument('--input', '-i', default='record/T-2.record', type=str, required=True,
                        help='cyber recording file')
    # parser.add_argument('--output', '-o', default='{}.record'.format(time_string), type=str,
    #                     help='reduced cyber recording file')
    # parser.add_argument('--report', '-r', default='{}.txt'.format(time_string), type=str,
    #                     help='the report file of causality analysis')
    args = parser.parse_args()
    map_params_dir = 'maps/{}'.format(args.map)
    vehicle_params_dir = 'vehicles/{}'.format(args.vehicle)
    original_record_dir = args.input
    string_segs = original_record_dir.split('/')
    file_name = string_segs[-1][:-7]
    reduced_segment_dir = 'The_Results/{}_reduced.record'.format(file_name)
    report_dir = 'The_Results/{}_report.txt'.format(file_name)
    # print(original_record_dir, map_params_dir, vehicle_params_dir, reduced_segment_dir, report_dir)
    main(original_record_dir, map_params_dir, vehicle_params_dir, reduced_segment_dir, report_dir)
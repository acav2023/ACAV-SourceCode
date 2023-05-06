from cyber_record.record import Record
from google.protobuf import json_format
import json
import os
import time
import datetime
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.routing.proto.routing_pb2 import RoutingRequest, RoutingResponse

def pb2json(proto_msg):
    """
    Parse a protobuf message into json format

    :param message: protobuf message
    :return: a json format string
    """
    str = json_format.MessageToJson(proto_msg)
    # print(type(str))  type:str
    json_msg = json.loads(str)
    # json_msg['header']['timestampSec'] = time.time()
    # print(type(json_msg))   type:dict
    return json_msg

def messages_aligning(record_dir):
    """
    1. Extract messages from multiple channels. The protobuf messages in a single channel are converted into json string
    frames, and added to the list of corresponding channel.

    2. Recording message alignment. choose channel {@/apollo/localization/pose} as the reference channel to align
    messages of other channels.

    Only five channels are selected:
        @ /apollo/localization/pose
        @ /apollo/perception/obstacles
        @ /apollo/perception/traffic_light
        @ /apollo/prediction
        @ /apollo/planning

    :param record_dir: the file directory of the recording file
    :return: aligned messages, seven lists
    """
    localization_msgs = []
    obstacle_msgs = []
    signal_msgs = []
    prediction_msgs = []
    planning_msgs = []

    localization_msg_count = 0
    obstacle_msg_count = 0
    signal_msg_count = 0
    prediction_msg_count = 0
    planning_msg_count = 0

    record = Record(record_dir)

    for channel, message, _ in record.read_messages():
        obstacle_msg_latest = ''
        signal_msg_latest = ''
        prediction_msg_latest = ''
        planning_msg_latest = ''
        routing_request_msg_latest = ''
        routing_response_msg_latest = ''

        routing_request_flag = False
        routing_response_flag = False

        if channel == "/apollo/localization/pose":
            frame = pb2json(message)
            localization_msgs.append(frame)
            localization_msg_count += 1

        elif channel == "/apollo/perception/obstacles":
            obstacle_msg_latest = pb2json(message)
            if localization_msg_count > obstacle_msg_count:
                for i in range(obstacle_msg_count, localization_msg_count):
                    obstacle_msgs.append(obstacle_msg_latest)
                    obstacle_msg_count += 1

        elif channel == "/apollo/perception/traffic_light":
            signal_msg_latest = pb2json(message)
            if localization_msg_count > signal_msg_count:
                for i in range(signal_msg_count, localization_msg_count):
                    signal_msgs.append(signal_msg_latest)
                    signal_msg_count += 1

        elif channel == "/apollo/prediction":
            prediction_msg_latest = pb2json(message)
            if localization_msg_count > prediction_msg_count:
                for i in range(prediction_msg_count, localization_msg_count):
                    prediction_msgs.append(prediction_msg_latest)
                    prediction_msg_count += 1

        elif channel == "/apollo/planning":
            planning_msg_latest = pb2json(message)
            if localization_msg_count > planning_msg_count:
                for i in range(planning_msg_count, localization_msg_count):
                    planning_msgs.append(planning_msg_latest)
                    planning_msg_count += 1

        # elif channel == '/apollo/routing_request':
        #     print('    routing_request')
        #     routing_request_msg_latest = pb2json(message)
        #     print(routing_request_msg_latest)
        #     # print(routing_request_msg_latest)
        #     if localization_msg_count > routing_request_msg_count:
        #         for i in range(routing_request_msg_count, localization_msg_count):
        #             routing_request_msgs.append(routing_request_msg_latest)
        #             routing_request_msg_count += 1
        #
        # elif channel == '/apollo/routing_response':
        #     print('    routing_response')
        #     routing_response_msg_latest = pb2json(message)
        #     print(routing_response_msg_latest)
        #     # print(routing_response_msg_latest)
        #     if localization_msg_count > routing_response_msg_count:
        #         for i in range(routing_response_msg_count, localization_msg_count):
        #             routing_response_msgs.append(routing_response_msg_latest)
        #             routing_response_msg_count += 1

        # elif channel == '/apollo/routing_request':
        #     print(localization_msg_count)
        #     if routing_request_flag == False:
        #         for i in range(localization_msg_count):
        #             routing_request_msgs.append(routing_request_msg_latest)
        #         routing_request_msg_latest = pb2json(message)
        #         routing_request_msgs[-1] = routing_request_msg_latest
        #         routing_request_msg_count = localization_msg_count
        #         routing_request_flag = True
        #     else:
        #         routing_request_msg_latest = pb2json(message)
        #         if localization_msg_count > routing_request_msg_count:
        #             for i in range(routing_request_msg_count, localization_msg_count):
        #                 routing_request_msgs.append(routing_request_msg_latest)
        #                 routing_request_msg_count += 1
        #
        # elif channel == '/apollo/routing_response':
        #     if routing_response_flag == False:
        #         for i in range(localization_msg_count):
        #             routing_response_msgs.append(routing_response_msg_latest)
        #         routing_response_msg_latest = pb2json(message)
        #         routing_response_msgs[-1] = routing_response_msg_latest
        #         routing_response_msg_count = localization_msg_count
        #         routing_response_flag = True
        #     else:
        #         routing_response_msg_latest = pb2json(message)
        #         if localization_msg_count > routing_response_msg_count:
        #             for i in range(routing_response_msg_count, localization_msg_count):
        #                 routing_response_msgs.append(routing_response_msg_latest)
        #                 routing_response_msg_count += 1

    msg_count = min(localization_msg_count, obstacle_msg_count, signal_msg_count, prediction_msg_count,
                    planning_msg_count)
    localization_msgs = localization_msgs[:msg_count]
    obstacle_msgs = obstacle_msgs[:msg_count]
    signal_msgs = signal_msgs[:msg_count]
    prediction_msgs = prediction_msgs[:msg_count]
    planning_msgs = planning_msgs[:msg_count]


    return localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs

def test_write_record(msg_dict):
    msg_dict['containLights'] = True
    # print(type(msg_dict))   type:dict
    return msg_dict

def json2pb_localization(json_str):
    """
    Parse a dictionary format message into protobuf format, for channel{@/apollo/localization/pose}

    :param json_str: dictionary format message
    :return: a protobuf message
    """
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, LocalizationEstimate())
    return pb_msg

def json2pb_obstacles(json_str):
    """
    Parse a dictionary format message into protobuf format, for channel{@/apollo/perception/obstacles}

    :param json_str: dictionary format message
    :return: a protobuf message
    """
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, PerceptionObstacles())
    return pb_msg

def json2pb_signal(json_str):
    """
    Parse a dictionary format message into protobuf format, for channel{@/apollo/perception/traffic_light}

    :param json_str: dictionary format message
    :return: a protobuf message
    """
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, TrafficLightDetection())
    return pb_msg

def json2pb_prediction(json_str):
    """
    Parse a dictionary format message into protobuf format, for channel{@/apollo/prediction}

    :param json_str: dictionary format message
    :return: a protobuf message
    """
    if 'predictionObstacle' in json_str:
        for po in json_str['predictionObstacle']:
            if 'interactiveTag' in po:
                del po['interactiveTag']
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, PredictionObstacles())
    return pb_msg

def json2pb_planning(json_str):
    """
    Parse a dictionary format message into protobuf format, for channel{@/apollo/planning}

    :param json_str: dictionary format message
    :return: a protobuf message
    """
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, ADCTrajectory())
    return pb_msg

def json2pb_routing_request(json_str):
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, RoutingRequest())
    return pb_msg

def json2pb_routing_response(json_str):
    json_msg = json.dumps(json_str)
    pb_msg = json_format.Parse(json_msg, RoutingResponse())
    return pb_msg

def record_rewrite(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs,
                   synthetic_record_dir, start, end):
    """
    Build record file with customized segment of aligned messages

    :param localization_msgs: aligned localization messages
    :param obstacle_msgs: aligned obstacle messages
    :param signal_msgs: aligned traffic light messages
    :param prediction_msgs: aligned prediction messages
    :param planning_msgs: aligned planning messages
    :param synthetic_record_dir: the directory of synthetic recording file
    :param start: start frame index
    :param end: end frame index
    :return: None
    """
    # msg_len = len(localization_msgs)
    with Record(synthetic_record_dir, mode='w') as record:
        # for i in range(0, msg_len):
        for i in range(start, end):
            ts = int(time.time() * 1e9)
            localization_msg_pb = json2pb_localization(localization_msgs[i])
            record.write('/apollo/localization/pose', localization_msg_pb, ts)
            obstacle_msg_pb = json2pb_obstacles(obstacle_msgs[i])
            record.write('/apollo/perception/obstacles', obstacle_msg_pb, ts)
            signal_msg_pb = json2pb_signal(signal_msgs[i])
            record.write('/apollo/perception/traffic_light', signal_msg_pb, ts)
            prediction_msg_pb = json2pb_prediction(prediction_msgs[i])
            record.write('/apollo/prediction', prediction_msg_pb, ts)
            planning_msg_pb = json2pb_planning(planning_msgs[i])
            record.write('/apollo/planning', planning_msg_pb, ts)
            time.sleep(0.08)

# def recording_rewrite(record_dir, directory, clips):
#     """
#     DEPRECATED! :(
#     """
#     clips_ts = []
#     print(clips)
#     for c in clips:
#         start = c[0]
#         end = c[-1]
#         start_ts = 0
#         end_ts = 0
#         counter = 0
#         with Record(record_dir) as reader:
#             for channel, message, _ in reader.read_messages():
#                 if channel == "/apollo/localization/pose":
#                     if counter == start:
#                         pose_msg = pb2json(message)
#                         time = datetime.datetime.fromtimestamp(pose_msg['header']['timestampSec'])
#                         start_ts = time.strftime('%Y-%m-%d %H:%M:%S.%f')
#                     if counter == end:
#                         pose_msg = pb2json(message)
#                         time = datetime.datetime.fromtimestamp(pose_msg['header']['timestampSec'])
#                         end_ts = time.strftime('%Y-%m-%d %H:%M:%S.%f')
#                     counter += 1
#         clips_ts.append((start_ts, end_ts))
#         # print(clips_ts[-1])
#
#     for i in range(len(clips_ts)):
#         cmd = """
# DOCKER_ID=e1fbc6e6d6a9
# docker exec -d $DOCKER_ID /bin/bash -c 'sh /apollo/recording/FaultLocalizer/segment_splitting.sh {} "{}" "{}"'
# """.format(i, clips_ts[i][0], clips_ts[i][-1])
#         # cmd = "sh recording_splitting.sh {} \"{}\" \"{}\"".format(i, clips_ts[i][0], clips_ts[i][-1])
#         print(cmd)
#         os.system(cmd)
#
#
#     # routing_request_msg = ''
#     # routing_response_msg = ''
#
#     # with Record(record_dir) as reader:
#     #     for channel, message, _ in reader.read_messages():
#     #         if channel == '/apollo/routing_request':
#     #             routing_request_msg = message
#     #         if channel == '/apollo/routing_response':
#     #             routing_response_msg = message
#     #             break
#
#         # for i in range(len(clips)):
#         #     start = clips[i][0]
#         #     end = clips[i][-1]
#         #     counter = 0
#         #     synthetic_record_dir = directory + 'segment-test-ordered-{}.record'.format(i)
#         #     with Record(synthetic_record_dir, mode='w') as writer:
#         #         writer.write('/apollo/routing_request', routing_request_msg)
#         #         time.sleep(0.0005)
#         #         writer.write('/apollo/routing_response', routing_response_msg)
#         #         for channel, message, _ in reader.read_messages():
#         #             if channel == "/apollo/localization/pose":
#         #                 if counter >= start and counter < end:
#         #                     writer.write('/apollo/localization/pose', message)
#         #                     time.sleep(0.01)
#         #                 counter += 1
#         #             elif channel == "/apollo/perception/obstacles":
#         #                 if counter >= start and counter < end:
#         #                     writer.write('/apollo/perception/obstacles', message)
#         #                     time.sleep(0.01)
#         #             elif channel == "/apollo/perception/traffic_light":
#         #                 if counter >= start and counter < end:
#         #                     writer.write('/apollo/perception/traffic_light', message)
#         #                     time.sleep(0.01)
#         #             elif channel == "/apollo/prediction":
#         #                 if counter >= start and counter < end:
#         #                     writer.write('/apollo/prediction', message)
#         #                     time.sleep(0.01)
#         #             elif channel == "/apollo/planning":
#         #                 if counter >= start and counter < end:
#         #                     writer.write('/apollo/planning', message)
#         #                     time.sleep(0.01)


def textfile_rewrite(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs,
                     localization_msg_dir, obstacle_msg_dir, signal_msg_dir, prediction_msg_dir, planning_msg_dir):
    """
    Save aligned messages from different channels to the
    corresponding channel text files

    :param localization_msgs: aligned localization messages
    :param obstacle_msgs: aligned obstacle messages
    :param signal_msgs: aligned traffic light messages
    :param prediction_msgs: aligned prediction messages
    :param planning_msgs: aligned planning messages
    :param localization_msg_dir: the directory of text file containing messages of localization
    :param obstacle_msg_dir: the directory of text file containing messages of perception obstacle
    :param signal_msg_dir: the directory of text file containing messages of traffic light
    :param prediction_msg_dir: the directory of text file containing messages of prediction
    :param planning_msg_dir: the directory of text file containing messages of planning
    :return: None
    """
    msg_len = len(localization_msgs)

    # for i in range(0, msg_len):
    #     if 'perceptionObstacle' in obstacle_msgs[i]:
    #         print("perception_msg:{}   obstacles: {}".format(i, len(obstacle_msgs[i]["perceptionObstacle"])))
    #
    # print()
    #
    # for i in range(0, msg_len):
    #     if 'predictionObstacle' in prediction_msgs[i]:
    #         print("prediction_msg:{}   obstacles: {}".format(i, len(prediction_msgs[i]["predictionObstacle"])))
    #
    # print()

    with open(localization_msg_dir, "a") as f:
        for i in range(0, msg_len):
            f.write(json.dumps(localization_msgs[i]))
            f.write('\n')

    with open(obstacle_msg_dir, "a") as f:
        for i in range(0, msg_len):
            f.write(json.dumps(obstacle_msgs[i]))
            f.write('\n')

    with open(signal_msg_dir, "a") as f:
        for i in range(0, msg_len):
            f.write(json.dumps(signal_msgs[i]))
            f.write('\n')

    with open(prediction_msg_dir, "a") as f:
        for i in range(0, msg_len):
            f.write(json.dumps(prediction_msgs[i]))
            f.write('\n')

    with open(planning_msg_dir, "a") as f:
        for i in range(0, msg_len):
            f.write(json.dumps(planning_msgs[i]))
            f.write('\n')

if __name__ == '__main__':
    directory = './The_Results/'
    # original_record_dir = 'record/A-3.record'
    original_record_dir = 'The_Results/test_main.record'
    localization_msg_dir = directory + 'localization_accident_A3.txt'
    obstacle_msg_dir = directory + 'obstacle_accident_A3.txt'
    signal_msg_dir = directory + 'signal_accident_A3.txt'
    prediction_msg_dir = directory + 'prediction_accident_A3.txt'
    planning_msg_dir = directory + 'planning_accident_A3.txt'
    synthetic_record_dir = directory + 'example-A3.record'
    localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs = \
        messages_aligning(original_record_dir)
    # textfile_rewrite(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs,
    #                  localization_msg_dir, obstacle_msg_dir, signal_msg_dir, prediction_msg_dir, planning_msg_dir)
    # start = 0
    # end = len(localization_msgs)
    for msg in planning_msgs:
        if 'decision' in msg:
            # print(msg['decision']['mainDecision'])
            print(msg['decision']['objectDecision'])
            print()
    # record_rewrite(localization_msgs, obstacle_msgs, signal_msgs, prediction_msgs, planning_msgs, synthetic_record_dir,
    #                start, end)

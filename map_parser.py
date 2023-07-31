from modules.map.proto import map_pb2
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt
import proto_utils


def is_on_lane(p_point, map):
    if len(p_point) == 2:
        p_point = Point(p_point)
    else:
        p_point = Point(p_point[0], p_point[1])

    for id, (lane_polygon, lane_type) in map.lanes.items():
        if lane_polygon.contains(p_point):
            return (id, lane_type)

    return None


def is_in_junction(p_point, map):
    p_point = Point(p_point)
    for id, lane_polygon in map.junctions.items():
        if lane_polygon.contains(p_point):
            return id

    return None


def is_on_crosswalk(p_point, map):
    p_point = Point(p_point)

    for id, (_, lane_polygon) in map.crosswalk.items():
        if lane_polygon.contains(p_point):
            return id

    return None


def near_junction(p_point, map):
    p_point = Point(p_point)
    closet_dis = 999999

    for id, lane_polygon in map.junctions.items():
        dis = lane_polygon.boundary.distance(p_point)
        if dis < closet_dis:
            closet_dis = dis

    if closet_dis <= 5:
        return True
    else:
        return False


def near_crosswalk(p_point, map):
    p_point = Point(p_point)
    closet_dis = 999999

    for id, lane_polygon in map.crosswalk.items():
        dis = lane_polygon[1].boundary.distance(p_point)
        if dis < closet_dis:
            closet_dis = dis

    if closet_dis <= 5:
        return True
    else:
        return False


def near_stop_sign(p_point, map):
    p_point = Point(p_point)
    closet_dis = 999999

    for id, lane_polygon in map.stop_signs.items():
        dis = lane_polygon[1].boundary.distance(p_point)
        if dis < closet_dis:
            closet_dis = dis

    if closet_dis <= 5:
        return True
    else:
        return False


class Map():
    def __init__(self, map_file):
        self.map = self.load_map(map_file)

    def load_map(self, map_file):
        map_pb = map_pb2.Map()
        result = proto_utils.get_pb_from_file(map_file, map_pb)
        return result

    def parse(self):
        self.lanes = {}
        self.junctions = {}
        self.traffic_signs = {}
        self.crosswalk = {}
        self.stop_signs = {}
        self.merge = {}
        self.signals = {}

        # parsing lanes
        for lane in self.map.lane:
            lane_id = str(lane.id.id)
            lane_type = str(lane.turn)
            lane_points = []
            for curve in lane.left_boundary.curve.segment:
                if curve.HasField('line_segment'):
                    for p in curve.line_segment.point:
                        lane_points.append((float(p.x), float(p.y)))

            for curve in lane.right_boundary.curve.segment:
                if curve.HasField('line_segment'):
                    for p in curve.line_segment.point[::-1]:
                        lane_points.append((float(p.x), float(p.y)))
            self.lanes[lane_id] = [Polygon(lane_points), lane_type]

        # parsing junctions
        for junction in self.map.junction:
            junction_id = str(junction.id.id)
            junction_points = []
            for p in junction.polygon.point:
                junction_points.append((float(p.x), float(p.y)))
            self.junctions[junction_id] = Polygon(junction_points)

        # parsing crosswalk
        for crosswalk in self.map.crosswalk:
            crosswalk_id = str(crosswalk.id.id)
            overlap_lanes = []
            crosswalk_points = []
            for p in crosswalk.polygon.point:
                crosswalk_points.append((float(p.x), float(p.y)))
            if len(crosswalk_points) >= 3:
                crosswalk_pos = Polygon(crosswalk_points)
            for overlap in crosswalk.overlap_id:
                if 'lane' in overlap.id:
                    overlap_lanes.append(str('lane_' + overlap.id.split('_')[-1]))
            self.crosswalk[crosswalk_id] = [overlap_lanes, crosswalk_pos]
        # parsing traffic light
        # the same x of three subsignals: the signal is vertical; the same y: horizontal
        # check whether an object is close to a signal. If an object is approaching a vertical signal,
        # check the distance by x; if approaching a horizontal signal, check the distance by y.

        for signal in self.map.signal:
            signal_id = str(signal.id.id)
            signal_stop_line_points = []
            for p in signal.stop_line[0].segment[0].line_segment.point:
                signal_stop_line_points.append((float(p.x), float(p.y)))
            if len(signal_stop_line_points) >= 3:
                signal_pos = Polygon(signal_stop_line_points)
                self.traffic_signs[signal_id] = signal_pos

            # enum
            # Type
            # {
            #     UNKNOWN = 1;
            # MIX_2_HORIZONTAL = 2;
            # MIX_2_VERTICAL = 3;
            # MIX_3_HORIZONTAL = 4;
            # MIX_3_VERTICAL = 5;
            # SINGLE = 6;
            # };
            subsignals = signal.subsignal
            if len(subsignals) == 1:
                signal_type = 6
            elif len(subsignals) == 2:
                signal_z = [subsignal.location.z for subsignal in subsignals]
                if max(signal_z) - min(signal_z) < 0.5:
                    signal_type = 2
                else:
                    signal_type = 3
            elif len(subsignals) == 3:
                signal_z = [subsignal.location.z for subsignal in subsignals]
                if max(signal_z) - min(signal_z) < 0.5:
                    signal_type = 4
                else:
                    signal_type = 5
            else:
                signal_type = 1

            in_junction = False
            for overlap in signal.overlap_id:
                if 'junction' in overlap.id:
                    in_junction = True
                    break
            # subsignal_type
            # enum
            # Type
            # {
            #     UNKNOWN = 1;
            # CIRCLE = 2;
            # ARROW_LEFT = 3;
            # ARROW_FORWARD = 4;
            # ARROW_RIGHT = 5;
            # ARROW_LEFT_AND_FORWARD = 6;
            # ARROW_RIGHT_AND_FORWARD = 7;
            # ARROW_U_TURN = 8;
            # };
            subsignal = signal.subsignal[0]
            if subsignal.type == 1:
                subsignal_type = 1
            elif subsignal.type == 2:
                subsignal_type = 2
            else:
                subsignal_type = 3

            self.signals[signal_id] = {"pos": signal_pos, "type": signal_type,
                                       "in_junction": in_junction, "subsignal_type": subsignal_type}

        # parsing stop sign {"id": [overlap lanes]}
        # check and record the overlap lanes with the stop sign
        for stop_sign in self.map.stop_sign:
            stop_sign_id = str(stop_sign.id.id)
            overlap_lanes = []
            stop_line_points = []

            for p in stop_sign.stop_line[0].segment[0].line_segment.point:
                stop_line_points.append((float(p.x), float(p.y)))
            if len(stop_line_points) >= 3:
                stop_sign_pos = Polygon(stop_line_points)
            for overlap in stop_sign.overlap_id:
                if 'lane' in overlap.id:
                    overlap_lanes.append(str('lane_' + overlap.id.split('_')[-1]))
            self.stop_signs[stop_sign_id] = [overlap_lanes, stop_sign_pos]

        # parsing merge lanes: two lanes have the same successor_lane {"id": [merging lanes]}
        for lane in self.map.lane:
            lane_id = str(lane.id.id)
            successors = []
            for successor_lane in lane.successor_id:
                successors.append(str(successor_lane.id))
            self.merge[lane_id] = successors


if __name__ == '__main__':
    map = Map('maps/san_francisco.bin')
    map.parse()
    x, y = map.lanes['lane_623'][0].exterior.xy
    plt.plot(x, y)
    plt.show()
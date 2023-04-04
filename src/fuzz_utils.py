import os
import json
import time
import shutil
import math

import numpy as np

import config
from driving_quality import *

config.set_carla_api_path()
try:
    import carla
except ModuleNotFoundError as e:
    print("Carla module not found. Make sure you have built Carla.")
    proj_root = config.get_proj_root()
    print("Try `cd {}/carla && make PythonAPI' if not.".format(proj_root))
    exit(-1)

import executor
import constants as c


# def dry_run(conf, client, tm, town, sp, wp, weather):
    # """
    # Dry-runs the base scenario to infer the oracle state
    # params: None
    # return: packed object of feedbacks (tbd)
    # """
    # return # debug

    # dry_run_states = []

    # for i in range(conf.num_dry_runs):
        # print("performing {}-th dry run".format(i+1))
        # executor.simulate(client, town, tm, sp, wp, weather, [], [])
        # state = {
            # "num_frames": states.NUM_FRAMES,
            # "elapsed_time": states.ELAPSED_TIME,
            # "crash": states.COLLISION_EVENT,
            # "lane_invasions": states.LANEINVASION_EVENT,
            # "isstuck": states.STUCK,
            # # "avg_iae_lon": sum(states.IAE_LON) / len(states.IAE_LON),
            # # "avg_iae_lat": sum(states.IAE_LAT) / len(states.IAE_LAT)
        # }
        # dry_run_states.append(state)

    # # get oracle states out of dry_run_states, and return
    # # now just consider raw states as an oracle
    # return dry_run_states


def get_carla_transform(loc_rot_tuples):
    """
    Convert loc_rot_tuples = ((x, y, z), (roll, pitch, yaw)) to
    carla.Transform object
    """

    if loc_rot_tuples is None:
        return None

    loc = loc_rot_tuples[0]
    rot = loc_rot_tuples[1]

    t = carla.Transform(
        carla.Location(loc[0], loc[1], loc[2]),
        carla.Rotation(roll=rot[0], pitch=rot[1], yaw=rot[2])
    )

    return t


def get_valid_xy_range(town):
    try:
        with open(os.path.join("town_info", town + ".json")) as fp:
            town_data = json.load(fp)
    except:
        return (-999, 999, -999, 999)

    x_list = []
    y_list = []
    for coord in town_data:
        x_list.append(town_data[coord][0])
        y_list.append(town_data[coord][1])

    return (min(x_list), max(x_list), min(y_list), max(y_list))


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    # Copied from
    # https://github.com/davheld/tf/blob/master/src/tf/transformations.py#L1100

    _AXES2TUPLE = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

    _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

    _NEXT_AXIS = [1, 2, 0, 1]

    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = np.empty((4, ), dtype=np.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion


class TestScenario:
    seed_data = {}
    town = None
    # sp = None
    # wp = None
    weather = {}
    actors = []
    puddles = []
    # oracle_state = None
    # client = None
    # tm = None
    # list_spawn_points = None
    # quota = 0
    # last_deduction = 0
    driving_quality_score = None
    found_error = False

    def __init__(self, conf, base="seed00.json"):
        """
        When initializing, perform dry run and get the oracle state
        """
        self.conf = conf

        # First, connect to the client once.
        # This is important because the same TM instance has to be reused
        # over multiple simulations for autopilot to be enabled.
        (client, tm) = executor.connect(self.conf)

        self.weather["cloud"] = 0
        self.weather["rain"] = 0
        self.weather["puddle"] = 0
        self.weather["wind"] = 0
        self.weather["fog"] = 0
        self.weather["wetness"] = 0
        self.weather["angle"] = 0
        self.weather["altitude"] = 90

        self.actors = []
        self.puddles = []
        self.driving_quality_score = 0
        self.found_error = False

        # seedfile = os.path.join("seed", base)
        seedfile = os.path.join(conf.seed_dir, conf.cur_scenario)
        with open(seedfile, "r") as fp:
            seed = json.load(fp)
            self.seed_data = seed

        # self.sp = carla.Transform(
            # carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"]+2),
            # carla.Rotation(seed["roll"], seed["yaw"], seed["pitch"])
        # )

        # self.wp = carla.Transform(
            # carla.Location(seed["wp_x"], seed["wp_y"], seed["wp_z"]),
            # carla.Rotation(0.0, seed["wp_yaw"], 0.0)
        # )

        self.town = seed["map"]
        executor.switch_map(conf, self.town)

        # self.oracle_state = dry_run(self.conf, self.client, self.tm,
                # self.town, self.sp, self.wp, self.weather)
        # print("oracle:", self.oracle_state)

        # executor.get_waypoints()

        # self.quota = self.conf.initial_quota

        print("[+] test case initialized")

    def get_seed_sp_transform(self, seed):
        sp = carla.Transform(
            carla.Location(seed["sp_x"], seed["sp_y"], seed["sp_z"]+2),
            carla.Rotation(seed["roll"], seed["yaw"], seed["pitch"])
        )

        return sp

    def get_seed_wp_transform(self, seed):
        wp = carla.Transform(
            carla.Location(seed["wp_x"], seed["wp_y"], seed["wp_z"]),
            carla.Rotation(0.0, seed["wp_yaw"], 0.0)
        )

        return wp


    def get_distance_from_player(self, location):
        sp = self.get_seed_sp_transform(self.seed_data)
        return location.distance(sp.location)


    def add_actor(self, actor_type, nav_type, location, rotation, speed,
            sp_idx, dp_idx):
        """
        Mutator calls `ret = add_actor` with the mutated parameters
        until ret == 0.

        actor_type: int
        location: (float x, float y, float z)
        rotation: (float yaw, float roll, float pitch)
        speed: float

        1) check if the location is within the map
        2) check if the location is not preoccupied by other actors
        3) check if the actor's path collides with player's path
        return 0 iif 1), 2), 3) are satisfied.
        """

        maneuvers = None

        # do validity checks
        if nav_type == c.LINEAR or nav_type == c.IMMOBILE:
            spawn_point = (location, rotation) # carla.Transform(location, rotation)
            dest_point = None

        elif nav_type == c.MANEUVER:
            spawn_point = (location, rotation) # carla.Transform(location, rotation)
            dest_point = None

            # [direction (-1: L, 0: Fwd, 1: R),
            #  velocity (m/s) if fwd / apex degree if lane change,
            #  frame_maneuver_performed]
            maneuvers = [
                [0, 0, 0],
                [0, 8, 0],
                [0, 8, 0],
                [0, 8, 0],
                [0, 8, 0],
            ]

        elif nav_type == c.AUTOPILOT:
            assert(executor.list_spawn_points)
            if sp_idx == dp_idx:
                return -1
            sp = executor.list_spawn_points[sp_idx]

            # prevent autopilot vehicles from being
            # spawned beneath the player vehicle
            sp.location.z = 1.5

            spawn_point = (
                (sp.location.x, sp.location.y, sp.location.z),
                (sp.rotation.roll, sp.rotation.pitch, sp.rotation.yaw)
            )

            dp = executor.list_spawn_points[dp_idx]

            dest_point = (
                (dp.location.x, dp.location.y, dp.location.z),
                (dp.rotation.roll, dp.rotation.pitch, dp.rotation.yaw)
            )

        dist = self.get_distance_from_player(
            get_carla_transform(spawn_point).location
        )

        if (dist > c.MAX_DIST_FROM_PLAYER):
            # print("[-] too far from player: {}m".format(int(dist)))
            return -1

        elif (dist < c.MIN_DIST_FROM_PLAYER) and nav_type != c.MANEUVER:
            # print("[-] too close to the player: {}m".format(int(dist)))
            return -1

        new_actor = {
                "type": actor_type,
                "nav_type": nav_type,
                "spawn_point": spawn_point,
                "dest_point": dest_point,
                "speed": speed,
                "maneuvers": maneuvers
            }
        self.actors.append(new_actor)

        return 0


    def add_puddle(self, level, location, size):
        """
        Mutator calls `ret = add_friction` with the mutated parameters
        until ret == 0.

        level: float [0.0:1.0]
        location: (float x, float y, float z)
        size: (float xlen, float ylen, float zlen)

        1) check if the location is within the map
        2) check if the friction box lies on the player's path
        return 0 iff 1, and 2) are satisfied.
        """

        if self.conf.function != "eval-os" and self.conf.function != "eval-us":
            dist = self.get_distance_from_player(
                carla.Location(location[0], location[1], location[2])
            )
            if (dist > c.MAX_DIST_FROM_PLAYER):
                # print("[-] too far from player: {}m".format(int(dist)))
                return -1

        rotation = (0, 0, 0)
        spawn_point = (location, rotation) # carla.Transform(location, carla.Rotation())

        new_puddle = {
                "level": level,
                "size": size,
                "spawn_point": spawn_point
            }

        self.puddles.append(new_puddle)

        return 0


    def dump_states(self, state, log_type):
        if self.conf.debug:
            print("[*] dumping {} data".format(log_type))

        state_dict = {}

        state_dict["fuzzing_start_time"] = self.conf.cur_time
        state_dict["determ_seed"] = self.conf.determ_seed
        state_dict["seed"] = self.seed_data
        state_dict["weather"] = self.weather
        state_dict["autoware_cmd"] = state.autoware_cmd
        state_dict["autoware_goal"] = state.autoware_goal

        actor_list = []
        for actor in self.actors: # re-convert from carla.transform to xyz
            actor_dict = {
                    "type": actor["type"],
                    "nav_type": actor["nav_type"],
                    "speed": actor["speed"],
                    }
            if actor["spawn_point"] is not None:
                actor_dict["sp_x"] = actor["spawn_point"][0][0]
                actor_dict["sp_y"] = actor["spawn_point"][0][1]
                actor_dict["sp_z"] = actor["spawn_point"][0][2]
                actor_dict["sp_roll"] = actor["spawn_point"][1][0]
                actor_dict["sp_pitch"] = actor["spawn_point"][1][1]
                actor_dict["sp_yaw"] = actor["spawn_point"][1][2]

            if actor["dest_point"] is not None:
                actor_dict["dp_x"] = actor["dest_point"][0][0]
                actor_dict["dp_y"] = actor["dest_point"][0][1]
                actor_dict["dp_z"] = actor["dest_point"][0][2]
            actor_list.append(actor_dict)
        state_dict["actors"] = actor_list

        puddle_list = []
        for puddle in self.puddles:
            puddle_dict = {
                    "level": puddle["level"],
                    "sp_x": puddle["spawn_point"][0][0],
                    "sp_y": puddle["spawn_point"][0][1],
                    "sp_z": puddle["spawn_point"][0][2],
                    }
            puddle_dict["size_x"] = puddle["size"][0]
            puddle_dict["size_y"] = puddle["size"][1]
            puddle_dict["size_z"] = puddle["size"][2]
            puddle_list.append(puddle_dict)
        state_dict["puddles"] = puddle_list

        state_dict["first_frame_id"] = state.first_frame_id
        state_dict["first_sim_elapsed_time"] = state.first_sim_elapsed_time
        state_dict["sim_start_time"] = state.sim_start_time
        state_dict["num_frames"] = state.num_frames
        state_dict["elapsed_time"] = state.elapsed_time

        state_dict["deductions"] = state.deductions

        vehicle_state_dict = {
                "speed": state.speed,
                "steer_wheel_angle": state.steer_angle_list,
                "yaw": state.yaw_list,
                "yaw_rate": state.yaw_rate_list,
                "lat_speed": state.lat_speed_list,
                "lon_speed": state.lon_speed_list,
                "min_dist": state.min_dist
                }
        state_dict["vehicle_states"] = vehicle_state_dict

        control_dict = {
                "throttle": state.cont_throttle,
                "brake": state.cont_brake,
                "steer": state.cont_steer
                }
        state_dict["control_cmds"] = control_dict

        event_dict = {
                "crash": state.crashed,
                "stuck": state.stuck,
                "lane_invasion": state.laneinvaded,
                "red": state.red_violation,
                "speeding": state.speeding,
                "other": state.other_error,
                "other_error_val": state.other_error_val
                }
        state_dict["events"] = event_dict

        config_dict = {
                "fps": c.FRAME_RATE,
                "max_dist_from_player": c.MAX_DIST_FROM_PLAYER,
                "min_dist_from_player": c.MIN_DIST_FROM_PLAYER,
                "abort_seconds": self.conf.timeout,
                "wait_autoware_num_topics": c.WAIT_AUTOWARE_NUM_TOPICS
                }
        state_dict["config"] = config_dict

        filename = "{}_{}_{}_{}.json".format(state.campaign_cnt,
                state.cycle_cnt, state.mutation, time.time())

        if log_type == "queue":
            out_dir = self.conf.queue_dir

        # elif log_type == "error":
            # out_dir = self.conf.error_dir
        # elif log_type == "cov":
            # out_dir = self.conf.cov_dir

        with open(os.path.join(out_dir, filename), "w") as fp:
            json.dump(state_dict, fp)

        if self.conf.debug:
            print("[*] dumped")

        return filename


    def run_test(self, state):
        if self.conf.debug:
            print("[*] call executor.simulate()")
            # print("Weather:", self.weather)
        # print("before sim", time.time())

        sp = self.get_seed_sp_transform(self.seed_data)
        wp = self.get_seed_wp_transform(self.seed_data)

        ret = executor.simulate(
            conf=self.conf,
            state=state,
            town=self.town,
            sp=sp,
            wp=wp,
            weather_dict=self.weather,
            frictions_list=self.puddles,
            actors_list=self.actors
        )

        # print("after sim", time.time())

        # print("before logging", time.time())
        log_filename = self.dump_states(state, log_type="queue")
        self.log_filename = log_filename
        # print("after logging", time.time())

        if state.spawn_failed:
            obj = state.spawn_failed_object
            if self.conf.debug:
                print("failed object:", obj)

            # don't try to spawn an infeasible actor in the next run
            # XXX: and we need a map of coordinates that represent
            #      spawn-feasibility
            if "level" in obj:
                self.puddles.remove(obj)
            else:
                self.actors.remove(obj)
            return -1

        # print("before error checking", time.time())
        if self.conf.debug:
            print("----- Check for errors -----")
        error = False
        if self.conf.check_dict["crash"] and state.crashed:
            if self.conf.debug:
                print("Crashed:", state.collision_event)
                oa = state.collision_event.other_actor
                print(f"  - against {oa.type_id}")
            error = True
        if self.conf.check_dict["stuck"] and state.stuck:
            if self.conf.debug:
                print("Vehicle stuck:", state.stuck_duration)
            error = True
        if self.conf.check_dict["lane"] and state.laneinvaded:
            if self.conf.debug:
                le_list = state.laneinvasion_event
                le = le_list[0] # only consider the very first invasion
                print("Lane invasion:", le)
                lm_list = le.crossed_lane_markings
                for lm in lm_list:
                    print("  - crossed {} lane (allows {} change)".format(
                        lm.color, lm.lane_change))
            error = True
        if self.conf.check_dict["red"] and state.red_violation:
            error = True
        if self.conf.check_dict["speed"] and state.speeding:
            if self.conf.debug:
                print("Speeding: {} km/h".format(state.speed[-1]))
            error = True
        if self.conf.check_dict["other"] and state.other_error:
            if state.other_error == "timeout":
                if self.conf.debug:
                    print("Simulation took too long")
            elif state.other_error == "goal":
                if self.conf.debug:
                    print("Goal is too far:", state.other_error_val, "m")
            error = True

        # print("before file ops", time.time())
        if self.conf.agent_type == c.AUTOWARE:
            if error:
                # print("copying bag & video files")
                shutil.copyfile(
                        os.path.join(self.conf.queue_dir, log_filename),
                        os.path.join(self.conf.error_dir, log_filename)
                        )
                shutil.copyfile(
                        "/tmp/fuzzerdata/bagfile.lz4.bag",
                        os.path.join(self.conf.rosbag_dir, log_filename.replace(".json", ".bag"))
                        )

            shutil.copyfile(
                    "/tmp/fuzzerdata/front.mp4",
                    os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-front.mp4"))
                    )
            shutil.copyfile(
                    "/tmp/fuzzerdata/rear.mp4",
                    os.path.join(self.conf.cam_dir, log_filename.replace(".json", "-rear.mp4"))
                    )

        elif self.conf.agent_type == c.BASIC or self.conf.agent_type == c.BEHAVIOR:
            if error:
                shutil.copyfile(
                    os.path.join(self.conf.queue_dir, log_filename),
                    os.path.join(self.conf.error_dir, log_filename)
                )

            shutil.copyfile(
                "/tmp/fuzzerdata/front.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-front.mp4")
                )
            )

            shutil.copyfile(
                "/tmp/fuzzerdata/top.mp4",
                os.path.join(
                    self.conf.cam_dir,
                    log_filename.replace(".json", "-top.mp4")
                )
            )

        # print("after file ops", time.time())

        if not self.conf.function.startswith("eval"):
            if ret == 128:
                return 128

        if error:
            self.found_error = True
            return 1

        if state.num_frames <= c.FRAME_RATE:
            # Trap for unlikely situation where test target didn't load
            # but we somehow got here.
            print("[-] Not enough data for scoring ({} frames)".format(
                state.num_frames))
            return 1

        # print("before scoring", time.time())
        if self.conf.debug:
            print("----- Scoring -----")
            # print("[debug] # frames:", state.num_frames)
            # print("[debug] elapsed time:", state.elapsed_time)
            # print("[debug] dist:", state.min_dist)
        np.set_printoptions(precision=3, suppress=True)

        # Attributes
        speed_list = np.array(state.speed)
        acc_list = np.diff(speed_list)

        Vx_list = np.array(state.lon_speed_list)
        Vy_list = np.array(state.lat_speed_list)
        SWA_list = np.array(state.steer_angle_list)

        # filter & process attributes
        Vx_light = get_vx_light(Vx_list)
        Ay_list = get_ay_list(Vy_list)
        Ay_diff_list = get_ay_diff_list(Ay_list)
        Ay_heavy = get_ay_heavy(Ay_list)
        SWA_diff_list = get_swa_diff_list(Vy_list)
        SWA_heavy_list = get_swa_heavy(SWA_list)
        Ay_gain = get_ay_gain(SWA_heavy_list, Ay_heavy)
        Ay_peak = get_ay_peak(Ay_gain)
        frac_drop = get_frac_drop(Ay_gain, Ay_peak)
        abs_yr = get_abs_yr(state.yaw_rate_list)

        deductions = 0

        # avoid infinitesimal md
        if int(state.min_dist) > 100:
            md = 0
        else:
            md = (1 / int(state.min_dist))

        ha = int(check_hard_acc(acc_list))
        hb = int(check_hard_braking(acc_list))
        ht = int(check_hard_turn(Vy_list, SWA_list))

        deductions += ha + hb + ht + md

        # check oversteer and understeer
        os_thres = 4
        us_thres = 4
        num_oversteer = 0
        num_understeer = 0
        for fid in range(len(Vy_list) - 2):
            SWA_diff = SWA_diff_list[fid]
            Ay_diff = Ay_diff_list[fid]
            yr = abs_yr[fid]

            Vx = Vx_light[fid]
            SWA2 = SWA_heavy_list[fid]
            fd = frac_drop[fid]
            os_level = get_oversteer_level(SWA_diff, Ay_diff, yr)
            us_level = get_understeer_level(fd)

            # TODO: add unstable event detection (section 3.5.1)

            if os_level >= os_thres:
                if Vx > 5 and Ay_diff > 0.1:
                    num_oversteer += 1
                    # print("OS @%d %.2f (SWA %.4f Ay %.4f AVz %.4f Vx %.4f)" %(
                        # fid, os_level, SWA_diff, Ay_diff, yr, Vx))
            if us_level >= us_thres:
                if Vx > 5 and SWA2 > 10:
                    num_understeer += 1
                    # print("US @%d %.2f (SA %.4f FD %.4f Vx %.4f)" %(
                        # fid, us_level, sa2, fd, Vx))

        if self.conf.debug:
            # print("[debug] # ha:", ha)
            # print("[debug] # hb:", hb)
            # print("[debug] # ht:", ht)
            # print("[debug] # oversteer:", num_oversteer)
            # print("[debug] # understeer:", num_understeer)
            pass

        ovs = int(num_oversteer)
        uds = int(num_understeer)
        deductions += ovs + uds
        state.deductions = {
            "ha": ha, "hb": hb, "ht": ht, "os": ovs, "us": uds, "md": md
        }

        self.driving_quality_score = -deductions

        print("[*] driving quality score: {}".format(
            self.driving_quality_score))

        # print("after scoring", time.time())

        with open(os.path.join(self.conf.score_dir, log_filename), "w") as fp:
            json.dump(state.deductions, fp)


if __name__ == "__main__":
    # call run_test from this module only for debugging
    ts = TestScenario()
    ts.run_test()

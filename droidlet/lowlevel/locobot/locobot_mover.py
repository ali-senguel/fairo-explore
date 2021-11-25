"""
Copyright (c) Facebook, Inc. and its affiliates.
"""
import os
import sys
import math
import copy
import time
import logging
from collections.abc import Iterable
from prettytable import PrettyTable
import Pyro4
import numpy as np

from droidlet.shared_data_structs import ErrorWithResponse
from agents.argument_parser import ArgumentParser
from droidlet.shared_data_structs import RGBDepth


Pyro4.config.SERIALIZER = "pickle"
Pyro4.config.SERIALIZERS_ACCEPTED.add("pickle")
Pyro4.config.PICKLE_PROTOCOL_VERSION = 4


def safe_call(f, *args, **kwargs):
    try:
        return f(*args, **kwargs)
    except Pyro4.errors.ConnectionClosedError as e:
        msg = "{} - {}".format(f._RemoteMethod__name, e)
        raise ErrorWithResponse(msg)
    except Exception as e:
        print("Pyro traceback:")
        print("".join(Pyro4.util.getPyroTraceback()))
        raise e


class LoCoBotMover:
    """
    Implements methods that call the physical interfaces of the Locobot.

    Arguments:
        ip (string): IP of the Locobot.
        backend (string): backend where the Locobot lives, either "habitat" or "locobot"
    """
    def __init__(self, ip=None, backend="habitat"):
    	
       #self.bot = Pyro4.Proxy("PYRONAME:remotefranka@" + ip)
        print (f'IP In Constructor:: >  {ip}')
        self.bot = Pyro4.core.Proxy('PYRO:remotefranka@' + ip + ':9090')
        
   

    def check(self):
        """
        Sanity checks all the mover interfaces.
        Checks move by moving the locobot around in a square and reporting L1 drift and total time taken
        for the two movement modes available to the locobot - using PyRobot slam (vslam),
        and without using any slam (default)
        Checks look and point by poiting and looking at the same target.
        """
        self.reset_camera()
        table = PrettyTable(["Command", "L1 Drift (meters)", "Time (sec)"])
        sq_table = PrettyTable(["Mode", "Total L1 drift (meters)", "Total time (sec)"])

        def l1_drift(a, b):
            return round(abs(a[0] - b[0]) + abs(a[1] - b[1]), ndigits=3)

        def execute_move(init_pos, dest_pos, cmd_text, use_map=False):
            logging.info("Executing {} ... ".format(cmd_text))
            start = time.time()
            self.move_absolute([dest_pos], use_map=use_map)
            end = time.time()
            tt = round((end - start), ndigits=3)
            pos_after = self.get_base_pos_in_canonical_coords()
            drift = l1_drift(pos_after, dest_pos)
            logging.info("Finished Executing. \nDrift: {} Time taken: {}".format(drift, tt))
            table.add_row([cmd_text, drift, tt])
            return drift, tt

        def move_in_a_square(magic_text, side=0.3, use_vslam=False):
            """
            Moves the locobot in a square starting from the bottom right - goes left, forward, right, back.

            Args:
                magic_text (str): unique text to differentiate each scenario
                side (float): side of the square
            Returns:
                total L1 drift, total time taken to move around the square.
            """
            pos = self.get_base_pos_in_canonical_coords()
            logging.info("Initial agent pos {}".format(pos))
            dl, tl = execute_move(
                pos,
                [pos[0] - side, pos[1], pos[2]],
                "Move Left " + magic_text,
                use_map=use_vslam,
            )
            df, tf = execute_move(
                pos,
                [pos[0] - side, pos[1] + side, pos[2]],
                "Move Forward " + magic_text,
                use_map=use_vslam,
            )
            dr, tr = execute_move(
                pos,
                [pos[0], pos[1] + side, pos[2]],
                "Move Right " + magic_text,
                use_map=use_vslam,
            )
            db, tb = execute_move(
                pos,
                [pos[0], pos[1], pos[2]],
                "Move Backward " + magic_text,
                use_map=use_vslam,
            )
            return dl + df + dr + db, tl + tf + tr + tb

        # move in a square of side 0.3 starting at current base pos
        d, t = move_in_a_square("default", side=0.3, use_vslam=False)
        sq_table.add_row(["default", d, t])

        d, t = move_in_a_square("use_vslam", side=0.3, use_vslam=True)
        sq_table.add_row(["use_vslam", d, t])

        print(table)
        print(sq_table)

        # Check that look & point are at the same target
        logging.info("Visually check that look and point are at the same target")
        pos = self.get_base_pos_in_canonical_coords()
        look_pt_target = [pos[0] + 0.5, 1, pos[1] + 1]

        # look
        self.look_at(look_pt_target, 0, 0)
        logging.info("Completed Look at.")

        # point
        self.point_at(look_pt_target)
        logging.info("Completed Point.")

    # TODO/FIXME!  instead of just True/False, return diagnostic messages
    # so e.g. if a grip attempt fails, the task is finished, but the status is a failure
    def bot_step(self):
        try:
            f = self.bot.command_finished()
        except:
            # do better here?
            f = True
        return f

    def get_pan(self):
        """get yaw in radians."""
        return self.bot.get_pan()

    def get_tilt(self):
        """get pitch in radians."""
        return self.bot.get_tilt()

    def reset_camera(self):
        """reset the camera to 0 pan and tilt."""
        return self.bot.reset()

    def move_relative(self, xyt_positions, blocking=True):
        """
        Command to execute a relative move.

        Args:
            xyt_positions: a list of relative (x,y,yaw) positions for the bot to execute.
            x,y,yaw are in the pyrobot's coordinates.
            blocking (boolean): If True, waits for navigation to complete.
        """
        if not isinstance(next(iter(xyt_positions)), Iterable):
            # single xyt position given
            xyt_positions = [xyt_positions]
        for xyt in xyt_positions:
            self.nav_result.wait() # wait for the previous navigation command to finish
            self.nav_result = safe_call(self.nav.go_to_relative, xyt)
            if blocking:
                self.nav_result.wait()

    def move_absolute(self, xyt_positions, use_map=False, blocking=True):

        return "finished"

    def look_at(self, obj_pos, yaw_deg, pitch_deg):


        return "finished"

    def point_at(self, target_pos):

        return "finished"

    def get_base_pos_in_canonical_coords(self):
        """
        get the current Locobot position in the canonical coordinate system
        instead of the Locobot's global coordinates as stated in the Locobot
        documentation: https://www.pyrobot.org/docs/navigation.
        The standard coordinate systems:
        Camera looks at (0, 0, 1),
        its right direction is (1, 0, 0) and
        its up-direction is (0, 1, 0)

         return:
         (x, z, yaw) of the Locobot base in standard coordinates
        """

        x_global, y_global, yaw = safe_call(self.bot.get_base_state, "odom")
        x_standard = -y_global
        z_standard = x_global
        return np.array([x_standard, z_standard, yaw])

    def get_base_pos(self):
        """
        Return Locobot (x, y, yaw) in the robot base coordinates as
        illustrated in the docs:
        https://www.pyrobot.org/docs/navigation
        """
        return self.bot.get_base_state("odom")

    def get_rgb_depth(self):
        pass

    def get_current_pcd(self, in_cam=False, in_global=False):
        """Gets the current point cloud"""
        return self.bot.get_current_pcd(in_cam=in_cam, in_global=in_global)

    def dance(self):
        self.bot.dance()

    def turn(self, yaw):
        """
        turns the bot by the yaw specified.

        Args:
            yaw: the yaw to execute in degree.
        """
        turn_rad = yaw * math.pi / 180
        self.bot.go_to_relative([0, 0, turn_rad], close_loop=self.close_loop)

    def grab_nearby_object(self, bounding_box=[(240, 480), (100, 540)]):
        """

        :param bounding_box: region in image to search for grasp
        """
        return self.bot.grasp(bounding_box)

    def explore(self):
        if self.nav_result.ready:
            self.nav_result = safe_call(self.nav.explore)
        else:
            print("navigator executing another call right now")
        return self.nav_result

    def get_obstacles_in_canonical_coords(self):
        pass

    def print_result(self):
        print ("Successful Connection")
        
        
if __name__ == "__main__":
    base_path = os.path.dirname(__file__)
    parser = ArgumentParser("Locobot", base_path)
    opts = parser.parse()
    mover = LoCoBotMover(ip=opts.ip, backend=opts.backend)
    mover.bot.move()
    mover.bot.go_home()
    mover.bot.move()
    print('client exiting')
    

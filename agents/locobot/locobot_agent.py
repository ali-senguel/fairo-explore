"""
Copyright (c) Facebook, Inc. and its affiliates.
"""
import os
import time
import signal
import random
import logging
import faulthandler
from multiprocessing import set_start_method
import shutil

import Pyro4

from droidlet import dashboard
#from droidlet.tools.data_scripts.try_download import try_download_artifacts

if __name__ == "__main__":
    # this line has to go before any imports that contain @sio.on functions
    # or else, those @sio.on calls become no-ops
    dashboard.start()
from droidlet.base_util import to_player_struct, Pos, Look, Player
from agents.droidlet_agent import DroidletAgent
from agents.argument_parser import ArgumentParser
from droidlet.memory.robot.loco_memory import LocoAgentMemory, DetectedObjectNode
from droidlet.perception.semantic_parsing.utils.interaction_logger import InteractionLogger

from droidlet.interpreter.robot import (
    dance,
    default_behaviors
)
from droidlet.dialog.robot import LocoBotCapabilities
import droidlet.lowlevel.rotation as rotation
from droidlet.event import sio

faulthandler.register(signal.SIGUSR1)

random.seed(0)
log_formatter = logging.Formatter(
    "%(asctime)s [%(filename)s:%(lineno)s - %(funcName)s() %(levelname)s]: %(message)s"
)
logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger().handlers.clear()

Pyro4.config.SERIALIZER = "pickle"
Pyro4.config.SERIALIZERS_ACCEPTED.add("pickle")
Pyro4.config.PICKLE_PROTOCOL_VERSION = 4


class LocobotAgent(DroidletAgent):
    """Implements an instantiation of the LocoMCAgent on a Locobot. It starts
    off the agent processes including launching the dashboard.

    Args:
        opts (argparse.Namespace): opts returned by the ArgumentParser with defaults set
            that you can override.
        name (string, optional): a name for your agent (default: Locobot)

    Example:
        >>> python locobot_agent.py --backend 'habitat'
    """

    coordinate_transforms = rotation

    def __init__(self, opts, name="Locobot"):
        self.backend = opts.backend
        super(LocobotAgent, self).__init__(opts)
        logging.info("LocobotAgent.__init__ started")
        self.agent_type = "locobot"
        self.opts = opts
        self.entityId = 0
        self.no_default_behavior = opts.no_default_behavior
        self.last_chat_time = -1000000000000
        self.name = name
        self.player = Player(100, name, Pos(0, 0, 0), Look(0, 0))
        self.pos = Pos(0, 0, 0)
        self.uncaught_error_count = 0
        self.last_task_memid = None
        self.point_targets = []
        self.init_event_handlers()
        

        
        # list of (prob, default function) pairs
        if self.backend == 'habitat':
            self.visible_defaults = [(1.0, default_behaviors.explore)]
        else:
            raise RuntimeError("Unknown backend specified {}" % (self.backend, ))
        self.interaction_logger = InteractionLogger()
        if os.path.exists("annotation_data/rgb"):
            shutil.rmtree("annotation_data/rgb")
        if os.path.exists("annotation_data/seg"):
            shutil.rmtree("annotation_data/seg")
            

    def init_event_handlers(self):
        super().init_event_handlers()

        @sio.on("movement command")
        def test_command(sid, commands, movement_values={}):
            if len(movement_values) == 0:
                movement_values["yaw"] = 0.01
                movement_values["velocity"] = 0.1

            movement = [0.0, 0.0, 0.0]
            for command in commands:
                if command == "MOVE_FORWARD":
                    movement[0] += movement_values["velocity"]
                    print("action: FORWARD")
                elif command == "MOVE_BACKWARD":
                    movement[0] -= movement_values["velocity"]
                    print("action: BACKWARD")
                elif command == "MOVE_LEFT":
                    movement[2] += movement_values["yaw"]
                    print("action: LEFT")
                elif command == "MOVE_RIGHT":
                    movement[2] -= movement_values["yaw"]
                    print("action: RIGHT")
                elif command == "PAN_LEFT":
                    self.mover.bot.set_pan(self.mover.bot.get_pan() + 0.08)
                elif command == "PAN_RIGHT":
                    self.mover.bot.set_pan(self.mover.bot.get_pan() - 0.08)
                elif command == "TILT_UP":
                    self.mover.bot.set_tilt(self.mover.bot.get_tilt() - 0.08)
                elif command == "TILT_DOWN":
                    self.mover.bot.set_tilt(self.mover.bot.get_tilt() + 0.08)
                elif command == "MOVE_JOINT_1":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_1()
                elif command == "MOVE_JOINT_2":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_2()
                elif command == "MOVE_JOINT_3":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_3()    
                elif command == "MOVE_JOINT_4":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_4()
                elif command == "MOVE_JOINT_5":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_5()
                elif command == "MOVE_JOINT_6":
                    print("MOVE JOINT BUTTON PRESSED")
                    self.mover.move_6() 
                elif command == "GO_HOME":
                    self.mover.go_home()   

            #self.mover.move_relative([movement])

        @sio.on("shutdown")
        def _shutdown(sid, data):
            self.shutdown()

        @sio.on("get_memory_objects")
        def objects_in_memory(sid):
            objects = DetectedObjectNode.get_all(self.memory)
            for o in objects:
                del o["feature_repr"]  # pickling optimization
            self.dashboard_memory["objects"] = objects
            sio.emit("updateState", {"memory": self.dashboard_memory})

        @sio.on("interaction data")
        def log_interaction_data(sid, interactionData):
            self.interaction_logger.logInteraction(interactionData)

        # Returns an array of objects with updated masks
        @sio.on("label_propagation")
        def label_propagation(sid, postData):
            objects = LP.label_propagation(postData)
            sio.emit("labelPropagationReturn", objects)


    def init_memory(self):
        """Instantiates memory for the agent.

        Uses the DB_FILE environment variable to write the memory to a
        file or saves it in-memory otherwise.
        """
        self.memory = LocoAgentMemory(
            db_file=os.environ.get("DB_FILE", ":memory:"),
            db_log_path=None,
            coordinate_transforms=self.coordinate_transforms,
        )
        dance.add_default_dances(self.memory)
        logging.info("Initialized agent memory")

    def init_perception(self):
        """Instantiates all perceptual modules.

        Each perceptual module should have a perceive method that is
        called by the base agent event loop.
        """
        if not hasattr(self, "perception_modules"):
            self.perception_modules = {}
        #self.perception_modules["language_understanding"] = NSPQuerier(self.opts, self)
        #self.perception_modules["self"] = SelfPerception(self)
        #self.perception_modules["vision"] = Perception(self.opts.perception_model_dir)
	

	
    def perceive(self, force=False):
        # 1. perceive from NLU parser
        print("PERCIEVE_DEBUG")
        super().perceive(force=force)
        # 2. perceive from robot perception modules
        #self.perception_modules["self"].perceive(force=force)
        #rgb_depth = self.mover.get_rgb_depth()
        #xyz = self.mover.get_base_pos_in_canonical_coords()
        #x, y, yaw = xyz
        #if self.backend == 'habitat':
        #    sio.emit(
        #        "map",
        #        {"x": x, "y": y, "yaw": yaw, "map": self.mover.get_obstacles_in_canonical_coords()},
        #    )

        #previous_objects = DetectedObjectNode.get_all(self.memory)
        # perception_output is a namedtuple of : new_detections, updated_detections, humans
        #perception_output = self.perception_modules["vision"].perceive(rgb_depth,
        #                                                       xyz,
        #                                                       previous_objects,
        #                                                       force=force)
        #self.memory.update(perception_output)


    def init_controller(self):
        """Instantiates controllers - the components that convert a text chat to task(s)."""
        dialogue_object_classes = {}
        print("CONTROLLER_DEBUG")
        dialogue_object_classes["bot_capabilities"] = {"task": LocoBotCapabilities, "data": {}}
        #dialogue_object_classes["interpreter"] = LocoInterpreter
        #dialogue_object_classes["get_memory"] = LocoGetMemoryHandler
        #dialogue_object_classes["put_memory"] = PutMemoryHandler
        #self.dialogue_manager = DialogueManager(
        #    memory=self.memory,
        #    dialogue_object_classes=dialogue_object_classes,
        #    dialogue_object_mapper=DialogueObjectMapper,
        #    opts=self.opts,
        #)

    def init_physical_interfaces(self):
        """Instantiates the interface to physically move the robot."""
        print(f"IP::: PHYSICAL INTERFACE {self.opts.ip}")
        self.mover = Pyro4.core.Proxy('PYRO:remotefranka@' + self.opts.ip + ':9090')

    def step(self):
        #super().step()
        time.sleep(0)

    def task_step(self, sleep_time=0.0):
        super().task_step(sleep_time=sleep_time)

    def shutdown(self):
        self._shutdown = True
        try:
            self.perception_modules["vision"].vprocess_shutdown.set()
        except:
            """
            the try/except is there in the event that
            self.perception_modules["vision"] has either:
            1. not been fully started yet
            2. already crashed / shutdown due to other effects
            """
            pass
        time.sleep(5)  # let the other threads die
        os._exit(0)  # TODO: remove and figure out why multiprocess sometimes hangs on exit


if __name__ == "__main__":
    base_path = os.path.dirname(__file__)
    parser = ArgumentParser("Locobot", base_path)
    opts = parser.parse()
    opts.ip = "192.168.89.166"
    print(f"IP::: LOCOBOT AGENT {opts.ip}")

    logging.basicConfig(level=opts.log_level.upper())
    # set up stdout logging
    sh = logging.StreamHandler()
    sh.setFormatter(log_formatter)
    logger = logging.getLogger()
    logger.addHandler(sh)
    logging.info("LOG LEVEL: {}".format(logger.level))

    # Check that models and datasets are up to date
    #if not opts.dev:
     #   try_download_artifacts(agent="locobot")

    #set_start_method("spawn", force=True)

    sa = LocobotAgent(opts)
    sa.start()


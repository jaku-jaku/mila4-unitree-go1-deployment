import time
import numpy as np
import torch

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller

from submodules.fast_and_efficient.potential_planner import Planner
from submodules.fast_and_efficient.src.convex_mpc_controller.A1_MPC_controller import StateEstimator, Fixer, _update_controller
from submodules.fast_and_efficient.src.convex_mpc_controller import locomotion_controller
from submodules.fast_and_efficient.src.convex_mpc_controller.locomotion_controller import ControllerMode
from submodules.fast_and_efficient.src.convex_mpc_controller.locomotion_controller import GaitType
from submodules.fast_and_efficient.src.worlds import plane_world, slope_world, stair_world, uneven_world

from threading import Lock
import numpy as np
from absl import app
from absl import flags

flags.DEFINE_string("logdir", "logs", "where to log trajectories.")
flags.DEFINE_bool("use_real_robot", False,
                  "whether to use real robot or simulation")
flags.DEFINE_bool("show_gui", True, "whether to show GUI.")
flags.DEFINE_float("max_time_secs", 1., "maximum time to run the robot.")
flags.DEFINE_enum("world", "plane",
                  ["plane", "slope", "stair", "uneven"],
                  "world type to choose from.")
FLAGS = flags.FLAGS

WORLD_NAME_TO_CLASS_MAP = dict(plane=plane_world.PlaneWorld,
                               slope=slope_world.SlopeWorld,
                               stair=stair_world.StairWorld,
                               uneven=uneven_world.UnevenWorld)


# TODO: Make sure to hang the robot from the gantry when testing this!!!

if __name__ == "__main__":
    controller = Go1Controller(policy_path="go1_flat_novel-Aug24_13-58-37_-jitted.pt")
    controller.connect_and_stand()

    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")
    cmd = torch.Tensor([0.5, 0, 0])
    time.sleep(1)


    # REAL state estimator and planner
    state_estimator = StateEstimator(controller._conf.timestep)
    planner = Planner(0.2)

    ### NEED TO REMOVE THIS AND MAKE SURE WE USE GO1CONTROLLER APIs
    controller = locomotion_controller.LocomotionController(
        FLAGS.use_real_robot,
        FLAGS.show_gui,
        world_class=WORLD_NAME_TO_CLASS_MAP[FLAGS.world],
        state_estimator=state_estimator)

    try:
        start_time = 0  # controller.time_since_reset
        current_time = start_time
        current_p = state_estimator.get_pos()
        
        def is_done(self, goal, robot_pose, delta):
            dist = np.linalg.norm(robot_pose - goal)
            done = (dist < delta).all()
            return done

        while not is_done(goal, current_p, 0.001):
            # update time
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # update position
            current_o = controller._robot.base_orientation_rpy()
            current_p = state_estimator.get_pos()

            v = planner.potential(current_p)
            o = Fixer.get_fix(current_o=current_o, dt=dt)
            v = state_estimator.world2robot(o, v)
            command = [v[0], v[1], o]
            _update_controller(controller, command)

            if not controller.is_safe:
                break

            # sleep until next time
            time.sleep(0.05)

    finally:
        controller.set_controller_mode(
            locomotion_controller.ControllerMode.TERMINATE)




    '''
    while True:
        state, obs, action = controller.control_highlevel(cmd)
        time.sleep(controller.dt) # very important - same amount of time between actions like in sim, respecting decimation
        print ("===")
        print (obs)
        print (action)
    '''

import rclpy
from rclpy.node import Node

if __name__ == "__main__":
    from real_robot import Ros2RealRobot, get_euler_xyz
else:
    from rlcontroller.real_robot import Ros2RealRobot, get_euler_xyz

import os
import os.path as osp
import json
import time
from collections import OrderedDict
from copy import deepcopy
import numpy as np
import torch
import torch.nn.functional as F
from torch.autograd import Variable

from rsl_rl import modules

class ZeroActModel(torch.nn.Module):
    def __init__(self, angle_tolerance= 0.15, delta= 0.2):
        super().__init__()
        self.angle_tolerance = angle_tolerance
        self.delta = delta

    def forward(self, dof_pos):
        target = torch.zeros_like(dof_pos)
        #diff = dof_pos - target
        #diff_large_mask = torch.abs(diff) > self.angle_tolerance
        #target[diff_large_mask] = dof_pos[diff_large_mask] - self.delta * torch.sign(diff[diff_large_mask])
        return target

class AmbotNode(Ros2RealRobot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def register_models(self, stand_model, task_model, task_policy):
        self.stand_model = stand_model
        self.task_model = task_model

        self.task_policy = task_policy
        self.use_stand_policy = True # Start with standing model

    def start_main_loop_timer(self, duration):
        self.main_loop_timer = self.create_timer(
            duration, # in sec
            self.main_loop,
        )
        
    def main_loop(self):
        if (self.joy_stick_buffer.motion_mode==1):
            self.get_logger().info("Left down button was pressed, using stand policy",throttle_duration_sec=1)
            self.use_stand_policy = True

        if (self.joy_stick_buffer.motion_mode==2) and self.use_stand_policy:
            self.get_logger().info("Right down button was pressed, stop using stand policy", throttle_duration_sec=1)
            self.use_stand_policy = False

        if (self.joy_stick_buffer.motion_mode==3):
            self.get_logger().info("Right button 1, reset the policy", throttle_duration_sec=1)
            self.task_model.reset()

        if self.use_stand_policy:
            obs = self._get_dof_pos_obs() # do not multiply by obs_scales["dof_pos"]
            action = self.stand_model(obs)
            if (action == 0).all():
                self.get_logger().info("All actions are zero, it's time to switch to the policy", throttle_duration_sec= 1)
            self.send_action(action / self.action_scale)
        else:
            # start_time = time.monotonic()
            obs = self.get_obs()
            # obs_time = time.monotonic()
            action = self.task_policy(obs)
            # policy_time = time.monotonic()
            self.send_action(action)
            # self.send_action(self._get_dof_pos_obs() / self.action_scale)
            # publish_time = time.monotonic()
            # print(
            #     "obs_time: {:.5f}".format(obs_time - start_time),
            #     "policy_time: {:.5f}".format(policy_time - obs_time),
            #     "publish_time: {:.5f}".format(publish_time - policy_time),
            # )

@torch.inference_mode()
def main():
    
    #1) load params
    import argparse
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--logdir", type= str, default= None, help= "The directory which contains the config.json and model_*.pt files")
    parser.add_argument("--nodryrun", action= "store_true", default= True, help= "Disable dryrun mode")
    parser.add_argument("--loop_mode", type= str, default= "timer",
        choices= ["while", "timer"],
        help= "Select which mode to run the main policy control iteration",
    )
    args, unknown = parser.parse_known_args()

    #2) init ros env
    rclpy.init()

    #3) find model logs
    assert args.logdir is not None, "Please provide a logdir"
    with open(osp.join(args.logdir,"config.json"), "r") as f:
        config_dict = json.load(f, object_pairs_hook= OrderedDict)
        #print(f"config: {config_dict}")
    
    #4) modify the config_dict if needed
    config_dict["control"]["computer_clip_torque"] = True
    
    duration = config_dict["sim"]["dt"] * config_dict["control"]["decimation"] # in sec
    device = "cpu"

    #5) create env node
    env_node = AmbotNode(
        namespace="/ambotw1_ns",
        robot_class_name="AmbotW1",
        # low_cmd_topic= "low_cmd_dryrun", # for the dryrun safety
        cfg= config_dict,
        replace_obs_with_embeddings= ["forward_depth"],
        model_device= device,
        dryrun= not args.nodryrun,
    )

    #6) load model
    model = getattr(modules, config_dict["runner"]["policy_class_name"])(
        num_actor_obs = env_node.num_obs,
        num_critic_obs = env_node.num_privileged_obs,
        num_actions= env_node.num_actions,
        obs_segments= env_node.obs_segments,
        privileged_obs_segments= env_node.privileged_obs_segments,
        **config_dict["policy"]
    )

    # load the model with the latest checkpoint
    model_names = [i for i in os.listdir(args.logdir) if i.startswith("model_")]
    model_names.sort(key= lambda x: int(x.split("_")[-1].split(".")[0]))
    state_dict = torch.load(osp.join(args.logdir, model_names[-1]), map_location= "cpu")
    model.load_state_dict(state_dict["model_state_dict"])
    model.eval()
    model.to(device)

    env_node.get_logger().info("Model loaded from: {}".format(osp.join(args.logdir, model_names[-1])))
    env_node.get_logger().info("Control Duration: {} sec".format(duration))
    env_node.get_logger().info("Motor Stiffness (kp): {}".format(env_node.p_gains))
    env_node.get_logger().info("Motor Damping (kd): {}".format(env_node.d_gains))

    # zero_act_model to start the safe standing
    zero_act_model = ZeroActModel()
    zero_act_model = torch.jit.script(zero_act_model)

    # magically modify the model to use the components other than the forward depth encoders
    memory_a = model.memory_a
    mlp = model.actor

    @torch.jit.script
    def policy(obs: torch.Tensor):
        rnn_embedding = memory_a(obs)
        action = mlp(rnn_embedding)
        return action

    if hasattr(model, "state_estimator") and not hasattr(model, "state_estimator"):
        # the case where lin_vel is estimated by the state estimator
        memory_s = model.memory_s
        estimator = model.state_estimator
        rnn_policy = policy

        @torch.jit.script
        def policy(obs: torch.Tensor):
            estimator_input = obs[:, 3:48]
            memory_s_embedding = memory_s(estimator_input)
            estimated_state = estimator(memory_s_embedding)
            obs[:, :3] = estimated_state
            return rnn_policy(obs)

    if hasattr(model, "encoders") and hasattr(model, "state_estimator"):
        encoder = model.encoders[0]
        memory_s = model.memory_s
        estimator = model.state_estimator
        rnn_policy = policy

        @torch.jit.script
        def policy(obs: torch.Tensor):
            encoder_obs = obs[:, 48:]
            encoder_obs = encoder(encoder_obs)

            estimator_input = obs[:, 3:48]
            memory_s_embedding = memory_s(estimator_input)
            estimated_state = estimator(memory_s_embedding)
            obs[:, :3] = estimated_state

            rnn_policy_input = torch.cat([obs[:,:48], encoder_obs],dim=-1)

            rnn_embedding = memory_a(rnn_policy_input)
            action = mlp(rnn_embedding)

            return action

    # resgiter models and policy
    env_node.register_models(
        zero_act_model,
        model,
        policy,
    )
    # start ros
    env_node.start_ros_handlers()

    #main loop
    if args.loop_mode == "while":
        rclpy.spin_once(env_node, timeout_sec= 0.)
        env_node.get_logger().info("Model and Policy are ready")
        while rclpy.ok():
            main_loop_time = time.monotonic()
            env_node.main_loop()
            rclpy.spin_once(env_node, timeout_sec= 0.)
            time.sleep(max(0, duration - (time.monotonic() - main_loop_time)))
    elif args.loop_mode == "timer":
        env_node.start_main_loop_timer(duration)
        rclpy.spin(env_node)

    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

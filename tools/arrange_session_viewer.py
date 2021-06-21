#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
import os

import magnum as mn

import habitat_sim
from habitat_sim.utils import viz_utils as vut


def make_configuration(dataset, scene):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = scene
    backend_cfg.scene_dataset_config_file = dataset
    assert os.path.exists(backend_cfg.scene_id)
    backend_cfg.enable_physics = True
    backend_cfg.scene_light_setup = ""
    backend_cfg.override_scene_light_defaults = True

    sensor_cfg = habitat_sim.CameraSensorSpec()
    sensor_cfg.resolution = [544, 720]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def make_quaternion(float_array):
    return mn.Quaternion(
        mn.Vector3(float_array[1], float_array[2], float_array[3]), float_array[0]
    )


def session_viewer(args):

    show_video = args.show_video
    make_video = args.make_video

    if make_video and not os.path.exists(args.output_folder):
        os.mkdir(args.output_folder)

    with open(args.session, "r") as f:
        json_root = json.load(f)
        assert "session" in json_root
        session = json_root["session"]

    scene_filepath = session["scene"]
    print("scene: " + scene_filepath)

    cfg = make_configuration(args.dataset, scene_filepath)

    sim = habitat_sim.Simulator(cfg)

    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["rgba_camera"]._sensor_object.object

    # initial agent transform
    agent_node.translation = mn.Vector3(-0.045473, 0.0, -0.418929)
    agent_node.rotation = mn.Quaternion(mn.Vector3(0, -0.256289, 0), 0.9666)

    # initial sensor local transform (relative to agent)
    sensor_node.translation = mn.Vector3(-0.045473, 0, -0.418929)
    sensor_node.rotation = mn.Quaternion(mn.Vector3(-0.271441, 0, 0), 0.962455)

    keyframes = session["keyframes"]
    assert keyframes

    rigid_obj_mgr = sim.get_rigid_object_manager()
    art_obj_mgr = sim.get_articulated_object_manager()

    observations = []

    for user_action_index, user_action in enumerate(session["userActions"]):

        print(
            "rendering "
            + str(user_action_index + 1)
            + "/"
            + str(len(session["userActions"]))
            + " actions"
        )

        # note off-by-one on endFrame
        for keyframe_index in range(
            user_action["startFrame"], user_action["endFrame"] + 1
        ):

            keyframe = keyframes[keyframe_index]

            for keyframe_rigid_obj in keyframe["rigidObjects"]:
                handles = rigid_obj_mgr.get_object_handles(keyframe_rigid_obj["name"])
                assert len(handles)
                rigid_obj = rigid_obj_mgr.get_object_by_handle(handles[0])
                rigid_obj.translation = mn.Vector3(keyframe_rigid_obj["pos"])
                rigid_obj.rotation = make_quaternion(keyframe_rigid_obj["rot"])

            for keyframe_art_obj in keyframe["articulatedObjects"]:
                handles = art_obj_mgr.get_object_handles(keyframe_art_obj["name"])
                assert len(handles)
                art_obj = art_obj_mgr.get_object_by_handle(handles[0])
                art_obj.translation = mn.Vector3(keyframe_art_obj["pos"])
                art_obj.rotation = make_quaternion(keyframe_art_obj["rot"])
                art_obj.joint_positions = keyframe_art_obj["joints"]

            obs = sim.get_sensor_observations()
            observations.append(obs)
            if keyframe_index == user_action["endFrame"]:
                # repeat the last frame in the action to produce a pause in the video
                for _ in range(30):
                    observations.append(obs)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera",
            "color",
            args.output_folder
            + "/"
            + os.path.splitext(os.path.basename(args.session))[0],
            open_vid=show_video,
        )

    # # simulate with empty scene
    # observations += simulate_with_moving_agent(
    #     sim,
    #     duration=1.0,
    #     agent_vel=np.array([0.5, 0.0, 0.0]),
    #     look_rotation_vel=25.0,
    #     get_frames=make_video,
    # )

    # if make_video:
    #     vut.make_video(
    #         observations,
    #         "rgba_camera",
    #         "color",
    #         output_path + "episode",
    #         open_vid=show_video,
    #     )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-video", dest="show_video", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument("--dataset")
    parser.add_argument("--session")
    parser.add_argument("--output-folder")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()

    session_viewer(args)

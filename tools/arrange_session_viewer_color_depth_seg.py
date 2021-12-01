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


    height, width = 544, 720

    sensor_specs = []
    color_sensor_spec = habitat_sim.CameraSensorSpec()
    color_sensor_spec.uuid = "color_sensor"
    color_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    color_sensor_spec.resolution = [height, width] # [settings["height"], settings["width"]]
    # color_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    color_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(color_sensor_spec)
    
    depth_sensor_spec = habitat_sim.CameraSensorSpec()
    depth_sensor_spec.uuid = "depth_sensor"
    depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_sensor_spec.resolution = [height, width] #[settings["height"], settings["width"]]
    # depth_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    depth_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(depth_sensor_spec)

    semantic_sensor_spec = habitat_sim.CameraSensorSpec()
    semantic_sensor_spec.uuid = "semantic_sensor"
    semantic_sensor_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
    semantic_sensor_spec.resolution = [height, width]  #[settings["height"], settings["width"]]
    # semantic_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]
    semantic_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(semantic_sensor_spec)
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs # [sensor_cfg]

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

    dataset_filepath = session["dataset"]
    scene_filepath = session["scene"]
    print("dataset: " + dataset_filepath)
    print("scene: " + scene_filepath)

    cfg = make_configuration(dataset_filepath, scene_filepath)

    sim = habitat_sim.Simulator(cfg)

    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["color_sensor"]._sensor_object.object
    depth_node = sim._sensors["depth_sensor"]._sensor_object.object
    semantic_node = sim._sensors["semantic_sensor"]._sensor_object.object
    # place agent at origin (we will never move the agent; we will only move the sensor)
    agent_node.translation = [0.0, 0.0, 0.0]
    agent_node.rotation = mn.Quaternion()

    if session["defaultCamera"]:
        cam_transform = session["defaultCamera"]        
        sensor_node.translation = depth_node.translation = semantic_node.translation = mn.Vector3(cam_transform["translation"])
        sensor_node.rotation = depth_node.rotation = semantic_node.rotation = make_quaternion(cam_transform["rotation"])
        
    else:
        print("warning: no defaultCamera in session")
        sensor_node.translation = mn.Vector3(0.0, 0.0, 0.0)
        sensor_node.rotation = mn.Quaternion()

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
            instance_counter = 0
            for keyframe_rigid_obj in keyframe["rigidObjects"]:
                instance_counter += 1 

                handles = rigid_obj_mgr.get_object_handles(keyframe_rigid_obj["name"])
                assert len(handles)
                rigid_obj = rigid_obj_mgr.get_object_by_handle(handles[0])
                rigid_obj.translation = mn.Vector3(keyframe_rigid_obj["pos"])
                rigid_obj.rotation = make_quaternion(keyframe_rigid_obj["rot"])
                rigid_obj.semantic_id = instance_counter   

            for keyframe_art_obj in keyframe["articulatedObjects"]:
                handles = art_obj_mgr.get_object_handles(keyframe_art_obj["name"])
                assert len(handles)
                art_obj = art_obj_mgr.get_object_by_handle(handles[0])
                art_obj.translation = mn.Vector3(keyframe_art_obj["pos"])
                art_obj.rotation = make_quaternion(keyframe_art_obj["rot"])
                art_obj.joint_positions = keyframe_art_obj["joints"]
                # add semantic ids only for dishwasher components
                if handles[0] == 'ktc_dishwasher_:0000':
                    for visual_scene_node in art_obj.visual_scene_nodes:
                        instance_counter += 1 
                        visual_scene_node.semantic_id = instance_counter

            obs = sim.get_sensor_observations()
            observations.append(obs)
            if keyframe_index == user_action["endFrame"]:
                # repeat the last frame in the action to produce a pause in the video
                for _ in range(30):
                    observations.append(obs)

    if make_video:
        vut.make_video(
            observations,
            "color_sensor",
            "color",
            args.output_folder
            + "/"
            + os.path.splitext(os.path.basename(args.session))[0]
            + '_color',
            open_vid=show_video,
        )
        vut.make_video(
            observations,
            "depth_sensor",
            "depth",
            args.output_folder
            + "/"
            + os.path.splitext(os.path.basename(args.session))[0]
            + '_depth',
            open_vid=show_video,
        )
        vut.make_video(
            observations,
            "semantic_sensor",
            "semantic",
            args.output_folder
            + "/"
            + os.path.splitext(os.path.basename(args.session))[0]
            + '_semantic',
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
    parser.add_argument("--session")
    parser.add_argument("--output-folder")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()

    session_viewer(args)

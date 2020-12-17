# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     name: Replay Tutorial
#     provenance: []
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.6.0
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %%
# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/master/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s

# %%

# @title Setup code { display-mode: "form" }
# @markdown (double click to show code)
# %cd /content/habitat-sim
import os
import random
import sys

import git
import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim import gfx_replay_utils
from habitat_sim.utils import viz_utils as vut

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_path = os.path.join(dir_path, "examples/tutorials/replay_tutorial_output/")


def remove_all_objects(sim):
    for id_ in sim.get_existing_object_ids():
        sim.remove_object(id_)


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = os.path.join(
        data_path, "scene_datasets/habitat-test-scenes/apartment_1.glb"
    )
    assert os.path.exists(backend_cfg.scene_id)
    backend_cfg.enable_physics = True

    # Enable gfx replay save. See also our call to sim.gfx_replay_manager.save_keyframe()
    # below.
    backend_cfg.enable_gfx_replay_save = True

    # sensor configuration
    sensor_specs = []
    sensor_spec = habitat_sim.SensorSpec()
    sensor_spec.uuid = "rgba_camera_1stperson"
    sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    sensor_spec.resolution = [544, 720]
    sensor_specs.append(sensor_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


# %%
# @title Helper function to move our agent, step physics, and showcase the replay recording API.


def simulate_with_moving_agent(
    sim,
    duration=1.0,
    agent_vel=np.array([0, 0, 0]),
    look_rotation_vel=0.0,
    get_frames=True,
):
    sensor_node = sim._sensors["rgba_camera_1stperson"]._sensor_object.object
    agent_node = sim.get_agent(0).body.object

    # simulate dt seconds at 60Hz to the nearest fixed timestep
    time_step = 1.0 / 60.0

    rotation_x = mn.Quaternion.rotation(
        mn.Deg(look_rotation_vel) * time_step, mn.Vector3(1.0, 0, 0)
    )

    print("Simulating " + str(duration) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + duration:

        # move agent
        agent_node.translation += agent_vel * time_step

        # rotate sensor
        sensor_node.rotation *= rotation_x

        # Add user transforms for the agent and sensor. We'll use these later during
        # replay playback.
        gfx_replay_utils.add_node_user_transform(sim, agent_node, "agent")
        gfx_replay_utils.add_node_user_transform(sim, sensor_node, "sensor")

        sim.step_physics(time_step)

        # save a replay keyframe after every physics step
        sim.gfx_replay_manager.save_keyframe()

        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations

# %%
# @title More setup

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-video", dest="show_video", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.show_video
    make_video = args.make_video
    if make_video and not os.path.exists(output_path):
        os.mkdir(output_path)

    cfg = make_configuration()
    sim = None
    random.seed(0)
    replay_filepaths = []
    num_episodes = 1

    for episode_index in range(num_episodes):
        episodeName = "episode{}".format(episode_index)
        replay_filepath = output_path + episodeName + ".json"
        replay_filepaths.append(replay_filepath)

    # %%
    # @title Run three episodes. Save videos and replays.

    for episode_index in range(num_episodes):

        if not sim:
            sim = habitat_sim.Simulator(cfg)
        else:
            sim.reconfigure(cfg)

        agent_state = habitat_sim.AgentState()
        agent = sim.initialize_agent(0, agent_state)

        agent_node = sim.get_agent(0).body.object
        sensor_node = sim._sensors["rgba_camera_1stperson"]._sensor_object.object

        # initial agent transform
        agent_node.translation = [-0.15, -1.5, 1.0]
        agent_node.rotation = mn.Quaternion.rotation(
            mn.Deg(-75), mn.Vector3(0.0, 1.0, 0)
        )

        # initial sensor local transform (relative to agent)
        sensor_node.translation = [0.0, 0.6, 0.0]
        sensor_node.rotation = mn.Quaternion.rotation(
            mn.Deg(-15), mn.Vector3(1.0, 0.0, 0)
        )

        observations = []

        # simulate with empty scene
        observations += simulate_with_moving_agent(
            sim,
            duration=1.0,
            agent_vel=np.array([0.5, 0.0, 0.0]),
            look_rotation_vel=25.0,
            get_frames=make_video,
        )

        obj_templates_mgr = sim.get_object_template_manager()

        obj_templates_mgr.load_configs(str(os.path.join(data_path, "objects")))
        chefcan_template_handle = obj_templates_mgr.get_template_handles(
            "data/objects/chefcan"
        )[0]

        y_dist = 0.1

        # drop some dynamic objects
        id_1 = sim.add_object_by_handle(chefcan_template_handle)
        sim.set_translation(
            np.array([2.4, -0.64 + random.uniform(-y_dist, y_dist), 0]), id_1
        )
        id_2 = sim.add_object_by_handle(chefcan_template_handle)
        sim.set_translation(
            np.array([2.4, -0.64 + random.uniform(-y_dist, y_dist), 0.28]), id_2
        )
        id_3 = sim.add_object_by_handle(chefcan_template_handle)
        sim.set_translation(
            np.array([2.4, -0.64 + random.uniform(-y_dist, y_dist), -0.28]), id_3
        )

        # simulate
        observations += simulate_with_moving_agent(
            sim,
            duration=2.0,
            agent_vel=np.array([0.0, 0.0, -0.4]),
            look_rotation_vel=-5.0,
            get_frames=make_video,
        )

        # remove some objects
        sim.remove_object(id_1)
        sim.remove_object(id_2)

        observations += simulate_with_moving_agent(
            sim,
            duration=2.0,
            agent_vel=np.array([0.4, 0.0, 0.0]),
            look_rotation_vel=-10.0,
            get_frames=make_video,
        )

        episodeName = "episode{}".format(episode_index)

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + episodeName,
                open_vid=show_video,
            )

        # save a replay at the end of an episode
        sim.gfx_replay_manager.write_saved_keyframes_to_file(
            replay_filepaths[episode_index]
        )

        remove_all_objects(sim)

    # %%
    # @title Reconfigure simulator for replay playback.
    if True:

        # use same agents/sensors from earlier, with different backend config
        playback_cfg = habitat_sim.Configuration(
            # gfx_replay_utils.make_backend_configuration_for_playback(need_separate_semantic_scene_graph=True),
            gfx_replay_utils.make_backend_configuration_for_playback(
                need_separate_semantic_scene_graph=False
            ),
            cfg.agents,
        )

        if not sim:
            sim = habitat_sim.Simulator(playback_cfg)
        else:
            sim.reconfigure(playback_cfg)

        agent_state = habitat_sim.AgentState()
        sim.initialize_agent(0, agent_state)

        agent_node = sim.get_agent(0).body.object
        sensor_node = sim._sensors["rgba_camera_1stperson"]._sensor_object.object

        # For replay playback, we place a dummy agent at the origin and then transform
        # the sensor using the "sensor" user transform stored in the replay.
        # In the future, Habitat will offer a cleaner way to play replays without an agent.
        agent_node.translation = [0.0, 0.0, 0.0]
        agent_node.rotation = mn.Quaternion()

        # %%
        # @title load replays

        players = []
        for filepath in replay_filepaths:
            player = sim.gfx_replay_manager.read_keyframes_from_file(filepath)
            assert player
            players.append(player)

        # %%
        # @title play replay #0

        observations = []
        print("play replay #0...")
        for frame in range(players[0].get_num_keyframes()):
            players[0].set_keyframe_index(frame)

            # todo: add comment
            (sensor_node.translation, sensor_node.rotation) = player.get_user_transform(
                "sensor"
            )

            observations.append(sim.get_sensor_observations())

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "replay_playback1",
                open_vid=show_video,
            )

        # %%
        # @title play in reverse at 3x

        observations = []
        print("play in reverse at 3x...")
        for frame in range(players[0].get_num_keyframes() - 2, -1, -3):
            players[0].set_keyframe_index(frame)
            (sensor_node.translation, sensor_node.rotation) = player.get_user_transform(
                "sensor"
            )
            observations.append(sim.get_sensor_observations())

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "replay_playback2",
                open_vid=show_video,
            )

        # %%
        # @title play from a different camera view, with agent visualization

        observations = []
        print("play from a different camera view, with agent visualization...")

        # place a third-person camera
        sensor_node.translation = [-1.1, -0.9, -0.2]
        sensor_node.rotation = mn.Quaternion.rotation(
            mn.Deg(-115), mn.Vector3(0.0, 1.0, 0)
        )

        prim_attr_mgr = sim.get_asset_template_manager()

        # visualize the recorded agent transform as a cylinder
        agent_viz_handle = prim_attr_mgr.get_template_handles("cylinderSolid")[0]
        agent_viz_id = sim.add_object_by_handle(agent_viz_handle)
        sim.set_object_motion_type(
            habitat_sim.physics.MotionType.KINEMATIC, agent_viz_id
        )
        sim.set_object_is_collidable(False, agent_viz_id)

        # visualize the recorded sensor transform as a cube
        sensor_viz_handle = prim_attr_mgr.get_template_handles("cubeSolid")[0]
        sensor_viz_id = sim.add_object_by_handle(sensor_viz_handle)
        sim.set_object_motion_type(
            habitat_sim.physics.MotionType.KINEMATIC, sensor_viz_id
        )
        sim.set_object_is_collidable(False, sensor_viz_id)

        for frame in range(players[0].get_num_keyframes()):
            players[0].set_keyframe_index(frame)

            (agent_translation, agent_rotation) = players[0].get_user_transform("agent")
            sim.set_translation(agent_translation, agent_viz_id)
            sim.set_rotation(agent_rotation, agent_viz_id)

            (sensor_translation, sensor_rotation) = players[0].get_user_transform(
                "sensor"
            )
            sim.set_translation(sensor_translation, sensor_viz_id)
            sim.set_rotation(sensor_rotation, sensor_viz_id)

            observations.append(sim.get_sensor_observations())

        if make_video:
            vut.make_video(
                observations,
                "rgba_camera_1stperson",
                "color",
                output_path + "replay_playback3",
                open_vid=show_video,
            )

        sim.remove_object(agent_viz_id)

        # print("play all {} replays together...".format(num_episodes))
        # for frame in range(players[0].get_num_keyframes()):
        #     for player in players:
        #         player.set_keyframe_index(frame)
        #     gfx_replay_utils.set_agent_from_user_transform(players[0], sim)
        #     observations.append(sim.get_sensor_observations())

        # if make_video:
        #     vut.make_video(
        #         observations,
        #         "rgba_camera_1stperson",
        #         "color",
        #         output_path + "replay_playback",
        #         open_vid=show_video,
        #     )

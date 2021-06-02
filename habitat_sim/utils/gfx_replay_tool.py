import argparse
from argparse import ArgumentParser

import magnum as mn

import habitat_sim
from habitat_sim.utils import gfx_replay_utils
from habitat_sim.utils import viz_utils as vut


# work in progress; don't use
def set_light_setup_keys_by_filepath(replay_str, instance_filepath, key):

    prev_index = -1
    replace_count = 0
    while True:
        index = replay_str.find(instance_filepath, prev_index + 1)
        if index == -1:
            break

        empty_light_setup_substring = '"lightSetupKey": ""'
        new_light_setup_substring = '"lightSetupKey": + "' + key + '"'
        index_of_light_setup = replay_str.find('"lightSetupKey":')
        index_of_empty_light_setup = replay_str.find(empty_light_setup_substring)
        if index_of_light_setup != index_of_empty_light_setup:
            print(
                "warning: set_light_setup_keys_by_filepath "
                + instance_filepath
                + " existing light setup key is not empty"
            )
            continue
        replay_str = replay_str.replace(
            empty_light_setup_substring, new_light_setup_substring, 1
        )

        prev_index = index
        replace_count += 1

    if replace_count == 0:
        print(
            "warning: set_light_setup_keys_by_filepath "
            + instance_filepath
            + " found zero instances"
        )

    return replay_str


def generic_find_replace(replay_str, old, new):

    if replay_str.find(old) == -1:
        print("warning: generic_find_replace " + old + " found zero instances")

    return replay_str.replace(old, new)


def do_hack_string_fixes(args):

    replay_filepath = args.input
    with open(replay_filepath) as in_file:
        replay_str = in_file.read()

    replay_str = generic_find_replace(
        replay_str,
        "data/scene_datasets/v3_sc3_staging_01.glb",
        "/home/eundersander/projects/blender/v3_sc2_staging_19_Baked_Ceiling_LittleClutter.glb",
    )

    modified_filepath = replay_filepath + ".modified"
    with open(modified_filepath, "w") as out_file:
        out_file.write(replay_str)

    print("do_hack_string_fixes: wrote modified replay to " + modified_filepath)
    return modified_filepath


def make_video_from_replay(args):

    output_path = "./data/"  # temp
    make_video = True
    show_video = False

    replay_filepath = args.input

    if args.do_hack_string_fixes:
        replay_filepath = do_hack_string_fixes(args)

    sensor_cfg = habitat_sim.CameraSensorSpec()
    sensor_cfg.resolution = [args.height, args.width]  # todo: expose resolution option
    # todo: expose fov option
    agent_cfg = habitat_sim.agent.AgentConfiguration()  # type: ignore
    agent_cfg.sensor_specifications = [sensor_cfg]
    agents = [agent_cfg]

    # use same agents/sensors from earlier, with different backend config
    playback_cfg = habitat_sim.Configuration(
        gfx_replay_utils.make_backend_configuration_for_playback(
            need_separate_semantic_scene_graph=False
        ),
        agents,
    )

    sim = habitat_sim.Simulator(playback_cfg)

    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["rgba_camera"]._sensor_object.object

    # For replay playback, we place a dummy agent at the origin and then transform the sensor using the "sensor" user transform stored in the replay. In the future, Habitat will offer a cleaner way to play replays without an agent.

    agent_node.translation = [0.0, 0.0, 0.0]
    agent_node.rotation = mn.Quaternion()

    # Load the requested replay
    player = sim.gfx_replay_manager.read_keyframes_from_file(replay_filepath)
    assert player

    # Play back the replay.
    # Note call to player.set_keyframe_index. Note also call to player.get_user_transform. For this playback, we restore our sensor to the original sensor transform from the episode. In this way, we reproduce the same observations. Note this doesn't happen automatically when using gfx replay; you must position your agent, sensor, or camera explicitly when playing a replay.

    observations = []
    error_count = 0
    for frame in range(player.get_num_keyframes()):
        player.set_keyframe_index(frame)

        user_transform_name = args.camera_name
        camera_trans_rot_pair = player.get_user_transform(user_transform_name)
        if not camera_trans_rot_pair:
            if error_count < 5:
                print(
                    "warning: skipping frame "
                    + str(frame)
                    + " because the user transform '"
                    + user_transform_name
                    + "' wasn't found"
                )
            elif error_count == 5:
                print("additional warnings will be disabled")
            error_count += 1
            continue

        (sensor_node.translation, sensor_node.rotation) = camera_trans_rot_pair

        obs = sim.get_sensor_observations()
        primary_sensor_name = "rgba_camera"
        primary_obs = obs[primary_sensor_name]
        if args.flip_y:
            new_primary_obs = []
            for i in range(len(primary_obs) - 1, -1, -1):
                row = primary_obs[i]
                new_primary_obs.append(row)
            obs[primary_sensor_name] = new_primary_obs
        observations.append(obs)

        if (frame - 1) % 200 == 0:
            print(
                "processed "
                + str(frame - 1)
                + " of "
                + str(player.get_num_keyframes())
                + " frames"
            )

    if len(observations) == 0:
        print("error: no valid frames found in replay")
        return

    if make_video:
        vut.make_video(
            observations,
            primary_sensor_name,
            "color",
            output_path + "replay_playback1",
            open_vid=show_video,
        )

    # clean up the player (not really necessary unless we're doing more rendering later)
    player.close()


def create_arg_parser() -> ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        help="Filepath of source replay",
    )
    parser.add_argument(
        "--flip-y",
        action="store_true",
        default=False,
        help="Flip the output vertically",
    )
    parser.add_argument(
        "--do-hack-string-fixes",
        action="store_true",
        default=False,
        help="See function do_hack_string_fixes()",
    )
    parser.add_argument(
        "--camera-name",
        default="camera",
        help='The name of the camera user transform in the replay file. Search for "userTransforms" in the replay json if you\'re not sure.',
    )
    parser.add_argument(
        "--height",
        default="480",
        help="Video output height",
    )
    parser.add_argument(
        "--width",
        default="640",
        help="Video output width",
    )

    return parser


def main():
    args = create_arg_parser().parse_args()
    make_video_from_replay(args)


if __name__ == "__main__":
    main()

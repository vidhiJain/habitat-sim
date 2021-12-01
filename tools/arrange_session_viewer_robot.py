#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
import os

import magnum as mn

import hydra

import habitat_sim
from habitat_sim.utils import viz_utils as vut

import torchcontrol as toco
from torchcontrol.modules.planning import (
    PositionMinJerkPlanner,
    JointSpaceMinJerkPlanner,
)

import numpy as np
import scipy.spatial.transform
import torch
from farsighted_mpc.datasets.graph_kitchen_dataset import (
    keyframe_to_obs_dict,
    kitchen_obs_to_graph,
    sim_to_obs_dict,
    discrete_rots,
)

# Dishwasher xyz position for open/close
z_offset = 0.15
top_closed = [-1.0518866, 0.6489163 + z_offset, 0.6096113]
top_open = [-1.0518866, 0.6489163 + z_offset, 1.2096113]
bottom_closed = [-1.0432647, 0.3537103 + z_offset, 0.5499439]
bottom_open = [-1.0432647, 0.3537103 + z_offset, 1.1499439]


USE_OFFSET_DIST_THRESHOLD = 0.15


def make_configuration(dataset, scene):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = os.path.join(hydra.utils.get_original_cwd(), scene)
    backend_cfg.scene_dataset_config_file = os.path.join(
        hydra.utils.get_original_cwd(), dataset
    )
    assert os.path.exists(backend_cfg.scene_id)
    backend_cfg.enable_physics = True
    backend_cfg.scene_light_setup = ""
    backend_cfg.override_scene_light_defaults = True

    sensor_cfg = habitat_sim.CameraSensorSpec()
    sensor_cfg.resolution = [544, 720]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]
    # breakpoint() 
    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def vec_to_list(vec: mn.Vector3) -> list:
    return [vec.x, vec.y, vec.z]


def list_to_vec(lst):
    assert len(lst) == 3
    return mn.Vector3(x=lst[0], y=lst[1], z=lst[2])


def gfx2robot_old(gfx_pos):
    """Flip y and z axis"""
    if isinstance(gfx_pos, np.ndarray):
        return gfx_pos[[0, 2, 1]] * [1, -1, 1]
    elif isinstance(gfx_pos, list):
        return [gfx_pos[0], -gfx_pos[2], gfx_pos[1]]
    else:
        raise Exception(f"Unknown type {type(gfx_pos)}")


def make_quaternion(float_array):
    return mn.Quaternion(
        mn.Vector3(float_array[1], float_array[2], float_array[3]), float_array[0]
    )


def place_robot_from_agent(
    sim: habitat_sim.Simulator,
    robot: habitat_sim._ext.habitat_sim_bindings.ManagedBulletArticulatedObject,
    local_base_pos: list,
    orientation_vector: list,
    angle_correction: float,
) -> None:
    """Moves robot to reasonable transformation relative to agent."""
    local_base_pos = np.array(local_base_pos)
    # place the robot root state relative to the agent
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    breakpoint()
    base_transform = mn.Matrix4.rotation(
        mn.Rad(angle_correction), mn.Vector3(*orientation_vector)
    )
    # base_transform = mn.Matrix4.rotation()
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    robot.transformation = base_transform


def get_action_from_index(num_bowls, num_plates, block_i, goal_i, dishwasher_i):
    if dishwasher_i > 0:  # dishwasher
        index = 0 if dishwasher_i == 1 else 2
        return "dishwasher", index
    else:  # select plate or bowl
        obj_type, obj_i, obj_goal_i = None, None, None
        if block_i < num_plates:
            obj_type = "plate"
            obj_i = block_i
        else:
            obj_type = "bowl"
            obj_i = block_i - num_plates

        if goal_i < num_plates:
            obj_goal_i = goal_i
        else:
            obj_goal_i = goal_i - num_plates

        return obj_type, obj_i, obj_goal_i


def goal_offset(dishwasher_obs, obj_pos):
    """Return the goal offset given the joint angles of the dishwasher object."""
    # if dishwasher bottom tray moved out, adjust the goal for plates
    if dishwasher_obs[2] > 0.5 and abs(obj_pos[1] - 0.3) < USE_OFFSET_DIST_THRESHOLD:
        return [0, 0, 0.6]
    # if dishwasher upper tray moved out, adjust the goal for bowls
    elif dishwasher_obs[0] > 0.5 and abs(obj_pos[1] - 0.56) < USE_OFFSET_DIST_THRESHOLD:
        return [0, 0, 0.6]
    else:
        return [0.0, 0.0, 0.0]


def session_viewer(
    session_path: str,
    robot_description_path: str,
    ee_link_name: str,
    policy_path: str = None,
    show_video: bool = False,
    make_video: bool = True,
):
    robot_model = toco.models.RobotModelPinocchio(robot_description_path, ee_link_name)

    def inverse_kinematics(ee_pos: list, ik_seed=None, desired_ori=None):
        ori = torch.Tensor([0, 1, 0, 0])
        if desired_ori is not None:
            print(ori)
            w, x, y, z = ori.cpu().numpy().tolist()
            ori_scipy = scipy.spatial.transform.Rotation.from_quat([x, y, z, w])
            x, y, z, w = (ori_scipy * desired_ori).as_quat()
            ori = torch.Tensor([w, x, y, z])
            print(ori)
        if ik_seed is not None:
            ik_seed = torch.Tensor(ik_seed)
        joint_positions = robot_model.inverse_kinematics(
            torch.Tensor(ee_pos), ori, rest_pose=ik_seed
        )
        return joint_positions.tolist()

    def forward_kinematics():
        return robot_model.forward_kinematics(torch.Tensor(robot.joint_positions))

    with open(session_path, "r") as f:
        json_root = json.load(f)
        assert "session" in json_root
        session = json_root["session"]

    dataset_filepath = session["dataset"]
    scene_filepath = session["scene"]
    print("dataset: " + dataset_filepath)
    print("scene: " + scene_filepath)

    cfg = make_configuration(dataset_filepath, scene_filepath)

    sim = habitat_sim.Simulator(cfg)

    artic_mgr = sim.get_articulated_object_manager()
    rigid_mgr = sim.get_rigid_object_manager()

    # Load robot
    robot = artic_mgr.add_articulated_object_from_urdf(
        robot_description_path,
        fixed_base=True,
        light_setup_key=habitat_sim.gfx.DEFAULT_LIGHTING_KEY,
    )
    assert robot is not None
    dishwasher = artic_mgr.get_object_by_handle("ktc_dishwasher_:0000")

    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["rgba_camera"]._sensor_object.object

    # place agent at origin (we will never move the agent; we will only move the sensor)
    agent_node.translation = [0.0, 0.0, 0.0]
    agent_node.rotation = mn.Quaternion()

    if session["defaultCamera"]:
        cam_transform = session["defaultCamera"]
        sensor_node.translation = mn.Vector3(cam_transform["translation"])
        sensor_node.rotation = make_quaternion(cam_transform["rotation"])

    else:
        print("warning: no defaultCamera in session")
        sensor_node.translation = mn.Vector3(0.0, 0.0, 0.0)
        sensor_node.rotation = mn.Quaternion()

    local_base_pos = mn.Vector3(cam_transform["translation"]) + mn.Vector3(
        [0.4, -0.7, -0.7]
    )
    # orientation_vector = [0, 1, 0]
    # angle_correction: float = -1.56
    orientation_vector = [0.57730541, 0.57110605, 0.58357206]
    angle_correction = 4.18872291008439
    place_robot_from_agent(
        sim, robot, local_base_pos, orientation_vector, angle_correction
    )

    keyframes = session["keyframes"]
    assert keyframes

    observations = []

    def render():
        obs = sim.get_sensor_observations()
        observations.append(obs)

    def reset_robot():
        num_steps = 100
        default_joint_positions = torch.Tensor(
            [
                0.1981443464756012,
                0.6331225633621216,
                -0.09332864731550217,
                2.158449649810791,
                -0.015762098133563995,
                -1.9249799251556396,
                0.5225799679756165,
            ]
            + [0.0, 0.0, 0.0, 0.0, 0.0]
        )

        plan = JointSpaceMinJerkPlanner(
            torch.Tensor(robot.joint_positions),
            default_joint_positions,
            steps=num_steps,
            time_to_go=1.0,
        )
        joint_traj = plan.q_traj.numpy()

        for joint_pos in joint_traj:
            robot.joint_positions = joint_pos.tolist()
            render()

    ee_offset = np.array([-0.13, 0.05, 0.1])

    def move_to(des_ee_pos, obj=None, des_obj_pos=None, desired_ori=None):
        num_steps = 100

        block_plan = None
        ori_plan = None
        xyz_traj = None
        if obj is not None:
            assert des_obj_pos is not None
            block_planner = PositionMinJerkPlanner(
                torch.Tensor(vec_to_list(obj.translation)),
                torch.Tensor(des_obj_pos),
                steps=num_steps,
                time_to_go=1.0,
            )
            block_plan = block_planner.q_traj.numpy()
            xyz_traj = block_plan.copy()
            for i in range(len(xyz_traj)):
                xyz_traj[i] = (
                    np.array(vec_to_list(gfx2robot(xyz_traj[i].tolist()).translation))
                    + ee_offset
                )

            if desired_ori is not None:
                w = obj.rotation.scalar
                x, y, z = obj.rotation.vector
                start_ori_list = [x, y, z, w]

                w, x, y, z = desired_ori
                end_ori_list = [x, y, z, w]

                oris = scipy.spatial.transform.Rotation.from_quat(
                    [start_ori_list, end_ori_list]
                )

                interpolator = scipy.spatial.transform.Slerp([0, 1], oris)
                interpolated_times = np.linspace(0, 1, num_steps)
                ori_plan = interpolator(interpolated_times)
        else:
            plan = PositionMinJerkPlanner(
                forward_kinematics()[0],
                torch.Tensor(des_ee_pos),
                steps=num_steps,
                time_to_go=1.0,
            )
            xyz_traj = plan.q_traj.numpy()
        joint_positions = []
        for i, xyz in enumerate(xyz_traj):
            prev_pos = (
                joint_positions[-1]
                if len(joint_positions) > 0
                else robot.joint_positions
            )
            # if ori_plan is not None:
            #     desired_ori = ori_plan[0].inv() * ori_plan[i]
            # else:
            desired_ori = None
            joint_positions.append(
                inverse_kinematics(
                    xyz.tolist(), ik_seed=prev_pos, desired_ori=desired_ori
                )
            )

        for i, joint_position in enumerate(joint_positions):
            joint_position[-5:] = [0, 0, 0, 0, 0]  # set all fingers to 0
            if obj is not None:
                obj.translation = block_plan[i]
            if ori_plan is not None:
                x, y, z, w = ori_plan[i].as_quat()
                obj.rotation = make_quaternion([w, x, y, z])
            robot.joint_positions = joint_position

            render()

    # Eric's code which resets everything to session, including presumably camera
    # Not sure why this is necessary, but it is to make nice videos
    def magic_camera_reset():
        # note off-by-one on endFrame
        for user_action_index, user_action in enumerate(
            reversed(session["userActions"])
        ):
            print(
                "rendering "
                + str(user_action_index + 1)
                + "/"
                + str(len(session["userActions"]))
                + " actions"
            )

            for keyframe_index in range(
                user_action["startFrame"], user_action["endFrame"] + 1
            ):

                keyframe = keyframes[keyframe_index]

                for keyframe_rigid_obj in keyframe["rigidObjects"]:
                    handles = rigid_mgr.get_object_handles(keyframe_rigid_obj["name"])
                    assert len(handles)
                    rigid_obj = rigid_mgr.get_object_by_handle(handles[0])
                    print('bb', rigid_obj.root_scene_node.cumulative_bb)
                    breakpoint()
                    rigid_obj.translation = mn.Vector3(keyframe_rigid_obj["pos"])
                    rigid_obj.rotation = make_quaternion(keyframe_rigid_obj["rot"])

                for keyframe_art_obj in keyframe["articulatedObjects"]:
                    handles = artic_mgr.get_object_handles(keyframe_art_obj["name"])
                    assert len(handles)
                    art_obj = artic_mgr.get_object_by_handle(handles[0])
                    art_obj.translation = mn.Vector3(keyframe_art_obj["pos"])
                    art_obj.rotation = make_quaternion(keyframe_art_obj["rot"])
                    art_obj.joint_positions = keyframe_art_obj["joints"]

                # Use keyframe data for visualization

                # obs = sim.get_sensor_observations()
                # observations.append(obs)
                # if keyframe_index == user_action["endFrame"]:
                #     # repeat the last frame in the action to produce a pause in the video
                #     for _ in range(30):
                #         observations.append(obs)

    def gfx2robot(obj):
        if isinstance(obj, list):
            transform = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], obj + [1]]
            return robot.transformation.inverted() @ mn.Matrix4(transform)
        else:
            return robot.transformation.inverted() @ obj.transformation

    def pick_and_place(rigid_obj, final_obj_pos, desired_ori=None):
        # ee_offset = np.array([0, 0, 0])
        # old_des_ee_pos = gfx2robot_old(vec_to_list(rigid_obj.translation - robot.translation))
        des_ee_pose = gfx2robot(rigid_obj)
        des_ee_pos = vec_to_list(des_ee_pose.translation)
        des_ee_pos = (np.array(des_ee_pos) + ee_offset + np.array([0, 0, 0.2])).tolist()
        reset_robot()
        move_to(des_ee_pos)
        move_to((np.array(des_ee_pos) + np.array([0, 0, -0.2])).tolist())

        des_ee_pos = (
            np.array(des_ee_pos) + np.array([0, -0.5, 0.3]) + ee_offset
        ).tolist()
        des_obj_pos = vec_to_list(rigid_obj.translation + mn.Vector3([0, 0.3, 0.5]))
        move_to(des_ee_pos, rigid_obj, des_obj_pos, desired_ori=desired_ori)

        # old_des_ee_pos = gfx2robot_old(
        #     vec_to_list(mn.Vector3(final_obj_pos) - robot.translation)
        # )

        des_ee_pose = gfx2robot(final_obj_pos)
        des_ee_pos = vec_to_list(des_ee_pose.translation)
        des_ee_pos = (np.array(des_ee_pos) + ee_offset).tolist()

        move_to(des_ee_pos, rigid_obj, final_obj_pos)

        # des_ee_pos = (np.array(des_ee_pos) + np.array([0, 0.6, 0.0])).tolist()
        # move_to(des_ee_pos)

    if policy_path is None:  # Directly replay the demonstration actions in the session
        magic_camera_reset()
        for user_action_index, user_action in enumerate(session["userActions"]):
            if user_action["rigidObj"]:
                rigid_obj = rigid_mgr.get_object_by_handle(user_action["rigidObj"])

                obj_end_states = keyframes[user_action["endFrame"]]["rigidObjects"]
                final_obj = [
                    x for x in obj_end_states if x["name"] == user_action["rigidObj"]
                ][0]
                final_obj_pos = final_obj["pos"]

                pick_and_place(rigid_obj=rigid_obj, final_obj_pos=final_obj_pos)

    else:  # Use loaded GNN policy in the same environment
        magic_camera_reset()

        obs = keyframe_to_obs_dict(keyframes[0])
        goal_obs = keyframe_to_obs_dict(keyframes[-1])
        num_plates = len(obs["plates"])
        num_bowls = len(obs["bowls"])

        def step_obs_from_action(action_tuple, num_steps=150, desired_ori=None):
            # for each action_tuple output from `get_action_from_index`, step the environment
            # according to the action taken and add the observations to the observations list

            if action_tuple[0] == "dishwasher":
                ranges = {  # map from dishwasher index to tuple(value when closed, value when open)
                    0: (0, 0.6),
                    1: (0, -1.5),
                    2: (0, 0.6),
                }

                action_index = action_tuple[1]
                offset_range = ranges[action_index]

                obj_offset = 0.6 / num_steps
                if (
                    abs(dishwasher.joint_positions[action_index] - offset_range[0])
                    < 0.1
                ):  # ~0, when it's closed
                    joint_angles = np.linspace(
                        offset_range[0], offset_range[1], num_steps
                    )
                    if action_index == 0:
                        move_to(vec_to_list(gfx2robot(bottom_closed).translation))
                    elif action_index == 2:
                        move_to(vec_to_list(gfx2robot(top_closed).translation))

                else:  # ~ -1.5 or 0.6, when it's open
                    joint_angles = np.linspace(
                        offset_range[1], offset_range[0], num_steps
                    )
                    obj_offset *= -1

                    if action_index == 0:
                        move_to(vec_to_list(gfx2robot(bottom_open).translation))
                    elif action_index == 2:
                        move_to(vec_to_list(gfx2robot(top_open).translation))
                all_joint_angles = dishwasher.joint_positions

                for joint_angle in joint_angles:
                    all_joint_angles[action_index] = joint_angle
                    dishwasher.joint_positions = all_joint_angles

                    if action_index != 1:
                        robot.joint_positions = inverse_kinematics(
                            forward_kinematics()[0] + torch.Tensor([obj_offset, 0, 0]),
                            ik_seed=robot.joint_positions,
                        )
                        for obj_id in range(rigid_mgr.get_num_objects()):
                            obj = rigid_mgr.get_object_by_id(obj_id)

                            translation = (
                                obj.translation
                            )  # compensate objects for dishwasher tray movement
                            if (
                                action_index == 0
                                and abs(obj.translation[1] - 0.3)
                                < USE_OFFSET_DIST_THRESHOLD
                            ):
                                translation[2] += obj_offset
                            elif (
                                action_index == 2
                                and abs(obj.translation[1] - 0.56)
                                < USE_OFFSET_DIST_THRESHOLD
                            ):
                                translation[2] += obj_offset
                            obj.translation = translation
                    render()
                reset_robot()
            else:
                obj_type = action_tuple[0]

                plate_id = action_tuple[1]
                plate_id_str = str(plate_id)
                plate_id_str = "0" * (4 - len(plate_id_str)) + plate_id_str

                if obj_type == "plate":
                    obj_handle = f"frl_apartment_plate_01_:{plate_id_str}"
                elif obj_type == "bowl":
                    obj_handle = f"frl_apartment_bowl_07_small_:{plate_id_str}"
                else:
                    raise Exception(f"Unknown obj type {obj_type}")

                rigid_obj = rigid_mgr.get_object_by_handle(obj_handle)
                final_obj_pos = goal_obs[f"{obj_type}s"][plate_id][:3]
                offset = goal_offset(dishwasher.joint_positions, final_obj_pos)
                final_obj_pos[-1] += offset[-1]

                pick_and_place(
                    rigid_obj=rigid_obj,
                    final_obj_pos=final_obj_pos,
                    desired_ori=desired_ori,
                )

        policy = torch.load(policy_path).to("cpu")

        def expert_policy():
            # Open dishwasher
            step_obs_from_action(("dishwasher", 1))

            # Bowls
            step_obs_from_action(("dishwasher", 2))
            for i in range(num_bowls):
                step_obs_from_action(("bowl", i, i))
            step_obs_from_action(("dishwasher", 2))

            # Plates
            step_obs_from_action(("dishwasher", 0))
            for i in range(num_plates):
                step_obs_from_action(("bowl", i, i))

            # Close dishwasher
            step_obs_from_action(("dishwasher", 1))

        # Run expert policy:
        # expert_policy()

        # step_obs_from_action(("dishwasher", 1))
        # step_obs_from_action(("dishwasher", 0))
        # step_obs_from_action(("dishwasher", 2))

        dishwasher.joint_positions = [0, -1.5, 0]

        for i in range(num_plates + num_bowls + 4):
            # for i in range(3):
            # if i == 5:
            #     step_obs_from_action(("dishwasher", 0))
            #     step_obs_from_action(("dishwasher", 2))
            obs = sim_to_obs_dict(sim)

            data = kitchen_obs_to_graph(obs, goal_obs)
            # breakpoint()
            block_pred, goal_pred, ori_pred, dishwasher_pred = policy.forward_on_data(
                data
            )

            block_i = block_pred.argmax().item()
            goal_i = goal_pred.argmax().item()
            ori_i = ori_pred.argmax().item()
            desired_ori = discrete_rots[ori_i].tolist()
            dishwasher_i = dishwasher_pred.argmax(axis=1).item()
            print(
                f"Policy picks: block_i={block_i} goal_i={goal_i}, dishwasher_i={dishwasher_i}"
            )

            action_tuple = get_action_from_index(
                num_bowls, num_plates, block_i, goal_i, dishwasher_i
            )
            print(f"Policy's resulting action tuple: {action_tuple}")
            step_obs_from_action(action_tuple=action_tuple, desired_ori=desired_ori)
        # step_obs_from_action(("dishwasher", 1))

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera",
            "color",
            os.path.join(hydra.utils.get_original_cwd(), "out.mp4"),
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


@hydra.main(config_path="../conf", config_name="arrange_session_viewer_robot")
def main(cfg):
    policy_path = cfg.policy_path if "policy_path" in cfg else None
    session_viewer(
        show_video=cfg.show_video,
        make_video=cfg.make_video,
        session_path=cfg.session_path,
        robot_description_path=cfg.robot_description_path,
        ee_link_name=cfg.ee_link_name,
        policy_path=policy_path,
    )


if __name__ == "__main__":
    main()

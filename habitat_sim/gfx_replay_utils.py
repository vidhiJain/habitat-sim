import habitat_sim


def add_agent_user_transform(sim):
    agent = sim.get_agent(0)
    sim.gfx_replay.add_user_transform_to_keyframe(
        "agent", agent.body.object.translation, agent.body.object.rotation
    )


def set_agent_from_user_transform(player, sim):
    agent = sim.get_agent(0)
    (agent_translation, agent_rotation) = player.get_user_transform("agent")
    agent.body.object.translation = agent_translation
    agent.body.object.rotation = agent_rotation


def make_backend_configuration_for_playback(
    need_separate_semantic_scene_graph=False,
):
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "NONE"  # see Asset.h EMPTY_SCENE
    backend_cfg.force_separate_semantic_scene_graph = need_separate_semantic_scene_graph

    return backend_cfg

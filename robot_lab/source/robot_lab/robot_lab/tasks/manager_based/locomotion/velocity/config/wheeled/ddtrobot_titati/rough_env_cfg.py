# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import robot_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from robot_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    ActionsCfg,
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

##
# Pre-defined configs
##
from robot_lab.assets.titati import DDTROBOT_TITATI_CFG  # isort: skip  # 预定义的机器人资产配置（URDF/关节组/执行器等）

@configclass
class TITATIActionsCfg(ActionsCfg):
    """Action specifications for the MDP."""  # 动作空间配置：定义动作类型、缩放、裁剪、是否带默认偏置等

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[""], scale=0.25, use_default_offset=True, clip=None, preserve_order=True
    )
    # joint_pos：用“关节位置（或Δ关节位置）”作为动作分量
    # - asset_name/joint_names：应用到哪些关节（在 __post_init__ 里会被设置成 leg_joint_names）
    # - scale：动作缩放（与 rl_sar 的 action_scale 对应，hip 后面会重写为更小）
    # - use_default_offset：是否在默认姿态基础上加偏置（常见为 Δq）
    # - clip：动作裁剪（此处先置 None，__post_init__ 再统一给）
    # - preserve_order：保持关节顺序不被正则重排

    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot", joint_names=[""], scale=5.0, use_default_offset=True, clip=None, preserve_order=True
    )
    # joint_vel：用“关节速度”作为动作分量
    # - 这里用于 wheel（脚/轮）关节，scale=5.0 对应你在 rl_sar 侧脚/轮 action_scale=5.0 的设计


@configclass
class TITATIRewardsCfg(RewardsCfg):
    """Reward terms for the MDP."""  # 奖励项集合（仅新增/重写与轮子相关的几项）

    joint_vel_wheel_l2 = RewTerm(
        func=mdp.joint_vel_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )
    # 轮关节速度 L2 惩罚（默认权重 0，不启用；__post_init__ 会把 joint_names 指到 wheel_joint_names）

    joint_acc_wheel_l2 = RewTerm(
        func=mdp.joint_acc_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )
    # 轮关节加速度 L2 惩罚（默认 0）

    joint_torques_wheel_l2 = RewTerm(
        func=mdp.joint_torques_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )
    # 轮关节力矩 L2 惩罚（默认 0）


@configclass
class TITATIRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    actions: TITATIActionsCfg = TITATIActionsCfg()   # 动作配置：腿用关节位置，轮用关节速度
    rewards: TITATIRewardsCfg = TITATIRewardsCfg()   # 奖励配置：在基类上叠加/重写

    base_link_name = "base"                           # 机体基座 link 名（用于高度、姿态、扰动等）
    foot_link_name = ".*_foot"                        # 脚/轮 link 的正则（用于接触/传感器/奖励等）

    # fmt: off
    leg_joint_names = [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ]
    wheel_joint_names = [
        "FR_foot_joint", "FL_foot_joint", "RR_foot_joint", "RL_foot_joint",
    ]
    joint_names = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FR_foot_joint",
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FL_foot_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RR_foot_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_foot_joint"]  # 关节总列表（腿 + 轮），顺序与训练/部署需一致
    # fmt: on

    def __post_init__(self):
        # post init of parent
        super().__post_init__()  # 调用父类后处理：初始化场景/观察/奖励/事件等基础结构

        # ------------------------------Sence------------------------------
        self.scene.robot = DDTROBOT_TITATI_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # 载入机器人资产：把 prim_path 放到每个并行环境的 Robot 节点下

        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        self.scene.height_scanner_base.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        # 高度扫描器附在 base 上（用于 rough 地形观测/奖励；这里后面关闭了 height_scan 观测）

        # ------------------------------Observations------------------------------
        self.observations.policy.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.policy.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        # 策略观测中的 joint_pos：使用“相对默认姿态，但不包含 wheel 关节”的函数
        # 通过 wheel_asset_cfg 告诉函数哪些是轮关节，便于剔除

        self.observations.critic.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.critic.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        # critic（价值函数）观测与 policy 保持一致，确保输入分布一致

        self.observations.policy.base_lin_vel.scale = 2.0   # 与 rl_sar config.yaml 的 lin_vel_scale 对齐
        self.observations.policy.base_ang_vel.scale = 0.25  # 与 rl_sar config.yaml 的 ang_vel_scale 对齐
        self.observations.policy.joint_pos.scale = 1.0      # 关节角观测缩放（与部署一致）
        self.observations.policy.joint_vel.scale = 0.05     # 关节角速度观测缩放（与部署一致）
        self.observations.policy.base_lin_vel = None        # 关闭线速度观测（该任务选择不用）
        self.observations.policy.height_scan = None         # 关闭高度扫描（rough 里可以选开/关，这里关）
        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names
        # 指定 joint_pos/vel 观测作用于所有关节（但 joint_pos 的函数内部会把 wheel 剔除）

        # ------------------------------Actions------------------------------
        # reduce action scale
        self.actions.joint_pos.scale = {".*_hip_joint": 0.125, "^(?!.*_hip_joint).*": 0.25}
        # 动作缩放：髋关节更小（0.125），其余腿关节 0.25 —— 与 rl_sar 的 action_scale 前 12 项匹配

        self.actions.joint_vel.scale = 5.0
        # 轮关节速度动作的缩放（5.0）—— 与 rl_sar 的后 4 项动作缩放匹配

        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}
        self.actions.joint_vel.clip = {".*": (-100.0, 100.0)}
        # 动作裁剪（±100，等效于基本不裁剪；部署端也设置了相同的 clip）

        self.actions.joint_pos.joint_names = self.leg_joint_names
        self.actions.joint_vel.joint_names = self.wheel_joint_names
        # 动作分配：腿关节 ← 位置动作；轮关节 ← 速度动作（与 rl_sar FSM 的 q/dq 路径一致）

        # ------------------------------Events------------------------------
        self.events.randomize_reset_base.params = {
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.2),
                "roll": (-3.14, 3.14),
                "pitch": (-3.14, 3.14),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        }
        # 随机重置：位姿与速度的范围（提升鲁棒性）

        self.events.randomize_rigid_body_mass_base.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_rigid_body_mass_others.params["asset_cfg"].body_names = [
            f"^(?!.*{self.base_link_name}).*"
        ]
        self.events.randomize_com_positions.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_apply_external_force_torque.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_apply_external_force_torque.params["force_range"] = (-30.0, 30.0)
        self.events.randomize_apply_external_force_torque.params["torque_range"] = (-10.0, 10.0)
        # 质量、质心、外力/外力矩等 domain randomization，模拟真实误差与扰动

        # ------------------------------Rewards------------------------------
        # General
        self.rewards.is_terminated.weight = 0    # 是否给“终止奖励”（这里 0=不使用）

        # Root penalties
        self.rewards.lin_vel_z_l2.weight = -2.0            # 垂直线速度惩罚（鼓励少跳动）
        self.rewards.ang_vel_xy_l2.weight = -0.05          # 横滚/俯仰角速度惩罚（抑制晃动）
        self.rewards.flat_orientation_l2.weight = 0        # 水平姿态惩罚（关闭）
        self.rewards.base_height_l2.weight = 0             # 基座高度偏差惩罚（关闭）
        self.rewards.base_height_l2.params["target_height"] = 0.60
        self.rewards.base_height_l2.params["asset_cfg"].body_names = [self.base_link_name]
        self.rewards.body_lin_acc_l2.weight = 0            # 基座线加速度惩罚（关闭）
        self.rewards.body_lin_acc_l2.params["asset_cfg"].body_names = [self.base_link_name]

        # Joint penalties
        self.rewards.joint_torques_l2.weight = -1e-5
        self.rewards.joint_torques_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        # 腿关节力矩 L2 惩罚（鼓励省力）

        self.rewards.joint_torques_wheel_l2.weight = 0
        self.rewards.joint_torques_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        # 轮力矩惩罚（默认 0）

        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_vel_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_vel_wheel_l2.weight = 0
        self.rewards.joint_vel_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        # 关节速度惩罚（这里默认 0）

        self.rewards.joint_acc_l2.weight = -1e-7
        self.rewards.joint_acc_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_acc_wheel_l2.weight = -2.5e-10
        self.rewards.joint_acc_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        # 关节加速度惩罚（平滑动作；轮子的权重更小）

        # self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_hip_l1", -0.2, [".*_hip_joint"])
        # 可选：髋关节偏差 L1 惩罚（关闭示例）

        self.rewards.joint_pos_limits.weight = -5.0
        self.rewards.joint_pos_limits.params["asset_cfg"].joint_names = self.leg_joint_names
        # 关节角越界惩罚（腿）

        self.rewards.joint_vel_limits.weight = 0
        self.rewards.joint_vel_limits.params["asset_cfg"].joint_names = self.wheel_joint_names
        # 关节速度越界惩罚（轮，默认 0）

        self.rewards.joint_power.weight = -1e-5
        self.rewards.joint_power.params["asset_cfg"].joint_names = self.leg_joint_names
        # 功率惩罚（鼓励省电）

        self.rewards.stand_still.weight = -2.0
        self.rewards.stand_still.params["asset_cfg"].joint_names = self.leg_joint_names
        # 非运动状态下 stillness 惩罚（防止乱动）

        self.rewards.joint_pos_penalty.weight = -1.0
        self.rewards.joint_pos_penalty.params["asset_cfg"].joint_names = self.leg_joint_names
        # 偏离名义姿态的惩罚（腿）

        self.rewards.wheel_vel_penalty.weight = 0
        self.rewards.wheel_vel_penalty.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.wheel_vel_penalty.params["asset_cfg"].joint_names = self.wheel_joint_names
        # 轮速度惩罚（默认 0；可用于限制轮滑）

        self.rewards.joint_mirror.weight = -0.05
        self.rewards.joint_mirror.params["mirror_joints"] = [
            ["FR_(hip|thigh|calf).*", "RL_(hip|thigh|calf).*"],
            ["FL_(hip|thigh|calf).*", "RR_(hip|thigh|calf).*"],
        ]
        # 镜像约束：鼓励对角腿对称性，提升步态稳定

        # Action penalties
        self.rewards.action_rate_l2.weight = -0.01  # 动作变化率惩罚（抑制抖动/忽快忽慢）

        # Contact sensor
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]
        # 非足部的接触惩罚（例如肚子、髋部撞地）

        self.rewards.contact_forces.weight = -1.5e-4
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [self.foot_link_name]
        # 足底接触力惩罚（防止过猛着地）

        # Velocity-tracking rewards
        self.rewards.track_lin_vel_xy_exp.weight = 3.0   # 追踪水平线速度（x,y）
        self.rewards.track_ang_vel_z_exp.weight = 1.5    # 追踪航向角速度（yaw）

        # Others
        self.rewards.feet_air_time.weight = 0
        self.rewards.feet_air_time.params["threshold"] = 0.5
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact.weight = 0
        self.rewards.feet_contact.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact_without_cmd.weight = 0.1
        self.rewards.feet_contact_without_cmd.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_stumble.weight = 0
        self.rewards.feet_stumble.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.weight = 0
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height.weight = 0
        self.rewards.feet_height.params["target_height"] = 0.1
        self.rewards.feet_height.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height_body.weight = 0
        self.rewards.feet_height_body.params["target_height"] = -0.4
        self.rewards.feet_height_body.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_gait.weight = 0
        self.rewards.feet_gait.params["synced_feet_pair_names"] = (("FL_foot", "RR_foot"), ("FR_foot", "RL_foot"))
        self.rewards.upward.weight = 3.0  # “抬头/向上”奖励（鼓励机体姿态更直立/不趴）

        # If the weight of rewards is 0, set rewards to None
        if self.__class__.__name__ == "TITATIRoughEnvCfg":
            self.disable_zero_weight_rewards()
        # 把权重为 0 的奖励项自动禁用（减少计算，防止无用项干扰）

        # ------------------------------Terminations------------------------------
        # self.terminations.illegal_contact.params["sensor_cfg"].body_names = [self.base_link_name, ".*_hip"]
        self.terminations.illegal_contact = None
        # 终止条件：非法接触（这里关闭，选择更“宽容”的训练，靠惩罚而非立即终止）

        # ------------------------------Curriculums------------------------------
        # self.curriculum.command_levels.params["range_multiplier"] = (0.2, 1.0)
        self.curriculum.command_levels = None
        # 课程学习：命令难度分级（此处关闭；若开启可在早期降低指令范围）

        # ------------------------------Commands------------------------------
        # self.commands.base_velocity.ranges.lin_vel_x = (-2.0, 2.0)
        # self.commands.base_velocity.ranges.lin_vel_y = (-2.0, 2.0)
        # self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)
        # 命令范围可按需放开/收紧；与部署端 commands_scale 一起决定“策略看到的命令分布”

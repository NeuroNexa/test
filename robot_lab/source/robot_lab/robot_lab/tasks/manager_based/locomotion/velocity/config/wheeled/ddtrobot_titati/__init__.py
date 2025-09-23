# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import gymnasium as gym

from . import agents  # 导入该目录下的 agents 包（存放 RL 算法配置，如 PPO 参数）

##
# Register Gym environments.
##  # 向 gym 注册环境，后续在训练/测试时直接用 gym.make("ID") 创建

gym.register(
    id="RobotLab-Isaac-Velocity-Flat-TITATI-v0",  # 环境 ID（平地）
    entry_point="isaaclab.envs:ManagerBasedRLEnv",  # 环境入口类：isaaclab 提供的 ManagerBasedRLEnv
    disable_env_checker=True,  # 关闭 gym 的环境检查（避免额外报错）
    kwargs={
        # 环境配置入口（指向 flat_env_cfg.TITATIFlatEnvCfg）
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:TITATIFlatEnvCfg",
        # RSL-RL 框架的 PPO Runner 配置（定义学习率、网络结构、采样步数等）
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:TITATIFlatPPORunnerCfg",
        # 自定义 RL 框架的 PPO Trainer 配置（比如 curriculum、训练超参）
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:TITATIFlatTrainerCfg",
    },
)

gym.register(
    id="RobotLab-Isaac-Velocity-Rough-TITATI-v0",  # 环境 ID（崎岖地形）
    entry_point="isaaclab.envs:ManagerBasedRLEnv",  # 同样是 ManagerBasedRLEnv
    disable_env_checker=True,
    kwargs={
        # 环境配置入口（指向 rough_env_cfg.TITATIRoughEnvCfg）
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:TITATIRoughEnvCfg",
        # RSL-RL 框架的 PPO Runner 配置
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:TITATIRoughPPORunnerCfg",
        # 自定义 RL 框架的 PPO Trainer 配置
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:TITATIRoughTrainerCfg",
    },
)

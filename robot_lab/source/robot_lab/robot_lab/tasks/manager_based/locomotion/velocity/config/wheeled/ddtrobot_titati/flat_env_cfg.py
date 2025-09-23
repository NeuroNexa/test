# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from .rough_env_cfg import TITATIRoughEnvCfg  # 继承“崎岖地形”配置，在其基础上做“平地”简化/覆盖


@configclass
class TITATIFlatEnvCfg(TITATIRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()  # 先调用父类（rough）把通用场景/观测/奖励/事件等全部初始化好

        # override rewards
        self.rewards.base_height_l2.params["sensor_cfg"] = None
        # 覆盖奖励：基座高度偏差项不再依赖额外的传感器配置（平地环境通常不需要复杂高度感知）

        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # 改为“无限平面”地形；移除地形生成器（不再采样坑洼/起伏）

        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None
        # 关闭高度扫描器与其观测（平地上不需要“看地形高度”）

        # no terrain curriculum
        self.curriculum.terrain_levels = None
        # 关闭地形难度的课程学习（平地不分等级）

        # If the weight of rewards is 0, set rewards to None
        if self.__class__.__name__ == "TITATIFlatEnvCfg":
            self.disable_zero_weight_rewards()
        # 把权重为 0 的奖励项禁用，减少无效计算

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import cusrl
from cusrl.environment.isaaclab import TrainerCfg
from isaaclab.utils import configclass


@configclass
class TITATIRoughTrainerCfg(TrainerCfg):
    # 训练配置（崎岖地形）
    max_iterations = 20000             # 最大迭代次数（rough terrain 收敛慢，需要更多训练）
    save_interval = 100                # 每隔多少迭代保存一次模型
    experiment_name = "titati_rough"   # 实验名，用于区分保存目录/日志

    # 定义一个 Actor-Critic 算法的工厂
    agent_factory = cusrl.ActorCritic.Factory(
        num_steps_per_update=24,       # 每个环境每次 rollout 的步数
        # Actor（策略网络）
        actor_factory=cusrl.Actor.Factory(
            backbone_factory=cusrl.Mlp.Factory(
                hidden_dims=[512, 256, 128],  # MLP 隐藏层结构
                activation_fn="ELU",          # 激活函数
                ends_with_activation=True     # 最后一层是否接激活函数
            ),
            distribution_factory=cusrl.NormalDist.Factory(), # 动作分布：高斯分布（连续动作空间）
        ),
        # Critic（价值函数网络）
        critic_factory=cusrl.Value.Factory(
            backbone_factory=cusrl.Mlp.Factory(
                hidden_dims=[512, 256, 128],  # 网络结构与 Actor 相同
                activation_fn="ELU",
                ends_with_activation=True
            ),
        ),
        optimizer_factory=cusrl.OptimizerFactory("AdamW", defaults={"lr": 1.0e-3}),
        # 优化器：AdamW，学习率 1e-3

        sampler=cusrl.AutoMiniBatchSampler(num_epochs=5, num_mini_batches=4),
        # 自动 mini-batch 切分：每个 batch 重复训练 5 个 epoch，每次分 4 份

        # 训练 hooks（训练循环中自动调用的“插件”）
        hooks=[
            cusrl.hook.ValueComputation(),                         # 计算价值函数
            cusrl.hook.GeneralizedAdvantageEstimation(gamma=0.99, lamda=0.95), # GAE 优势估计
            cusrl.hook.AdvantageNormalization(),                   # 优势归一化（稳定训练）
            cusrl.hook.ValueLoss(),                                # value 损失
            cusrl.hook.OnPolicyPreparation(),                      # 准备 On-policy 数据
            cusrl.hook.PpoSurrogateLoss(),                         # PPO surrogate loss
            cusrl.hook.EntropyLoss(weight=0.008),                  # 策略熵损失（鼓励探索）
            cusrl.hook.GradientClipping(max_grad_norm=1.0),        # 梯度裁剪（防止爆炸）
            cusrl.hook.OnPolicyStatistics(sampler=cusrl.AutoMiniBatchSampler()), # 统计信息
            cusrl.hook.AdaptiveLRSchedule(desired_kl_divergence=0.01),           # 自适应学习率（基于 KL）
        ],
    )


@configclass
class TITATIFlatTrainerCfg(TITATIRoughTrainerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.max_iterations = 5000         # 平地任务训练更快，迭代数少
        self.experiment_name = "titati_flat" # 实验名区分开（避免覆盖）

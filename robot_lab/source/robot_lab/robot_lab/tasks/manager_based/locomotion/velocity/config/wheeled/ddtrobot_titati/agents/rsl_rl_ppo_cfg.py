# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class TITATIRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # Runner 配置：决定训练流程（采样步长、总迭代次数、保存频率等）
    num_steps_per_env = 24          # 每个环境每次 rollout 的步数（24*并行环境数 = batch 大小）
    max_iterations = 20000          # 最大迭代次数（rough terrain 任务需要更多迭代）
    save_interval = 100             # 每隔多少迭代保存一次 checkpoint
    experiment_name = "titati_rough" # 实验名，用于保存日志和模型文件夹

    # 策略（Actor-Critic 网络结构和初始化）
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,               # 动作初始化时的高斯噪声标准差（鼓励探索）
        actor_obs_normalization=False,    # actor 输入是否做标准化（这里关闭）
        critic_obs_normalization=False,   # critic 输入是否做标准化（这里关闭）
        actor_hidden_dims=[512, 256, 128],# actor MLP 网络层数与每层维度
        critic_hidden_dims=[512, 256, 128],# critic MLP 网络层数与每层维度
        activation="elu",                 # 激活函数类型（ELU：平滑，梯度不易消失）
    )

    # 算法（PPO 的核心超参数）
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,        # value function loss 权重
        use_clipped_value_loss=True,# 是否使用 clipped value loss（稳定训练）
        clip_param=0.2,             # PPO 的 clipping ratio ε（控制更新步长）
        entropy_coef=0.01,          # 策略熵的系数（鼓励探索）
        num_learning_epochs=5,      # 每次采样后更新 epoch 数（同一 batch 用多少次）
        num_mini_batches=4,         # 将 batch 切分成多少个 mini-batch 进行更新
        learning_rate=1.0e-3,       # 初始学习率
        schedule="adaptive",        # 学习率调度方式（这里自适应，跟踪 KL）
        gamma=0.99,                 # 折扣因子（未来奖励折扣）
        lam=0.95,                   # GAE(lambda) 衰减系数（平衡偏差/方差）
        desired_kl=0.01,            # 目标 KL 散度（控制策略更新幅度）
        max_grad_norm=1.0,          # 梯度裁剪阈值，防止梯度爆炸
    )


@configclass
class TITATIFlatPPORunnerCfg(TITATIRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 5000         # 平地任务更简单，迭代次数少
        self.experiment_name = "titati_flat" # 保存目录使用不同名字，避免覆盖

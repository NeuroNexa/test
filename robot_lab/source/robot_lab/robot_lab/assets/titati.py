# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Configuration for DDT robots.
Reference: https://github.com/DDTRobot
"""

import isaaclab.sim as sim_utils
# 导入 isaaclab 的仿真工具模块，别名为 sim_utils

from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg  # noqa: F401
# 从 isaaclab.actuators 模块中导入直流电机配置 (DCMotorCfg) 和隐式执行器配置 (ImplicitActuatorCfg)

from isaaclab.assets.articulation import ArticulationCfg
# 从 isaaclab.assets.articulation 模块中导入关节配置 (ArticulationCfg)，
# 用于描述机器人本体结构和运动学属性

from robot_lab.assets import ISAACLAB_ASSETS_DATA_DIR
# 从 robot_lab.assets 导入资源数据路径常量 ISAACLAB_ASSETS_DATA_DIR，
# 用于指定 urdf 文件所在的目录

##
# Configuration
##
# 配置部分

DDTROBOT_TITATI_CFG = ArticulationCfg(
    # 定义名为 DDTROBOT_TITATI_CFG 的配置，使用 ArticulationCfg 进行构造
    spawn=sim_utils.UrdfFileCfg(
        # 配置机器人从 urdf 文件中生成的参数
        fix_base=False,
        # 机器人基座是否固定在地面，False 表示不固定（机器人可以自由运动）

        merge_fixed_joints=True,
        # 将固定关节进行合并以简化模型

        replace_cylinders_with_capsules=False,
        # 是否将圆柱体替换为胶囊体，这里设为 False，保持原始几何形状

        asset_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/ddt/titati_description/urdf/titati.urdf",
        # urdf 文件路径，指定机器人结构定义文件的位置

        activate_contact_sensors=True,
        # 激活接触传感器，用于检测碰撞与接触信息

        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # 刚体属性配置
            disable_gravity=False,
            # 是否禁用重力，False 表示启用重力

            retain_accelerations=False,
            # 是否保留加速度数据，False 表示不保留（节省计算开销）

            linear_damping=0.0,
            # 线性阻尼，影响速度衰减，这里设为 0 表示无衰减

            angular_damping=0.0,
            # 角阻尼，影响旋转速度衰减，这里设为 0 表示无衰减

            max_linear_velocity=1000.0,
            # 最大线速度限制，单位 m/s，这里设置为 1000，基本无限制

            max_angular_velocity=1000.0,
            # 最大角速度限制，单位 rad/s，这里设置为 1000，基本无限制

            max_depenetration_velocity=1.0,
            # 最大去穿透速度（防止刚体相互穿透后弹出过快）
        ),

        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # 关节系统的根节点属性配置
            enabled_self_collisions=False,
            # 是否启用自碰撞检测，False 表示禁用（提升效率）

            solver_position_iteration_count=4,
            # 物理求解器的位置迭代次数，影响精度与稳定性，这里为 4

            solver_velocity_iteration_count=0
            # 物理求解器的速度迭代次数，这里为 0（不额外迭代）
        ),

        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            # 关节驱动配置
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                # PD 控制增益配置
                stiffness=0,
                # 刚度系数（比例增益），设为 0 表示不使用刚度控制

                damping=0
                # 阻尼系数（微分增益），设为 0 表示不使用阻尼控制
            )
        ),
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        # 初始状态配置
        pos=(0.0, 0.0, 0.35),
        # 机器人初始位置 (x, y, z)，放置在 0.3 米的高度

        joint_pos={
            # 初始关节角度配置，使用正则表达式匹配关节名称
            ".*hip_joint": 0.0,
            # 髋关节初始角度为 0

            "FR_thigh_joint": 0.40,
            "FR_calf_joint": -0.917,

            "FL_thigh_joint": 0.40,
            "FL_calf_joint": -0.917,

            "RR_thigh_joint": -0.40,
            "RR_calf_joint": 0.917,

            "RL_thigh_joint": -0.40,
            "RL_calf_joint": 0.917,
            

            # "F.*_thigh_joint": 0.80,
            # # 前腿的大腿关节设为 0.8 弧度

            # "R.*_thigh_joint": -0.80,
            # # 后腿的大腿关节设为 -0.8 弧度

            # "F.*_calf_joint": -1.50,
            # # 前腿的小腿关节设为 -1.5 弧度

            # "R.*_calf_joint": 1.50,
            # # 后腿的小腿关节设为 1.5 弧度

            ".*_foot_joint": 0.0,
            # 脚关节设为 0
        },

        joint_vel={".*": 0.0},
        # 所有关节初始速度设为 0
    ),

    soft_joint_pos_limit_factor=0.9,
    # 关节位置软限制因子，设置为物理范围的 90%

    actuators={
        # 执行器配置，定义不同关节组所使用的电机参数        
        "hip": DCMotorCfg(
            # 髋关节执行器配置
            joint_names_expr=[".*_hip_joint"],
            # 匹配的关节名称，这里是左右两侧的第 1 个关节（髋关节）

            effort_limit=85.0,
            # 最大驱动力/力矩限制 (Nm)

            saturation_effort=85.0,
            # 电机饱和力矩，与 effort_limit 相同，用于避免过载

            velocity_limit=20.0,
            # 最大关节速度限制 (rad/s)

            stiffness=40.0,
            # 电机刚度（比例增益），越大电机越“硬”，响应更快

            damping=1.0,
            # 电机阻尼（微分增益），用于抑制振动和过冲

            friction=0.0,
            # 摩擦系数，设为 0 表示不考虑摩擦
        ),

        "thigh": DCMotorCfg(
            # 大腿关节执行器配置
            joint_names_expr=[".*_thigh_joint"],
            # 左右两条腿的第 2 个关节（大腿）

            effort_limit=85.0,
            # 最大力矩限制

            saturation_effort=85.0,
            # 力矩饱和限制

            velocity_limit=20.0,
            # 最大角速度

            stiffness=40.0,
            # 刚度系数

            damping=1.0,
            # 阻尼系数

            friction=0.0,
            # 摩擦系数
        ),

        "calf": DCMotorCfg(
            # 小腿关节执行器配置
            joint_names_expr=[".*_calf_joint"],
            # 左右两条腿的第 3 个关节（小腿）

            effort_limit=85.0,
            # 最大力矩

            saturation_effort=85.0,
            # 力矩饱和限制

            velocity_limit=20.0,
            # 最大角速度

            stiffness=40.0,
            # 刚度系数

            damping=1.0,
            # 阻尼系数

            friction=0.0,
            # 摩擦系数
        ),

        "wheel": DCMotorCfg(
            # 轮子关节执行器配置
            joint_names_expr=[".*_foot_joint"],
            # 使用正则匹配所有“脚部关节”，视作轮子驱动关节

            effort_limit=7.5,
            # 最大力矩限制较小（15 Nm），比腿部小

            saturation_effort=7.5,
            # 力矩饱和限制

            velocity_limit=26.0,
            # 最大角速度 (rad/s)，比腿部低一些

            stiffness=0.0,
            # 刚度系数设为 0，表示不进行位置刚性控制

            damping=1.0,
            # 阻尼系数，避免过快运动导致振荡

            friction=0.0,
            # 摩擦系数
        ),
    },

)
# 结束配置定义

"""Configuration of DDT TITATI  using DC motor.
"""
# 使用直流电机的 DDT TITATI 配置说明
RoboMaster 机甲大师机器人的“小陀螺”（Top Mode）功能是机甲控制算法中非常经典且具有挑战性的模块。它的核心在于**坐标系变换**和**闭环控制的解耦**。

简单来说，你需要同时做三件事：

1. **底盘自旋**：让底盘以设定的速度（如 $6 \text{rad/s}$）旋转。
2. **云台稳像**：无论底盘怎么转，云台要始终指向操作手想要的方向（绝对坐标系不动）。
3. **运动解耦**：当底盘在高速旋转时，操作手推“前”，机器人应该相对于云台的方向向前移动，而不是跟着底盘乱跑。

以下是基于你现有传感器和控制量的详细系统设计方案。

---

### 一、 系统架构与坐标系定义

首先，必须明确两个核心坐标系：

* **云台坐标系 (Gimbal Frame, $G$)**：以云台朝向为X轴正方向。操作手的“前后左右”指令是基于这个坐标系的。
* **底盘坐标系 (Chassis Frame, $C$)**：以底盘麦克纳姆轮的几何中心朝向为参考。底盘电机执行的是这个坐标系的指令。

**关键变量：**

* $\theta_{rel}$：云台与底盘的相对角度（即 **云台Yaw电机编码器位置**）。
* 定义：$\theta_{rel} = \theta_{gimbal} - \theta_{chassis}$ （需根据实际安装方向校准正负）。



---

### 二、 控制系统详细设计

我们将系统分为三个并行的控制闭环：**云台姿态环**、**底盘速度环**、**底盘运动合成**。

#### 1. 云台姿态控制（稳像）

**目标**：保持云台在世界坐标系下的Yaw角不变，抵消底盘旋转带来的干扰。

* **输入**：目标云台世界角度 $\theta_{target}$，当前云台世界角度 $\theta_{cur}$（通常由磁力计+陀螺仪融合得到，或者仅用陀螺仪积分）。
* **传感器**：云台陀螺仪Yaw角速度 $\omega_{gimbal}$。
* **控制量**：云台Yaw关节电机位置（这是你提供的接口，实际上如果是位置环接口，响应可能不够快，建议做前馈）。

**控制逻辑 (串级PID)**：
由于底盘在剧烈旋转，单纯的位置环可能跟不上，导致云台“被带着跑”。

1. **位置环（外环）**：

$$\text{Error}_{pos} = \theta_{target} - \theta_{cur}$$


$$\omega_{cmd} = K_{p\_pos} \cdot \text{Error}_{pos}$$


2. **速度环（内环）** + **前馈（关键）**：
你需要控制的是电机相对于底盘的位置，但电机需要抵消底盘的旋转。

$$\text{Target\_Speed}_{motor} = \omega_{cmd} - \omega_{chassis}$$


* $\omega_{chassis}$ 是底盘的角速度。**这是前馈项**。如果底盘以 300°/s 向左转，电机必须立刻以 300°/s 向右转，才能保持云台不动。


3. **输出转换**：
如果你只能控制“电机位置”，你需要将计算出的速度积分成下一时刻的目标位置发送给电调：

$$\text{Pos}_{cmd}(t) = \text{Pos}_{cmd}(t-1) + \text{Target\_Speed}_{motor} \cdot \Delta t$$



#### 2. 底盘自旋控制

**目标**：让底盘维持在一个恒定的角速度旋转。

* **输入**：小陀螺目标转速 $\omega_{spin\_set}$ (例如 300 rpm)。
* **传感器**：底盘陀螺仪 Yaw 角速度 $\omega_{chassis}$。
* **控制量**：底盘 Yaw 旋转速度 $V_{\omega}$。

**控制逻辑 (单级或增量式PID)**：


$$V_{\omega} = \text{PID}(\omega_{spin\_set} - \omega_{chassis})$$

* 注意：为了保持平滑，可以使用**斜波发生器**（Ramp）让设定速度缓慢增加到最大值，避免瞬间启动导致电流过载或丢步。

#### 3. 底盘平移运动解耦（核心算法）

**目标**：将操作手基于云台视角的速度指令 ($V_{gx}, V_{gy}$)，转换为旋转中的底盘视角指令 ($V_{cx}, V_{cy}$)。

* **输入**：
* $V_{gx}, V_{gy}$：遥控器输入的平移速度（相对于云台）。
* $\theta_{rel}$：云台电机编码器角度（底盘与云台的夹角）。


* **数学模型（旋转矩阵）**：
我们需要将速度向量从云台坐标系旋转 $-\theta_{rel}$ 到底盘坐标系。
$$\begin{bmatrix}
V_{cx} \\
V_{cy}
\end{bmatrix}
=
\begin{bmatrix}
\cos(-\theta_{rel}) & -\sin(-\theta_{rel}) \\
\sin(-\theta_{rel}) & \cos(-\theta_{rel})
\end{bmatrix}
\begin{bmatrix}
V_{gx} \\
V_{gy}
\end{bmatrix}$$


简化后（利用三角函数性质 $\cos(-x)=\cos x, \sin(-x)=-\sin x$）：

$$V_{cx} = V_{gx} \cos(\theta_{rel}) + V_{gy} \sin(\theta_{rel})$$


$$V_{cy} = -V_{gx} \sin(\theta_{rel}) + V_{gy} \cos(\theta_{rel})$$


* **最终底盘输出**：
将解耦后的平移速度与自旋速度叠加：
* 底盘 X 轴输出 = $V_{cx}$
* 底盘 Y 轴输出 = $V_{cy}$
* 底盘 Yaw 输出 = $V_{\omega}$ (来自上面的自旋控制)


然后通过麦克纳姆轮的运动学解算（通常底盘MCU内部会处理，或者你需要手动解算到4个轮子速度）发送给电机。

---

### 三、 关键信号处理与滤波方法

在小陀螺模式下，传感器噪声会被放大，且角度跳变是常见Bug来源。

#### 1. 角度归一化 (Angle Normalization)

编码器角度通常是 0~8191 或者 -180~180。当底盘旋转经过 180 度边界时，数值会突变（例如从 179 跳到 -179）。如果不处理，PID控制器会计算出一个巨大的误差，导致机器人抽搐。
**方法**：在计算 $\sin$ 和 $\cos$ 以及 PID 误差时，必须处理过零点。

```c
float Loop_Error_Handler(float set, float get) {
    float err = set - get;
    // 将误差限制在 -PI 到 PI 之间
    if (err > PI) err -= 2 * PI;
    if (err < -PI) err += 2 * PI;
    return err;
}

```

#### 2. 低通滤波 (Low Pass Filter)

底盘和云台的陀螺仪角速度数据通常会有高频噪声，直接用于微分（D项）或前馈会引起震荡。
**方法**：使用一阶低通滤波器（IIR）。


$$Y_{n} = \alpha \cdot X_{n} + (1 - \alpha) \cdot Y_{n-1}$$

* $X_n$: 当前传感器读数
* $Y_n$: 滤波后输出
* $\alpha$: 滤波系数 (0~1)，越小越平滑但延迟越大。推荐 0.1~0.3。

#### 3. 陷波滤波器 (Notch Filter) [进阶]

如果你的机械结构在特定转速下有共振（导致陀螺仪数据在该频率激增），可以使用陷波滤波器滤除特定频率的噪声。但这通常在调试后期才考虑。

#### 4. 卡尔曼滤波 (Kalman Filter) [进阶]

如果你发现单纯用陀螺仪积分得到的云台角度 $\theta_{cur}$ 漂移严重（时间久了云台歪了），需要融合加速度计（修正俯仰/横滚）和磁力计（修正Yaw）。最简单的是用 **互补滤波** 融合陀螺仪和磁力计。

---

### 四、 代码实现伪代码 (C语言风格)

```c
// 定义结构体和变量
float chassis_yaw_speed_ref = 6.0; // 底盘目标自旋速度 rad/s
float gimbal_yaw_angle_ref = 0.0;  // 云台目标指向 (世界坐标系)
float remote_vx, remote_vy;        // 遥控器输入

void Robot_Control_Loop() {
    // 1. 获取传感器数据
    float chassis_gyro_speed = Get_Chassis_Gyro();
    float gimbal_gyro_speed = Get_Gimbal_Gyro();
    float gimbal_encoder_angle = Get_Gimbal_Encoder_Angle(); // 归一化到 -PI ~ PI
    float gimbal_world_angle = Get_Gimbal_IMU_Angle();

    // ===========================
    // 模块 A: 云台稳像控制
    // ===========================
    float gimbal_pos_error = Loop_Error_Handler(gimbal_yaw_angle_ref, gimbal_world_angle);

    // 外环：位置环
    float gimbal_speed_target = PID_Calc(&gimbal_pos_pid, gimbal_pos_error);

    // 前馈：这一步极其重要！让云台主动抵消底盘的旋转
    // 目标电机速度 = 想要的世界速度 - 底盘的世界速度
    // 注意方向：如果底盘左转，电机要右转，根据实际电机安装方向调整符号
    float motor_feedforward = -chassis_gyro_speed; 

    float final_motor_speed = gimbal_speed_target + motor_feedforward;

    // 因为你的接口是位置，这里做积分转换 (V = dx/dt -> x += v*dt)
    float motor_target_pos = Get_Current_Motor_Pos() + final_motor_speed * DT;
    Set_Gimbal_Motor_Position(motor_target_pos);


    // ===========================
    // 模块 B: 底盘运动合成
    // ===========================
    
    // 1. 自旋速度 PID
    float spin_output = PID_Calc(&chassis_spin_pid, chassis_yaw_speed_ref - chassis_gyro_speed);

    // 2. 向量旋转 (平移解耦)
    // sin 和 cos 的输入必须是弧度
    float cos_theta = cos(gimbal_encoder_angle);
    float sin_theta = sin(gimbal_encoder_angle);

    // 旋转矩阵应用
    float chassis_vx = remote_vx * cos_theta + remote_vy * sin_theta;
    float chassis_vy = -remote_vx * sin_theta + remote_vy * cos_theta;

    // 3. 最终输出给底盘
    // 叠加平移速度和旋转速度
    Set_Chassis_Velocity(chassis_vx, chassis_vy, spin_output);
}

```

### 五、 调试建议与避坑指南

1. **确定 $\theta_{rel}$ 的零点**：
* 一定要校准编码器。通常将云台正对底盘车头时的编码器值设为0（或记录偏置量）。


2. **正负号判断**：
* 推导公式时的正负号依赖于电机安装方向和传感器坐标系定义。
* **测试方法**：
1. 锁死底盘不转。推前，看车是否向前。
2. 让底盘慢速旋转。推前，看车是否依然大致沿直线向前。如果车走出了一个圆或者螺旋线，说明旋转矩阵的 `sin` `cos` 符号反了，或者 $\theta_{rel}$ 的更新频率太低/延迟太大。




3. **云台的“硬”度**：
* 小陀螺模式下，云台电机的刚性（Stiffness）要求很高。PID的 $P$ 和 $D$ 需要给得比普通模式大，否则底盘转起来云台会被“甩”歪。


4. **功率限制**：
* 底盘旋转加平移非常耗电。需要加入**功率限制算法**，如果总功率超过上限（如80W），优先保证底盘旋转（防止失稳）还是优先保证平移，通常策略是等比例衰减 $V_x, V_y$。



希望这套方案能帮到你！如果有具体的硬件型号（如C型开发板或GM6020电机），可以补充细节，我再做针对性调整。
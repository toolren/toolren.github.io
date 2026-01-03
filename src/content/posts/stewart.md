---
title: 【学习笔记“Stewart平台的运动控制仿真”】
date: 2025-10-28
summary: stewart并联机器人ROS控制与仿真
category: 学习笔记
tags: [逐晖计划]
sticky: 1
---

# 前言

本学习笔记为个人学习参考所用。相关实验依托 GitHub 开源项目完成，尽管项目开源可复用，但从建模到运动学求解，再到代码编写实现，直至最终通过 ROS 通信在仿真平台上完成机器人动态仿真，整个完整的机器人项目仿真流程，每个环节都让我直面未知、攻克了诸多挑战，也让我从中积累了部分实践经验。

# 问题

# 预期实现目标

1. stewart并联型机器人的结构建模,并用urdf文件描述。
2. stewart并联型机器人逆运动学（inverse kinematics，以下简称ik）求解。
3. 编写手柄控制程序，通过手柄控制gazebo中的机器人运动。
4. 通过ROS通讯在gazebo中实现对机器人平台的运动仿真

## 第一阶段：Stewart平台的结构建模

### 建模讲解

![stewart机器平台系统结构示意图](/img/stewart.png)

stewart机器人仿真平台示意图，机器人平台由30个Joint和26个Link组成，每个Link之间通过Joint连接。

stewart分为静平台（黑色部分）和动平台（红色部分），根据动平台位姿变化通过ik求解六个杆长。

### 控制逻辑

1. Joy 手柄输入 → 控制动平台的姿态  
   $\downarrow$
2. 逆运动学计算（IK）  
   $\downarrow$
3. 通过ROS发布指令 → 生成六个执行器长度目标  
   $\downarrow$
4. 驱动层接收指令，控制伺服电机完成动作

---

## 第二阶段：Stewart平台ik解算

### 机器人数学原理

已知动平台的姿态参数：$(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$  
活动活塞两端铰链坐标：
动平台端铰链坐标 $p$（4×6矩阵，每行对应1个铰链齐次坐标）：

$$
p = \begin{bmatrix}
-0.101 & 0.8 & 0.25 & 1 \\
0.101 & 0.8 & 0.25 & 1 \\
0.743 & -0.313 & 0.25 & 1 \\
0.642 & -0.487 & 0.25 & 1 \\
-0.643 & -0.486 & 0.25 & 1 \\
-0.744 & -0.311 & 0.25 & 1
\end{bmatrix}
$$

基座端（静平台）铰链坐标 $b$（4×6矩阵，每行对应1个铰链齐次坐标）：

$$
b = \begin{bmatrix}
-0.642 & 0.487 & -0.05 & 1 \\
0.642 & 0.487 & -0.05 & 1 \\
0.743 & 0.313 & -0.05 & 1 \\
0.101 & -0.8 & -0.05 & 1 \\
-0.101 & -0.8 & -0.05 & 1 \\
-0.743 & 0.313 & -0.05 & 1
\end{bmatrix}
$$

求解六根执行器的长度：$L_1, L_2, L_3, L_4, L_5, L_6$。

1. 求解变换矩阵$T = \begin{bmatrix}\boldsymbol{R} & \boldsymbol{t} \\ \boldsymbol{0} & 1\end{bmatrix}$。
2. 求解动平台铰链相对于静平台的坐标 ${}_b^p P_i=p(row_i)*T$。
3. 现在对一根活塞杆来讲已知了在静平台坐标系中两端铰链的坐标，求杆长$Length$。  
   $Length = \sqrt{(x_1-x_2)^2+(y_1-y_2)^2+(z_1-z_2)^2}$。

### 求解变换矩阵$T$详解

Roll-Pitch-Yaw: Deriving the Rotation Matrix

已知 roll（滚转）、pitch（俯仰）、yaw（偏航）三个欧拉角后，求解旋转矩阵 $\boldsymbol{R}$ 的核心是：将绕x/y/z轴的单轴旋转矩阵按固定顺序相乘。机器人/工程领域最常用的顺序为 **「先绕x轴roll→再绕y轴pitch→最后绕z轴yaw」** 或 **「绕z轴yaw→绕y轴pitch→绕x轴roll」** ，需明确旋转顺序，否则结果会存在差异。

以下是完整的推导过程、核心公式及工程实现示例，兼顾理论原理与实操落地。

#### 一、核心前提：明确欧拉角的旋转顺序

工程中最通用的旋转模式是 “绕参考坐标系的轴旋转”（固定轴旋转），其标准顺序及对应旋转矩阵定义如下：

- roll（滚转角，记为 $\phi$）：绕参考坐标系X轴旋转，对应旋转矩阵 $\boldsymbol{R}_x(\phi)$；

- pitch（俯仰角，记为 $\theta$）：绕参考坐标系Y轴旋转，对应旋转矩阵 $\boldsymbol{R}_y(\theta)$；

- yaw（偏航角，记为 $\psi$）：绕参考坐标系Z轴旋转，对应旋转矩阵 $\boldsymbol{R}_z(\psi)$。

固定轴旋转模式下，最终的旋转矩阵 $\boldsymbol{R}$ 是三个单轴旋转矩阵的右乘（旋转顺序：X→Y→Z），数学表达式为：

$\boldsymbol{R} = \boldsymbol{R}_z(\psi) \cdot \boldsymbol{R}_y(\theta) \cdot \boldsymbol{R}_x(\phi)$

#### 二、单轴旋转矩阵公式（弧度制）

以下所有旋转矩阵的输入角度均为弧度制，若使用角度制需先进行单位转换（弧度 = 角度 × $\pi/180.0$）。

1.  绕X轴旋转（roll，$\phi$）

$\boldsymbol{R}_x(\phi) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}$

2.  绕Y轴旋转（pitch，$\theta$）

$\boldsymbol{R}_y(\theta) = \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{bmatrix}$

3.  绕Z轴旋转（yaw，$\psi$）

$\boldsymbol{R}_z(\psi) = \begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}$

#### 三、合并为最终旋转矩阵

将三个单轴旋转矩阵按 $\boldsymbol{R}_z \cdot \boldsymbol{R}_y \cdot \boldsymbol{R}_x$ 的顺序执行矩阵乘法，展开后得到最终的3×3旋转矩阵 $\boldsymbol{R}$：

$\boldsymbol{R} = \begin{bmatrix}
\cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\
\sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}$

#### 四、关键注意事项

1.  角度单位转换

所有公式及代码均要求输入为弧度，若原始角度为角度制（如45°），需通过以下公式转换：

$\text{弧度} = \text{角度} \times \frac{\pi}{180.0}$

2.  旋转顺序的影响

旋转顺序直接决定旋转矩阵的结果，工程中需根据场景区分两种核心模式：

- 固定轴（参考坐标系）：旋转轴始终为世界/基坐标系的X/Y/Z轴，旋转矩阵为 $\boldsymbol{R} = \boldsymbol{R}_z \cdot \boldsymbol{R}_y \cdot \boldsymbol{R}_x$；

- 动态轴（物体自身坐标系）：旋转轴随物体姿态变化（如机器人自身轴），旋转顺序为X→Y→Z，矩阵为 $\boldsymbol{R} = \boldsymbol{R}_x \cdot \boldsymbol{R}_y \cdot \boldsymbol{R}_z$。

3.  万向锁问题

当俯仰角 pitch = ±90° 时，滚转角 roll 与偏航角 yaw 会产生耦合（即“万向锁”现象），此时欧拉角无法唯一描述旋转姿态。

## 第三阶段：ps4控制器与ROS通信

### 手柄控制逻辑编写

![Xbox手柄按键使能示意图](/img/手柄按键详解_01.png)

1. 发布与订阅话题

| 话题名                    | 消息类型                    | 含义               |
| ------------------------- | --------------------------- | ------------------ |
| `/joy`                    | `sensor_msgs/Joy`           | 手柄原始输入       |
| `/stewart/platform_twist` | `geometry_msgs/Twist`       | 末端速度指令       |
| `/stewart/platform_pose`  | `geometry_msgs/PoseStamped` | 积分得到的末端位姿 |

2. 数据流逻辑
   PS4 手柄  
   $\downarrow$  
   Joy 消息 (/joy)  
   $\downarrow$  
   解析为速度 Twist  
   $\downarrow$  
   对速度进行时间积分  
   $\downarrow$  
   得到位姿 Pose  
   $\downarrow$  
   发布 /stewart/platform_pose

3. 代码逻辑

```
        // ========== 从手柄读取速度指令 ==========
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = msg->axes[0] * 0.1;   // 速度缩放 0.1 m/s
        twist_msg.linear.y = msg->axes[1] * 0.1;
        twist_msg.linear.z = msg->axes[7] * 0.1; // 上升下降

        twist_msg.angular.x = msg->axes[4] * 0.1;  // 绕X旋转速度
        twist_msg.angular.y = msg->axes[3] * 0.1;  // 绕Y旋转速度
        twist_msg.angular.z = msg->axes[6] * 0.1; // 绕Z旋转速度

        twist_pub.publish(twist_msg); // 仍然发布速度话题（方便调试）

        // ========== 对速度积分得到位姿 ==========
        pose.pose.position.x += twist_msg.linear.x * dt;
        pose.pose.position.y += twist_msg.linear.y * dt;
        pose.pose.position.z += twist_msg.linear.z * dt;

        roll  += twist_msg.angular.x * dt;
        pitch += twist_msg.angular.y * dt;
        yaw   += twist_msg.angular.z * dt;
```

${}^C({}^A\Omega_B)$  
${}^A_BA$

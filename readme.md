# StationLinearPath

| 技术方向 | 开源部分                       | 开源技术点                 | 编写负责人 |
| -------- | ------------------------------ | -------------------------- | ---------- |
| 电控方案 | 视觉辅助自动兑换的电控驱动部分 | 视觉辅助兑换的电控驱动方案 | 曾熙朗     |

> 由于此readme涉及到Latex公式，强烈建议使用Typora打开进行阅读。
>
> - 本次开源作品StationLinearPath，出自华南理工大学华南虎战队工程电控曾熙朗。作品仅用于技术交流，未经作者允许，不得作任何商业用途。
>
> - 本作品的声明以及其修改权、保护作品完整权及最终解释权均归华南理工大学华南虎战队所有。

# 背景

​		23赛季工程兑换难度增大，在经过分区赛的尝试和国赛的规则改动后，所有的队伍都可以兑换最高等级的矿石。在此情景下，充分缩短兑换时间是提高赛场队伍作战能力的重要战略。缩短兑换时间的方法目前有2种，一种是使用自定义控制器由操作手手动兑换，另一种方案是通过视觉对兑换站的灯条进行识别从而驱动机械臂自动兑换。采用第二种方案的原因是，自定义控制器的表现始终取决于操作手的赛场表现，有较大的不确定性，而视觉自动兑换仅取决于硬件的稳定性和算法的稳定性，在保证稳定性的情况下，可以充分保证赛场的表现，并减轻操作手的赛场负担。

# 功能

- 接收视觉传来的姿态数据和空间位置数据
- 将以上数据转换为机械臂的目标位点并计算中间点和终点，中间点为机械臂前往终点的辅助点，终点为兑换站前表面向外移动一个矿石的距离（20cm，可自定义距离）
- 提供根据视觉姿态数据得出的机械臂3维运动向量数据
- 逆运动学解算及其有效性检测（机械臂机械限位）（配套华南虎23赛季国赛工程的机械臂结构）

# 依赖

- C库`math.h`
- 标准库`stdint.h`

# 文件架构

StationLinearPath:.
│  readme.md
│  StationLinearPath.cpp
│  StationLinearPath.h
│
└─兑换站路径解算文档

# 使用方法

- 云台下方地面为原点，吸盘方向为x，竖直向上为z，建立机器人坐标系

- 使用方法：

​     1. 使用结构体SLPConstantStructdef进行初始化传参，详细参数含义见代码注释，注意，该参数配套华南虎23赛季工程机器人结构

​     2. 循环调用recVisionTarget和Calculate，接收Calculate返回的结果判断

​     3. 根据calculate返回的结果决定执行的动作

# 原理与理论支持分析

参考配套文档：

[纯小三轴机械臂解算](兑换站路径解算文档/纯小三轴机械臂解算.md)

[路径规划解算](兑换站路径解算文档/路径规划解算.md)

1. 考虑到机器人坐标系和视觉坐标系的差异，首先需要一个变换矩阵实现机器人坐标系和视觉坐标系的转换：

设车身机器人坐标W、车身相机坐标C、目标机器人坐标G、目标相机坐标A

$$ {_{C}^{W} T=\begin{bmatrix}  0& 0& 1& 0\\ -1& 0& 0& 0\\  0&-1& 0& 0\\  0& 0& 0& 1 \end{bmatrix} }$$

$${ _{A}^{C} T=\begin{bmatrix} r_{11}&r_{12}&r_{13}&p_{x}\\ r_{21}&r_{22}&r_{23}&p_{y}\\ r_{31}&r_{32}&r_{33}&p_{z}\\      0&     0&     0& 1 \end{bmatrix} }$$

$${ _{G}^{A} T=\begin{bmatrix}  0&-1& 0& 0\\  0& 0&-1& 0\\  1& 0& 0& 0\\  0& 0& 0& 1 \end{bmatrix} }$$

从而

$${ _{G}^{W} T={_{C}^{W}T} \space {_{A}^{C}T} \space {_{G}^{A}T} =\begin{bmatrix}  r_{33}&-r_{31}&-r_{32}& p_{z}\\ -r_{13}& r_{11}& r_{12}&-p_{x}\\ -r_{23}& r_{21}& r_{22}&-p_{y}\\  0& 0& 0& 1 \end{bmatrix}} $$

从而

$${ _{W}^{G} T=\begin{bmatrix}  r_{33}&-r_{13}&-r_{23}& p_{y}r_{13} - p_{x}r_{33} + p_{z}r_{23}\\ -r_{31}& r_{11}& r_{21}& p_{x}r_{31} - p_{y}r_{11} - p_{z}r_{21}\\ -r_{32}& r_{12}& r_{22}& p_{x}r_{32} - p_{y}r_{12} - p_{z}r_{22}\\  0& 0& 0& 1 \end{bmatrix} }$$

1. 为了减少通信数据量，兑换站姿态数据采用四元数的方式进行传输，因此，需要将四元数数据转换为旋转矩阵数据，方便后期直观地对姿态进行控制：

```C++
void SLPClassdef::quaCoord2TMatrix(float qx, float qy, float qz, float qw, float x, float y, float z, float TMat[3][4])
{
  float norm = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
  float qxn = qx/norm;
  float qyn = qy/norm;
  float qzn = qz/norm;
  float qwn = qw/norm;

  TMat[0][0] = 1 - 2 * qyn * qyn - 2 * qzn * qzn;     TMat[0][1] = 2 * qxn * qyn - 2 * qzn * qwn;       TMat[0][2] = 2 * qxn * qzn + 2 * qyn * qwn;       TMat[0][3] = x;
  TMat[1][0] = 2 * qxn * qyn + 2 * qzn * qwn;         TMat[1][1] = 1 - 2 * qxn * qxn - 2 * qzn * qzn;   TMat[1][2] = 2 * qyn * qzn - 2 * qxn * qwn;       TMat[1][3] = y;
  TMat[2][0] = 2 * qxn * qzn - 2 * qyn * qwn;         TMat[2][1] = 2 * qyn * qzn + 2 * qxn * qwn;       TMat[2][2] = 1 - 2 * qxn * qxn - 2 * qyn * qyn;   TMat[2][3] = z;
}
```

1. 为直观考虑，以兑换站的前表面中心为原点，向内为X轴，向左为Y轴，向上为Z轴，建立坐标系。兑换站尺寸：288x288x288mm。

![img](兑换站路径解算文档/兑换站坐标系.png)

从而需要知道两个坐标系之间的转换关系为：

```C++
quaCoord2TMatrix(visionPack.qx,visionPack.qy,visionPack.qz,visionPack.qw,visionPack.x,visionPack.y,visionPack.z,visionT);//四元数转变换矩阵visionT
TWorldGoal[0][0] = visionT[2][2];   TWorldGoal[0][1] = -visionT[2][0];    TWorldGoal[0][2] = -visionT[2][1];     TWorldGoal[0][3] = visionT[2][3];
TWorldGoal[1][0] = -visionT[0][2];   TWorldGoal[1][1] = visionT[0][0];    TWorldGoal[1][2] = visionT[0][1];     TWorldGoal[1][3] = -visionT[0][3];
TWorldGoal[2][0] = -visionT[1][2];   TWorldGoal[2][1] = visionT[1][0];    TWorldGoal[2][2] = visionT[1][1];     TWorldGoal[2][3] = -visionT[1][3];    //机器人坐标系转换到兑换站坐标系



//机器人坐标系转换到兑换站坐标系
TGoalWorld[0][0] = visionT[2][2];   TGoalWorld[0][1] = -visionT[0][2];    TGoalWorld[0][2] = -visionT[1][2];    TGoalWorld[0][3] = -visionT[0][3] * visionT[0][2] - visionT[1][3] * visionT[1][2] - visionT[2][3] * visionT[2][2];
TGoalWorld[1][0] = -visionT[2][0];   TGoalWorld[1][1] = visionT[0][0];    TGoalWorld[1][2] = visionT[1][0];     TGoalWorld[1][3] = visionT[0][3] * visionT[0][0] + visionT[1][3] * visionT[1][0] + visionT[2][3] * visionT[2][0];
TGoalWorld[2][0] = -visionT[2][1];   TGoalWorld[2][1] = visionT[0][1];    TGoalWorld[2][2] = visionT[1][1];     TGoalWorld[2][3] = visionT[0][3] * visionT[0][1] + visionT[1][3] * visionT[1][1] + visionT[2][3] * visionT[2][1];
```

1. 前期我们考虑了机构相对于兑换站的位置，并通过几何法进行了碰撞规避，这个可以从源码中的中间点计算得到体现。但综合考虑机构设计和机器人底盘可到达的位置，我们认为可以不考虑矿石和机构碰撞，故只要计算：
   1. 机械臂末端能否到达兑换站前表面
   2. 机械臂末端能不能到达距离兑换站前表面20cm的位置（相当于隔了一个矿石的距离），可以考虑安全多预留一些距离。

```C++
stationWorld[0] = TWorldGoal[0][3];
stationWorld[1] = TWorldGoal[1][3];
stationWorld[2] = TWorldGoal[2][3];//兑换站前表面中心的空间位置

goalWorld[0] = -safeR * TWorldGoal[0][0] + TWorldGoal[0][3];
goalWorld[1] = -safeR * TWorldGoal[1][0] + TWorldGoal[1][3];
goalWorld[2] = -safeR * TWorldGoal[2][0] + TWorldGoal[2][3];//距离兑换站前表面中心20cm的空间位置
```

1. 计算出机械臂末端姿态和平移机构位置

```C++
void SLPClassdef::attitudeCal(float &yawOri, float &pitchOri, float &rollOri,float &yaw, float &pitch, float &roll)
{
  pitchOri = acosf(TWorldGoal[0][0]);
  if (pitchOri != 0.0f)
  {
    rollOri = atan2f(TWorldGoal[0][1] / sinf(pitchOri), TWorldGoal[0][2] / sinf(pitchOri));
    yawOri = atan2f(TWorldGoal[1][0] / sinf(pitchOri), -TWorldGoal[2][0] / sinf(pitchOri));
  }
  else
  {
    rollOri = 0.0f;
    yawOri = atan2f(TWorldGoal[2][1], TWorldGoal[2][2]);
  }

  pitch = 90.0f - Degrees(pitchOri);
  roll = Degrees(rollOri);
  if (-63.0f <= Degrees(yawOri) && Degrees(yawOri) <= 180.0f)
    yaw = -Degrees(yawOri) + 180.0f;
  else if (-180.0f <= Degrees(yawOri) && Degrees(yawOri) < -63.0f)
    yaw = -Degrees(yawOri) - 180.0f;
}

void SLPClassdef::xyzTracGene(float point[3], float _attiTrac[3], float &lift, float &extend, float &translate)
{
  /* 常数项为世界坐标原点与平移机构零点的偏差 */
  extend = point[0] - errx - (x1 + x2 + x3 * cosf(_attiTrac[1]));
  translate = - (point[1] - erry - (y1 + y2 * cosf(_attiTrac[0]) - z2 * sinf(_attiTrac[0]) + x3 * sinf(_attiTrac[1]) * sinf(_attiTrac[0])));
  lift = point[2] - errz - (z1 + z2 * cosf(_attiTrac[0]) + y2 * sin(_attiTrac[0]) - x3 * cosf(_attiTrac[0]) * sinf(_attiTrac[1]));
}
```

后将关节目标量下发到机构进行驱动即可。

```C++
/* 视觉兑换 */
  if(VisionPack.visionMode != 0 && ParamServer.VisionAdjState != 0)
      ParamServer.VisionAdjState = SLP.Calculate() + 2;
  if (IsKeyPressOnce(DR16_KEY_V) && (ParamServer.VisionAdjState == 3 || ParamServer.VisionAdjState == 4 || ParamServer.VisionAdjState == 5))//带矿石移动到兑换站前表面
  {
    ParamServer.VisionAimed = true;
    SLP.getAttitudeTrac(ParamServer.vyaw, ParamServer.vpitch, ParamServer.vroll);
    SLP.getGoalxyzTrac(ParamServer.vglift, ParamServer.vgextend, ParamServer.vgtrans);
    SLP.getStationxyzTrac(ParamServer.vslift, ParamServer.vsextend, ParamServer.vstrans);
    SLP.getForWardIncxyzTrac(ParamServer.vIncForward[0], ParamServer.vIncForward[1], ParamServer.vIncForward[2]);
    SLP.getUpIncxyzTrac(ParamServer.vIncUp[0], ParamServer.vIncUp[1], ParamServer.vIncUp[2]);
    SLP.getRightIncxyzTrac(ParamServer.vIncRight[0], ParamServer.vIncRight[1], ParamServer.vIncRight[2]);
    if(ParamServer.VisionAdjState == 3 || ParamServer.VisionAdjState == 4)
    {
      float curL = ParamServer.JointTarget[(uint8_t)JointEnumdef::Lift]; // curL ： current lift target
      float curE = ParamServer.JointTarget[(uint8_t)JointEnumdef::Extend];
      float curR = ParamServer.JointTarget[(uint8_t)JointEnumdef::Rotate];
      float curT = ParamServer.JointTarget[(uint8_t)JointEnumdef::Translate];
      float curY = ParamServer.JointTarget[(uint8_t)JointEnumdef::Yaw];
      float curP = ParamServer.JointTarget[(uint8_t)JointEnumdef::Pitch];
      float curr = ParamServer.JointTarget[(uint8_t)JointEnumdef::Roll];

      ParamServer.trajectory.num = 3;
      ParamServer.trajectory.status = 1;
      ParamServer.trajectory.tim    (0,     1.0f,               1.8f                );
      ParamServer.trajectory.lift   (curL,  curL,               ParamServer.vglift  );
      ParamServer.trajectory.extend (curE,  curE,               ParamServer.vgextend);
      ParamServer.trajectory.rotate (curR,  ROTATEUP,           ROTATEUP            );
      ParamServer.trajectory.trans  (curT,  curT,               ParamServer.vgtrans );
      ParamServer.trajectory.yaw    (curY,  ParamServer.vyaw,   ParamServer.vyaw    );
      ParamServer.trajectory.pitch  (curP,  ParamServer.vpitch, ParamServer.vpitch  );
      ParamServer.trajectory.roll   (curr,  ParamServer.vroll,  ParamServer.vroll   );
    }
    else if(ParamServer.VisionAdjState == 5)
    {
      float curL = ParamServer.JointTarget[(uint8_t)JointEnumdef::Lift]; // curL ： current lift target
      float curE = ParamServer.JointTarget[(uint8_t)JointEnumdef::Extend];
      float curR = ParamServer.JointTarget[(uint8_t)JointEnumdef::Rotate];
      float curT = ParamServer.JointTarget[(uint8_t)JointEnumdef::Translate];
      float curY = ParamServer.JointTarget[(uint8_t)JointEnumdef::Yaw];
      float curP = ParamServer.JointTarget[(uint8_t)JointEnumdef::Pitch];
      float curr = ParamServer.JointTarget[(uint8_t)JointEnumdef::Roll];

      ParamServer.trajectory.num = 2;
      ParamServer.trajectory.status = 1;
      ParamServer.trajectory.tim    (0,     1.0f);
      ParamServer.trajectory.lift   (curL,  curL);
      ParamServer.trajectory.extend (curE,  curE);
      ParamServer.trajectory.rotate (curR,  ROTATEUP);
      ParamServer.trajectory.trans  (curT,  curT);
      ParamServer.trajectory.yaw    (curY,  ParamServer.vyaw);
      ParamServer.trajectory.pitch  (curP,  ParamServer.vpitch);
      ParamServer.trajectory.roll   (curr,  ParamServer.vroll);
    }
  }

  if(IsKeyPressOnce(DR16_KEY_C) && ParamServer.VisionAimed == true)//一键推入矿石
  {
    ParamServer.VisionAimed = false;
    float curL = ParamServer.JointTarget[(uint8_t)JointEnumdef::Lift]; // curL ： current lift target
    float curE = ParamServer.JointTarget[(uint8_t)JointEnumdef::Extend];
    float curR = ParamServer.JointTarget[(uint8_t)JointEnumdef::Rotate];
    float curT = ParamServer.JointTarget[(uint8_t)JointEnumdef::Translate];
    float curY = ParamServer.JointTarget[(uint8_t)JointEnumdef::Yaw];
    float curP = ParamServer.JointTarget[(uint8_t)JointEnumdef::Pitch];
    float curr = ParamServer.JointTarget[(uint8_t)JointEnumdef::Roll];

    ParamServer.trajectory.num = 2;
    ParamServer.trajectory.status = 1;
    ParamServer.trajectory.tim    (0,      3.0f                   );
    ParamServer.trajectory.lift   (curL,   ParamServer.vslift     );
    ParamServer.trajectory.extend (curE,   ParamServer.vsextend   );
    ParamServer.trajectory.rotate (curR,   ROTATEUP               );
    ParamServer.trajectory.trans  (curT,   ParamServer.vstrans    );
    ParamServer.trajectory.yaw    (curY,   curY                   );
    ParamServer.trajectory.pitch  (curP,   curP                   );
    ParamServer.trajectory.roll   (curr,   curr                   );
  }
```

1. 进一步优化，考虑到视觉识别的视野比较小，而高兑换等级的情况下，在相机能成功识别时，机构不一定能够完全能将矿石送进兑换站，故设计警告点的计算和机构是否超过限位的计算，若能到达警告点，仍可以进行一键全自动兑换，若不能到达警告点，则尝试仅辅助姿态对准。

```C++
warnPointWorld[0] = -warnR * TWorldGoal[0][0] + TWorldGoal[0][3];
warnPointWorld[1] = -warnR * TWorldGoal[1][0] + TWorldGoal[1][3];
warnPointWorld[2] = -warnR * TWorldGoal[2][0] + TWorldGoal[2][3];//警告点空间位置计算

bool SLPClassdef::limitCheck(float &_lift, float &_extend, float &_translate, float &_yaw, float &_pitch, float &_roll)
{
  if(liftMin      <= _lift        && _lift      <= liftMax
  && extendMin    <= _extend      && _extend    <= extendMax
  && translateMin <= _translate   && _translate <= translateMax
  && yawMin       <= _yaw         && _yaw       <= yawMax
  && pitchMin     <= _pitch       && _pitch     <= pitchMax
  && rollMin      <= _roll        && _roll      <= rollMax)
  {
    return true;
  }
  else
    return false;
}
```

为了在这种特殊情况下方便操作手的操作，通过视觉数据引入机械臂的直线增量驱动

```C++
stationUpIncWorld[0] = 1.0f * TWorldGoal[0][2] + TWorldGoal[0][3];
stationUpIncWorld[1] = 1.0f * TWorldGoal[1][2] + TWorldGoal[1][3];
stationUpIncWorld[2] = 1.0f * TWorldGoal[2][2] + TWorldGoal[2][3];

stationRightIncWorld[0] = -1.0f * TWorldGoal[0][1] + TWorldGoal[0][3];
stationRightIncWorld[1] = -1.0f * TWorldGoal[1][1] + TWorldGoal[1][3];
stationRightIncWorld[2] = -1.0f * TWorldGoal[2][1] + TWorldGoal[2][3];

UpIncxyzTrac[0] = UpIncxyzTrac[0] - StationxyzTrac[0];
UpIncxyzTrac[1] = UpIncxyzTrac[1] - StationxyzTrac[1];
UpIncxyzTrac[2] = UpIncxyzTrac[2] - StationxyzTrac[2];

RightIncxyzTrac[0] = RightIncxyzTrac[0] - StationxyzTrac[0];
RightIncxyzTrac[1] = RightIncxyzTrac[1] - StationxyzTrac[1];
RightIncxyzTrac[2] = RightIncxyzTrac[2] - StationxyzTrac[2];

/**
 * @brief 获取上移增量，单位m
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getUpIncxyzTrac(float &lift, float &extend, float &translate)
{
  lift = UpIncxyzTrac[0];
  extend = UpIncxyzTrac[1];
  translate = UpIncxyzTrac[2];
}

/**
 * @brief 获取右移增量，单位m
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getRightIncxyzTrac(float &lift, float &extend, float &translate)
{
  lift = RightIncxyzTrac[0];
  extend = RightIncxyzTrac[1];
  translate = RightIncxyzTrac[2];
}

/**
 * @brief 获取前移增量，单位m
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getForWardIncxyzTrac(float &lift, float &extend, float &translate)
{
  lift = StationxyzTrac[0] - GoalxyzTrac[0];
  extend = StationxyzTrac[1] - GoalxyzTrac[1];
  translate = StationxyzTrac[2] - GoalxyzTrac[2];
}
```

综合计算策略如下：

```C++
/**
 * @brief 计算轨迹点
 * 
 * @return uint8_t 0：轨迹生成失败，1：轨迹生成成功，2：勉强兑换警告，3：仅姿态可用
 */
uint8_t SLPClassdef::Calculate()
{
  /* 姿态计算 */
  attitudeCal(AttiOri[0], AttiOri[1], AttiOri[2],AttiTrac[0], AttiTrac[1], AttiTrac[2]);
  /* 兑换站位点计算 */
  xyzTracGene(stationWorld,AttiOri,StationxyzTrac[0],StationxyzTrac[1],StationxyzTrac[2]);
  /* 向上增量计算 */
  xyzTracGene(stationUpIncWorld, AttiOri, UpIncxyzTrac[0], UpIncxyzTrac[1], UpIncxyzTrac[2]);
  UpIncxyzTrac[0] = UpIncxyzTrac[0] - StationxyzTrac[0];
  UpIncxyzTrac[1] = UpIncxyzTrac[1] - StationxyzTrac[1];
  UpIncxyzTrac[2] = UpIncxyzTrac[2] - StationxyzTrac[2];
  /* 向右增量计算 */
  xyzTracGene(stationRightIncWorld, AttiOri, RightIncxyzTrac[0], RightIncxyzTrac[1], RightIncxyzTrac[2]);
  RightIncxyzTrac[0] = RightIncxyzTrac[0] - StationxyzTrac[0];
  RightIncxyzTrac[1] = RightIncxyzTrac[1] - StationxyzTrac[1];
  RightIncxyzTrac[2] = RightIncxyzTrac[2] - StationxyzTrac[2];
  /* 警告点位点计算 */
  xyzTracGene(warnPointWorld,AttiOri,WarnxyzTrac[0],WarnxyzTrac[1],WarnxyzTrac[2]);

  /* 判断兑换站位点是否超限 */
  if(true == limitCheck(StationxyzTrac[0], StationxyzTrac[1], StationxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
  {
    /* 终点计算 */
    xyzTracGene(goalWorld, AttiOri, GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]);
    /* 判断终点是否超限 */
    if (true == limitCheck(GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
      return 1;
  }
  else
  {
    /* 判断警告点是否超限 */
    if (true == limitCheck(WarnxyzTrac[0], WarnxyzTrac[1], WarnxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
    {
      /* 终点计算 */
      xyzTracGene(goalWorld, AttiOri, GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]);
      /* 判断终点是否超限 */
      if (true == limitCheck(GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
        return 2;
    }
  }
  if (true == attiCheck(AttiTrac[0], AttiTrac[1], AttiTrac[2]))
    return 3;
  else
    return 0;
}
```

# 未来优化方向

- 优化轨迹生成方法（使用RRT算法等
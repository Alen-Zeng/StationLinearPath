/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    StationLinearPath.cpp
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   SLP 兑换站线性路径规划算法
  * @date    2023/06/10 09:33:41
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2023-6-10 <td> 1.0 <td>zxl <td>Creator
  * <tr><td>2023-6-16 <td> 1.1 <td>zxl <td>complete,wait for simulation.
  * </table>
  *
  ==============================================================================
                               How to use this Lib  
  ==============================================================================
    @note
      -# 云台下方地面为原点，吸盘方向为x，竖直向上为z，建立机器人坐标系
      -# 使用方法：
          1. 使用结构体SLPConstantStructdef进行初始化传参
          2. 循环调用recVisionTarget和Calculate，接收Calculate返回的结果判断
          3. 如果Calculate返回的结果为true，此时可以调用getMidxyzTrac和getGoalxyzTrac获取轨迹点数据，并执行
      -# 代码涉及较多的符号，请参考代码配套的说明文档
      -# 没有太多时间想架构，修修补补，内部全是shit，凑合着看吧，有时间再改
    @warning
      -# 如果Calculate返回的结果为false，请勿调用getMidxyzTrac和getGoalxyzTrac，否则会获得错误数据
      -# 
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "StationLinearPath.h"
/* function prototypes -------------------------------------------------------*/

namespace SLP_NP
{
  template <typename T>
  const T abs(const T &input)
  {
    return input < (T)0 ? -input : input;
  }
}

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef Radians
#define Radians(degrees) ((degrees)*PI / 180.0f)  //角度转弧度
#endif

#ifndef Degrees
#define Degrees(radians) ((radians)*180.0f / PI)  //弧度转角度
#endif

SLPClassdef::SLPClassdef(SLPConstantStructdef &_SLPCon) 
{
  /* 安全距离 */
  safeR = _SLPCon.safeR;
  /* 关节限位参数 */
  liftMin = _SLPCon.liftMin;
  liftMax = _SLPCon.liftMax;
  extendMin = _SLPCon.extendMin;
  extendMax = _SLPCon.extendMax;
  translateMin = _SLPCon.translateMin;
  translateMax = _SLPCon.translateMax;
  yawMin = _SLPCon.yawMin;
  yawMax = _SLPCon.yawMax;
  pitchMin = _SLPCon.pitchMin;
  pitchMax = _SLPCon.pitchMax;
  rollMin = _SLPCon.rollMin;
  rollMax = _SLPCon.rollMax;
  /* 小三轴变换矩阵中用到的常数 */
  x1 = _SLPCon.x1, y1 = _SLPCon.y1, z1 = _SLPCon.z1; // translate--yaw
  x2 = _SLPCon.x2, y2 = _SLPCon.y2, z2 = _SLPCon.z2; // yaw--pitch
  x3 = _SLPCon.x3;                                   // pitch--roll
  /* 平移机构坐标原点与世界坐标原点的误差 */
  errx = _SLPCon.errx;
  erry = _SLPCon.erry;
  errz = _SLPCon.errz;
  /* 视觉兑换自动组的平移常数项 */
  autoConExt = _SLPCon.autoConExt;
  autoConTrans = _SLPCon.autoConTrans;
  autoConLif = _SLPCon.autoConLif;
  /* 警告点距离 */
  warnR = _SLPCon.warnR;
}

/**
 * @brief 四元数与xyz坐标转齐次变换矩阵
 * 
 * @param qx 
 * @param qy 
 * @param qz 
 * @param qw 
 * @param x 
 * @param y 
 * @param z 
 * @param TMat 
 */
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

/**
 * @brief 接收视觉四元数+位置数据，并计算齐次变换矩阵
 * 
 * @param visionPack 
 */
void SLPClassdef::recVisionTarget(VisionPackStructdef &visionPack)
{
  quaCoord2TMatrix(visionPack.qx,visionPack.qy,visionPack.qz,visionPack.qw,visionPack.x,visionPack.y,visionPack.z,visionT);
  TWorldGoal[0][0] = visionT[2][2];   TWorldGoal[0][1] = -visionT[2][0];    TWorldGoal[0][2] = -visionT[2][1];     TWorldGoal[0][3] = visionT[2][3];
  TWorldGoal[1][0] = -visionT[0][2];   TWorldGoal[1][1] = visionT[0][0];    TWorldGoal[1][2] = visionT[0][1];     TWorldGoal[1][3] = -visionT[0][3];
  TWorldGoal[2][0] = -visionT[1][2];   TWorldGoal[2][1] = visionT[1][0];    TWorldGoal[2][2] = visionT[1][1];     TWorldGoal[2][3] = -visionT[1][3];

  stationWorld[0] = TWorldGoal[0][3];
  stationWorld[1] = TWorldGoal[1][3];
  stationWorld[2] = TWorldGoal[2][3];

  TGoalWorld[0][0] = visionT[2][2];   TGoalWorld[0][1] = -visionT[0][2];    TGoalWorld[0][2] = -visionT[1][2];    TGoalWorld[0][3] = -visionT[0][3] * visionT[0][2] - visionT[1][3] * visionT[1][2] - visionT[2][3] * visionT[2][2];
  TGoalWorld[1][0] = -visionT[2][0];   TGoalWorld[1][1] = visionT[0][0];    TGoalWorld[1][2] = visionT[1][0];     TGoalWorld[1][3] = visionT[0][3] * visionT[0][0] + visionT[1][3] * visionT[1][0] + visionT[2][3] * visionT[2][0];
  TGoalWorld[2][0] = -visionT[2][1];   TGoalWorld[2][1] = visionT[0][1];    TGoalWorld[2][2] = visionT[1][1];     TGoalWorld[2][3] = visionT[0][3] * visionT[0][1] + visionT[1][3] * visionT[1][1] + visionT[2][3] * visionT[2][1];

  goalWorld[0] = -safeR * TWorldGoal[0][0]/*  - 0.02*TWorldGoal[0][1] + 0.025*TWorldGoal[0][2] */ + TWorldGoal[0][3];
  goalWorld[1] = -safeR * TWorldGoal[1][0]/*  - 0.02*TWorldGoal[1][1] + 0.025*TWorldGoal[1][2] */ + TWorldGoal[1][3];
  goalWorld[2] = -safeR * TWorldGoal[2][0]/*  - 0.02*TWorldGoal[2][1] + 0.025*TWorldGoal[2][2] */ + TWorldGoal[2][3];
  
  warnPointWorld[0] = -warnR * TWorldGoal[0][0] + TWorldGoal[0][3];
  warnPointWorld[1] = -warnR * TWorldGoal[1][0] + TWorldGoal[1][3];
  warnPointWorld[2] = -warnR * TWorldGoal[2][0] + TWorldGoal[2][3];

  stationUpIncWorld[0] = 1.0f * TWorldGoal[0][2] + TWorldGoal[0][3];
  stationUpIncWorld[1] = 1.0f * TWorldGoal[1][2] + TWorldGoal[1][3];
  stationUpIncWorld[2] = 1.0f * TWorldGoal[2][2] + TWorldGoal[2][3];

  stationRightIncWorld[0] = -1.0f * TWorldGoal[0][1] + TWorldGoal[0][3];
  stationRightIncWorld[1] = -1.0f * TWorldGoal[1][1] + TWorldGoal[1][3];
  stationRightIncWorld[2] = -1.0f * TWorldGoal[2][1] + TWorldGoal[2][3];
}

/**
 * @brief 小三轴姿态计算
 * 
 */
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
  if (false == limitCheck(StationxyzTrac[0], StationxyzTrac[1], StationxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
  {
    /* 判断警告点是否超限 */
    if (false == limitCheck(WarnxyzTrac[0], WarnxyzTrac[1], WarnxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
      return 0;
    else
    {
      /* 终点计算 */
      xyzTracGene(goalWorld, AttiOri, GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]);
      /* 判断终点是否超限 */
      if (false == limitCheck(GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
      {
        if(false == attiCheck(AttiTrac[0], AttiTrac[1], AttiTrac[2]))
          return 0;
        else
          return 3;
      }

      // /* 计算中间点 */
      // endEffLocCal(AttiTrac[0], AttiTrac[1], AttiTrac[2]);
      // /* 如果中间点无解，直接返回错误 */
      // if (!midPointCal())
      //   return 0;
      // xyzTracGene(midWorld, AttiOri, MidxyzTrac[0], MidxyzTrac[1], MidxyzTrac[2]);
      // /* 判断中间点是否超限 */
      // if (false == limitCheck(MidxyzTrac[0], MidxyzTrac[1], MidxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
      //   return 0;
      else
        return 2; // 兑换站不可达，但警告点可达，返回警告
    }
  }
  else
  {
    /* 终点计算 */
    xyzTracGene(goalWorld, AttiOri, GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]);
    /* 判断终点是否超限 */
    if (false == limitCheck(GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
      return 0;

    // /* 计算中间点 */
    // endEffLocCal(AttiTrac[0], AttiTrac[1], AttiTrac[2]);
    // /* 如果中间点无解，直接返回错误 */
    // if (!midPointCal())
    //   return 0;
    // xyzTracGene(midWorld, AttiOri, MidxyzTrac[0], MidxyzTrac[1], MidxyzTrac[2]);
    // /* 判断中间点是否超限 */
    // if (false == limitCheck(MidxyzTrac[0], MidxyzTrac[1], MidxyzTrac[2], AttiTrac[0], AttiTrac[1], AttiTrac[2]))
    //   return 0;
    else
      return 1;
  }

}

/**
 * @brief 获取小三轴姿态
 * 
 * @param _pitch 
 * @param _yaw 
 * @param _roll 
 */
void SLPClassdef::getAttitudeTrac(float &_yaw, float &_pitch, float &_roll)
{
  _yaw = AttiTrac[0];
  _pitch = AttiTrac[1];
  _roll = AttiTrac[2];
}

/**
 * @brief 获取中间点
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getMidxyzTrac(float &lift, float &extend, float &translate)
{
  lift = MidxyzTrac[0];
  extend = MidxyzTrac[1];
  translate = MidxyzTrac[2];
}

/**
 * @brief 获取目标目标点
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getGoalxyzTrac(float &lift, float &extend, float &translate)
{
  lift = GoalxyzTrac[0];
  extend = GoalxyzTrac[1];
  translate = GoalxyzTrac[2];
}


/**
 * @brief 获取兑换站目标点
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 */
void SLPClassdef::getStationxyzTrac(float &lift, float &extend, float &translate)
{
  lift = StationxyzTrac[0];
  extend = StationxyzTrac[1];
  translate = StationxyzTrac[2];
}

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

/**
 * @brief 解算出小三轴末端相对于兑换站的xyz位置
 * 
 * @param yaw 
 * @param pitch 
 * @param roll 
 */
void SLPClassdef::endEffLocCal(float _yaw, float _pitch, float _roll)
{
  /* 考虑小三轴位移, 第一个常数项为3个平移常数项，需要结合动作组调整 */
  endEffWorld[0] = autoConExt + errx + x1 + x2 + x3 * cosf(_pitch);
  endEffWorld[1] = -autoConTrans + erry + y1 + y2 * cosf(_yaw) - z2 * sinf(_yaw) + x3 * sin(_pitch) * sinf(_yaw);
  endEffWorld[2] = autoConLif + errz + z1 + z2 * cosf(_yaw) + y2 * sin(_yaw) - x3 * cosf(_yaw) * sinf(_pitch);

  endEffGoal[0] = endEffWorld[0]*TGoalWorld[0][0]+endEffWorld[1]*TGoalWorld[0][1]+endEffWorld[2]*TGoalWorld[0][2]+TGoalWorld[0][3];
  endEffGoal[1] = endEffWorld[0]*TGoalWorld[1][0]+endEffWorld[1]*TGoalWorld[1][1]+endEffWorld[2]*TGoalWorld[1][2]+TGoalWorld[1][3];
  endEffGoal[2] = endEffWorld[0]*TGoalWorld[2][0]+endEffWorld[1]*TGoalWorld[2][1]+endEffWorld[2]*TGoalWorld[2][2]+TGoalWorld[2][3];
}

/**
 * @brief 通过决策面计算小三轴
 * 
 * @param endEffGoal 
 * @return uint8_t 0：无解，1：前表面，2：上表面，3：左表面，4：右表面，5：下表面
 */
uint8_t SLPClassdef::decSurfaceCal(float endEffGoal[3])
{
  if (endEffGoal[0] + safeR < 0)        //前表面
          return 1;
  else if (endEffGoal[0] + safeR >= 0   //上表面
        && ((R[1]-G[1])*(U[2]-G[2])-(R[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(U[2]-G[2])-(R[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(U[1]-G[1])-(R[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) <= 0
        && ((R[1]-G[1])*(S[2]-G[2])-(R[2]-G[2])*(S[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(S[2]-G[2])-(R[2]-G[2])*(S[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(S[1]-G[1])-(R[1]-G[1])*(S[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0
        && ((T[1]-G[1])*(U[2]-G[2])-(T[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((T[0]-G[0])*(U[2]-G[2])-(T[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((T[0]-G[0])*(U[1]-G[1])-(T[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0)
        {
          return 2;
        }
  else if (endEffGoal[0] + safeR >= 0   //左表面
        && ((R[1]-G[1])*(S[2]-G[2])-(R[2]-G[2])*(S[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(S[2]-G[2])-(R[2]-G[2])*(S[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(S[1]-G[1])-(R[1]-G[1])*(S[0]-G[0]))*(endEffGoal[2]-G[2]) < 0
        && ((T[1]-G[1])*(U[2]-G[2])-(T[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((T[0]-G[0])*(U[2]-G[2])-(T[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((T[0]-G[0])*(U[1]-G[1])-(T[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0
        && ((R[1]-G[1])*(O[2]-G[2])-(R[2]-G[2])*(O[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(O[2]-G[2])-(R[2]-G[2])*(O[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(O[1]-G[1])-(R[1]-G[1])*(O[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0)
        {
          return 3;
        }
  else if (endEffGoal[0] + safeR >= 0   //右表面
        && ((R[1]-G[1])*(S[2]-G[2])-(R[2]-G[2])*(S[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(S[2]-G[2])-(R[2]-G[2])*(S[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(S[1]-G[1])-(R[1]-G[1])*(S[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0
        && ((T[1]-G[1])*(U[2]-G[2])-(T[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((T[0]-G[0])*(U[2]-G[2])-(T[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((T[0]-G[0])*(U[1]-G[1])-(T[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) < 0
        && ((N[1]-G[1])*(U[2]-G[2])-(N[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((N[0]-G[0])*(U[2]-G[2])-(N[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((N[0]-G[0])*(U[1]-G[1])-(N[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0)
        {
          return 4;
        }
  else if (endEffGoal[0] + safeR >= 0   //下表面
        && ((R[1]-G[1])*(S[2]-G[2])-(R[2]-G[2])*(S[1]-G[1]))*(endEffGoal[0]-G[0]) - ((R[0]-G[0])*(S[2]-G[2])-(R[2]-G[2])*(S[0]-G[0]))*(endEffGoal[1]-G[1]) + ((R[0]-G[0])*(S[1]-G[1])-(R[1]-G[1])*(S[0]-G[0]))*(endEffGoal[2]-G[2]) < 0
        && ((T[1]-G[1])*(U[2]-G[2])-(T[2]-G[2])*(U[1]-G[1]))*(endEffGoal[0]-G[0]) - ((T[0]-G[0])*(U[2]-G[2])-(T[2]-G[2])*(U[0]-G[0]))*(endEffGoal[1]-G[1]) + ((T[0]-G[0])*(U[1]-G[1])-(T[1]-G[1])*(U[0]-G[0]))*(endEffGoal[2]-G[2]) < 0
        && ((O[1]-G[1])*(N[2]-G[2])-(O[2]-G[2])*(N[1]-G[1]))*(endEffGoal[0]-G[0]) - ((O[0]-G[0])*(N[2]-G[2])-(O[2]-G[2])*(N[0]-G[0]))*(endEffGoal[1]-G[1]) + ((O[0]-G[0])*(N[1]-G[1])-(O[1]-G[1])*(N[0]-G[0]))*(endEffGoal[2]-G[2]) >= 0)
        {
          return 5;
        }
  else                                  //无解
          return 0;
}

/**
 * @brief 根据面决策结果生成中间点，中间点由世界坐标系描述
 * 
 * @return true 中间点有解
 * @return false 中间点无解
 */
bool SLPClassdef::midPointCal()
{
  surfaceRes = decSurfaceCal(endEffGoal);
  if (SLP_NP::abs(endEffGoal[1]) <= 0.144f + safeR && SLP_NP::abs(endEffGoal[2]) <= 0.144f + safeR)
  {
    midGoal[0] = -safeR;
    midGoal[1] = endEffGoal[1];
    midGoal[2] = endEffGoal[2];
  }
  else
  {
    switch (surfaceRes)
    {
    case 1:       //前表面
      midGoal[0] = G[0];
      midGoal[1] = G[1];
      midGoal[2] = G[2];
      break;
    case 2:       //上表面
      midGoal[0] = -safeR;
      midGoal[1] = (endEffGoal[1] * (0.144f + safeR)) / (endEffGoal[2]);
      midGoal[2] = 0.144f + safeR;
      break;
    case 3:       //左表面
      midGoal[0] = -safeR;
      midGoal[1] = 0.144f + safeR;
      midGoal[2] = (endEffGoal[2] * (0.144f + safeR)) / (endEffGoal[1]);
      break;
    case 4:       //右表面
      midGoal[0] = -safeR;
      midGoal[1] = -0.144f - safeR;
      midGoal[2] = (endEffGoal[2] * (-0.144f - safeR)) / (endEffGoal[1]);
      break;
    case 5:       //下表面
      midGoal[0] = -safeR;
      midGoal[1] = (endEffGoal[1] * (-0.144f - safeR)) / (endEffGoal[2]);
      midGoal[2] = -0.144f - safeR;
      break;
    
    default:
      return false;
      break;
    }
  }

  if(surfaceRes != 0)
  {
    midWorld[0] = TWorldGoal[0][0] * midGoal[0] + TWorldGoal[0][1] * midGoal[1] + TWorldGoal[0][2] * midGoal[2] + TWorldGoal[0][3];
    midWorld[1] = TWorldGoal[1][0] * midGoal[0] + TWorldGoal[1][1] * midGoal[1] + TWorldGoal[1][2] * midGoal[2] + TWorldGoal[1][3];
    midWorld[2] = TWorldGoal[2][0] * midGoal[0] + TWorldGoal[2][1] * midGoal[1] + TWorldGoal[2][2] * midGoal[2] + TWorldGoal[2][3];
  }
  return true;
}

/**
 * @brief 产生平移轨迹
 * 
 * @param point 世界坐标系下的x--y--z
 * @param _attiTrac 小三轴小三轴yaw--pitch--roll
 * @param lift 
 * @param extend 
 * @param translate 
 * @return true 
 * @return false 
 */
void SLPClassdef::xyzTracGene(float point[3], float _attiTrac[3], float &lift, float &extend, float &translate)
{
  /* 常数项为世界坐标原点与平移机构零点的偏差 */
  extend = point[0] - errx - (x1 + x2 + x3 * cosf(_attiTrac[1]));
  translate = - (point[1] - erry - (y1 + y2 * cosf(_attiTrac[0]) - z2 * sinf(_attiTrac[0]) + x3 * sinf(_attiTrac[1]) * sinf(_attiTrac[0])));
  lift = point[2] - errz - (z1 + z2 * cosf(_attiTrac[0]) + y2 * sin(_attiTrac[0]) - x3 * cosf(_attiTrac[0]) * sinf(_attiTrac[1]));
}

/**
 * @brief 判断关节位置有无超限
 * 
 * @param lift 
 * @param extend 
 * @param translate 
 * @return true 
 * @return false 
 */
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

/**
 * @brief 姿态超限判断
 * 
 * @param _yaw 
 * @param _pitch 
 * @param _roll 
 * @return true 
 * @return false 
 */
bool SLPClassdef::attiCheck(float &_yaw, float &_pitch, float &_roll)
{
  if(yawMin       <= _yaw         && _yaw       <= yawMax
  && pitchMin     <= _pitch       && _pitch     <= pitchMax
  && rollMin      <= _roll        && _roll      <= rollMax)
  {
    return true;
  }
  else
    return false;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

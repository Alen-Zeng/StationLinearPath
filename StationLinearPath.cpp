/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    StationLinearPath.cpp
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   
  * @date    2023/06/10 09:33:41
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2022-11-01 <td> 1.0 <td>S.B. <td>Creator
  * </table>
  *
  ==============================================================================
                               How to use this Lib  
  ==============================================================================
    @note
      -# 云台下方地面为原点，吸盘方向为x，竖直向上为z，建立机器人坐标系
      -# 使用方法：
          1. 循环调用recVisionTarget和Calculate，接收Calculate返回的结果判断
          2. 如果Calculate返回的结果为true，此时可以调用getMidxyzTrac和getGoalxyzTrac获取轨迹点数据，并执行
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
  TMat[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;     TMat[0][1] = 2 * qx * qy - 2 * qz * qw;       TMat[0][2] = 2 * qx * qz + 2 * qy * qw;       TMat[0][3] = x;
  TMat[1][0] = 2 * qx * qy + 2 * qz * qw;         TMat[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;   TMat[1][2] = 2 * qy * qz - 2 * qx * qw;       TMat[1][3] = y;
  TMat[2][0] = 2 * qx * qz - 2 * qy * qw;         TMat[2][1] = 2 * qy * qz + 2 * qx * qw;       TMat[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;   TMat[2][3] = z;
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

  goalWorld[0] = TWorldGoal[0][3];
  goalWorld[1] = TWorldGoal[1][3];
  goalWorld[2] = TWorldGoal[2][3];

  TGoalWorld[0][0] = visionT[2][2];   TGoalWorld[0][1] = -visionT[0][2];    TGoalWorld[0][2] = -visionT[1][2];     TGoalWorld[0][3] = visionT[1][3] * visionT[0][2] - visionT[0][3] * visionT[2][2] + visionT[2][3] * visionT[1][2];
  TGoalWorld[1][0] = -visionT[2][0];   TGoalWorld[1][1] = visionT[0][0];    TGoalWorld[1][2] = visionT[1][0];     TGoalWorld[1][3] = visionT[0][3] * visionT[2][0] - visionT[1][3] * visionT[0][0] - visionT[2][3] * visionT[1][0];
  TGoalWorld[2][0] = -visionT[2][1];   TGoalWorld[2][1] = visionT[0][1];    TGoalWorld[2][2] = visionT[1][1];     TGoalWorld[2][3] = visionT[0][3] * visionT[2][1] - visionT[1][3] * visionT[0][1] - visionT[2][3] * visionT[1][1];
}


/**
 * @brief 计算轨迹点
 * 
 * @return true 
 * @return false 
 */
bool SLPClassdef::Calculate()
{
  /* 判断终点是否超限 */
  xyzTracGene(goalWorld, GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]);
  if (false == limitCheck(GoalxyzTrac[0], GoalxyzTrac[1], GoalxyzTrac[2]))
    return false;

  /* 计算中间点 */
  endEffLocCal();
  midPointCal();
  xyzTracGene(midWorld,MidxyzTrac[0],MidxyzTrac[1],MidxyzTrac[2]);
  /* 判断中间点是否超限 */
  return  limitCheck(MidxyzTrac[0], MidxyzTrac[1], MidxyzTrac[2]);
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
 * @brief 获取目标点
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
 * @brief 解算出小三轴末端相对于兑换站的xyz位置
 * @note 考虑到小三轴的位移很小，先试试不考虑小三轴末端的位移
 * @param yaw 
 * @param pitch 
 * @param roll 
 */
void SLPClassdef::endEffLocCal(float yaw, float pitch, float roll)
{
  /* 考虑小三轴位移 */
  /* code */
  /* 不考虑小三轴位移 TODO应该根据动作组移动到一个确定的位置 */
  endEffWorld[0] = 1;
  endEffWorld[1] = 1;
  endEffWorld[2] = 1;

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
 * @param mid 
 */
void SLPClassdef::midPointCal()
{
  uint8_t surfaceRes = decSurfaceCal(G);
  static float midGoal[3] = {0};
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
    break;
  }

  if(surfaceRes != 0)
  {
    midWorld[0] = TWorldGoal[0][0] * midGoal[0] + TWorldGoal[0][1] * midGoal[1] + TWorldGoal[0][2] * midGoal[2] + TWorldGoal[0][3];
    midWorld[1] = TWorldGoal[1][0] * midGoal[0] + TWorldGoal[1][1] * midGoal[1] + TWorldGoal[1][2] * midGoal[2] + TWorldGoal[1][3];
    midWorld[2] = TWorldGoal[2][0] * midGoal[0] + TWorldGoal[2][1] * midGoal[1] + TWorldGoal[2][2] * midGoal[2] + TWorldGoal[2][3];
  }
}

/**
 * @brief 产生平移轨迹
 * @note TODO常数项需要微调
 * @param point 世界坐标系下的x--y--z
 * @param lift 
 * @param extend 
 * @param translate 
 * @return true 
 * @return false 
 */
void SLPClassdef::xyzTracGene(float point[3], float &lift, float &extend, float &translate)
{
  lift = point[2] - 0.1f;
  extend = point[0] - 0.5f;
  translate = point[1] + 0.2f;
}

/**
 * @brief 判断关节位置有无超限
 * @note TODO 关节限位需要微调
 * @param lift 
 * @param extend 
 * @param translate 
 * @return true 
 * @return false 
 */
bool SLPClassdef::limitCheck(float &lift, float &extend, float &translate)
{
  if(0.0f <= lift      && lift      <= 0.61f
  && 0.0f <= extend    && extend    <= 0.339f
  && 0.0f <= translate && translate <= 0.348f)
  {
    return true;
  }
  else
    return false;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

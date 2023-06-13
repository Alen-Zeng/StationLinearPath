/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    StationLinearPath.h
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   Header file 
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
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/

#pragma pack(1)
typedef struct _VisionPackStructdef
{
  uint8_t visionMode;
  float qx;
  float qy;
  float qz;
  float qw;
  float x;
  float y;
  float z;
}VisionPackStructdef;
#pragma pack()


class SLPClassdef
{
private:
  float visionT[3][4];            //视觉旋转矩阵与位置
  float TWorldGoal[3][4];         //世界坐标系描述兑换站坐标系
  float TGoalWorld[3][4];         //兑换站坐标系描述世界坐标系
  float endEffWorld[3];           // 末端小三轴世界坐标系xyz
  float endEffGoal[3];            // 末端小三轴兑换站坐标系xyz
  float midWorld[3];              // 中间点xyz
  float goalWorld[3];              // 目标点xyz
  /* 在Goal坐标系下 */
  float O[3] = {0.288 + safeR, 0.144 + safeR, -0.144 - safeR};
  float P[3] = {0 - safeR, 0.144 + safeR, -0.144 - safeR};
  float Q[3] = {0 - safeR, -0.144 - safeR, -0.144 - safeR};
  float N[3] = {0.288 + safeR, -0.144 - safeR, -0.144 - safeR};
  float R[3] = {0.288 + safeR, 0.144 + safeR, 0.144 + safeR};
  float S[3] = {0 - safeR, 0.144 + safeR, 0.144 + safeR};
  float T[3] = {0 - safeR, -0.144 - safeR, 0.144 + safeR};
  float U[3] = {0.288 + safeR, -0.144 - safeR, 0.144 + safeR};
  float G[3] = {-safeR, 0, 0};    // 目标点xyz
  /* ↑↑在Goal坐标系下 */
  float MidxyzTrac[3];            //lift--extend--translate
  float GoalxyzTrac[3];           //lift--extend--translate

  void quaCoord2TMatrix(float qx,float qy,float qz,float qw,float x,float y,float z,float TMat[3][4]);
  void endEffLocCal(float yaw = 0.0f, float pitch = 0.0f, float roll = 0.0f);
  uint8_t decSurfaceCal(float endEffGoal[3]);
  void midPointCal();
  void xyzTracGene(float point[3],float &lift, float &extend, float &translate);
  bool limitCheck(float &lift, float &extend, float &translate);
public:
  float safeR = 0.2;

  SLPClassdef(){};
  ~SLPClassdef(){};

  void recVisionTarget(VisionPackStructdef &visionPack);
  bool Calculate();
  void getMidxyzTrac(float &lift, float &extend, float &translate);
  void getGoalxyzTrac(float &lift, float &extend, float &translate);
};


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

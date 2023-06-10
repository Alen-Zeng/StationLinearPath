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
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

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
  float visionT[3][4];
  float TWorldGoal[3][4];
  float TGoalWorld[3][4];
  float endEffWorld[3];
  float endEffGoal[3];
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
  float midWorld[3];                   // 中间点xyz

  void quaCoord2TMatrix(float qx,float qy,float qz,float qw,float x,float y,float z,float TMat[3][4]);
  void endEffLocCal(float yaw,float pitch,float roll);
  bool limitCheck(float point[3]);
public:
  float safeR = 0.2;
  bool CalSuccess = false;

  SLPClassdef(){};
  ~SLPClassdef(){};

  void recVisionTarget(VisionPackStructdef &visionPack);
  uint8_t decSurfaceCal(float endEffGoal[3]);
  void midPointGenerate(float mid[3]);
  bool xyzTracGenerate(float point[3],float &lift, float &extend, float &translate);
};





/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

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
  float VisionT[3][4];
  float TWorldGoal[3][4];
  float TGoalWorld[3][4];
  float SafeR;
  float EndEffWorld[3];
  float EndEffGoal[3];
  float O[3];
  float P[3];
  float Q[3];
  float N[3];
  float R[3];
  float S[3];
  float T[3];
  float U[3];
  float G[3];   //目标点xyz
  float Mid[3]; //中间点xyz

  void QuaCoord2TMatrix(float qx,float qy,float qz,float qw,float x,float y,float z,float TMat[3][4]);
public:
  bool CalSuccess = false;

  SLPClassdef(){};
  ~SLPClassdef(){};

  void recVisionTarget(VisionPackStructdef visionPack);
  uint8_t decSurfaceCal(float endEffGoal[3]);
};





/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

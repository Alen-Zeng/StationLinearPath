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
      -# 
      -# 
    @warning
      -# 
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
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

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
void SLPClassdef::QuaCoord2TMatrix(float qx, float qy, float qz, float qw, float x, float y, float z, float TMat[3][4])
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
void SLPClassdef::recVisionTarget(VisionPackStructdef visionPack)
{
  QuaCoord2TMatrix(visionPack.qx,visionPack.qy,visionPack.qz,visionPack.qw,visionPack.x,visionPack.y,visionPack.z,VisionT);
  TWorldGoal[0][0] = VisionT[2][2];   TWorldGoal[0][1] = -VisionT[2][0];    TWorldGoal[0][2] = -VisionT[2][1];     TWorldGoal[0][3] = VisionT[2][3];
  TWorldGoal[1][0] = -VisionT[0][2];   TWorldGoal[1][1] = VisionT[0][0];    TWorldGoal[1][2] = VisionT[0][1];     TWorldGoal[1][3] = -VisionT[0][3];
  TWorldGoal[2][0] = -VisionT[1][2];   TWorldGoal[2][1] = VisionT[1][0];    TWorldGoal[2][2] = VisionT[1][1];     TWorldGoal[2][3] = -VisionT[1][3];

  TGoalWorld[0][0] = VisionT[2][2];   TGoalWorld[0][1] = -VisionT[0][2];    TGoalWorld[0][2] = -VisionT[1][2];     TGoalWorld[0][3] = VisionT[1][3] * VisionT[0][2] - VisionT[0][3] * VisionT[2][2] + VisionT[2][3] * VisionT[1][2];
  TGoalWorld[1][0] = -VisionT[2][0];   TGoalWorld[1][1] = VisionT[0][0];    TGoalWorld[1][2] = VisionT[1][0];     TGoalWorld[1][3] = VisionT[0][3] * VisionT[2][0] - VisionT[1][3] * VisionT[0][0] - VisionT[2][3] * VisionT[1][0];
  TGoalWorld[2][0] = -VisionT[2][1];   TGoalWorld[2][1] = VisionT[0][1];    TGoalWorld[2][2] = VisionT[1][1];     TGoalWorld[2][3] = VisionT[0][3] * VisionT[2][1] - VisionT[1][3] * VisionT[0][1] - VisionT[2][3] * VisionT[1][1];
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

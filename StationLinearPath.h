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
#include "math.h"
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

typedef struct _SLPConstantStructdef
{
  /* 安全距离 */
  float safeR;
  /* 关节限位参数 */
  float liftMin;
  float liftMax;
  float extendMin;
  float extendMax;
  float translateMin;
  float translateMax;
  float yawMin;
  float yawMax;
  float pitchMin;
  float pitchMax;
  float rollMin;
  float rollMax;
  /* 小三轴变换矩阵中用到的常数 */
  float x1 = 0, y1 = 0, z1 = 0; // translate--yaw
  float x2 = 0, y2 = 0, z2 = 0; // yaw--pitch
  float x3 = 0;                 // pitch--roll
  /* 平移机构坐标原点与世界坐标原点的误差 */
  float errx;
  float erry;
  float errz;
  /* 视觉兑换自动组的平移常数项，需要结合动作组调整 */
  float autoConExt;
  float autoConTrans;
  float autoConLif;

  _SLPConstantStructdef(float safeR,
                       float liftMin, float liftMax,
                       float extendMin, float extendMax,
                       float translateMin, float translateMax,
                       float yawMin, float yawMax,
                       float pitchMin, float pitchMax,
                       float rollMin, float rollMax,
                       float x1, float y1, float z1,
                       float x2, float y2, float z2,
                       float x3,
                       float errx, float erry, float errz,
                       float autoConExt, float autoConTrans, float autoConLif) : 
                      safeR(safeR),
                      liftMin(liftMin), liftMax(liftMax),
                      extendMin(extendMin), extendMax(extendMax),
                      translateMin(translateMin), translateMax(translateMax),
                      yawMin(yawMin), yawMax(yawMax),
                      pitchMin(pitchMin), pitchMax(pitchMax),
                      rollMin(rollMin), rollMax(rollMax),
                      x1(x1), y1(y1), z1(z1),
                      x2(x2), y2(y2), z2(z2),
                      x3(x3),
                      errx(errx), erry(erry), errz(errz),
                      autoConExt(autoConExt), autoConTrans(autoConTrans), autoConLif(autoConLif) {}
}SLPConstantStructdef;

class SLPClassdef
{
private:
  /* 关节限位参数 */
  float liftMin;
  float liftMax;
  float extendMin;
  float extendMax;
  float translateMin;
  float translateMax;
  float yawMin;
  float yawMax;
  float pitchMin;
  float pitchMax;
  float rollMin;
  float rollMax;
  /* 小三轴变换矩阵中用到的常数 */
  float x1 = 0, y1 = 0, z1 = 0; // translate--yaw
  float x2 = 0, y2 = 0, z2 = 0; // yaw--pitch
  float x3 = 0;                 // pitch--roll
  /* 平移机构坐标原点与世界坐标原点的误差 */
  float errx;
  float erry;
  float errz;
  /* 视觉兑换自动组的平移常数项,需要结合动作组调整 */
  float autoConExt;
  float autoConTrans;
  float autoConLif;

  uint8_t surfaceRes;     // 决策面判断结果
  float visionT[3][4];    // 视觉旋转矩阵与位置
  float TWorldStation[3][4]; // 世界坐标系描述兑换站坐标系
  float TStationWorld[3][4]; // 兑换站坐标系描述世界坐标系
  float TWorldGoal[3][4]; // 世界坐标系描述目标点坐标系
  float TGoalWorld[3][4]; // 目标点坐标系描述世界坐标系
  float endEffWorld[3];   // 末端小三轴世界坐标系xyz
  float endEffGoal[3];    // 末端小三轴兑换站坐标系xyz
  float midWorld[3];      // 中间点xyz
  float goalWorld[3];     // 目标点xyz
  float stationWorld[3];  // 兑换站xyz
  /* 在Goal坐标系下 */
  float O[3] = {0.288f + safeR, 0.144f + safeR, -0.144f - safeR};
  float P[3] = {0.0f - safeR, 0.144f + safeR, -0.144f - safeR};
  float Q[3] = {0.0f - safeR, -0.144f - safeR, -0.144f - safeR};
  float N[3] = {0.288f + safeR, -0.144f - safeR, -0.144f - safeR};
  float R[3] = {0.288f + safeR, 0.144f + safeR, 0.144f + safeR};
  float S[3] = {0.0f - safeR, 0.144f + safeR, 0.144f + safeR};
  float T[3] = {0.0f - safeR, -0.144f - safeR, 0.144f + safeR};
  float U[3] = {0.288f + safeR, -0.144f - safeR, 0.144f + safeR};
  float G[3] = {-safeR, 0.0f, 0.0f}; // 目标点xyz
  /* ↑↑在Goal坐标系下 */
  float AttiTrac[3];    //小三轴yaw--pitch--roll目标
  float MidxyzTrac[3];  // lift--extend--translate
  float GoalxyzTrac[3]; // lift--extend--translate
  float StationxyzTrac[3]; // lift--extend--translate

  void quaCoord2TMatrix(float qx, float qy, float qz, float qw, float x, float y, float z, float TMat[3][4]);
  void attitudeCal(float &yaw, float &pitch, float &roll);
  void endEffLocCal(float _yaw = 0.0f, float _pitch = 0.0f, float _roll = 0.0f);
  uint8_t decSurfaceCal(float endEffGoal[3]);
  bool midPointCal();
  void xyzTracGene(float point[3], float _attiTrac[3], float &lift, float &extend, float &translate);
  bool limitCheck(float &_lift, float &_extend, float &_translate, float &_yaw, float &_pitch, float &_roll);

public:
  float safeR = 0.2;  //安全距离

  SLPClassdef(SLPConstantStructdef &_SLPCon);
  ~SLPClassdef(){};

  void recVisionTarget(VisionPackStructdef &visionPack);
  bool Calculate();
  void getAttitudeTrac(float &_yaw, float &_pitch, float &_roll);
  void getMidxyzTrac(float &lift, float &extend, float &translate);
  void getGoalxyzTrac(float &lift, float &extend, float &translate);
};


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

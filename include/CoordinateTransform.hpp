#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include <iostream>
#include <cmath>
#include <iomanip>
using namespace std;

// ===============================
// フラグ定義
// ===============================

// 座標系フラグ　（使ってはいない）
#define ECI_FLAG 0x0001        // 地球中心慣性座標系
#define ECEF_FLAG 0x0002       // 地球固定座標系
#define NCS_FLAG 0x0004        // 航法座標系
#define TSUKUTO_FLAG 0x0008    // TSUKUTO機体座標系

// センサータイプフラグ
#define MAG_FLAG 0x0010        // 地磁気ベクトル（ECI経由）
#define IMU_FLAG 0x0020        // 加速度ベクトル（ECEF経由）
#define SUNSENSOR_FLAG 0x0040  // 太陽方向ベクトル（ECI経由）

// センサー座標系フラグ
#define MAG_SENSOR_COORD_FLAG 0x0100      // 磁気センサ座標系
#define IMU_SENSOR_COORD_FLAG 0x0200      // IMU座標系
#define SUNSENSOR_1_COORD_FLAG 0x0400     // 太陽センサ1座標系
#define SUNSENSOR_2_COORD_FLAG 0x0800     // 太陽センサ2座標系
#define SUNSENSOR_3_COORD_FLAG 0x1000     // 太陽センサ3座標系

// ===============================
// クラス宣言
// ===============================
class CoordinateTransform {
private:
    float latitude, longitude;
    float roll, pitch, yaw;
    float current_time;

    void Multiply_Matrix(float AB[3][3], const float A[3][3], const float B[3][3]);
    
    // ECI→ECEF変換用の内部関数
    float Calculate_GMST(float unix_time);
    float Calculate_JulianDate(float unix_time);

public:
    CoordinateTransform();

    void Set_Position(float lat, float lon);
    void Set_Attitude(float r, float p, float y);
    void Set_Time(float t);

    void Get_CT_matrix(float ct_matrix[3][3], int flag);
    void Transform(float result[3][3], float input[3][3], int flag);
    void ShowMatrix(const float mat[3][3]) const;
    
    
};

#endif
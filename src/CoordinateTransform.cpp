#include "CoordinateTransform.hpp"

// ---------- コンストラクタ ----------
CoordinateTransform::CoordinateTransform()
    : latitude(0), longitude(0), roll(0), pitch(0), yaw(0), current_time(0) {}

// ---------- パラメータ設定 ----------
void CoordinateTransform::Set_Position(float lat, float lon) {
    latitude = lat;
    longitude = lon;
}

void CoordinateTransform::Set_Attitude(float r, float p, float y) {
    roll = r;
    pitch = p;
    yaw = y;
}

void CoordinateTransform::Set_Time(float t) {
    current_time = t;
}

// ---------- 行列積 　AにBを左からかけてくれる　（A、Bを受け取る = BA  であることに注意）----------
void CoordinateTransform::Multiply_Matrix(float BA[3][3], const float A[3][3], const float B[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            BA[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                BA[i][j] += B[i][k] * A[k][j];
        }
    }
}

// ---------- ユリウス日の計算 （今はよくわからないからここは未定）----------
float CoordinateTransform::Calculate_JulianDate(float unix_time) {
    // Unix時間（1970年1月1日 00:00:00からの秒数）をユリウス日に変換
    // Unix epoch のJD（1970-01-01 00:00:00）= 2440587.5
    return (unix_time / 86400.0) + 2440587.5;
}

// ---------- グリニッジ平均恒星時（GMST）の計算 ----------
float CoordinateTransform::Calculate_GMST(float unix_time) {
    // ユリウス日の計算
    float jd = Calculate_JulianDate(unix_time);
    
    // J2000.0（JD 2451545.0）からのユリウス世紀数（ここから先は確定のコードを書きたい）
    float T = (jd - 2451545.0) / 36525.0;
    
    // GMSTの計算（秒単位） - IAU 2000規約による
    float gmst_sec = 67310.54841 
                    + (876600.0 * 3600.0 + 8640184.812866) * T
                    + 0.093104 * T * T
                    - 6.2e-6 * T * T * T;
    
    // 86400秒（1日）での剰余を取る
    gmst_sec = fmod(gmst_sec, 86400.0);
    
    // 負の値を正に補正
    if (gmst_sec < 0) gmst_sec += 86400.0;
    
    // ラジアンに変換（2π rad = 86400秒）
    float gmst_rad = gmst_sec * (2.0 * M_PI / 86400.0);
    
    return gmst_rad;
}


// ---------- 変換行列の生成 ----------
void CoordinateTransform::Get_CT_matrix(float ct_matrix[3][3], int flag) {

    /// ECEF→ body   加速度ベクトル → Body ///
    if (flag ==IMU_FLAG ) {
        float sinl = sin(latitude), cosl = cos(latitude);
        float sinm = sin(longitude), cosm = cos(longitude);
        float sinp = sin(pitch), cosp = cos(pitch);
        float sinr = sin(roll), cosr = cos(roll);
        float siny = sin(yaw), cosy = cos(yaw);

        float C1[3][3] = {
            {-sinl * cosm, -sinl * sinm,  cosl},
            {     -sinm,        cosm,       0 },
            {-cosl * cosm, -cosl * sinm, -sinl}
        };

        float C3[3][3] = {
            { cosp * cosy,  cosp * siny,  -sinp },
            { -cosr * siny + sinr * sinp * cosy,
               cosr * cosy + sinr * sinp * siny,
               sinr * cosp },
            {  sinr * siny + cosr * sinp * cosy,
              -sinr * cosy + cosr * sinp * siny,
               cosr * cosp }
        };

        Multiply_Matrix(ct_matrix, C1, C3);
    }

    /// ECI → body  (太陽方向ベクトルと地磁気ベクトルからbodyへの変換行列の生成）///
    else if (flag == MAG_FLAG || flag == SUNSENSOR_FLAG){
        float gmst = Calculate_GMST(current_time);
    
        // θ + μ の計算
        float theta_plus_mu = gmst + longitude;
        float sin_tpm = sin(theta_plus_mu);
        float cos_tpm = cos(theta_plus_mu);
    
        float sinl = sin(latitude), cosl = cos(latitude);
        float sinp = sin(pitch), cosp = cos(pitch);
        float sinr = sin(roll), cosr = cos(roll);
        float siny = sin(yaw), cosy = cos(yaw);
    
    // C2: ECI → 航法座標系 
    float C2[3][3] = {
        {-sinl * cos_tpm, -sinl * sin_tpm,  cosl},
        {       -sin_tpm,         cos_tpm,     0},
        {-cosl * cos_tpm, -cosl * sin_tpm, -sinl}
    };
    
    // C3: 航法座標系 → 機体座標系 (再掲)
    float C3[3][3] = {
        { cosp * cosy,  cosp * siny,  -sinp },
        { -cosr * siny + sinr * sinp * cosy,
           cosr * cosy + sinr * sinp * siny,
           sinr * cosp },
        {  sinr * siny + cosr * sinp * cosy,
          -sinr * cosy + cosr * sinp * siny,
           cosr * cosp }
    };
    
    // C3 × C2 で座標変換行列が完成
    Multiply_Matrix(ct_matrix, C2, C3);
}


    /// IMU → boby ///
    else if (flag == IMU_SENSOR_COORD_FLAG) {
        float R1[3][3] = {
            { 0, -1,  0 },
            { 1,  0,  0 },
            { 0,  0,  1 }
        };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ct_matrix[i][j] = R1[i][j];
    }

    /// 太陽センサ1 → Body ///
    else if (flag == SUNSENSOR_1_COORD_FLAG ) {
        float Rs1[3][3] = {
            { 0,  0,  1 },
            { 0,  1,  0 },
            {-1,  0,  0 }
        };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ct_matrix[i][j] = Rs1[i][j];
    }

    /// 太陽センサ2 → Body ///
    else if (flag == SUNSENSOR_2_COORD_FLAG) {
        float Rs2[3][3] = {
            { 1,  0,  0 },
            { 0,  0, -1 },
            { 0,  1,  0 }
        };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ct_matrix[i][j] = Rs2[i][j];
    }

    /// 太陽センサ3 → Body ///
    else if (flag == SUNSENSOR_3_COORD_FLAG) {
        float Rs3[3][3] = {
            { 1,  0,  0 },
            { 0,  1,  0 },
            { 0,  0,  1 }
        };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ct_matrix[i][j] = Rs3[i][j];
    }
    
    /// 地磁気センサ → Body ///
    else if (flag == MAG_SENSOR_COORD_FLAG) {
        float Rmg[3][3] = {
            { -1,  0,  0 },
            { 0,  -1,  0 },
            { 0,   0,  1 }
        };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ct_matrix[i][j] = Rmg[i][j];
    }
    else {
        cerr << "Error: Unknown flag." << endl;
    }
}

// ---------- 任意の変換行列との掛け算をする ----------
void CoordinateTransform::Transform(float result[3][3], float input[3][3], int flag) {
    float ct_matrix[3][3];
    Get_CT_matrix(ct_matrix, flag);
    Multiply_Matrix(result, input, ct_matrix);
}

// ---------- 出力 ----------
void CoordinateTransform::ShowMatrix(const float mat [3][3]) const {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            cout << setw(10) << fixed << setprecision(6) << mat[i][j] << " ";
        cout << "\n";
    }
}
#include "CoordinateTransform.hpp"
#include <ctime>

// 日時からUnix時間に変換する関数　（ここも未定）
float dateTimeToUnixTime(int year, int month, int day, int hour, int minute, int second) {
    struct tm timeinfo = {};
    timeinfo.tm_year = year - 1900;  // 1900年からの年数
    timeinfo.tm_mon = month - 1;     // 0-11の月
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = second;
    timeinfo.tm_isdst = -1;          // 夏時間の自動判定
    
    // UTCとして扱う
    time_t t = mktime(&timeinfo);
    
    // ローカル時間との差を補正してUTCにする
    struct tm* utc_time = gmtime(&t);
    time_t utc_t = mktime(utc_time);
    t = t + (t - utc_t);
    
    return static_cast<float>(t);
}

int main() {
    CoordinateTransform ct;
    float lat, lon, roll, pitch, yaw;
    float input[3][3];
    float result[3][3];
    int year, month, day, hour, minute, second;
    int flag;

    // フラグ入力
    cout<<"Input flag (10)"<<endl;
    cin >> flag;

    // 各モードごとの処理
    if (flag == IMU_FLAG) {
        // ECEF →　機体座標系　：　緯度・経度を入力
        cout<<"Input latitude and longitude (rad): "<<endl;
        cin >> lat >> lon ;
        // 姿勢角を入力
        cout<<"Input roll, pitch, yaw (rad):"<<endl;
        cin >> roll >> pitch >> yaw;
        ct.Set_Position(lat, lon);
        ct.Set_Attitude(roll, pitch, yaw);
    }
    else if (flag == MAG_FLAG || flag == SUNSENSOR_FLAG ) {
        // ECI→ECEF変換：年月日時分秒で入力
        cout<<"Input year, month, day, hour, minute, second:"<<endl;
        cin >> year >> month >> day >> hour >> minute >> second;
        // ECEF →　機体座標系　：　緯度・経度を入力
        cout<<"Input latitude and longitude (rad): "<<endl;
        cin >> lat >> lon ;
        // 姿勢角を入力
        cout<<"Input roll, pitch, yaw (rad):"<<endl;
        cin >> roll >> pitch >> yaw;
        
        float unix_time = dateTimeToUnixTime(year, month, day, hour, minute, second);
        ct.Set_Time(unix_time);
        ct.Set_Position(lat, lon);      
        ct.Set_Attitude(roll, pitch, yaw);
        
        cout << "Input: " << year << "/" << month << "/" << day 
             << " " << hour << ":" << minute << ":" << second << " UTC" << endl;
        cout << "Unix time: " << unix_time << endl;
    }
    else if (flag == IMU_SENSOR_COORD_FLAG||
             flag == SUNSENSOR_1_COORD_FLAG||
             flag == SUNSENSOR_2_COORD_FLAG ||
             flag == SUNSENSOR_3_COORD_FLAG ||
             flag == MAG_SENSOR_COORD_FLAG) {
        // 角度入力なし
    }
    else {
        cerr << "Error: Unknown flag (" << flag << ")" << endl;
        return 1;
    }

    // 行列入力（3×3行列）
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cin >> input[i][j];

    // 計算 & 表示
    ct.Transform(result, input, flag);
    ct.ShowMatrix(result);

    return 0;
}
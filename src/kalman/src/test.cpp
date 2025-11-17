#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <vector>

// 定义宏，将弧度转换为角度
#define RAD2DEG(x) ((x)*180.0 / M_PI)

/**
 * @brief 四元数转换为欧拉角 (Roll, Pitch, Yaw)
 * @param w, x, y, z 输入四元数分量
 * @param roll, pitch, yaw 输出欧拉角（弧度制）
 */
void quaternionToEuler(double w, double x, double y, double z, double& roll, double& pitch, double& yaw)
{
    const double PI        = 3.14159265358979323846;
    const double Epsilon   = 0.0009765625;
    const double Threshold = 0.5 - Epsilon;

    double TEST = w * y - x * z;

    // 处理万向节死锁（Gimbal Lock）或奇异姿态：pitch ≈ ±90°
    if (TEST < -Threshold || TEST > Threshold)
    {
        // 手动 Sign 函数
        int sign = (TEST > 0.0) ? 1 : -1;

        yaw   = -2.0 * sign * std::atan2(x, w);  // Z: yaw
        pitch = sign * (PI / 2.0);               // Y: pitch = ±90°
        roll  = 0.0;                             // X: roll = 0
    }
    else  // 正常情况
    {
        // roll = atan2(2*(y*z + w*x), w² - x² - y² + z²)
        roll = std::atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);

        // pitch = asin(-2*(x*z - w*y))
        double sinp = -2.0 * (x * z - w * y);
        if (sinp < -1.0)
            sinp = -1.0;
        else if (sinp > 1.0)
            sinp = 1.0;
        pitch = std::asin(sinp);

        // yaw = atan2(2*(x*y + w*z), w² + x² - y² - z²)
        yaw = std::atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
    }

    // 可选：归一化到 [-π, π]
    auto normalize = [&](double& angle) {
        while (angle > PI)
            angle -= 2.0 * PI;
        while (angle <= -PI)
            angle += 2.0 * PI;
    };
    normalize(roll);
    normalize(pitch);
    normalize(yaw);
}

int main()
{
    // 输入输出文件路径
    std::string inputPath  = "/mnt/planB/planning_explore/config/txt_imu.txt";
    std::string outputPath = "/mnt/planB/planning_explore/config/output_rpy_and_quat.txt";  // 修改输出文件名

    std::ifstream inputFile(inputPath);
    std::ofstream outputFile(outputPath);

    if (!inputFile.is_open())
    {
        std::cerr << "Error: Cannot open input file: " << inputPath << std::endl;
        return 1;
    }
    if (!outputFile.is_open())
    {
        std::cerr << "Error: Cannot open output file: " << outputPath << std::endl;
        return 1;
    }

    // 设置浮点精度
    outputFile << std::fixed << std::setprecision(6);

    // 【修改表头】同时保存四元数和欧拉角
    outputFile << "Quat_X\tQuat_Y\tQuat_Z\tQuat_W\tRoll(deg)\tPitch(deg)\tYaw(deg)\n";

    std::string line;
    int         lineNum = 0;

    while (std::getline(inputFile, line))
    {
        lineNum++;
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream   ss(line);
        std::vector<double> values;
        double              val;
        while (ss >> val)
        {
            values.push_back(val);
        }

        if (values.size() < 4)
        {
            std::cerr << "Skip line " << lineNum << ": less than 4 values.\n";
            continue;
        }

        // 【修改逻辑】：确定地取最后四列作为 x, y, z, w
        double q_x_raw = values[values.size() - 4];  // 倒数第 4 列: X
        double q_y_raw = values[values.size() - 3];  // 倒数第 3 列: Y
        double q_z_raw = values[values.size() - 2];  // 倒数第 2 列: Z
        double q_w_raw = values[values.size() - 1];  // 倒数第 1 列: W

        // 归一化 (用于计算 RPY)
        double norm = std::sqrt(q_w_raw * q_w_raw + q_x_raw * q_x_raw + q_y_raw * q_y_raw + q_z_raw * q_z_raw);
        double q_w  = q_w_raw;
        double q_x  = q_x_raw;
        double q_y  = q_y_raw;
        double q_z  = q_z_raw;

        if (norm > 1e-6)
        {
            q_w /= norm;
            q_x /= norm;
            q_y /= norm;
            q_z /= norm;
        }

        double roll, pitch, yaw;
        // 注意：传入的参数顺序是 (w, x, y, z)
        quaternionToEuler(q_w, q_x, q_y, q_z, roll, pitch, yaw);

        // 【修改输出】：先写入原始四元数（未归一化），再写入 RPY
        outputFile << q_x_raw << "\t" << q_y_raw << "\t" << q_z_raw << "\t" << q_w_raw << "\t" << RAD2DEG(roll) << "\t" << RAD2DEG(pitch) << "\t"
                   << RAD2DEG(yaw) << "\n";
    }

    std::cout << "Success! Quaternions (x, y, z, w) and RPY saved to: " << outputPath << std::endl;
    return 0;
}
#include "ekf.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace Eigen;

/**
 * @brief 结构体，用于存储每一帧 Odometry 数据
 */
struct OdometryData
{
    double      timestamp;
    Vector3d    p;  // position (x, y, z)
    Quaterniond q;  // quaternion (x, y, z, w)
};

/**
 * @brief 从 txt 文件读取 Odometry 数据
 * @param filename 文件名
 * @param data_vec 存储数据的向量
 * @return bool 是否成功读取
 */
bool readOdometryData(const std::string& filename, std::vector<OdometryData>& data_vec)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    std::string line;
    // 跳过可能的标题行
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::stringstream   ss(line);
        std::string         token;
        std::vector<double> values;

        // 读取所有列
        while (std::getline(ss, token, ' '))
        {  // 假设数据用空格分隔
            try
            {
                values.push_back(std::stod(token));
            }
            catch (const std::exception& e)
            {
                // 忽略无效或不完整的行
                continue;
            }
        }

        // 检查列数是否正确 (8列: ts, px, py, pz, qx, qy, qz, qw)
        if (values.size() >= 8)
        {
            OdometryData data;
            data.timestamp = values[0];
            data.p         = Vector3d(values[1], values[2], values[3]);
            // 注意四元数的顺序: txt中是 (x, y, z, w)，Eigen::Quaterniond 构造函数是 (w, x, y, z) 或用 setFrom
            // 这里我们直接用 x, y, z, w 赋值
            data.q.x() = values[4];
            data.q.y() = values[5];
            data.q.z() = values[6];
            data.q.w() = values[7];
            data.q.normalize();  // 确保四元数有效

            data_vec.push_back(data);
        }
    }
    std::cout << "Successfully read " << data_vec.size() << " odometry records." << std::endl;
    return true;
}

/**
 * @brief 将结果写入 CSV 文件供 Python 脚本绘图
 * @param filename 输出文件名
 * @param raw_data 原始数据
 * @param est_states 估计的状态
 * @return bool 是否成功写入
 */
bool writeResultsToCSV(const std::string& filename, const std::vector<OdometryData>& raw_data, const std::vector<Matrix<double, EKF::N, 1>>& est_states)
{
    if (raw_data.size() != est_states.size())
    {
        std::cerr << "Error: Raw data size and estimated states size mismatch." << std::endl;
        return false;
    }

    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return false;
    }

    // 设置输出精度
    file << std::fixed << std::setprecision(12);

    // 写入 CSV 标题
    file << "Timestamp,px_raw,py_raw,pz_raw,r_raw,p_raw,y_raw,px_est,py_est,pz_est,r_est,p_est,y_est\n";

    for (size_t i = 0; i < raw_data.size(); ++i)
    {
        const auto& raw = raw_data[i];
        const auto& est = est_states[i];

        // 原始数据: Quaternion -> RPY
        Eigen::Vector3d rpy_raw = quaternionToRPY(raw.q);

        // 估计数据: State (x) -> RPY
        Eigen::Quaterniond q_est(est(6), est(3), est(4), est(5));  // w, x, y, z
        Eigen::Vector3d    rpy_est = quaternionToRPY(q_est);

        // 写入一行数据
        file << raw.timestamp << "," << raw.p.x() << "," << raw.p.y() << "," << raw.p.z() << "," << rpy_raw[0] << "," << rpy_raw[1] << "," << rpy_raw[2] << ","
             << est(0) << "," << est(1) << "," << est(2) << "," << rpy_est[0] << "," << rpy_est[1] << "," << rpy_est[2] << "\n";
    }

    std::cout << "Successfully wrote results to " << filename << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    // 检查参数
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_input_txt_file>" << std::endl;
        return 1;
    }

    std::string input_filename  = argv[1];
    std::string output_filename = "/mnt/planB/planning_explore/config/ekf.csv";

    // 1. 读取数据
    std::vector<OdometryData> raw_data;
    if (!readOdometryData(input_filename, raw_data) || raw_data.empty())
    {
        return 1;
    }

    // 2. 初始化 EKF
    EKF ekf;
    ekf.setInitialState(raw_data[0].p, raw_data[0].q);

    double                                 last_timestamp = raw_data[0].timestamp;
    std::vector<Matrix<double, EKF::N, 1>> estimated_states;
    estimated_states.push_back(ekf.getState());

    // 3. 循环处理数据
    for (size_t i = 1; i < raw_data.size(); ++i)
    {
        const auto& current_data = raw_data[i];
        double      dt           = current_data.timestamp - last_timestamp;

        if (dt < 0)
        {
            std::cerr << "Warning: Time went backwards. Skipping data point " << i << std::endl;
            last_timestamp = current_data.timestamp;
            continue;
        }

        // --- 预测步骤 ---
        ekf.predict(dt);

        // --- 更新步骤 ---
        // 观测向量 z = [px, py, pz, qx, qy, qz, qw]^T
        Matrix<double, EKF::M, 1> z;
        z.block<3, 1>(0, 0) = current_data.p;
        z(3)                = current_data.q.x();
        z(4)                = current_data.q.y();
        z(5)                = current_data.q.z();
        z(6)                = current_data.q.w();

        ekf.update(z);

        estimated_states.push_back(ekf.getState());
        last_timestamp = current_data.timestamp;
    }

    // 4. 将结果写入 CSV 文件
    if (!writeResultsToCSV(output_filename, raw_data, estimated_states))
    {
        return 1;
    }

    std::cout << "\nEKF filtering complete. Run the Python script to visualize." << std::endl;

    return 0;
}
#include <iostream>
#include <vector>
#include <iomanip>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/shape/sphere.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;

// 获取轨迹信息
bool read_trajectory(vector<fcl::Vector3d>& trajectory, string& txt_path);
// 计算所有轨迹点距离障碍的最近距离
vector<double> Cal_collision_distance(vector<fcl::Vector3d>& trajectory, vector<fcl::Vector3<double>>& b_size, vector<fcl::Vector3<double>>& b_position);
// 可视化
void scene_vis(pcl::visualization::PCLVisualizer::Ptr vis, vector<fcl::Vector3d>& trajectory, vector<fcl::Vector3<double>>& b_size, vector<fcl::Vector3<double>>& b_position, double danger_dis, vector<double>& all_dis);


// box 尺寸
vector<fcl::Vector3<double>> boxs_size{ {50.0, 50.0, 0.0 } };
//{ 50.0, 50.0, 20.0 }};
// box 中心点位置
vector<fcl::Vector3<double>> boxs_position{ {73.785, -15.0, 0.0 } };
//{ 90.0, 50.0, 18.0 } };

double safe_distance = 3.0;

// 轨迹地址
string txt_path = "your_file_path\\path1_data_xyz.txt";

int main(int argc, char** argv) {
    vector<fcl::Vector3d> trajectory;
    vector<double> trajectory_min_dis;
    trajectory_min_dis.resize(trajectory.size());
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Trajectory Visualizer"));
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->addCoordinateSystem(100.0);

    read_trajectory(trajectory, txt_path);
    trajectory_min_dis = Cal_collision_distance(trajectory, boxs_size, boxs_position);

    auto minDistance = min_element(trajectory_min_dis.begin(), trajectory_min_dis.end());

    cout << "minDistance= " << *minDistance << endl;

    scene_vis(visualizer, trajectory, boxs_size, boxs_position, safe_distance, trajectory_min_dis);

    visualizer->setCameraPosition(200, 15, 45, 0, 1, 0, 0, 0, 1);
    // 显示可视化窗口
    while (!visualizer->wasStopped()) {
        visualizer->spinOnce(100);
    }
    return 0;
}


bool read_trajectory(vector<fcl::Vector3d>& trajectory, string& txt_path) {

    // 读取txt文件
    ifstream file(txt_path);
    if (!file.is_open()) {
        cerr << "Failed to open file." << endl;
        return false;
    }
    // 获取每行
    string line;
    while (getline(file, line)) {
        if (line.size() >= 2) {
            // 去掉第一个字符 '[' 和最后一个字符 ']'
            line = line.substr(1, line.size() - 2);
        }
        else {
            cerr << "Line is too short: " << line << endl;
            continue;
        }
        stringstream ss(line);
        string token;
        vector<string> tokens;

        // 逐个逗号分隔的值
        while (getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() != 3) {
            cerr << "Invalid line format: " << line << endl;
            continue;
        }

        // 转换为浮点数
        double x = stod(tokens[0]);
        double y = stod(tokens[1]);
        // double z = stod(tokens[2]);
        double z = 0;

        // 添加到向量
        trajectory.emplace_back(x, y, z);
    }
    cout << "Successfully acquired all " << trajectory.size() << " track points" << endl;
}

vector<double> Cal_collision_distance(vector<fcl::Vector3d>& trajectory, vector<fcl::Vector3<double>>& b_size, vector<fcl::Vector3<double>>& b_position) {
    // 所有碰撞对象
    vector<std::shared_ptr<fcl::CollisionObjectd>> objects;
    // 
    for (int i = 0; i < b_position.size(); ++i) {
        shared_ptr<fcl::Boxd> box = make_shared<fcl::Boxd>(b_size.at(i));
        fcl::CollisionObjectd obj(box);
        obj.setTranslation(b_position.at(i));
        objects.push_back(make_shared<fcl::CollisionObjectd>(obj));
        cout << "create the " << "box " << objects.size() << endl;
    }

    vector<double> distance;
    // 计算轨迹点到障碍的最近距离
    for (auto iter = trajectory.begin(); iter != trajectory.end(); iter++) {
        shared_ptr<fcl::Sphered> sphere = make_shared<fcl::Sphered>(0.0);
        fcl::CollisionObjectd query_object(sphere);
        query_object.setTranslation(*iter);

        double min_distance = DBL_MAX;
        for (auto obj : objects) {
            fcl::DistanceResultd distance_result;
            fcl::DistanceRequestd distance_request;
            distance_request.enable_nearest_points = true;
            // 计算距离
            double point_distance = fcl::distance(&query_object, obj.get(), distance_request, distance_result);

            if (point_distance < 0) {
                min_distance = point_distance;
                break;
            }
            else if (point_distance < min_distance) {
                min_distance = point_distance;
            }
        }
        distance.push_back(min_distance);
        // cout << min_distance << endl;
    }
    return distance;
}

void scene_vis(pcl::visualization::PCLVisualizer::Ptr vis, vector<fcl::Vector3d>& trajectory, vector<fcl::Vector3<double>>& b_size, vector<fcl::Vector3<double>>& b_position, double danger_dis, vector<double>& all_dis) {
    // 将轨迹点转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr safe_trajectory_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr danger_trajectory_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < trajectory.size(); ++i) {
        pcl::PointXYZ p;
        p.x = trajectory.at(i)[0];
        p.y = trajectory.at(i)[1];
        p.z = trajectory.at(i)[2];
        if (all_dis.at(i) > danger_dis) {
            safe_trajectory_cloud->push_back(p);
        }
        else {
            danger_trajectory_cloud->push_back(p);
            // cout << "[" << setprecision(6) << p.x << "," << p.y << "," << p.z << "]" << endl;
            printf("[%.6f,%.6f], dis = %f\n", p.x, p.y, all_dis.at(i));
        }
    }

    for (int i = 0; i < b_position.size(); ++i) {
        // 定义立方体的旋转（无旋转）
        Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
        Eigen::Vector3f center(b_position.at(i)[0], b_position.at(i)[1], b_position.at(i)[2]);

        string id = "cube_" + to_string(i);

        vis->addCube(center, rotation, b_size.at(i)[0], b_size.at(i)[1], b_size.at(i)[2], id);
        vis->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.4, 0.0, id);
        vis->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id);
    }

    // 将轨迹点添加到可视化窗口
    vis->addPointCloud<pcl::PointXYZ>(safe_trajectory_cloud, "safe_trajectory");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "safe_trajectory");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "safe_trajectory");

    vis->addPointCloud<pcl::PointXYZ>(danger_trajectory_cloud, "anger_trajectory");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "anger_trajectory");
    vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "anger_trajectory");

}

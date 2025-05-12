#include <iostream>
#include <vector>
#include <iomanip>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/shape/sphere.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <time.h>  
using namespace std;


// 获取轨迹信息
bool read_trajectory(pcl::PointCloud<pcl::PointXYZ>::Ptr& trajectory, string& txt_path);
// 轨迹生成凸包
void generate_convex(pcl::PointCloud<pcl::PointXYZ>::Ptr& trajectory, std::vector<pcl::Vertices>& polygons, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

// box 尺寸
vector<fcl::Vector3<double>> boxs_size{ {50.0, 50.0, 20.0 } };
//{ 50.0, 50.0, 20.0 }};
// box 中心点位置
vector<fcl::Vector3<double>> boxs_position{ {115.0, -10.0, -15.0 } };
//{ 90.0, 50.0, 18.0 } };
double safe_distance = 3.0;
// 轨迹地址
string txt_path = "your_file_path\\path3_data_xyz.txt";

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZ>);
    read_trajectory(trajectory, txt_path);


    double start, mid,end;
    start = clock();
    std::vector<pcl::Vertices> polygons;
  
    // polygons保存的是所有凸包多边形的顶点在surface_hull中的下标
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    // surface_hull是所有凸包多边形的顶点
    generate_convex(trajectory,polygons,surface_hull); 
    mid = clock();
    cout <<  "From points to convex hull have cost  " << mid - start << "ms" << endl;
    // 顶点转换换为fcl格式
    std::vector<fcl::Vector3d> hull_point;
    for (auto point : *surface_hull) {
        hull_point.push_back(fcl::Vector3d{point.x, point.y, point.z});
    }
    // 平面索引转换为fcl格式
    std::vector<int> polygon_index;
    for (auto ver : polygons) {
        // fcl的平面索引格式为[构成平面点数 点1索引 点2索引 点3索引]
        polygon_index.push_back(3);
        polygon_index.push_back(ver.vertices.at(0));
        polygon_index.push_back(ver.vertices.at(1));
        polygon_index.push_back(ver.vertices.at(2));
    }

    // 创建FCL凸包对象
    std::shared_ptr<const std::vector<fcl::Vector3d>> vertices_ptr = std::make_shared<const std::vector<fcl::Vector3d>>(hull_point);
    std::shared_ptr<const std::vector<int>> faces_ptr = std::make_shared<const std::vector<int>>(polygon_index);
    shared_ptr<fcl::Convex<double>> convex = make_shared<fcl::Convex<double>>(vertices_ptr, polygons.size() , faces_ptr, true);
    fcl::CollisionObject<double> convex_obj(convex);

    // 构建box 对象
    shared_ptr<fcl::Boxd> box = make_shared<fcl::Boxd>(boxs_size.at(0));
    fcl::CollisionObjectd box_obj(box);
    box_obj.setTranslation(boxs_position.at(0));

    // 创建距离请求和结果
    fcl::DistanceRequest<double> request;
    fcl::DistanceResult<double> result;

    // 计算距离
    fcl::distance(&convex_obj, &box_obj, request, result);

    end = clock();

    cout << "minDistance= " << result.min_distance << ",time: " << end - start << "ms" << endl;


    // ---------------------- Visualizer -------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_3d(new pcl::visualization::PCLVisualizer("surface_3d"));
    viewer_3d->setBackgroundColor(0, 0, 0);
    viewer_3d->addCoordinateSystem(100.0);
    viewer_3d->addPointCloud<pcl::PointXYZ>(surface_hull, "cloud_3d");
    viewer_3d->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_3d");
    viewer_3d->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polygons_3d");
    viewer_3d->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_3d");
    // viewer_3d->setRepresentationToWireframeForAllActors();
    // 
        // 定义立方体的旋转（无旋转）
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f center(boxs_position.at(0)[0], boxs_position.at(0)[1], boxs_position.at(0)[2]);

    string id = "cube_" + to_string(0);

    viewer_3d->addCube(center, rotation, boxs_size.at(0)[0], boxs_size.at(0)[1], boxs_size.at(0)[2], id);
    viewer_3d->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.4, 0.0, id);
    viewer_3d->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id);

    viewer_3d->setCameraPosition(200, 15, 45, 0, 1, 0, 0, 0, 1);
    while (!viewer_3d->wasStopped())
    {
        viewer_3d->spinOnce(100);
    }

    return 0;
}


bool read_trajectory(pcl::PointCloud<pcl::PointXYZ>::Ptr& trajectory, string& txt_path) {

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
        double z = stod(tokens[2]);

        // 添加到向量
        trajectory->emplace_back(x, y, z);
    }
    cout << "Successfully acquired all " << trajectory->size() << " track points" << endl;
}

void generate_convex(pcl::PointCloud<pcl::PointXYZ>::Ptr& trajectory, std::vector<pcl::Vertices>& polygons, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    // 创建凸包
    pcl::ConvexHull<pcl::PointXYZ> hull;
    // 放置点云信息
    hull.setInputCloud(trajectory);
    // 设置凸包维度
    hull.setDimension(2);  
    // 不计算凸包面积
    hull.setComputeAreaVolume(false);

    // 生成凸包 主要是顶点和构成凸包平面的顶点索引
    hull.reconstruct(*cloud_out, polygons);

    cout << "凸包边界点的数量：" << cloud_out->size() << " 凸包平面数量：" << polygons.size() << endl;
    //for (auto ver : polygons) {
    //    printf("凸包平面索引：[%3d,%3d,%3d]\r\n", ver.vertices.at(0), ver.vertices.at(1), ver.vertices.at(2));
    //}
}
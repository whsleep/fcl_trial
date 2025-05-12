#include <iostream>
#include <chrono>
#include <vector>
#include <time.h>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Portal.h>
#include <Portal_Manager.h>


using namespace std;



void draw_Portal(int id, pcl::visualization::PCLVisualizer::Ptr& vis, pcl::PointXYZ base, vector<pcl::PointXYZ> connect, pcl::PointXYZ end);
void dynamic_part(int id, pcl::visualization::PCLVisualizer::Ptr& vis, vector<pcl::PointXYZ> connect, pcl::PointXYZ end);
void updateDynamicPart(int id, pcl::visualization::PCLVisualizer::Ptr& vis, std::vector<pcl::PointXYZ> connect, pcl::PointXYZ end);

int PortalCrane::Portal::id = 0;

int main(int argc, char** argv) {

    double targer = 6;
    double times=1000;
    int cnt = 0;

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Visualizer"));
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->addCoordinateSystem(10.0);

    // 0.8415 -1.1515
    shared_ptr<PortalCrane::Portal> portal1 = make_shared<PortalCrane::Portal>(Eigen::Vector2d(0, 0), 0);
    shared_ptr<PortalCrane::Portal> portal2 = make_shared<PortalCrane::Portal>(Eigen::Vector2d(45, 0), -1.1515);
    
    vector<shared_ptr<PortalCrane::Portal>> portals;
    portals.push_back(portal1);
    portals.push_back(portal2);

    PortalCrane::Portal_Manager manager(portals);

    draw_Portal(portal1->personal_id, visualizer, portal1->get_base(), portal1->get_connect(), portal1->get_end());
    draw_Portal(portal2->personal_id, visualizer, portal2->get_base(), portal2->get_connect(), portal2->get_end());
    visualizer->spinOnce();
    double start, mid, end;
    start = clock();
    while (!visualizer->wasStopped()) {
        cnt++;
        portals.at(0)->set_state(targer * cnt / times);
        portals.at(1)->set_state(targer * (times - cnt) / times);
        manager.cal_distance();
        if (cnt % 10==0) { 
            updateDynamicPart(portal1->personal_id, visualizer, portal1->get_connect(), portal1->get_end());
            updateDynamicPart(portal2->personal_id, visualizer, portal2->get_connect(), portal2->get_end());
            visualizer->spinOnce();
        }
        if (cnt > times)
            break;
    }
    end = clock();
    cout << "Calculate diatance cost " << end - start << "ms" << endl;
    return 0;
}
  
void draw_Portal(int id, pcl::visualization::PCLVisualizer::Ptr& vis, pcl::PointXYZ base, vector<pcl::PointXYZ> connect, pcl::PointXYZ end) {
    // 定义圆的参数
    double center_x = base.x;
    double center_y = base.y;
    double center_z = 0.0;
    double radius = base.z;
    int num_points = 100;
    // 生成圆形的点
    std::vector<pcl::PointXYZ> points;
    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points; // 角度从 0 到 2π
        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);
        double z = center_z; // 圆在 z=0 平面上
        points.push_back(pcl::PointXYZ(x, y, z));
    }

    // 添加线段到可视化窗口
    for (size_t i = 0; i < points.size() - 1; ++i) {
        std::stringstream ss;
        ss << to_string(id) + "line_" << i;
        vis->addLine<pcl::PointXYZ>(points[i], points[i + 1], 1.0, 0.0, 0.0, ss.str());
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, ss.str());
    }
    dynamic_part(id,vis,connect,end);
}

void dynamic_part(int id, pcl::visualization::PCLVisualizer::Ptr& vis, vector<pcl::PointXYZ> connect, pcl::PointXYZ end) {

    vis->addLine(connect.at(0), connect.at(1), "1_" + to_string(id));
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "1_" + to_string(id));
    vis->addLine(connect.at(1), connect.at(2), "2_" + to_string(id));
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "2_" + to_string(id));
    vis->addLine(connect.at(2), connect.at(3), "3_" + to_string(id));
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "3_" + to_string(id));
    vis->addLine(connect.at(3), connect.at(0), "4_" + to_string(id));
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "4_" + to_string(id));

    // 定义圆的参数
    double center_x = end.x;
    double center_y = end.y;
    double center_z = 0.0;
    double radius = end.z;
    int num_points = 100;
    // 生成圆形的点
    std::vector<pcl::PointXYZ> points;
    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points; // 角度从 0 到 2π
        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);
        double z = center_z; // 圆在 z=0 平面上
        points.push_back(pcl::PointXYZ(x, y, z));
    }

    // 添加线段到可视化窗口
    for (size_t i = 0; i < points.size() - 1; ++i) {
        std::stringstream ss;
        ss << to_string(id) + "end_line_" << i;
        vis->addLine<pcl::PointXYZ>(points[i], points[i + 1], 1.0, 0.0, 0.0, ss.str());
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, ss.str());
    }
}

// 更新动态部分的函数
void updateDynamicPart(int id, pcl::visualization::PCLVisualizer::Ptr& vis, std::vector<pcl::PointXYZ> connect, pcl::PointXYZ end) {

    // 定义线段的ID
    std::string line1_id = "1_" + std::to_string(id);
    std::string line2_id = "2_" + std::to_string(id);
    std::string line3_id = "3_" + std::to_string(id);
    std::string line4_id = "4_" + std::to_string(id);

    // 删除旧的线段
    vis->removeShape(line1_id);
    vis->removeShape(line2_id);
    vis->removeShape(line3_id);
    vis->removeShape(line4_id);

    // 重新绘制新的线段
    vis->addLine(connect.at(0), connect.at(1), line1_id);
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, line1_id);
    vis->addLine(connect.at(1), connect.at(2), line2_id);
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, line2_id);
    vis->addLine(connect.at(2), connect.at(3), line3_id);
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, line3_id);
    vis->addLine(connect.at(3), connect.at(0), line4_id);
    vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, line4_id);

    // 定义圆的参数
    double center_x = end.x;
    double center_y = end.y;
    double center_z = 0.0;
    double radius = end.z;
    int num_points = 100;

    // 生成圆形的点
    std::vector<pcl::PointXYZ> points;
    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points; // 角度从 0 到 2π
        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);
        double z = center_z; // 圆在 z=0 平面上
        points.push_back(pcl::PointXYZ(x, y, z));
    }

    // 删除旧的圆形线段
    for (size_t i = 0; i < points.size() - 1; ++i) {
        std::stringstream ss;
        ss << std::to_string(id) + "end_line_" << i;
        vis->removeShape(ss.str());
    }

    // 重新绘制新的圆形线段
    for (size_t i = 0; i < points.size() - 1; ++i) {
        std::stringstream ss;
        ss << std::to_string(id) + "end_line_" << i;
        vis->addLine(points[i], points[i + 1], 1.0, 0.0, 0.0, ss.str());
        vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, ss.str());
    }
}
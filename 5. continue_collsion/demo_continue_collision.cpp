#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/box.h>
#include <Portal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense> 
#include <iostream>
#include <chrono>

#define M_PI 3.1415926

int PortalCrane::Portal::id = 0;

int main() {
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Trajectory Visualizer"));
    visualizer->setBackgroundColor(1, 1, 1);
    visualizer->addCoordinateSystem(10);

    auto sphere = std::make_shared<fcl::Sphere<double>>(1.0);
    auto box = std::make_shared<fcl::Box<double>>(2.0, 2.0, 2.0);

    float x0 = 3.0, y0 = 4.0, z0 = 0.0; 
    float r = 5.0; 

    // 初始位置
    fcl::Transform3<double> tf_box_beg = fcl::Transform3<double>::Identity();
    tf_box_beg.translation() = fcl::Vector3<double>(x0 + r, y0, z0); 

    // 结束位置 
    double theta = -M_PI/2; 
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    Eigen::Quaternionf quaternion;
    quaternion = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());

     
    fcl::Transform3<double> tf_box_end = fcl::Transform3<double>::Identity();
    tf_box_end.linear() = rotation_matrix; 
    tf_box_end.translation() = fcl::Vector3<double>(x0 + r * cos(theta), y0 + r * sin(theta), z0); 

    // 设置球体的变换
    fcl::Transform3<double> tf_sphere_beg = fcl::Transform3<double>::Identity();
    tf_sphere_beg.translation() = fcl::Vector3<double>(200.0, 0.0, 0.0);
    fcl::Transform3<double> tf_sphere_end = tf_sphere_beg;


    // 可视化
    visualizer->addCube(Eigen::Vector3f{ x0 + r, y0, z0 }, Eigen::Quaternionf::Identity(), 2.0, 2.0, 2.0, "cube");
    visualizer->setShapeRenderingProperties( 
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.4, 0.0, "cube");
    visualizer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cube");

    visualizer->addCube(Eigen::Vector3f(x0 + r * cos(theta), y0 + r * sin(theta), z0), quaternion, 2.0, 2.0, 2.0, "cube1");
    visualizer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.4, 0.0, "cube1");
    visualizer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cube1");

     
    visualizer->addSphere(pcl::PointXYZ(200.0, 0.0, 0.0), 1.0, "sphere");
    visualizer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.4, 0.0, "sphere");
    visualizer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "sphere");

    
    // 创建碰撞检测请求
    fcl::ContinuousCollisionRequest<double> request;
    request.ccd_motion_type = fcl::CCDM_SCREW;
    request.num_max_iterations = 1000; 
    //request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
    //request.ccd_solver_type = fcl::CCDC_NAIVE;
    //request.ccd_solver_type = fcl::CCDC_RAY_SHOOTING;
    request.ccd_solver_type = fcl::CCDC_POLYNOMIAL_SOLVER;
     

    // 创建碰撞检测结果
    fcl::ContinuousCollisionResult<double> result; 
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();
    // 执行连续碰撞检测
    bool collide = fcl::continuousCollide(sphere.get(), tf_sphere_beg, tf_sphere_end,
        box.get(), tf_box_beg, tf_box_end,
        request, result);
    // 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();

    // 计算总时间（微秒）
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    cout << "Calculate diatance cost " << duration.count() << "us" << endl;

    // 输出结果
    if (collide) {
        std::cout << "发生碰撞！" << std::endl;
        Eigen::Quaternionf quaternion2(result.contact_tf2.rotation().cast<float>());
        Eigen::Vector3f center = result.contact_tf2.translation().cast<float>();
        cout << center << endl;

        visualizer->addCube(center, quaternion2, 2.0, 2.0, 2.0, "cube2");
        visualizer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cube2");
        visualizer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube2");

    }
    else {
        std::cout << "未发生碰撞。" << std::endl;
    }

    // 显示可视化窗口
    while (!visualizer->wasStopped()) {
        visualizer->spinOnce(100);
    }
    return 0;
}
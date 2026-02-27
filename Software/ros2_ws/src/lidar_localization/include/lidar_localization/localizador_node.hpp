#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Librerías de conversión y geometría
#include "laser_geometry/laser_geometry.hpp"
#include <pcl_conversions/pcl_conversions.h>

// PCL y Eigen
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class LocalizadorBeacons : public rclcpp::Node {
    public:
        LocalizadorBeacons();

    private:
        // Callback principal del LiDAR
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

        // Funciones auxiliares (la lógica irá aquí)
        void procesar_beacons(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void detectar_obstaculos(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        // Suscriptores y Publicadores
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstaculos_pub_;

        // Herramienta para proyectar el láser a puntos 3D
        laser_geometry::LaserProjection projector_;
};
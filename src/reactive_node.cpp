#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node") {
        // Publicadores para AutoDRIVE
        throttle_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        // Suscriptor al LiDAR
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 10,
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        auto ranges = scan_msg->ranges;
        int size = ranges.size();

        // 1. LIMITAR VISIÓN FRONTAL
        float view_angle = 70.0 * M_PI / 180.0; 
        int min_idx = ((-view_angle) - scan_msg->angle_min) / scan_msg->angle_increment;
        int max_idx = ((view_angle) - scan_msg->angle_min) / scan_msg->angle_increment;
        min_idx = std::max(0, min_idx);
        max_idx = std::min(size - 1, max_idx);

        // 2. PRE-PROCESAMIENTO
        for (int i = 0; i < size; i++) {
            if (!std::isfinite(ranges[i]) || i < min_idx || i > max_idx) {
                ranges[i] = 0.0;
            }
            if (ranges[i] > 10.0) ranges[i] = 10.0;
        }

        // 3. BURBUJA DE SEGURIDAD (AJUSTADA A 25)
        int closest_index = min_idx;
        float min_dist = 10.0;
        for (int i = min_idx; i <= max_idx; i++) {
            if (ranges[i] > 0.1 && ranges[i] < min_dist) {
                min_dist = ranges[i];
                closest_index = i;
            }
        }

        int bubble_radius = 25; // Punto medio para no "asustar" al carro en pasillos
        for (int i = -bubble_radius; i <= bubble_radius; i++) {
            int idx = closest_index + i;
            if (idx >= 0 && idx < size) ranges[idx] = 0.0;
        }

        // 4. ENCONTRAR EL HUECO MÁS GRANDE (THRESHOLD AJUSTADO A 1.8)
        int max_start = min_idx, max_end = min_idx;
        int current_start = min_idx;
        bool in_gap = false;

        for (int i = min_idx; i <= max_idx; i++) {
            if (ranges[i] > 1.8) { // Umbral equilibrado para ver caminos estrechos
                if (!in_gap) {
                    current_start = i;
                    in_gap = true;
                }
            } else {
                if (in_gap) {
                    if ((i - current_start) > (max_end - max_start)) {
                        max_start = current_start;
                        max_end = i;
                    }
                    in_gap = false;
                }
            }
        }
        if (in_gap && (size - 1 - current_start) > (max_end - max_start)) {
            max_start = current_start;
            max_end = size - 1;
        }

        // 5. CÁLCULO DE CONTROL
        int best_index = (max_start + max_end) / 2;
        float angle = scan_msg->angle_min + best_index * scan_msg->angle_increment;

        auto throttle_msg = std_msgs::msg::Float32();
        auto steering_msg = std_msgs::msg::Float32();

        steering_msg.data = angle; 
        
        // Velocidad conservadora para evitar choques por inercia
        if (std::abs(angle) < 0.1) {
            throttle_msg.data = 0.25; 
        } else {
            throttle_msg.data = 0.12; 
        }

        throttle_pub->publish(throttle_msg);
        steering_pub->publish(steering_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

#include <random>
#include <cmath>
#include <chrono>

class VacuumNode : public rclcpp::Node {
public:
    VacuumNode() : Node("vacuum_node"), rand(std::random_device{}()) {
        cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        wall_sub = create_subscription<std_msgs::msg::Bool>( "/wall", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            wall = m->data;
        });

        done_sub = create_subscription<std_msgs::msg::Bool>( "/done", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) {
                done = true; RCLCPP_INFO(get_logger(), "Minden kosz felszívva, porszívó leáll.");
            }
        });

        pose_sub = create_subscription<geometry_msgs::msg::Pose2D>( "/pose", 10, [this](geometry_msgs::msg::Pose2D::SharedPtr m) {
            pose = *m;
        });

        target_sub = create_subscription<geometry_msgs::msg::Point>( "/target_dust", 10, [this](geometry_msgs::msg::Point::SharedPtr m) {
            target = *m; has_target = true;
        });

        timer = create_wall_timer(std::chrono::milliseconds(80), std::bind(&VacuumNode::tick, this));

        RCLCPP_INFO(get_logger(), "Porszívó elindult.");
    }

private:
    // állapot
    geometry_msgs::msg::Pose2D pose;
    geometry_msgs::msg::Point target;
    bool has_target{false};
    bool wall{false};
    bool done{false};
    std::mt19937 rand;
    int wall_escape_timer = 0;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wall_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub;
    rclcpp::TimerBase::SharedPtr timer;

    void tick() {
        geometry_msgs::msg::Twist cmd;
        if (done) { // ha kész, ne csináljon semmit
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub->publish(cmd);
            return;
        }

        if (wall_escape_timer > 0)
            wall_escape_timer--;

        if (wall && wall_escape_timer == 0) { // 1) falhoz ért -> kitérés
            std::uniform_real_distribution<double> turn(-1.5, 1.5);
            cmd.linear.x = 0.0; // állj meg
            cmd.angular.z = turn(rand) + 1.0; // fordulj el a faltól
            wall_escape_timer = 20; // kb. 1 másodperc kitérés
            cmd_pub->publish(cmd);
            return;
        }

        if (has_target) { // 2) van célpont -> mozgás felé
            const double dx = target.x - pose.x;
            const double dy = target.y - pose.y;
            const double dist = std::hypot(dx, dy); // távolság a célig
            const double th = std::atan2(dy, dx); // cél iránya
            const double err = normalize(th - pose.theta); // szögeltérés
            if (dist > 0.05) { // még nem értük el a célt
                cmd.angular.z = std::clamp(err * 2.8, -2.2, 2.2); // fordulás a cél felé
                cmd.linear.x = std::clamp(0.3 + 1.2*std::cos(err), 0.2, 1.0); // előre mozgás
            } else { // célpont elérve
                has_target = false;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            cmd_pub->publish(cmd);
            return;
        }

        // 3) Nincs célpont -> véletlenszerű mozgás
        std::uniform_real_distribution<double> noise(-0.4, 0.4); // kis zaj
        cmd.linear.x = 0.7;
        cmd.angular.z = noise(rand);

        // néha véletlenül nagyobbat fordul (kb. 10%-ban)
        std::uniform_real_distribution<double> chance(0.0, 1.0); // esély
        if (chance(rand) < 0.1)
            cmd.angular.z += (noise(rand) * 3.0); // nagyobb fordulás

        cmd_pub->publish(cmd);
    }

    static double normalize(double a) { // szög normalizálása [-pi, pi] tartományba
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
};

int main(int argc,char**argv) {
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<VacuumNode>()); // porszívó csomópont elindítása
    
    rclcpp::shutdown();
    return 0;
}
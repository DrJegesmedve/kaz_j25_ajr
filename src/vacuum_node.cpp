#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

#include <random>
#include <cmath>
#include <chrono>
#include <algorithm>

class VacuumNode : public rclcpp::Node {
public:
    VacuumNode() : Node("vacuum_node"), rand(std::random_device{}()) {

        cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        wall_sub = create_subscription<std_msgs::msg::Bool>("/wall", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            wall = m->data;
        });

        done_sub = create_subscription<std_msgs::msg::Bool>("/done", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) {
                done = true;
                RCLCPP_INFO(get_logger(), "Minden kosz felszívva, porszívó leáll.");
            }
        });

        pose_sub = create_subscription<geometry_msgs::msg::Pose2D>("/pose", 10, [this](geometry_msgs::msg::Pose2D::SharedPtr m) {
            pose = *m;
        });

        target_sub = create_subscription<geometry_msgs::msg::Point>("/target_dust", 10, [this](geometry_msgs::msg::Point::SharedPtr m) {
            target = *m;
            has_target = true;
        });

        obstacle_nearby_sub = create_subscription<std_msgs::msg::Bool>("/obstacle_nearby", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            obstacle_nearby = m->data;
        });

        obstacle_collision_sub = create_subscription<std_msgs::msg::Bool>("/obstacle_collision", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
            obstacle_collision = m->data;
        });

        timer = create_wall_timer(
            std::chrono::milliseconds(80),
            std::bind(&VacuumNode::tick, this));

        RCLCPP_INFO(get_logger(), "Porszívó elindult.");
    }

private:
    // állapot
    geometry_msgs::msg::Pose2D pose;
    geometry_msgs::msg::Point target;

    bool has_target{false}; // van célpont?
    bool wall{false}; // fal mellett vagyunk?
    bool done{false}; // végeztünk?

    bool obstacle_nearby{false}; // akadály van a közelben?
    bool obstacle_collision{false}; // akadállyal ütköztünk?

    std::mt19937 rand;

    // időzítők
    int wall_escape_timer{0}; // fal időzítő
    int avoid_timer{0}; // kikerülési időzítő
    int backoff_timer{0}; // hátrálási időzítő
    int stuck_counter{0}; // beragadási időzítő

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wall_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_nearby_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_collision_sub;

    rclcpp::TimerBase::SharedPtr timer;

    void tick() {
        geometry_msgs::msg::Twist cmd;

        // 0) Ha már kész, álljon le
        if (done) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub->publish(cmd);
            return;
        }

        // 1) Backoff mód – ha többször egymás után beszorul, hátráljunk és forduljunk el
        if (backoff_timer > 0) {
            backoff_timer--;

            cmd.linear.x = -1.5; // erős hátrálás
            cmd.angular.z = 1.8; // erős fordulás
            cmd_pub->publish(cmd);
            return;
        }

        // 2) Kikerülő mód – oldalirányú elhaladás akadály mellett
        if (avoid_timer > 0) {
            avoid_timer--;

            cmd.linear.x = 0.4;
            cmd.angular.z = 1.2;
            cmd_pub->publish(cmd);
            return;
        }

        // 3) Akadály a közelben -> ha érzékeljük, ne menjünk felé
        if (obstacle_nearby) {
            avoid_timer = 30;
            cmd.linear.x = 0.1;
            cmd.angular.z = 1.5;
            cmd_pub->publish(cmd);
            return;
        }

        // 4) Ütközne akadállyal -> kerüljük el
        if (obstacle_collision) {
            stuck_counter++;

            if (stuck_counter >= 3) {
                // már sokszor ütközött -> backoff mód
                backoff_timer = 15;
                stuck_counter = 0;
                cmd.linear.x = -0.5;
                cmd.angular.z = 2.5;
                cmd_pub->publish(cmd);
                return;
            }

            // normál ütközéskezelés
            avoid_timer = 10;
            cmd.linear.x = -0.2;
            cmd.angular.z = 2.0;
            cmd_pub->publish(cmd);
            return;
        }

        // 5) Fal mellett -> kitérés
        if (wall_escape_timer > 0)
            wall_escape_timer--;

        if (wall && wall_escape_timer == 0) { // fal mellett vagyunk
            std::uniform_real_distribution<double> turn(-1.5, 1.5); // véletlenszerű fordulás
            cmd.linear.x = 0.0;
            cmd.angular.z = turn(rand) + 1.0;
            wall_escape_timer = 20;
            cmd_pub->publish(cmd);
            return;
        }

        // 6) Van célpont -> menjünk felé
        if (has_target) {
            double dx = target.x - pose.x;
            double dy = target.y - pose.y;
            double dist = std::hypot(dx, dy);

            double th = std::atan2(dy, dx); // cél iránya
            double err = normalize(th - pose.theta); // szögeltérés

            if (dist > 0.05) { // még nem értünk célba
                cmd.angular.z = std::clamp(err * 2.8, -2.2, 2.2);
                cmd.linear.x = std::clamp(0.3 + 1.2 * std::cos(err), 0.2, 1.0);
            } else {
                has_target = false;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }

            cmd_pub->publish(cmd);
            return;
        }

        // 7) Nincs célpont -> véletlenszerű felfedezés
        std::uniform_real_distribution<double> noise(-0.4, 0.4); // kis zaj
        cmd.linear.x = 0.7;
        cmd.angular.z = noise(rand);

        std::uniform_real_distribution<double> chance(0.0, 1.0);
        if (chance(rand) < 0.1)
            cmd.angular.z += noise(rand) * 3.0;

        cmd_pub->publish(cmd);

        // 8) Ha már nincs akadály körülötte, nullázzuk a beragadási számlálót
        if (!obstacle_nearby && !obstacle_collision) {
            stuck_counter = 0;
        }
    }

    static double normalize(double a) { // szög normalizálása [-pi, pi] tartományban
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VacuumNode>()); // ROS eseménykezelő ciklus
    rclcpp::shutdown();
    return 0;
}

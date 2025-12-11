#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

#include <QApplication>
#include <QPainter>
#include <QWidget>

#include <vector>
#include <random>
#include <cmath>
#include <set>
#include <optional>
#include <thread>
#include <algorithm>

enum class Cell { Empty, Dust, Cleaned, Sensed, Obstacle }; // cella állapotok: üres, kosz, felszívott, érzékelt, akadály

class World : public QWidget {
public:
    World(rclcpp::Node::SharedPtr node_in) : node_(node_in), rand(std::random_device{}()) {
        resize(520, 520);
        setWindowTitle("Porszívó szimuláció");

        // Kezdő pozíció
        pose.x = 5.0;
        pose.y = 5.0;
        pose.theta = 0.0;

        init_grid(); // Rács inicializálás
        place_obstacles(); // Akadályok elhelyezése
        place_dust(); // Koszok elhelyezése

        cmd_sub = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,[this](geometry_msgs::msg::Twist::SharedPtr m) { cmd = *m; });

        pose_pub = node_->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);
        target_pub = node_->create_publisher<geometry_msgs::msg::Point>("/target_dust", 10);
        wall_pub = node_->create_publisher<std_msgs::msg::Bool>("/wall", 10);
        done_pub = node_->create_publisher<std_msgs::msg::Bool>("/done", 10);

        obstacle_nearby_pub = node_->create_publisher<std_msgs::msg::Bool>("/obstacle_nearby", 10);
        obstacle_collision_pub = node_->create_publisher<std_msgs::msg::Bool>("/obstacle_collision", 10);

        timer = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&World::tick, this));

        show(); // Megjelenítés
    }

protected:
    void paintEvent(QPaintEvent*) override { // Rajzolás
        QPainter p(this);
        p.fillRect(rect(), QColor(235, 235, 235));

        const double scale = width() / 10.0;

        // rács kirajzolása
        for (int i = 0; i < grid_h; ++i) {
            for (int j = 0; j < grid_w; ++j) {

                QColor c;
                bool draw = false;

                if (known_dust.count({j, i})) { // ismert kosz
                    c = QColor(255, 0, 0); // piros
                    draw = true;
                }
                else if (grid[i][j] == Cell::Cleaned) { // felszívott kosz
                    c = QColor(180, 120, 255); // lila
                    draw = true;
                }
                else if (grid[i][j] == Cell::Sensed) { // érzékelt cella
                    c = QColor(255, 240, 140); // sárga
                    draw = true;
                }
                else if (grid[i][j] == Cell::Dust) { // kosz
                    c = QColor(90, 90, 90); // szürke
                    draw = true;
                }
                else if (grid[i][j] == Cell::Obstacle) { // akadály
                    c = QColor(0, 0, 255); // kék
                    draw = true;
                }

                // kirajrolás
                if (draw) p.fillRect(QRectF(j * cell_size * scale, i * cell_size * scale, cell_size * scale, cell_size * scale), c);
            }
        }

        // sense range rajz
        p.setBrush(QColor(0, 255, 0, 60));
        p.setPen(Qt::NoPen);
        p.drawEllipse(QPointF(pose.x * scale, pose.y * scale), sense_range * scale, sense_range * scale);

        // porszívó
        p.setBrush(Qt::blue);
        p.setPen(Qt::black);
        p.drawEllipse(QPointF(pose.x * scale, pose.y * scale), 7, 7);

        // maradék kosz
        p.setPen(Qt::black);
        p.setBrush(Qt::NoBrush);
        p.drawText(10, 20, QString("Hátralévő kosz: %1").arg(remaining_dust));
    }

private:
    // paraméterek
    const double sense_range = 1.0;
    const double cell_size = 0.2;
    const int grid_w = int(10.0 / cell_size);
    const int grid_h = int(10.0 / cell_size);

    // állapot
    std::vector<std::vector<Cell>> grid;
    geometry_msgs::msg::Pose2D pose;
    geometry_msgs::msg::Twist cmd;
    int remaining_dust{0};
    std::set<std::pair<int, int>> known_dust;

    // ROS/Qt
    rclcpp::Node::SharedPtr node_;
    std::mt19937 rand;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_nearby_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_collision_pub;

    rclcpp::TimerBase::SharedPtr timer;

    void init_grid() { // Rács inicializálása
        grid.assign(grid_h, std::vector<Cell>(grid_w, Cell::Empty));
    }

    void place_dust() { // Koszok elhelyezése
        std::uniform_real_distribution<double> pos(0.6, 9.4);
        std::uniform_int_distribution<int> dust_count_dist(1, 100);

        int dust_count = dust_count_dist(rand);
        remaining_dust = 0;

        for (int i = 0; i < dust_count; ++i) { // kosz elhelyezs
            int gx = int(pos(rand) / cell_size);
            int gy = int(pos(rand) / cell_size);

            if (inside(gx, gy) && grid[gy][gx] == Cell::Empty) { // Csak üres cellába kerülhet kosz
                grid[gy][gx] = Cell::Dust;
                ++remaining_dust;
            }
        }
    }

    void place_obstacles() { // Akadályok elhhelyezése
        std::uniform_int_distribution<int> count_dist(8, 20); // akadály-csoportok száma
        std::uniform_int_distribution<int> size_dist(5, 20); // egy csoport mérete (cellák száma)
        std::uniform_real_distribution<double> prob(0.0, 1.0);

        int obstacle_groups = count_dist(rand); // akadály-csoportok száma

        for (int k = 0; k < obstacle_groups; ++k) { // Akadály-csoport generálás

            int size = size_dist(rand); // csoport mérete

            // minél nagyobb, annál ritkább
            double p = 0.05 * size;
            if (prob(rand) < p) continue;
            
            // kezdőpont kiválasztása
            std::uniform_int_distribution<int> gx_dist(0, grid_w - 1);
            std::uniform_int_distribution<int> gy_dist(0, grid_h - 1);

            int sx = gx_dist(rand);
            int sy = gy_dist(rand);

            // ha nem üres, kimarad
            if (!inside(sx, sy) || grid[sy][sx] != Cell::Empty) continue;

            grid[sy][sx] = Cell::Obstacle;

            std::vector<std::pair<int, int>> frontier; // csoport széle
            frontier.push_back({sx, sy});

            for (int i = 1; i < size; ++i) { // további cellák hozzáadása a csoporthoz
                if (frontier.empty()) break;

                // véletlen cella kiválasztása a csoport szélei közül
                std::uniform_int_distribution<int> pick(0, (int)frontier.size() - 1);
                auto [cx, cy] = frontier[pick(rand)];

                // 8 irány (egy cella szomszédai)
                std::vector<std::pair<int, int>> neigh = {
                    {cx + 1, cy}, {cx - 1, cy},
                    {cx, cy + 1}, {cx, cy - 1},
                    /*{cx + 1, cy + 1}, {cx + 1, cy - 1},
                    {cx - 1, cy + 1}, {cx - 1, cy - 1}*/
                };

                std::shuffle(neigh.begin(), neigh.end(), rand);

                bool placed = false;
                for (auto [nx, ny] : neigh) { // új akadálycella elhelyezése
                    if (!inside(nx, ny)) continue;
                    if (grid[ny][nx] != Cell::Empty) continue;
                    grid[ny][nx] = Cell::Obstacle;
                    frontier.push_back({nx, ny});
                    placed = true;
                    break;
                }

                if (!placed) break;
            }
        }
    }

    // DDA line-of-sight – akadály mögé nem lát a sense range
    bool has_line_of_sight(double x0, double y0, double x1, double y1) const {
        double dx = x1 - x0;
        double dy = y1 - y0;

        // lépések száma
        int steps = (int)(std::max(std::abs(dx), std::abs(dy)) / (cell_size * 0.1));
        if (steps <= 0) return true;

        // végiglépkedés a vonalon
        for (int i = 1; i <= steps; ++i) {
            double t = i / double(steps);
            double rx = x0 + dx * t;
            double ry = y0 + dy * t;

            int gx = int(rx / cell_size);
            int gy = int(ry / cell_size);

            // akadály ellenőrzése
            if (inside(gx, gy) && grid[gy][gx] == Cell::Obstacle) {
                return false;
            }
        }
        return true;
    }

    // tick
    void tick() {
        // először kiszámoljuk a következő pozíciót, és megnézzük, akadályba menne-e
        double step = 0.05;
        double trial_x = pose.x + cmd.linear.x * step * std::cos(pose.theta);
        double trial_y = pose.y + cmd.linear.x * step * std::sin(pose.theta);
        double trial_theta = pose.theta + cmd.angular.z * step; // irány

        int gx = int(trial_x / cell_size);
        int gy = int(trial_y / cell_size);

        // akadály ellenőrzése
        bool obstacle_hit = false;
        if (inside(gx, gy) && grid[gy][gx] == Cell::Obstacle) {
            obstacle_hit = true;
        }

        // fal ellenőrzés
        bool wall_hit = (trial_x <= 0.5 || trial_x >= 9.5 || trial_y <= 0.5 || trial_y >= 9.5);

        // csak akkor lépünk, nincs se fal, se akadály cella
        if (!obstacle_hit && !wall_hit) {
            pose.x = trial_x;
            pose.y = trial_y;
            pose.theta = trial_theta;
        }

        // pozíció korlátozás
        pose.x = std::clamp(pose.x, 0.6, 9.4);
        pose.y = std::clamp(pose.y, 0.6, 9.4);

        // falérzékelés
        const bool wall = (pose.x <= 0.7 || pose.x >= 9.3 || pose.y <= 0.7 || pose.y >= 9.3);

        // felszívás, érzékelés
        mark_cleaned_here(); // felszívás
        sense_and_update_memory(); // érzékelés és a memória frissítése

        // célpont választása
        auto target = choose_target();

        // /obstacle_collision: tényleg nekimenne-e
        std_msgs::msg::Bool msg;
        msg.data = obstacle_hit;
        obstacle_collision_pub->publish(msg);

        // területfelderítés vége? (nincs több Empty cella)
        bool any_unknown_left = false;
        for (int i = 0; i < grid_h; ++i) {
            for (int j = 0; j < grid_w; ++j) {
                if (grid[i][j] == Cell::Empty) {
                    any_unknown_left = true;
                    break;
                }
            }
            if (any_unknown_left) break;
        }

        if (!any_unknown_left) { // nincs empty -> kész
            std_msgs::msg::Bool done;
            done.data = true;
            done_pub->publish(done);
            timer->cancel();
        }

        // publikálás
        std_msgs::msg::Bool w;
        w.data = wall;
        wall_pub->publish(w);

        pose_pub->publish(pose);

        if (target.has_value())
            target_pub->publish(target.value());

        update();
    }

    void mark_cleaned_here() { // felszívás
        int gx = int(pose.x / cell_size);
        int gy = int(pose.y / cell_size);
        if (inside(gx, gy) && grid[gy][gx] != Cell::Obstacle) {
            grid[gy][gx] = Cell::Cleaned;
        }
    }

    void sense_and_update_memory() { // érzékelés és memória frissítése
        bool obstacle_nearby = false;

        for (int i = 0; i < grid_h; ++i) {
            for (int j = 0; j < grid_w; ++j) {

                double cx = j * cell_size + cell_size / 2;
                double cy = i * cell_size + cell_size / 2;
                double d = std::hypot(cx - pose.x, cy - pose.y);

                if (d < sense_range && has_line_of_sight(pose.x, pose.y, cx, cy)) {
                    // van akadály a közelben?
                    if (grid[i][j] == Cell::Obstacle && d < 0.8) {
                        obstacle_nearby = true;
                    }

                    if (grid[i][j] == Cell::Dust) { // kosz?
                        known_dust.insert({j, i});
                        if (d < 0.35) {
                            grid[i][j] = Cell::Cleaned;
                            known_dust.erase({j, i});
                            remaining_dust = std::max(0, remaining_dust - 1);
                        }
                    }
                    else if (grid[i][j] == Cell::Empty) {
                        grid[i][j] = Cell::Sensed;
                    }
                }
            }
        }

        // /obstacle_nearby - akadály a közelben
        std_msgs::msg::Bool msg;
        msg.data = obstacle_nearby;
        obstacle_nearby_pub->publish(msg);
    }

    std::optional<geometry_msgs::msg::Point> choose_target() { // célpont választása
        std::optional<geometry_msgs::msg::Point> result;
        double best = 1e9; // legnagyobb távolság

        // 1) Előbb az ismert koszok közül a legközelebbit szívjuk fel
        for (auto [jx, iy] : known_dust) {
            if (!inside(jx, iy)) continue;
            if (grid[iy][jx] != Cell::Dust) continue;

            double cx = jx * cell_size + cell_size / 2;
            double cy = iy * cell_size + cell_size / 2;
            double d = std::hypot(cx - pose.x, cy - pose.y);

            if (d < best) {
                best = d;
                geometry_msgs::msg::Point p;
                p.x = cx; p.y = cy; p.z = 0.0;
                result = p;
            }
        }
        if (result) return result;

        // 2) nincs ismert kosz -> legközelebbi Empty cella (frontier/ismert terület széle)
        best = 1e9;
        for (int i = 0; i < grid_h; ++i) {
            for (int j = 0; j < grid_w; ++j) {
                if (grid[i][j] == Cell::Empty) {
                    double cx = j * cell_size + cell_size / 2; // cella közepe
                    double cy = i * cell_size + cell_size / 2;
                    double d = std::hypot(cx - pose.x, cy - pose.y); // táv
                    if (d < best) { // legközelebbi
                        best = d;
                        geometry_msgs::msg::Point p;
                        p.x = cx; p.y = cy; p.z = 0.0;
                        result = p;
                    }
                }
            }
        }
        return result;
    }

    bool inside(int x, int y) const{ // rácson határán belül?
        return x >= 0 && y >= 0 && x < grid_w && y < grid_h;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("world_node");
    World w(node);

    std::thread spin_thread([&]() { // ROS eseménykezelő ciklus külön szálon
        rclcpp::spin(node);
    });

    int ret = app.exec(); // Qt eseménykezelő ciklus
    rclcpp::shutdown();
    spin_thread.join();
    return ret;
}

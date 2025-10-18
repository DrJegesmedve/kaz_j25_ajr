#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // sebesség (linear.x, angular.z)
#include "geometry_msgs/msg/pose2_d.hpp" // pozíció (x, y, theta)
#include "geometry_msgs/msg/point.hpp" // célpont (x, y, z)
#include "std_msgs/msg/bool.hpp" // Logikai üzenet (pl. fal érzékeléséhez)

#include <QApplication>
#include <QPainter> // Qt GUI
#include <QWidget>

#include <vector>
#include <random>
#include <cmath>
#include <set>
#include <optional>
#include <thread>

enum class Cell { Empty, Dust, Cleaned, Sensed };
// Cell állapotok: felfedezetlen, koszos, tisztított, érzékelt üres

class World : public QWidget {
public:
    World(rclcpp::Node::SharedPtr node_in) : node_(node_in), rand(std::random_device{}()) {
        resize(520, 520); setWindowTitle("Porszívó szimuláció"); // Ablak inicializálás

        pose.x = 5.0; pose.y = 5.0; // középre
        pose.theta = 0.0;

        init_grid(); // üres rács
        place_dust(); // véletlenszerű kosz (30)

        cmd_sub = node_->create_subscription<geometry_msgs::msg::Twist>( "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr m){ cmd = *m; }); // A porszívó sebességparancsa (lineáris + szögsebesség)
        pose_pub = node_->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10); //A porszívó aktuális pozíciója (x, y, szög)
        target_pub = node_->create_publisher<geometry_msgs::msg::Point>("/target_dust", 10); // A porszívó által megcélzott kosz pozíciója (x, y, z=0)
        wall_pub = node_->create_publisher<std_msgs::msg::Bool>("/wall", 10); // Igaz, ha a porszívó falhoz ér
        done_pub = node_->create_publisher<std_msgs::msg::Bool>("/done", 10); // Igaz, ha a porszívó befejezte a takarítást

        timer = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&World::tick, this)); // 50 ms-onként tick függvény meghívása

        show(); // Ablak megjelenítése
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this); p.fillRect(rect(), QColor(235,235,235)); const double scale = width()/10.0;
        // cellák
        for (int i = 0; i < grid_h; i++) for (int j = 0; j < grid_w; j++) {
            QColor c; bool draw = false;
            if (grid[i][j] == Cell::Cleaned) {
                c = QColor(180, 120, 255); draw = true; // lila
            } else if (grid[i][j] == Cell::Sensed) {
                c = QColor(255, 240, 140); draw = true; // sárga
            } else if (grid[i][j] == Cell::Dust) {
                c = QColor(90, 90, 90); draw = true; // szürke
            }

            if (draw) p.fillRect(QRectF(j * cell_size * scale, i * cell_size * scale, cell_size * scale, cell_size * scale), c); // cella rajzolása
        }
        
        // érzékelési kör (zöld)
        p.setBrush(QColor(0, 255, 0, 60));
        p.setPen(Qt::NoPen); // nincs keret
        p.drawEllipse(QPointF(pose.x * scale, pose.y * scale), sense_range * scale, sense_range * scale);
        
        // porszívó (kék)
        p.setBrush(Qt::blue);
        p.setPen(Qt::black); // fekete keret
        p.drawEllipse(QPointF(pose.x * scale, pose.y * scale), 7, 7);
        
        // maradék kosz
        p.setPen(Qt::black); // fekete szöveg
        p.setBrush(Qt::NoBrush); // nincs kitöltés
        p.drawText(10, 20, QString("Hátralévő kosz: %1").arg(remaining_dust));
    }

private:
    // paraméterek
    const double sense_range = 2.0; // érzékelési távolság
    const double cell_size = 0.2; // cella méret
    const int grid_w = int(10.0 / cell_size); // rács szélesség
    const int grid_h = int(10.0 / cell_size); // rács magasság

    // állapot
    std::vector<std::vector<Cell>> grid; // rács
    geometry_msgs::msg::Pose2D pose; // porszívó pozíciója
    geometry_msgs::msg::Twist cmd; // porszívó sebességparancs
    int remaining_dust{0}; // maradék kosz
    std::set<std::pair<int,int>> known_dust; // ismert kosz pozíciók (rács koordináták)

    // ROS/Qt
    rclcpp::Node::SharedPtr node_;
    std::mt19937 rand; // véletlen szám
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub; // porszívó pozíciója
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub; // célzott kosz pozíciója
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_pub; // fal érzékelés
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub; // felszívás kész
    rclcpp::TimerBase::SharedPtr timer;
    
    // rács inicializálás
    void init_grid() {
        grid.assign(grid_h, std::vector<Cell>(grid_w, Cell::Empty)); // minden cella üres
    }
    
    void place_dust() {
        std::uniform_real_distribution<double> pos(0.6, 9.4); // kosz pozíciók
        
        remaining_dust = 0;
        for (int i = 0; i < 30; i++) {
            int gx = int(pos(rand) / cell_size);
            int gy = int(pos(rand) / cell_size);
            if (inside(gx, gy) && grid[gy][gx] == Cell::Empty) {
                grid[gy][gx] = Cell::Dust;
                remaining_dust++;
            }
        }
    }
    
    // tick
    void tick() {
        // porszívó mozgása (cmd_vel alapján)
        pose.x += cmd.linear.x * 0.05 * std::cos(pose.theta); // 50 ms időlépés
        pose.y += cmd.linear.x * 0.05 * std::sin(pose.theta);
        pose.theta += cmd.angular.z * 0.05; // szög
        pose.x = std::clamp(pose.x,0.6,9.4); // határok (ablak szélétől 0.6 távolság)
        pose.y = std::clamp(pose.y,0.6,9.4);

        // fal érzékelés (ablak széléhez közel)
        const bool wall = (pose.x <= 0.7 || pose.x >= 9.3 || pose.y <= 0.7 || pose.y >= 9.3); // falhoz érés

        // felszívás jelölés
        mark_cleaned_here();
        
        sense_and_update_memory(); // sárgára jelölés és kosz felismerése
        
        auto target = choose_target(); // célpont választás

        // vége?
        if (remaining_dust == 0) {
            std_msgs::msg::Bool done;
            done.data = true; // minden kosz felszívva
            done_pub->publish(done); // jelzés a porszívónak
            timer->cancel(); // időzítő leállítása
            // GUI marad, logika leáll
        }

        // publikálás
        std_msgs::msg::Bool wallmsg; // fal üzenet
        wallmsg.data = wall; // falhoz érés
        wall_pub->publish(wallmsg); // fal üzenet küldése
        pose_pub->publish(pose); // porszívó pozíció küldése

        if (target.has_value()) target_pub->publish(target.value()); // célzott kosz pozíció küldése

        update(); // GUI frissítése
    }

    void mark_cleaned_here() {
        int gx = int(pose.x / cell_size); // porszívó rács koordinátái
        int gy = int(pose.y / cell_size);
        if (inside(gx, gy)) grid[gy][gx] = Cell::Cleaned; // felszívott kosz jelölése
    }

    void sense_and_update_memory() {
        for (int i = 0; i < grid_h; i++) {
            for (int j = 0; j < grid_w; j++) {
                const double cx = j * cell_size + cell_size / 2, cy = i * cell_size + cell_size / 2; // cella középpontja
                const double d = std::hypot(cx - pose.x, cy - pose.y); // távolság a porszívótól
                if (d < sense_range) { // érzékelési tartományban
                    if (grid[i][j] == Cell::Dust) {
                        known_dust.insert({j, i}); // felfedeztük -> memóriába
                        if (d < 0.35) { // közel van -> felszívás
                            grid[i][j] = Cell::Cleaned;
                            if (known_dust.count({j, i})) known_dust.erase({j, i}); // eltávolítás a memóriából
                            remaining_dust = std::max(0, remaining_dust - 1);
                        }
                    } else if (grid[i][j] == Cell::Empty) { // üres cella
                        grid[i][j] = Cell::Sensed; // sárgára színezés
                    }
                }
            }
        }
    }

    std::optional<geometry_msgs::msg::Point> choose_target() {
        // 1) Van ismert kosz a memóriában -> Legközelebbi kiválasztása
        std::vector<std::pair<double, geometry_msgs::msg::Point>> candidates; // (távolság, pont) párok
        double best = 1e9; // legkisebb távolság
        for (auto [jx, iy] : known_dust) { // memóriában lévő koszok
            if (!inside(jx, iy)) continue; // érvényes koordináta
            if (grid[iy][jx] != Cell::Dust) continue; // még kosz-e?

            double cx = jx * cell_size + cell_size / 2; // cella középpontja
            double cy = iy * cell_size + cell_size / 2;

            double d = std::hypot(cx - pose.x, cy - pose.y); // távolság a porszívótól

            if (d < best - 1e-6) { // új legjobb
                best = d; // legkisebb távolság frissítése
                candidates.clear();
                geometry_msgs::msg::Point pt; // pont létrehozása
                pt.x = cx;
                pt.y = cy;
                pt.z = 0.0;
                candidates.push_back({d, pt}); // hozzáadás a jelöltekhez
            } else if (std::abs(d - best) < 0.05) { // közel azonos távolságú koszok
                geometry_msgs::msg::Point pt;
                pt.x = cx;
                pt.y = cy;
                pt.z = 0.0;
                candidates.push_back({d, pt}); // hozzáadás a jelöltekhez
            }
        }
        if (!candidates.empty()) { // több jelölt esetén véletlenszerű választás
            std::uniform_int_distribution<int> pick(0, candidates.size() - 1);
            return candidates[pick(rand)].second;
        }

        // 2) Nincs ismert kosz -> Felfedezés: legközelebbi ismeretlen cella
        double frontier_best = 1e9;
        geometry_msgs::msg::Point pt;
        bool ok = false;
        for (int i = 0; i < grid_h; i++) {
            for (int j = 0; j < grid_w; j++) {
                if (grid[i][j] == Cell::Empty) {
                    double cx = j * cell_size + cell_size / 2;
                    double cy = i * cell_size + cell_size / 2;
                    double d = std::hypot(cx - pose.x, cy - pose.y);
                    if (d < frontier_best) { // új legjobb felfedezési célpont
                        frontier_best = d;
                        pt.x = cx;
                        pt.y = cy;
                        pt.z = 0.0;
                        ok = true;
                    }
                }
            }
        }

        if (ok) return pt; // van felfedezési célpont
        return std::nullopt;
    }

    bool inside(int x, int y) const {
        return x >= 0 && y >= 0 && x < grid_w && y < grid_h;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // ROS inicializálás
    QApplication app(argc, argv); // Qt inicializálás
    auto node = std::make_shared<rclcpp::Node>("world_node"); // ROS csomópont létrehozása

    World w(node);

    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    const int ret = app.exec(); // Qt ciklus

    rclcpp::shutdown(); // ROS leállítás

    spin_thread.join();
    return ret;
}
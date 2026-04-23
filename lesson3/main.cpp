#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <cstdio>
#include "json.hpp"
using json = nlohmann::json;

const double g = 9.81;

struct Coord {
    float x{0.0f};
    float y{0.0f};


    Coord operator+(const Coord& other) const {
        return Coord{static_cast<float>(x + other.x), static_cast<float>(y + other.y)};
    }

    Coord operator-(const Coord& other) const {
        return Coord{static_cast<float>(x - other.x), static_cast<float>(y - other.y)};
    }

    Coord operator*(float s) const {
        return Coord{static_cast<float>(x * s), static_cast<float>(y * s)};
    }

    Coord operator/(float s) const {
        return Coord{static_cast<float>(x / s), static_cast<float>(y / s)};
    }

    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y;
    }
};


static inline float length(const Coord& c) {
    return std::hypot(c.x, c.y);
}
static inline Coord normalize(const Coord& c) {
    float len = length(c);
    if (len == 0.0f) return Coord{0.0f, 0.0f};
    return c / len;
}


struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

static AmmoParams* g_ammo = nullptr;
static size_t g_ammo_count = 0;


static bool loadAmmoJSON(const char* path = "ammo.json") {
    std::ifstream fa(path);
    if (!fa) return false;
    json ja; fa >> ja;
    if (!ja.is_array()) return false;
    if (g_ammo) { delete[] g_ammo; g_ammo = nullptr; g_ammo_count = 0; }
    g_ammo_count = static_cast<size_t>(ja.size());
    g_ammo = new AmmoParams[g_ammo_count];
    for (size_t i = 0; i < g_ammo_count; ++i) {
        AmmoParams ap{};
        if (ja[i].contains("name")) {
            const nlohmann::json::string_t& nmref = ja[i]["name"].get_ref<const nlohmann::json::string_t&>();
            std::strncpy(ap.name, nmref.c_str(), sizeof(ap.name)-1);
        } else {
            ap.name[0] = '\0';
        }
        ap.mass = ja[i].value("mass", 0.0);
        ap.drag = ja[i].value("drag", 0.0);
        ap.lift = ja[i].value("lift", 0.0);
        g_ammo[i] = ap;
    }
    return g_ammo_count > 0;
}


struct DroneConfig {
    Coord startPos;      // x,y start
    float altitude;      // z
    float initialDir;     // radians
    float attackSpeed;    // m/s
    float accelPath;      // m
    char  ammoName[32];   // selected ammo
    float arrayTimeStep;  // targets time step
    float simTimeStep;    // simulation step
    float hitRadius;      // m
    float angularSpeed;   // rad/s
    float turnThreshold;  // rad
};


struct SimStep {
    Coord pos;             // drone position
    float direction;       // radians
    int   state;          // 0-4
    int   targetIdx;       // current target index
    Coord dropPoint;       // release point
    Coord aimPoint;      // where bomb would fall if dropped now
    Coord predictedTarget; // predicted target position
};


struct TrajectoryData {
    double xd;            // Координати дрона X
    double yd;            // Координати дрона Y
    double zd;            // висота (м)

    double initialDir;    // Початковий напрямок (рад, від осі X)
    double attackSpeed;   // Швидкість атаки (м/с)
    double accelerationPath; // Довжина розгону/гальмування (м)

    char ammo_name[50];   // Назва боєприпасу

    double arrayTimeStep; // Крок часу масиву координат цілей (с)
    double simTimeStep;   // Крок часу симуляції (с)
    double hitRadius;     // Радіус ураження (м)
    double angularSpeed;  // Кутова швидкість (рад/с)
    double turnThreshold; // Пороговий кут (рад)

    // Динамічні масиви координат цілей
    Coord** targets{nullptr};

    int targetCount{0};
    int samplesPerTarget{0};

    double V0;            // початкова швидкість (для фізики) = attackSpeed

    double h;             // горизонтальна дистанція польоту
    double D;             // відстань до цілі по горизонталі
};

struct Point {
    double x;
    double y;
    double xd_new;        // точка маневру X
    double yd_new;        // точка маневру Y
};

// Стани дрона
enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

struct Drone {
    double x{0};
    double y{0};
    double dir{0};
    double v{0};
    DroneState state{STOPPED};
    int targetIdx{0};
    double remainingTurnTime{0};
};


bool readInput(TrajectoryData& data) {
    // Load config.json
    std::ifstream icfg("config.json");
    if (!icfg) return false;
    json jc; icfg >> jc;

    // Drone parameters
    data.xd = jc["drone"]["position"].value("x", 0.0);
    data.yd = jc["drone"]["position"].value("y", 0.0);
    data.zd = jc["drone"].value("altitude", 0.0);
    data.initialDir = jc["drone"].value("initialDirection", 0.0);
    data.attackSpeed = jc["drone"].value("attackSpeed", 0.0);
    data.accelerationPath = jc["drone"].value("accelerationPath", 0.0);
    if (jc.contains("ammo")) {
        const nlohmann::json::string_t& an = jc["ammo"].get_ref<const nlohmann::json::string_t&>();
        std::strncpy(data.ammo_name, an.c_str(), sizeof(data.ammo_name)-1);
    } else {
        data.ammo_name[0] = '\0';
    }
    data.angularSpeed = jc["drone"].value("angularSpeed", 0.0);
    data.turnThreshold = jc["drone"].value("turnThreshold", 0.0);


    data.simTimeStep = jc["simulation"].value("timeStep", 0.1);
    data.hitRadius = jc["simulation"].value("hitRadius", 1.0);


    data.arrayTimeStep = jc.value("targetArrayTimeStep", 1.0);


    data.V0 = data.attackSpeed;

    std::ifstream ft("targets.json");
    if (!ft) {
        data.targets = nullptr;
        data.targetCount = 0;
        data.samplesPerTarget = 0;
        return true; // Allow running without targets
    }
    json jt; ft >> jt;
    data.targetCount = jt.value("targetCount", 0);
    data.samplesPerTarget = jt.value("timeSteps", 0);
    if (data.targetCount <= 0 || data.samplesPerTarget <= 0) {
        data.targets = nullptr; data.targetCount = 0; data.samplesPerTarget = 0; return true;
    }
    data.targets = new Coord*[data.targetCount];
    for (int i = 0; i < data.targetCount; i++) {
        data.targets[i] = new Coord[data.samplesPerTarget];
        for (int j = 0; j < data.samplesPerTarget; j++) {
            data.targets[i][j].x = jt["targets"][i]["positions"][j].value("x", 0.0);
            data.targets[i][j].y = jt["targets"][i]["positions"][j].value("y", 0.0);
        }
    }

    return true;
}

static void writeSimulationJSON(const SimStep* steps, int count) {
    json jo;
    // totalSteps should correspond to number of transitions (excluding initial state)
    jo["totalSteps"] = count;
    jo["steps"] = json::array();
    for (int i = 0; i < count; ++i) {
        const auto& s = steps[i];
        json js;
        js["position"] = { {"x", s.pos.x}, {"y", s.pos.y} };
        js["direction"] = s.direction;
        js["state"] = s.state;
        js["targetIndex"] = s.targetIdx;
        js["dropPoint"] = { {"x", s.dropPoint.x}, {"y", s.dropPoint.y} };
        js["aimPoint"] = { {"x", s.aimPoint.x}, {"y", s.aimPoint.y} };
        js["predictedTarget"] = { {"x", s.predictedTarget.x}, {"y", s.predictedTarget.y} };
        jo["steps"].push_back(js);
    }
    std::ofstream out("simulation.json");
    out << jo.dump(2);
}

int findAmmoIndex(const char* name) {
    if (!name) return -1;
    for (size_t i = 0; i < g_ammo_count; ++i) {
        if (std::strncmp(g_ammo[i].name, name, sizeof(g_ammo[i].name)) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}


double clamp(double &x) {
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

// Кардано
double solveTime(double &a, double &b, double &c) {
    double p = - (b * b) / (3.0 * a * a);
    double q = (2.0 * pow(b, 3)) / (27.0 * pow(a, 3)) + c / a;

    double inside = (3.0 * q) / (2.0 * p) * sqrt(-3.0 / p);
    inside = clamp(inside);

    double phi = acos(inside);

    double t = 2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

    return t;
}


double computeHorizontal(double t, double V0, double m, double d, double l) {
    double term1 = V0 * t;
    double term2 = - (t * t * d * V0) / (2.0 * m);

    double term3 = (pow(t, 3) * (6 * d * g * l * m - 6 * d * d * (l * l - 1) * V0)) / (36.0 * m * m);

    double term4 = (pow(t, 4) * (
        -6 * d * d * g * l * (1 + l * l + pow(l, 4)) * m
        + 3 * pow(d, 3) * l * l * (1 + l * l) * V0
        + 6 * pow(d, 3) * pow(l, 4) * (1 + l * l) * V0
    )) / (36.0 * pow(1 + l * l, 2) * pow(m, 3));

    double term5 = (pow(t, 5) * (
        3 * pow(d, 3) * g * pow(l, 3) * m
        - 3 * pow(d, 4) * l * l * (1 + l * l) * V0
    )) / (36.0 * (1 + l * l) * pow(m, 4));

    return term1 + term2 + term3 + term4 + term5;
}

bool prepareData(TrajectoryData& data) {
    int idx = findAmmoIndex(data.ammo_name);
    if (idx == -1) {
        return false;
    }

    double m = g_ammo[idx].mass;
    double d = g_ammo[idx].drag;
    double l = g_ammo[idx].lift;

    // коефіцієнти
    double a = d * g * m - 2 * d * d * l * data.V0;
    double b = -3 * g * m * m + 3 * d * l * m * data.V0;
    double c = 6 * m * m * data.zd;

    // час
    double t = solveTime(a, b, c);

    // горизонтальна дистанція
    data.h = computeHorizontal(t, data.V0, m, d, l);

    double tx0 = 0.0, ty0 = 0.0;
    if (data.targets && data.targetCount > 0 && data.samplesPerTarget > 0) {
        tx0 = data.targets[0][0].x;
        ty0 = data.targets[0][0].y;
    }
    data.D = sqrt(pow(tx0 - data.xd, 2) + pow(ty0 - data.yd, 2));

    return true;
}



// Допоміжні функції для кутів
static inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}
static inline double angleDiff(double a, double b) {
    return normalizeAngle(a - b);
}

static inline double clampVal(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline void getTargetXY(const TrajectoryData& data, int i, double t, double& outX, double& outY) {
    if (!data.targets || data.targetCount <= 0 || data.samplesPerTarget <= 0) {
        outX = 0.0; outY = 0.0; return;
    }
    if (i < 0) i = 0;
    if (i >= data.targetCount) i = data.targetCount - 1;
    if (data.arrayTimeStep <= 0) {
        // fallback: без кроку часу повертаємо перший елемент
        outX = data.targets[i][0].x;
        outY = data.targets[i][0].y;
        return;
    }
    int N = data.samplesPerTarget;
    int idx = (int)floor(t / data.arrayTimeStep) % N;
    if (idx < 0) idx += N; // на випадок від'ємних t
    int next = (idx + 1) % N;
    double frac = (t - idx * data.arrayTimeStep) / data.arrayTimeStep;
    outX = data.targets[i][idx].x + (data.targets[i][next].x - data.targets[i][idx].x) * frac;
    outY = data.targets[i][idx].y + (data.targets[i][next].y - data.targets[i][idx].y) * frac;
}


static inline void getTargetVel(const TrajectoryData& data, int i, double t, double dt, double& vx, double& vy) {
    double x1, y1, x2, y2;
    getTargetXY(data, i, t, x1, y1);
    getTargetXY(data, i, t + dt, x2, y2);
    vx = (x2 - x1) / dt;
    vy = (y2 - y1) / dt;
}

static inline double computeBallisticH(const TrajectoryData& data) {
    int idx = findAmmoIndex(data.ammo_name);
    if (idx < 0) return 0;
    double m = g_ammo[idx].mass, d = g_ammo[idx].drag, l = g_ammo[idx].lift;
    double a = d * g * m - 2 * d * d * l * data.V0;
    double b = -3 * g * m * m + 3 * d * l * m * data.V0;
    double c = 6 * m * m * data.zd;
    double t = solveTime(a, b, c);
    return computeHorizontal(t, data.V0, m, d, l);
}


static inline double timeToFullStop(const TrajectoryData& data, const Drone& dr) {
    double acc = (data.attackSpeed * data.attackSpeed) / (2.0 * data.accelerationPath);
    switch (dr.state) {
        case STOPPED: return 0.0;
        case ACCELERATING:
        case MOVING:  return dr.v / acc; // час гальмування до 0
        case DECELERATING: return dr.v / acc; // залишок
        case TURNING: return dr.remainingTurnTime; // залишок повороту
        default: return 0.0;
    }
}

static inline double estimateTravelTime(const TrajectoryData& data, const Drone& dr, double dx, double dy, double desiredDir, bool& needTurn) {
    double dist = std::sqrt(dx*dx + dy*dy);
    double acc = (data.attackSpeed * data.attackSpeed) / (2.0 * data.accelerationPath);
    double turnAngle = std::fabs(angleDiff(desiredDir, dr.dir));
    needTurn = (turnAngle > data.turnThreshold);

    double time = 0.0;
    double v0 = dr.v;

    if (needTurn) {
        // зупиняємось, повертаємось, розганяємось
        double tBrake = v0 / acc;
        time += tBrake;
        time += turnAngle / data.angularSpeed;
        v0 = 0.0; // починаємо з нуля
    }

    // рух на прямій з можливим розгоном до attackSpeed
    double s_acc = std::max(0.0, (data.attackSpeed*data.attackSpeed - v0*v0) / (2.0*acc));
    double t_acc = std::max(0.0, (data.attackSpeed - v0) / acc);

    if (dist <= s_acc) {
        // не досягаємо крейсерської швидкості
        // s = v0*t + 0.5*a*t^2 -> вирішимо квадр. рівняння
        double A = 0.5*acc;
        double B = v0;
        double C = -dist;
        double disc = B*B - 4*A*C;
        time += (-B + std::sqrt(std::max(0.0, disc))) / (2*A);
        return time;
    }

    time += t_acc;
    double s_remain = dist - s_acc;
    // далі на крейсерській швидкості
    time += s_remain / data.attackSpeed;
    return time;
}

int main() {
    // Load ammo list first
    if (!loadAmmoJSON("ammo.json")) {
        return 1;
    }

    TrajectoryData data{};

    if (!readInput(data)) {
         return 1;
    }

    if (!prepareData(data)) {
        return 1;
    }


    const int MAX_STEPS = 10000;
    const double hBall = computeBallisticH(data);
    double currentTime = 0.0;

    // Ініціалізація дрона
    Drone dr;
    dr.x = data.xd;
    dr.y = data.yd;
    dr.dir = data.initialDir;
    dr.v = 0.0;
    dr.state = STOPPED;
    dr.targetIdx = 0;
    dr.remainingTurnTime = 0.0;

    // Логи для JSON виводу (сирий масив)
    const int MAX_LOG_STEPS = MAX_STEPS + 5;
    SimStep* stepsBuf = new SimStep[MAX_LOG_STEPS];
    int stepsCount = 0;
    SimStep init{};
    init.pos = Coord{static_cast<float>(dr.x), static_cast<float>(dr.y)};
    init.direction = static_cast<float>(dr.dir);
    init.state = (int)dr.state;
    init.targetIdx = dr.targetIdx;
    init.dropPoint = Coord{0,0};
    init.aimPoint = Coord{static_cast<float>(dr.x + std::cos(dr.dir) * hBall), static_cast<float>(dr.y + std::sin(dr.dir) * hBall)};
    double tx0, ty0; getTargetXY(data, dr.targetIdx, currentTime, tx0, ty0);
    init.predictedTarget = Coord{static_cast<float>(tx0), static_cast<float>(ty0)};
    if (stepsCount < MAX_LOG_STEPS) stepsBuf[stepsCount++] = init;

    // Якщо немає цілей — одразу завершуємо після запису початкового стану
    if (data.targetCount <= 0) {
        writeSimulationJSON(stepsBuf, stepsCount);
        delete[] stepsBuf;
        if (data.targets) {
            for (int i = 0; i < data.targetCount; ++i) { delete[] data.targets[i]; }
            delete[] data.targets; data.targets = nullptr;
        }
        return 0;
    }

    // Симуляція
    int steps = 0;
    int activeTarget = 0;
    while (steps < MAX_STEPS) {

        double bestTime = 1e18;
        int bestIdx = 0;
        double bestRelX=0, bestRelY=0; // точка скиду
        double bestDesiredDir=0;
        double bestPredX=0, bestPredY=0; // прогнозована позиція

        for (int i = 0; i < data.targetCount; ++i) {
            double tx, ty; getTargetXY(data, i, currentTime, tx, ty);
            double dx0 = tx - dr.x; double dy0 = ty - dr.y;
            double desired = std::atan2(dy0, dx0);
            bool needTurnTmp=false;
            double tTravel = estimateTravelTime(data, dr, dx0, dy0, desired, needTurnTmp);

            double vx, vy; double dtv = (data.arrayTimeStep>0? data.arrayTimeStep : data.simTimeStep>0? data.simTimeStep:0.1);
            getTargetVel(data, i, currentTime, dtv, vx, vy);

            double px = tx + vx * tTravel;
            double py = ty + vy * tTravel;

            Coord predicted{static_cast<float>(px), static_cast<float>(py)};
            Coord dronePos{static_cast<float>(dr.x), static_cast<float>(dr.y)};
            Coord delta = predicted - dronePos;
            float distToPred = length(delta);
            if (distToPred < 1e-6f) continue;
            Coord firePoint = predicted - normalize(delta) * static_cast<float>(hBall);
            double rx = firePoint.x;
            double ry = firePoint.y;

            // оновлюємо напрям і відстань до release point
            double rdx = rx - dr.x; double rdy = ry - dr.y;
            double desiredDir = std::atan2(rdy, rdx);
            bool needTurn=false;
            double tToRelease = estimateTravelTime(data, dr, rdx, rdy, desiredDir, needTurn);

            // штраф за зміну цілі
            double penalty = 0.0;
            if (i != activeTarget) penalty = timeToFullStop(data, dr);

            double total = tToRelease + penalty;
            if (total < bestTime) {
                bestTime = total; bestIdx = i; bestRelX = rx; bestRelY = ry; bestDesiredDir = desiredDir; bestPredX = px; bestPredY = py;
            }
        }

        // якщо обрана ціль змінилась-оновлюємо activeTarget
        if (bestIdx != activeTarget) {
            activeTarget = bestIdx;
        }

        // Керування поворотом
        double delta = std::fabs(angleDiff(bestDesiredDir, dr.dir));
        double acc = (data.attackSpeed * data.attackSpeed) / (2.0 * data.accelerationPath);

        if (delta > data.turnThreshold) {
            if (dr.v > 0) {
                dr.state = DECELERATING;
                double dv = acc * data.simTimeStep;
                dr.v = std::max(0.0, dr.v - dv);
            }
            if (dr.v == 0.0) {

                dr.state = TURNING;
                double maxTurn = data.angularSpeed * data.simTimeStep;
                double dirErr = angleDiff(bestDesiredDir, dr.dir);
                double turnStep = clampVal(dirErr, -maxTurn, maxTurn);
                dr.dir = normalizeAngle(dr.dir + turnStep);
                if (std::fabs(angleDiff(bestDesiredDir, dr.dir)) <= data.turnThreshold) {
                    dr.state = ACCELERATING;
                }
            }
        } else {

            if (dr.v < data.attackSpeed) {
                dr.state = ACCELERATING;
                dr.v = std::min(data.attackSpeed, dr.v + acc * data.simTimeStep);
            } else {
                dr.state = MOVING;
            }
            // плавно коригуємо напрямок до цілі
            double maxTurn = data.angularSpeed * data.simTimeStep;
            double dirErr = angleDiff(bestDesiredDir, dr.dir);
            double turnStep = clampVal(dirErr, -maxTurn, maxTurn);
            dr.dir = normalizeAngle(dr.dir + turnStep);
        }

        // оновлення позиції
        if (dr.state != TURNING) {
            dr.x += std::cos(dr.dir) * dr.v * data.simTimeStep;
            dr.y += std::sin(dr.dir) * dr.v * data.simTimeStep;
        }

        // логування одного кроку для JSON
        SimStep step{};
        step.pos = Coord{static_cast<float>(dr.x), static_cast<float>(dr.y)};
        step.direction = static_cast<float>(dr.dir);
        step.state = (int)dr.state;
        step.targetIdx = activeTarget;
        step.dropPoint = Coord{static_cast<float>(bestRelX), static_cast<float>(bestRelY)};
        step.aimPoint = Coord{static_cast<float>(dr.x + std::cos(dr.dir) * hBall), static_cast<float>(dr.y + std::sin(dr.dir) * hBall)};
        step.predictedTarget = Coord{static_cast<float>(bestPredX), static_cast<float>(bestPredY)};
        if (stepsCount < MAX_LOG_STEPS) stepsBuf[stepsCount++] = step;

        double dx = bestRelX - dr.x; double dy = bestRelY - dr.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist <= data.hitRadius) {
            break;
        }

        currentTime += data.simTimeStep;
        ++steps;
    }

    writeSimulationJSON(stepsBuf, stepsCount);


    delete[] stepsBuf;
    if (data.targets) {
        for (int i = 0; i < data.targetCount; ++i) { delete[] data.targets[i]; }
        delete[] data.targets; data.targets = nullptr;
    }
    return 0;
}
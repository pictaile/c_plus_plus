#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

// константи та коефіцієнти
const double g = 9.81;

const int ammoCount = 5;
const char* ammo_names[] = {
    "VOG-17",
    "M67",
    "RKG-3",
    "GLIDING-VOG",
    "GLIDING-RKG"
};

double m_arr[] = {0.35, 0.6, 1.2, 0.45, 1.4}; // m масса
double d_arr[] = {0.07, 0.10, 0.10, 0.10, 0.10}; // d опір
double l_arr[] = {0.0, 0.0, 0.0, 1.0, 1.0}; // l підйомна сила



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

    // Масиви координат цілей: 5 цілей по 60 значень для X та Y
    double targetXInTime[5][60]{};
    double targetYInTime[5][60]{};


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
    std::ifstream in("./input.txt");
    if (!in) return false;

    in >> data.xd
       >> data.yd
       >> data.zd
       >> data.initialDir
       >> data.attackSpeed
       >> data.accelerationPath
       >> data.ammo_name
       >> data.arrayTimeStep
       >> data.simTimeStep
       >> data.hitRadius
       >> data.angularSpeed
       >> data.turnThreshold;

    if (!in) return false;

    // використовуємо V0 як attackSpeed
    data.V0 = data.attackSpeed;


    // зчитуємо координати цілей з targets.txt, із fallback на target.txt
    const char* filesToTry[] = {"targets.txt", "target.txt"};
    std::ifstream tfile;
    for (const char* fname : filesToTry) {
        tfile.open(fname);
        if (tfile.is_open()) break;
    }
    if (tfile.is_open()) {
        std::string line;

        for (int i = 0; i < 5; ++i) {
            if (!std::getline(tfile, line)) return false;
            std::istringstream iss(line);
            for (int k = 0; k < 60; ++k) {
                if (!(iss >> data.targetXInTime[i][k])) return false;
            }
        }

        for (int i = 0; i < 5; ++i) {
            if (!std::getline(tfile, line)) return false;
            std::istringstream iss(line);
            for (int k = 0; k < 60; ++k) {
                if (!(iss >> data.targetYInTime[i][k])) return false;
            }
        }
    } else {
        // Якщо файл недоступний — залишаємо масиви нульовими
    }


    return true;
}


// Вивід симуляції
static void writeSimulation(const std::vector<double>& xs,
                            const std::vector<double>& ys,
                            const std::vector<double>& dirs,
                            const std::vector<int>& states,
                            const std::vector<int>& targetIdxs) {
    std::ofstream out("simulation.txt");
    size_t N = xs.size() > 0 ? xs.size() - 1 : 0; // N кроків (0..N)
    out << N << "\n";
    // координати дрона
    for (size_t i = 0; i < xs.size(); ++i) {
        if (i) out << ' ';
        out << xs[i] << ' ' << ys[i];
    }
    out << "\n";
    // напрямки
    for (size_t i = 0; i < dirs.size(); ++i) {
        if (i) out << ' ';
        out << dirs[i];
    }
    out << "\n";
    // стани
    for (size_t i = 0; i < states.size(); ++i) {
        if (i) out << ' ';
        out << states[i];
    }
    out << "\n";
    // індекси цілей
    for (size_t i = 0; i < targetIdxs.size(); ++i) {
        if (i) out << ' ';
        out << targetIdxs[i];
    }
    out << "\n";
}


// пошук боєприпаса індексу
int findAmmoIndex(const char* name) {
    for (int i = 0; i < ammoCount; i++) {
        if (strcmp(ammo_names[i], name) == 0) {
            return i;
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

// горизонтальна дистанція
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
    // пошук типу боєприпаса
    int idx = findAmmoIndex(data.ammo_name);
    if (idx == -1) {
        return false;
    }

    double m = m_arr[idx];
    double d = d_arr[idx];
    double l = l_arr[idx];

    // коефіцієнти
    double a = d * g * m - 2 * d * d * l * data.V0;
    double b = -3 * g * m * m + 3 * d * l * m * data.V0;
    double c = 6 * m * m * data.zd;

    // час
    double t = solveTime(a, b, c);

    // горизонтальна дистанція
    data.h = computeHorizontal(t, data.V0, m, d, l);

    // відстань до першого положення цілі (ціль 0, t=0)
    double tx0 = data.targetXInTime[0][0];
    double ty0 = data.targetYInTime[0][0];
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

// Portable clamp for pre-C++17
static inline double clampVal(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Лінійна інтерполяція положення цілі i у час t з циклічним індексом
static inline void getTargetXY(const TrajectoryData& data, int i, double t, double& outX, double& outY) {
    if (data.arrayTimeStep <= 0) {
        // fallback: без кроку часу повертаємо перший елемент
        outX = data.targetXInTime[i][0];
        outY = data.targetYInTime[i][0];
        return;
    }
    int idx = (int)floor(t / data.arrayTimeStep) % 60;
    if (idx < 0) idx += 60; // на випадок від'ємних t
    int next = (idx + 1) % 60;
    double frac = (t - idx * data.arrayTimeStep) / data.arrayTimeStep;
    outX = data.targetXInTime[i][idx] + (data.targetXInTime[i][next] - data.targetXInTime[i][idx]) * frac;
    outY = data.targetYInTime[i][idx] + (data.targetYInTime[i][next] - data.targetYInTime[i][idx]) * frac;
}

// Оцінка швидкості цілі через кінцеві різниці
static inline void getTargetVel(const TrajectoryData& data, int i, double t, double dt, double& vx, double& vy) {
    double x1, y1, x2, y2;
    getTargetXY(data, i, t, x1, y1);
    getTargetXY(data, i, t + dt, x2, y2);
    vx = (x2 - x1) / dt;
    vy = (y2 - y1) / dt;
}

// Розрахунок горизонтальної дистанції h (балiстика) з наявних функцій
static inline double computeBallisticH(const TrajectoryData& data) {
    int idx = findAmmoIndex(data.ammo_name);
    if (idx < 0) return 0;
    double m = m_arr[idx], d = d_arr[idx], l = l_arr[idx];
    double a = d * g * m - 2 * d * d * l * data.V0;
    double b = -3 * g * m * m + 3 * d * l * m * data.V0;
    double c = 6 * m * m * data.zd;
    double t = solveTime(a, b, c);
    return computeHorizontal(t, data.V0, m, d, l);
}

// Час, щоб зупинитися з поточного стану
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

// Оцінка часу польоту дрона до точки скиду вздовж прямої з урахуванням розгону/гальмування
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
    TrajectoryData data{};

    if (!readInput(data)) {
         return 1;
    }

    if (!prepareData(data)) {
        return 1;
    }

    // Параметри
    const int MAX_STEPS = 10000;
    const double hBall = computeBallisticH(data); // фіксована балістична горизонтальна дистанція
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

    // Логи
    std::vector<double> xs, ys, dirs;
    std::vector<int> states, targetIdxs;

    xs.push_back(dr.x);
    ys.push_back(dr.y);
    dirs.push_back(dr.dir);
    states.push_back((int)dr.state);
    targetIdxs.push_back(dr.targetIdx);

    // Симуляція
    int steps = 0;
    int activeTarget = 0;
    while (steps < MAX_STEPS) {
        // Для кожної цілі оцінюємо загальний час
        double bestTime = 1e18;
        int bestIdx = 0;
        double bestRelX=0, bestRelY=0; // точка скиду
        double bestDesiredDir=0;

        for (int i = 0; i < 5; ++i) {
            // поточна позиція цілі
            double tx, ty; getTargetXY(data, i, currentTime, tx, ty);
            // початкова оцінка напрямку на поточну ціль
            double dx0 = tx - dr.x; double dy0 = ty - dr.y;
            double desired = std::atan2(dy0, dx0);
            bool needTurnTmp=false;
            double tTravel = estimateTravelTime(data, dr, dx0, dy0, desired, needTurnTmp);

            // швидкість цілі
            double vx, vy; double dtv = (data.arrayTimeStep>0? data.arrayTimeStep : data.simTimeStep>0? data.simTimeStep:0.1);
            getTargetVel(data, i, currentTime, dtv, vx, vy);

            // прогнозована позиція цілі на момент прибуття
            double px = tx + vx * tTravel;
            double py = ty + vy * tTravel;

            // перерахунок балістики до прогнозованої позиції: точка скиду на прямій між дроном і прогнозом на відстані hBall від прогнозу назад
            double ddx = px - dr.x; double ddy = py - dr.y;
            double distToPred = std::sqrt(ddx*ddx + ddy*ddy);
            if (distToPred < 1e-6) continue;
            double ratio = std::max(0.0, (distToPred - hBall) / distToPred);
            double rx = dr.x + ddx * ratio;
            double ry = dr.y + ddy * ratio;

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
                bestTime = total; bestIdx = i; bestRelX = rx; bestRelY = ry; bestDesiredDir = desiredDir;
            }
        }

        // якщо обрана ціль змінилась — оновлюємо activeTarget
        if (bestIdx != activeTarget) {
            activeTarget = bestIdx;
        }

        // Керування поворотом
        double delta = std::fabs(angleDiff(bestDesiredDir, dr.dir));
        double acc = (data.attackSpeed * data.attackSpeed) / (2.0 * data.accelerationPath);

        if (delta > data.turnThreshold) {
            // переходимо у DECELERATING
            if (dr.v > 0) {
                dr.state = DECELERATING;
                double dv = acc * data.simTimeStep;
                dr.v = std::max(0.0, dr.v - dv);
            }
            if (dr.v == 0.0) {
                // крутимось
                dr.state = TURNING;
                double maxTurn = data.angularSpeed * data.simTimeStep;
                double dirErr = angleDiff(bestDesiredDir, dr.dir);
                double turnStep = clampVal(dirErr, -maxTurn, maxTurn);
                dr.dir = normalizeAngle(dr.dir + turnStep);
                if (std::fabs(angleDiff(bestDesiredDir, dr.dir)) <= data.turnThreshold) {
                    dr.state = ACCELERATING; // готові розганятись
                }
            }
        } else {
            // можемо їхати у напрямку без зупинки
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

        // логування
        xs.push_back(dr.x);
        ys.push_back(dr.y);
        dirs.push_back(dr.dir);
        states.push_back((int)dr.state);
        targetIdxs.push_back(activeTarget);

        // умова завершення: якщо ми досягли точки скиду для активної цілі
        double dx = bestRelX - dr.x; double dy = bestRelY - dr.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist <= data.hitRadius) {
            break;
        }

        currentTime += data.simTimeStep;
        ++steps;
    }

    writeSimulation(xs, ys, dirs, states, targetIdxs);
    return 0;
}
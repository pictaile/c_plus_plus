#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>

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
    double xd;
    double yd;
    double zd;            // висота підриву/цілі по Z (з вхідних даних)
    double targetX;
    double targetY;
    double V0;            // початкова швидкість
    double accelerationPath;

    char ammoName[50];    // назва боєприпаса з вхідних даних

    double h;             // горизонтальна дистанція польоту
    double D;             // відстань до цілі по горизонталі
};

struct Point {
    double x;
    double y;
    double xd_new;        // точка маневру X
    double yd_new;        // точка маневру Y
};


bool readInput( TrajectoryData& data) {
       std::ifstream in("input.txt");

    in >> data.xd >> data.yd >> data.zd >> data.targetX >> data.targetY >> data.V0 >> data.accelerationPath >> data.ammoName;
    return static_cast<bool>(in);
}


void writeOutput(const TrajectoryData& data, const Point& fp) {
   //    auto &out = std::cout;
    std::ofstream out("output.txt");
    if (data.h + data.accelerationPath > data.D) {
        out << "точка маневру: " << fp.xd_new << " " << fp.yd_new << std::endl;
    }
    out << "точка скиду: " << fp.x << " " << fp.y << std::endl;
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
    int idx = findAmmoIndex(data.ammoName);
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

    // відстань
    data.D = sqrt(pow(data.targetX - data.xd, 2) + pow(data.targetY - data.yd, 2));

    return true;
}



Point firePoint(const TrajectoryData& data) {
    Point p{};

    // маневр (оновлюємо координати, якщо умова виконується)
    if (data.h + data.accelerationPath > data.D) {
        p.xd_new = data.targetX - (data.targetX - data.xd) * (data.h + data.accelerationPath) / data.D;
        p.yd_new = data.targetY - (data.targetY - data.yd) * (data.h + data.accelerationPath) / data.D;
    } else {
        // якщо маневр не потрібен — залишаємо початкові координати
        p.xd_new = data.xd;
        p.yd_new = data.yd;
    }

    // точка скиду (логіка незмінна)
    double ratio = (data.D - data.h) / data.D;
    p.x = data.xd + (data.targetX - data.xd) * ratio;
    p.y = data.yd + (data.targetY - data.yd) * ratio;

    return p;
}

int main() {
    TrajectoryData data{};
    Point fp;

    if (!readInput(data)) {
        return 1;
    }

    if (!prepareData(data)) {
        std::cout << "Бєприпас не знайдено" << std::endl;
        return -1;
    }

    // точка скиду та маневр обчислюються у firePoint
    fp = firePoint(data);

    // вивід
    writeOutput(data, fp);

    return 0;
}
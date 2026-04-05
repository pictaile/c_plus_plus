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


// пошук боєприпаса індексу
int findAmmoIndex(const char* name) {
    for (int i = 0; i < ammoCount; i++) {
        if (strcmp(ammo_names[i], name) == 0) {
            return i;
        }
    }
    return -1;
}


double clamp(double x) {
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

// Кардано
double solveTime(double a, double b, double c) {
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

int main() {
//    auto &out = std::cout;
    std::ifstream in("input.txt");
    std::ofstream out("output.txt");

    double xd, yd, zd;
    double targetX, targetY;
    double V0, accelerationPath;
    char ammoName[50];

    in >> xd >> yd >> zd >> targetX >> targetY >> V0 >> accelerationPath >> ammoName;

    int idx = findAmmoIndex(ammoName);

    if (idx == -1) {
        out << "Бєприпас не знайдено" << std::endl;
        return 1;
    }

    double m = m_arr[idx];
    double d = d_arr[idx];
    double l = l_arr[idx];

    // коефіцієнти
    double a = d * g * m - 2 * d * d * l * V0;
    double b = -3 * g * m * m + 3 * d * l * m * V0;
    double c = 6 * m * m * zd;

    // час
    double t = solveTime(a, b, c);

    // горизонтальна дистанція
    double h = computeHorizontal(t, V0, m, d, l);

    // відстань
    double D = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));

    double xd_new = xd;
    double yd_new = yd;

    // маневр
    if (h + accelerationPath > D) {
        xd_new = targetX - (targetX - xd) * (h + accelerationPath) / D;
        yd_new = targetY - (targetY - yd) * (h + accelerationPath) / D;

        out << "точка маневру: " << xd_new << " " << yd_new << std::endl;
    }

    // точка скиду
    double ratio = (D - h) / D;

    double fireX = xd + (targetX - xd) * ratio;
    double fireY = yd + (targetY - yd) * ratio;

    out << "точка скиду: " << fireX << " " << fireY << std::endl;

    return 0;
}
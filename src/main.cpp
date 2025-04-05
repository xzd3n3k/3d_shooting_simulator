#include "entt/entt.hpp"
#include <cmath>
#include <optional>
#include <iostream>
using namespace std;

constexpr double GRAVITY = 9.81;
constexpr double DRAG_COEFF = 0.001;
constexpr double HIT_RADIUS = 2.0;

struct Coordinates { 
    double x;
    double y;
    double z;
};

class ProjectileSolver {
public:
    static optional<double> solveTrajectory(
        const Coordinates &shooter,
        const Coordinates &target,
        double velocity,
        double mass,
        double dt = 0.01
    ) {
        
        double dx = target.x - shooter.x;
        double dz = target.z - shooter.z;
        double hDist = hypot(dx, dz);
        double azimuth = atan2(dx, dz);
        
        double dy = target.y - shooter.y;

        if (hDist < 0.001) {
            return (abs(dy) < 0.001) ? 0.0 : (dy > 0 ? 89.0 : -89.0);
        }
        if (dz < 0 && abs(dx) < 0.001) {
            return 0.0;
        }

        double v2 = velocity * velocity;
        double discriminant = v2*v2 - GRAVITY*(GRAVITY*hDist*hDist + 2*dy*v2);
        if (discriminant < 0) return nullopt;
        
        double sqrtDisc = sqrt(discriminant);
        double angle1 = atan((v2 + sqrtDisc) / (GRAVITY * hDist)) * 180.0/M_PI;
        double angle2 = atan((v2 - sqrtDisc) / (GRAVITY * hDist)) * 180.0/M_PI;

        if (dy < 0) {
            angle1 = max(-85.0, angle1);
            angle2 = max(-85.0, angle2);
        }
        if (abs(angle1) > abs(angle2)) {
            swap(angle1, angle2);
        }

        if (simulateShot(shooter, target, velocity, angle1, azimuth, mass, dt)) return angle1;
        if (simulateShot(shooter, target, velocity, angle2, azimuth, mass, dt)) return angle2;
        
        double startAngle = (dy < 0) ? -85.0 : 1.0;
        double endAngle = (dy < 0) ? -1.0 : 85.0;
        for (double angle = startAngle; angle <= endAngle; angle += 1.0) {
            if (simulateShot(shooter, target, velocity, angle, azimuth, mass, dt)) {
                return angle;
            }
        }
        
        return nullopt;
    }

private:
    static bool simulateShot(
        const Coordinates &shooter,
        const Coordinates &target,
        double velocity,
        double elevation,
        double azimuth,
        double mass,
        double dt
    ) {
        
        double elevRad = elevation * M_PI / 180.0;
        double azimRad = azimuth;
        
        double vHorizontal = velocity * cos(elevRad);
        double vx = vHorizontal * sin(azimRad);
        double vz = vHorizontal * cos(azimRad);
        double vy = velocity * sin(elevRad);
        
        Coordinates pos = shooter;
        
        for (int i = 0; i < 5000; ++i) {
            pos.x += vx * dt;
            pos.y += vy * dt;
            pos.z += vz * dt;
            
            vy -= GRAVITY * dt;
            
            double speed = sqrt(vx*vx + vy*vy + vz*vz);
            if (speed > 0.1) {
                double drag = 0.5 * DRAG_COEFF * speed * dt;
                vx -= vx * drag;
                vy -= vy * drag;
                vz -= vz * drag;
            }
            
            double dist2 = pow(pos.x-target.x, 2) + pow(pos.y-target.y, 2) + pow(pos.z-target.z, 2);
            if (dist2 < HIT_RADIUS*HIT_RADIUS) return true;
            if (pos.z > target.z + 10.0 && hypot(pos.x-target.x, pos.y-target.y) > 20.0) {
                break;
            }
        }
        return false;
    }
};

int main() {
    vector<pair<Coordinates, Coordinates>> tests = {
        {{0,0,0}, {0,10,20}},
        {{0,0,0}, {0,0,50}},
        {{0,5,0}, {0,0,20}},
        {{0,0,0}, {10,5,30}},
        {{0,0,10}, {0,0,0}},
        {{0,0,0}, {0,5,10}},
        {{0,0,0}, {0,-2,15}},
        {{0,2,0}, {0,0,12}},
        {{0,0,0}, {5,3,20}},
        {{0,0,0}, {-4,2,15}},
        {{3,1,0}, {-2,0,18}},
        {{0,10,0}, {0,0,25}},
        {{0,0,0}, {0,0,5}},
        {{0,0,10}, {0,0,0}},
        {{5,5,5}, {5,5,5}},
        {{0,0,0}, {0,50,100}},
        {{0,100,0}, {0,0,50}},
        {{0,0,0}, {20,0,0}},
        {{10,0,0}, {0,0,0}},
        {{0,0,0}, {0,30,30}}
    };
    
    double velocity = 400.0;
    double mass = 0.0097;
    
    for (size_t i = 0; i < tests.size(); ++i) {
        const auto &[shooter, target] = tests[i];
        auto angle = ProjectileSolver::solveTrajectory(shooter, target, velocity, mass);
        
        cout << "Test " << i+1 << ":\n";
        cout << "Shooter: (" << shooter.x << "," << shooter.y << "," << shooter.z << ")\n";
        cout << "Target: (" << target.x << "," << target.y << "," << target.z << ")\n";
        
        if (angle) {
            cout << "Elevation: " << *angle << "°\n";
            double dx = target.x - shooter.x;
            double dz = target.z - shooter.z;
            double azimuth = atan2(dx, dz) * 180.0 / M_PI;
            cout << "Azimuth: " << azimuth << "°\n";
        } else {
            cout << "No solution found\n";
        }
        cout << "----------------\n";
    }
    
    return 0;
}
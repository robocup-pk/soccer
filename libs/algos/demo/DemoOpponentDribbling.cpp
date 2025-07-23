#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RRT.h"
#include "Waypoint.h"
#include "Kinematics.h"
#include "Utils.h"
#include "TrapezoidalTrajectoryVi3D.h"
#include "TrajectoryManager.h"
#include "Dribble.h"

int main(int argc, char* argv[]) {
    std::cout << "[DemoOpponentDribbling] Two player RRT dribbling demo" << std::endl;

    if (cfg::SystemConfig::num_robots < 2) {
        std::cout << "[Demo] Need at least two robots" << std::endl;
        return 0;
    }

    std::vector<state::SoccerObject> objects;
    state::InitSoccerObjects(objects);

    vis::GLSimulation sim;
    sim.InitGameObjects(objects);

    state::SoccerObject* robotA = nullptr;
    state::SoccerObject* robotB = nullptr;
    state::SoccerObject* ball = nullptr;

    for (auto& obj : objects) {
        if (obj.name == "robot0") {
            robotA = &obj;
            obj.position = Eigen::Vector3d(-1.0, -0.5, 0);
        } else if (obj.name == "robot1") {
            robotB = &obj;
            obj.position = Eigen::Vector3d(1.0, -0.5, M_PI);
        } else if (obj.name == "ball") {
            ball = &obj;
            obj.position = Eigen::Vector3d(0, 0, 0);
        }
    }

    ctrl::TrajectoryManager trajA;
    ctrl::TrajectoryManager trajB;
    bool planA = false;
    bool planB = false;
    bool dribbleA = false;
    bool dribbleB = false;

    algos::RRTParams params = algos::DefaultRRTParams();
    params.max_iterations = 500;

    double startTime = util::GetCurrentTime();

    while (true) {
        double t = util::GetCurrentTime();
        double dt = util::CalculateDt();

        if (!planA) {
            double fw = vis::SoccerField::GetInstance().width_mm;
            double fh = vis::SoccerField::GetInstance().height_mm;
            state::Waypoint swp(fw/2 + robotA->position.x()*1000, fh/2 + robotA->position.y()*1000, 0);
            state::Waypoint gwp(fw/2 + ball->position.x()*1000, fh/2 + ball->position.y()*1000, 0);
            state::Path path = algos::FindSinglePath(swp, gwp, params);
            if (!path.empty()) {
                std::vector<Eigen::Vector3d> wps;
                for (size_t i=0;i<path.size();++i) {
                    double wx = (path[i].x - fw/2)/1000.0;
                    double wy = (path[i].y - fh/2)/1000.0;
                    double theta = 0;
                    if (i < path.size()-1) {
                        double dx = path[i+1].x - path[i].x;
                        double dy = path[i+1].y - path[i].y;
                        theta = std::atan2(dy,dx);
                    }
                    wps.push_back(Eigen::Vector3d(wx,wy,theta));
                }
                trajA.CreateTrajectoriesFromPath(wps, t);
                planA = true;
            }
        }

        if (!planB) {
            double fw = vis::SoccerField::GetInstance().width_mm;
            double fh = vis::SoccerField::GetInstance().height_mm;
            state::Waypoint swp(fw/2 + robotB->position.x()*1000, fh/2 + robotB->position.y()*1000, 0);
            state::Waypoint gwp(fw/2 + ball->position.x()*1000, fh/2 + ball->position.y()*1000, 0);
            state::Path path = algos::FindSinglePath(swp, gwp, params);
            if (!path.empty()) {
                std::vector<Eigen::Vector3d> wps;
                for (size_t i=0;i<path.size();++i) {
                    double wx = (path[i].x - fw/2)/1000.0;
                    double wy = (path[i].y - fh/2)/1000.0;
                    double theta = 0;
                    if (i < path.size()-1) {
                        double dx = path[i+1].x - path[i].x;
                        double dy = path[i+1].y - path[i].y;
                        theta = std::atan2(dy,dx);
                    }
                    wps.push_back(Eigen::Vector3d(wx,wy,theta));
                }
                trajB.CreateTrajectoriesFromPath(wps, t);
                planB = true;
            }
        }

        if (planA) {
            auto [fin, vel] = trajA.Update(robotA->position);
            Eigen::Vector3d velW = util::RotateAboutZ(vel, robotA->position.z());
            robotA->velocity = velW;
            if (fin) planA = false;
        }

        if (planB) {
            auto [fin, vel] = trajB.Update(robotB->position);
            Eigen::Vector3d velW = util::RotateAboutZ(vel, robotB->position.z());
            robotB->velocity = velW;
            if (fin) planB = false;
        }

        double distA = (robotA->GetCenterPosition().head<2>() - ball->GetCenterPosition().head<2>()).norm();
        double distB = (robotB->GetCenterPosition().head<2>() - ball->GetCenterPosition().head<2>()).norm();

        if (distA < 0.35) {
            dribbleA = true;
        }
        if (distB < 0.35) {
            dribbleB = true;
        }

        if (dribbleA) {
            kin::Dribble(*robotA, *ball, 1.0, false);
        }
        if (dribbleB) {
            kin::Dribble(*robotB, *ball, 1.0, false);
        }

        vis::ProcessInput(sim.GetRawGLFW(), objects);
        kin::UpdateKinematics(objects, dt);
        kin::CheckAndResolveCollisions(objects);

        if (!sim.RunSimulationStep(objects, dt)) {
            break;
        }

        if (util::GetCurrentTime() - startTime > 20.0) {
            std::cout << "[Demo] Finished" << std::endl;
            break;
        }
    }

    return 0;
}

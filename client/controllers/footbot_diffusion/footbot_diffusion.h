#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_encoder_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_turret_encoder_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_turret_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <queue>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <rpc/client.h>
#include <fstream>
#include <filesystem>
#include <boost/asio.hpp>

using namespace argos;
using outputTuple = std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>;

#define EPS 0.01f
#define MOVE_DIS 0.5f

struct Action {
    Real x;
    Real y;
    Real angle;
    std::deque<int> nodeIDS;
    enum Type {
        MOVE,
        TURN,
        STOP
    } type;
};

struct Pos {
    Real x;
    Real y;

    bool operator<(const Pos &other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

class PIDController {
public:
    PIDController(Real kp, Real ki, Real kd)
            : kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0) {}

    Real calculate(Real error) {
        integral_ += error * dt;
        Real derivative = (error - prevError_) / dt;
        prevError_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    Real kp_, ki_, kd_;
    Real prevError_;
    Real integral_;
    Real dt = 0.1;
};

class CFootBotDiffusion : public CCI_Controller {
public:
    CFootBotDiffusion();
    virtual ~CFootBotDiffusion() {}
    virtual void Init(TConfigurationNode &t_node);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
    int getRobotID() const { return agent_idx_in_server; }

private:
    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_FootBotProximitySensor *m_pcProximity;
    CCI_PositioningSensor *m_pcPosSens;
    int m_tickPerSec = 10;
    std::string robot_id;

    CDegrees m_cAlpha;
    int port_number;
    Real m_rotateWheelVelocity;
    Real m_angularVelocity;
    Real m_linearAcceleration;
    Real m_currVelocity;
    Real m_fWheelVelocity;
    CRange<CRadians> m_cGoStraightAngleRange;
    std::string m_outputDir;

    Real pidLinear(Real error);
    std::pair<Real, Real> Move(CVector3& targetPos, CVector3& currPos, Real currAngle, Real tolerance);
    void MoveForward(Real x, Real y, Real tolerance);
    std::pair<Real, Real> inline pidAngular(Real error);
    std::pair<Real, Real> Turn(Real targetAngle, Real currAngle, Real tolerance);
    void CalculateVelocity(Real x, Real y);
    Real ChangeCoordinateFromMapToArgos(Real x);
    Real ChangeCoordinateFromArgosToMap(Real x);
    void insertActions(std::vector<outputTuple> actions);
    double getReferenceSpeed(double dist);
    void updateQueue();

    std::deque<Action> q;
    std::queue<Real> velocityQueue;
    int moveForwardFlag;
    int count = 0;
    Real offset = 15.0;
    std::shared_ptr<rpc::client> client;

    std::shared_ptr<PIDController> angular_pid_ptr;
    std::shared_ptr<PIDController> linear_pid_ptr;

    Real dt = 0.1;
    Real prev_turn_error=0.0;
    Real integral_turn_error=0.0;
    Real kp_turn_ = 0.8;
    Real ki_turn_ = 0.0;
    Real kd_turn_ = 0.1;

    Real prevLeftVelocity_ = 0.0;
    Real prevRightVelocity_ = 0.0;
    Real prevVelocity_ = 0.0;
    Real prev_move_error=0.0;
    Real integral_move_error=0.0;
    Real kp_move_ = 0.6;
    Real ki_move_ = 0.0;
    Real kd_move_ = 0.0;
    std::string debug_id = "-1";
    int agent_idx_in_server = -1;


    int lineExistFlag = 0;
    bool terminateFlag = false;

    // std::ofstream outputFile;
    // std::string outputDir;

    bool is_initialized = false;
};

#endif

#include "footbot_diffusion.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <rpc/client.h>


CFootBotDiffusion::CFootBotDiffusion() :
        m_pcWheels(NULL),
        m_pcProximity(NULL),
        m_pcPosSens(NULL),
        m_cAlpha(10.0f),
        m_angularVelocity(0.5f),
        m_fWheelVelocity(2.5f),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                ToRadians(m_cAlpha)) {}

void CFootBotDiffusion::insertActions(std::vector<outputTuple> actions)
{
    for (const auto &action: actions) {
        std::string action1 = std::get<3>(action);
        int nodeID = std::get<1>(action);
        std::tuple<double, double> end_pos = std::get<5>(action);
        double x = ChangeCoordinateFromMapToArgos(std::get<1>(end_pos));
        double y = ChangeCoordinateFromMapToArgos(std::get<0>(end_pos));
        double angle;
        if (std::get<2>(action) == 0) {
            angle = 0.0;
        } else if (std::get<2>(action) == 1) {
            angle = 270.0;
        } else if (std::get<2>(action) == 2) {
            angle = 180.0;
        } else {
            angle = 90.0;
        }
        if (robot_id == debug_id) {
            std::cout << "Action: " << action1 << " NodeID: " << nodeID << " End Position: " << x << " " << y << " Angle: " << angle << std::endl;
        }
        if (action1 == "M") {
            std::deque<int> prev_ids;
            if (not q.empty() and q.back().type == Action::MOVE) {
                prev_ids = q.back().nodeIDS;
                q.pop_back();
            }
            prev_ids.push_back(nodeID);
            q.push_back({x, y, angle, prev_ids, Action::MOVE});
        } else if (action1 == "T") {
            q.push_back({x, y, angle, std::deque<int>{nodeID}, Action::TURN});
        } else {
            q.push_back({x, y, angle, std::deque<int>{nodeID}, Action::STOP});
        }
    }
}

void CFootBotDiffusion::Init(TConfigurationNode &t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");

    angular_pid_ptr = std::make_shared<PIDController>(1.0, 0.0, 0.1);
    linear_pid_ptr = std::make_shared<PIDController>(2.0, 0.0, 0.1);
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "omega", m_angularVelocity, m_angularVelocity);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "acceleration", m_linearAcceleration, m_linearAcceleration);
    GetNodeAttributeOrDefault(t_node, "portNumber", port_number, 8080);
    GetNodeAttributeOrDefault(t_node, "outputDir", m_outputDir,std::string("metaData/"));
    m_linearVelocity = 1.22 * m_angularVelocity;
    m_currVelocity = 0.0;
    CVector3 currPos = m_pcPosSens->GetReading().Position;
    robot_id = std::to_string((int) ChangeCoordinateFromArgosToMap(currPos.GetY())) + "_" +
               std::to_string((int) ChangeCoordinateFromArgosToMap(currPos.GetX()));
}


Real normalizeAngle(Real angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

double angleDifference(double curr_angle, double target_angle) {
    double diff = normalizeAngle(target_angle) - normalizeAngle(curr_angle);
    if (diff > 180.0) {
        diff -= 360.0;
    } else if (diff < -180.0) {
        diff += 360.0;
    }

    return abs(diff);
}

bool is_port_open(const std::string& ip, int port) {
    boost::asio::io_context io_context;
    boost::asio::ip::tcp::socket socket(io_context);
    
    try {
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);
        socket.connect(endpoint);
        return true;
    } catch (const boost::system::system_error& e) {
        return false;
    }
}

void CFootBotDiffusion::updateQueue() {
    if (q.empty()) {
        return;
    }
    auto first_act = q.front();
    if (first_act.type == Action::MOVE) {
        q.pop_front();
        while (not q.empty() and q.front().type == Action::MOVE) {
            auto next_act = q.front();
            q.pop_front();
            first_act.x = next_act.x;
            first_act.y = next_act.y;
            first_act.nodeIDS.insert(first_act.nodeIDS.end(), next_act.nodeIDS.begin(), next_act.nodeIDS.end());
        }
        q.push_front(first_act);
    }
}

void CFootBotDiffusion::ControlStep() {
    if (not is_initialized) {
        if (is_port_open("127.0.0.1", port_number)) {
            client = std::make_shared<rpc::client>("127.0.0.1", port_number);
        } else {
            return;
        }
        is_initialized = true;

        std::vector<outputTuple> actions = client->call("init", robot_id).as<std::vector<outputTuple>>();
        outputDir = client->call("get_config").as<std::string>();
        outputDir = "client_output/"+outputDir+"/";
        insertActions(actions);

        std::filesystem::path dirPath(outputDir);
        if (!std::filesystem::exists(dirPath)) {
            std::filesystem::create_directories(dirPath);
        }

        std::ifstream inputFile;
        inputFile.open(outputDir+robot_id+".csv");
        outputFile.open(outputDir+robot_id+".csv", std::ios::trunc);
        std::string line;
        bool lineExists = false;
        while(std::getline(inputFile, line)){
            if (line.find("Robot ID") != std::string::npos) {
                lineExists = true;
                break;
            }
        }
        inputFile.close();
        if (outputFile.is_open()) {
            if (!lineExists) {
                outputFile << std::setw(10) << "Robot ID"
                        << std::setw(20) << "Current Position X"
                        << std::setw(20) << "Current Position Y"
                        << std::setw(20) << "Current Angle"
                        << std::setw(20) << "Queue Length"
                        << std::setw(20) << "Left Velocity"
                        << std::setw(20) << "Right Velocity"
                        << std::setw(20) << "Count" 
                        << std::setw(20) << "Sim Time" << std::endl;
            }
        } else {
            std::cerr << "Unable to open output, errono:" << strerror(errno) << std::endl;
        }
        return;
    }
    Action a;
    a.type = Action::STOP;
    CQuaternion currOrient = m_pcPosSens->GetReading().Orientation;
    CRadians cZAngle, cYAngle, cXAngle;
    currOrient.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    Real currAngle = ToDegrees(cZAngle).GetValue();
    if (currAngle < 0.0) {
        currAngle = currAngle + 360.0;
    }
    Real left_v, right_v;
    CVector3 currPos = m_pcPosSens->GetReading().Position;
    if (count % 10 == 0) {
        std::vector<outputTuple> updateActions = client->call("update", robot_id).as<std::vector<outputTuple>>();
        if (updateActions.size() != 0) {
            insertActions(updateActions);
        }
    }
    count++;
    std::string receive_msg = "";
    updateQueue();
    while (!q.empty()) {
        a = q.front();
        CVector3 currPos = m_pcPosSens->GetReading().Position;
        CVector3 targetPos = CVector3(a.x, a.y, 0.0f);

        if (a.type == Action::MOVE && ((currPos - targetPos).Length() < EPS) and (abs(prevVelocity_)) <= dt*m_fWheelVelocity) {
            a.type = Action::STOP;
            q.pop_front();
            for (auto tmp_nodeId: a.nodeIDS) {
                receive_msg = client->call("receive_update", robot_id, tmp_nodeId).as<std::string>();
            }
            continue;
        }
        else if (a.type == Action::MOVE && (abs((currPos - targetPos).Length() + 0.5*(-static_cast<double>(a.nodeIDS.size()) + 1))) < EPS) {
            if (robot_id == debug_id) {
                std::cout << "Action: " << a.type << ", Target Position: (" << a.x << ", " << a.y << ")" <<
                ", Current Position: (" << currPos.GetX() << ", " << currPos.GetY() << "). Previous speed is: "
                << prevVelocity_ << std::endl;
                std::cout << "size of node id: " << a.nodeIDS.size() << std::endl;
                for (auto nodeId : a.nodeIDS) {
                        std::cout << "Node ID: " << nodeId << std::endl;
                    }
            }
            if (a.nodeIDS.size() > 1) {
                receive_msg = client->call("receive_update", robot_id, a.nodeIDS.front()).as<std::string>();
                q.front().nodeIDS.pop_front();
            }
            if ((currPos - targetPos).Length() < EPS) {
                if ((abs(prevVelocity_)) <= dt*m_fWheelVelocity) {
                    a.type = Action::STOP;
                    q.pop_front();
                    for (auto tmp_nodeId: a.nodeIDS) {
                        receive_msg = client->call("receive_update", robot_id, tmp_nodeId).as<std::string>();
                    }
                    continue;
                }
            }
        }
        else if (a.type == Action::TURN && angleDifference(currAngle, a.angle) < 0.5f) {
            a.type = Action::STOP;
            receive_msg = client->call("receive_update", robot_id, a.nodeIDS.front()).as<std::string>();
            q.pop_front();
            continue;
        }
        break;
    }

    if (a.type == Action::MOVE) {
        CVector3 targetPos = CVector3(a.x, a.y, 0.0f);
        CVector3 currPos = m_pcPosSens->GetReading().Position;
        std::pair<Real, Real> velocities = Move(targetPos, currPos, currAngle, 1.0f);
        left_v = velocities.first;
        right_v = velocities.second;
    } else if (a.type == Action::TURN) {
        std::pair<Real, Real> turn_velocities = Turn(a.angle, currAngle, 1.0f);
        left_v = turn_velocities.first;
        right_v = turn_velocities.second;

    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        left_v = 0.0f;
        right_v = 0.0f;
    }

    if (count % 200 == 0 && !terminateFlag){
        if (outputFile.is_open()) {
            outputFile << std::setw(10) << robot_id
            << std::setw(20) << ChangeCoordinateFromArgosToMap(currPos.GetY())
            << std::setw(20)<<ChangeCoordinateFromArgosToMap(currPos.GetX())
            << std::setw(20)<< currAngle
            << std::setw(20)<< q.size()
            << std::setw(20)<< left_v
            << std::setw(20)<< right_v
            << std::setw(20) << count
            << std::setw(20) << count/m_tickPerSec << std::endl;
        } else {
            std::cout << "Unable to open output file" << std::endl;
        }
    }

    if (receive_msg == "end" || receive_msg == "exit") {
        client->call("update_finish_agent", robot_id, count);
        if (outputFile.is_open()) {
            outputFile << std::setw(10) << robot_id 
            << std::setw(20) << ChangeCoordinateFromArgosToMap(currPos.GetY()) 
            << std::setw(20)<<ChangeCoordinateFromArgosToMap(currPos.GetX()) 
            << std::setw(20)<< currAngle 
            << std::setw(20)<< q.size() 
            << std::setw(20)<< left_v 
            << std::setw(20)<< right_v 
            << std::setw(20) << count
            << std::setw(20) << count/m_tickPerSec << std::endl;
            outputFile.close();
            terminateFlag = true;
        } else {
            std::cout << "Unable to open output file" << std::endl;
        }
    }
    if (receive_msg == "exit") {
        client->async_call("closeServer");
        exit(0);
    }
}


std::pair<Real, Real> CFootBotDiffusion::pidAngular(Real error)
{
    integral_turn_error += error * dt;
    Real derivative = (error - prev_turn_error) / dt;
    prev_turn_error = error;

    Real output = kp_turn_ * error + ki_turn_ * integral_turn_error + kd_turn_ * derivative;
    output = std::clamp(output, -m_linearVelocity, m_linearVelocity);
    Real left_v = -output, right_v = output;
    return std::make_pair(left_v, right_v);
}

Real CFootBotDiffusion::pidLinear(Real error)
{
    integral_move_error += error * dt;
    Real derivative = (error - prev_move_error) / dt;
    prev_move_error = error;
    Real desiredVelocity = kp_move_ * error + ki_move_ * integral_move_error + kd_move_ * derivative;
    return desiredVelocity;
}

std::pair<Real, Real> CFootBotDiffusion::Turn(Real targetAngle, Real currAngle, Real tolerance = 1.0f)
{
    Real error = normalizeAngle(targetAngle - currAngle);
    auto turn_v = pidAngular(error);
    Real left_v = turn_v.first;
    Real right_v = turn_v.second; 

    m_pcWheels->SetLinearVelocity(left_v, right_v);
    return std::make_pair(left_v, right_v);
}

inline Real toAngle(Real deltaX, Real deltaY) {
    Real targetAngle = std::atan2(deltaY, deltaX);
    Real tmp_angle = targetAngle/M_PI * 180.0 + 360;
    if (tmp_angle > 360) {
        tmp_angle -= 360;
    }
    assert(tmp_angle >= 0.0);
    return tmp_angle;
}

double CFootBotDiffusion::getReferenceSpeed(double dist)
{
    int dist_flag = 0;
    if (dist < 0) {
        dist_flag = -1;
    } else if (dist > 0) {
        dist_flag = 1;
    }
    return dist_flag * std::min(sqrt(2*m_linearAcceleration*std::abs(dist)*10), m_fWheelVelocity);
}

std::pair<Real, Real> CFootBotDiffusion::Move(CVector3& targetPos, CVector3& currPos, Real currAngle, Real tolerance = 1.0f)
{
    Real deltaX = targetPos.GetX() - currPos.GetX();
    Real deltaY = targetPos.GetY() - currPos.GetY();
    Real distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    Real targetAngle = toAngle(deltaX, deltaY);
    Real targetAngle2 = toAngle(-deltaX, -deltaY);

    Real angleError1 = normalizeAngle(targetAngle - currAngle);
    Real angleError2 = normalizeAngle(targetAngle2 - currAngle);
    Real angleError = 0.0;
    Real flag = 0.0;
    if (std::abs(angleError1) < std::abs(angleError2)) {
        angleError = angleError1;
        flag = 1.0;
    } else {
        angleError = angleError2;
        flag = -1.0;
    }

    Real refer_velocity = flag*getReferenceSpeed(distance);
    Real control_acc = pidLinear(refer_velocity - prevVelocity_);
    auto angularVelocity = pidAngular(angleError);

    Real maxDeltaV = m_linearAcceleration*dt;
    Real linearVelocity = prevVelocity_ + std::clamp(control_acc,  - maxDeltaV, maxDeltaV);

    Real left_v_total = linearVelocity;
    Real right_v_total = linearVelocity;

    left_v_total = std::clamp(left_v_total, -m_fWheelVelocity, m_fWheelVelocity);
    right_v_total = std::clamp(right_v_total, -m_fWheelVelocity, m_fWheelVelocity);
    left_v_total += (1/m_fWheelVelocity)*std::abs(linearVelocity)*angularVelocity.first;
    right_v_total += (1/m_fWheelVelocity)*std::abs(linearVelocity)*angularVelocity.second;

    prevLeftVelocity_ = left_v_total;
    prevRightVelocity_ = right_v_total;
    prevVelocity_ = linearVelocity;

    m_pcWheels->SetLinearVelocity(left_v_total, right_v_total);

    return std::make_pair(left_v_total, right_v_total);
}

void CFootBotDiffusion::TurnLeft(Real targetAngle, Real currAngle, Real tolerance = 1.0f) {
    Real angleDifference = targetAngle - currAngle;
    Real left_v, right_v;
    if (angleDifference > 0.0 && angleDifference < 180.0) {
        if (abs(angleDifference) <= 10.0) {
            left_v = -m_linearVelocity / (11.0 - abs(angleDifference));
            right_v = +m_linearVelocity / (11.0 - abs(angleDifference));
        } else {
            left_v = -m_linearVelocity;
            right_v = m_linearVelocity;
        }
    } else {
        if (abs(angleDifference) <= 10.0) {
            left_v = m_linearVelocity / (11.0 - abs(angleDifference));
            right_v = -m_linearVelocity / (11.0 - abs(angleDifference));
        } else {
            left_v = m_linearVelocity;
            right_v = -m_linearVelocity;
        }
    }
    m_pcWheels->SetLinearVelocity(left_v, right_v);
}

Real CFootBotDiffusion::ChangeCoordinateFromMapToArgos(Real x) {
    if (x == 0) {
        return 0;
    }
    return -x;
}

Real CFootBotDiffusion::ChangeCoordinateFromArgosToMap(Real x) {
    if (x == 0) {
        return 0;
    }
    return -x;
}

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

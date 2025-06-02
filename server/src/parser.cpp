#include "parser.h"

int getOrientation(int x1, int y1, int x2, int y2) {
    if (x1 == x2 and y1 == y2) {
        // indicate inplace waiting
        return -1;
    }
    if (x2 == x1) {
        return y2 > y1 ? 1 : 3; // North or South
    } else if (y2 == y1) {
        return x2 > x1 ? 2 : 0; // East or West
    } else {
        std::cerr << "Error with orientation!" << std::endl;
        exit(-2);
    }
}

void processAgentActions(const vector<Point>& points, vector<Step>& steps, bool flipped_coord) {
    steps.clear();
    int currentOrientation = 0;
    double currentTime = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == 0) {
            steps.push_back({points[i].x, points[i].y, currentOrientation, currentTime});
        } else {
            int neededOrientation = getOrientation(points[i-1].x, points[i-1].y, points[i].x, points[i].y);

            if (not flipped_coord) {
                neededOrientation = 3 - neededOrientation;
            }

            if (neededOrientation == -1 or neededOrientation == 4) {
                neededOrientation = currentOrientation;
            }

            if (neededOrientation != currentOrientation) {
                steps.push_back({points[i-1].x, points[i-1].y, neededOrientation, currentTime});
                currentOrientation = neededOrientation;
            }
            steps.push_back({points[i].x, points[i].y, currentOrientation, ++currentTime});
        }
    }
}

void processAgentActionsContinuous(const vector<Point>& points, vector<Step>& steps, bool flipped_coord) {
    steps.clear();
    int currentOrientation = 0;
    double currentTime = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == 0) {
            currentTime = points[i].time;
            steps.push_back({points[i].x, points[i].y, currentOrientation, currentTime});
        } else {
            int neededOrientation = getOrientation(points[i-1].x, points[i-1].y, points[i].x, points[i].y);
            if (not flipped_coord) {
                neededOrientation = 3 - neededOrientation;
            }
            if (neededOrientation != currentOrientation) {
                steps.push_back({points[i-1].x, points[i-1].y, neededOrientation, currentTime});
                currentOrientation = neededOrientation;
            }
            currentTime = points[i].time;
            steps.push_back({points[i].x, points[i].y, currentOrientation, currentTime});
        }
    }
}

vector<Point> parseLine(const string& line) {
    vector<Point> points;
    stringstream ss(line);
    char ignore;
    bool first_loc = true;
    int prev_x = 0, prev_y = 0;
    int x, y;
    double time = 1.0;

    ss.ignore(100, ':');

    while (ss >> ignore >> x >> ignore >> y >> ignore) {
        if (first_loc) {
            prev_x = x;
            prev_y = y;
            first_loc = false;
        }
        if (std::abs(prev_x - x) + std::abs(prev_y - y) >= 2) {
            raiseError("Invalid Plan");
            // std::cerr << "Invalid Plan, move from " << prev_x << "," << prev_y << ", to: " <<
            // x << "," << y << std::endl;
        }
        points.push_back({x, y, time++});
        prev_x = x;
        prev_y = y;
        ss >> ignore >> ignore;
    }

    return points;
}

vector<Point> parseLineContinuous(const string& line) {
    vector<Point> points;
    stringstream ss(line);
    char ignore;
    bool first_loc = true;
    int prev_x = 0, prev_y = 0;
    int x, y;
    double t;

    ss.ignore(100, ':');
    while (ss >> ignore >> x >> ignore >> y >> ignore >> t >> ignore) {
        if (first_loc) {
            prev_x = x;
            prev_y = y;
            first_loc = false;
        }
        if (std::abs(prev_x - x) + std::abs(prev_y - y) >= 2) {
            raiseError("Invalid Plan");
        }
        points.push_back({x, y, t});
        prev_x = x;
        prev_y = y;
        ss >> ignore >> ignore;
    }

    return points;
}

std::vector<std::vector<Action>> processActions(const std::vector<std::vector<Step>>& raw_steps, bool flipped_coord) {
    std::vector<std::vector<Action>> plans;
    int node_id=0;
    for (size_t i = 0; i < raw_steps.size(); ++i) {
        std::vector<Action> processedActions;
        for (size_t j = 1; j < raw_steps[i].size(); j++) {
            double prev_step_x, prev_step_y, curr_step_x, curr_step_y;
            if (flipped_coord) {
                prev_step_x = raw_steps[i][j-1].y;
                prev_step_y = raw_steps[i][j-1].x;
                curr_step_x = raw_steps[i][j].y;
                curr_step_y = raw_steps[i][j].x;
            } else {
                prev_step_x = raw_steps[i][j-1].x;
                prev_step_y = raw_steps[i][j-1].y;
                curr_step_x = raw_steps[i][j].x;
                curr_step_y = raw_steps[i][j].y;
            }
            Action processedAction;
            processedAction.robot_id = (int) i;
            processedAction.time = raw_steps[i][j-1].time;
            processedAction.start.first = prev_step_x;
            processedAction.start.second = prev_step_y;
            processedAction.goal.first = curr_step_x;
            processedAction.goal.second = curr_step_y;
            processedAction.orientation = raw_steps[i][j].orientation;
            processedAction.nodeID = node_id;
            Action accesoray_action = processedAction;

            if (processedAction.start == processedAction.goal &&
                raw_steps[i][j-1].orientation != raw_steps[i][j].orientation) {
                processedAction.type = 'T';
            } else if (processedAction.start != processedAction.goal &&
                        raw_steps[i][j-1].orientation == raw_steps[i][j].orientation) {
                double mid_x = (prev_step_x + curr_step_x)/2.0;
                double mid_y = (prev_step_y + curr_step_y)/2.0;
                processedAction.type = 'M';
                processedAction.start.first = prev_step_x;
                processedAction.start.second = prev_step_y;
                processedAction.goal.first = mid_x;
                processedAction.goal.second = mid_y;
                accesoray_action.type = 'M';
                accesoray_action.start.first = mid_x;
                accesoray_action.start.second = mid_y;
                accesoray_action.goal.first = curr_step_x;
                accesoray_action.goal.second = curr_step_y;
                accesoray_action.nodeID = node_id+1;
            } else if (processedAction.start == processedAction.goal &&
                       raw_steps[i][j-1].orientation == raw_steps[i][j].orientation) {
                continue;
            } else {
                std::cerr << "Invalid case exiting ..." << std::endl;
                exit(-1);
            }

            if (processedAction.type == 'T' and j == (raw_steps[i].size()-1)) {
                continue;
            }
            processedActions.push_back(processedAction);
            if (processedAction.type == 'M') {
                processedActions.push_back(accesoray_action);
                node_id++;
            }
            node_id++;
        }
        plans.push_back(processedActions);
    }
    return plans;
}

void showStepPoints(std::vector<std::vector<Step>>& raw_plan){
    for (size_t i = 0; i < raw_plan.size(); i++) {
        printf("Path of agent: %lu\n", i);
        for (auto& tmp_point: raw_plan[i]) {
            std::cout << "(" << tmp_point.x << ", " << tmp_point.y << "), "
                      << tmp_point.orientation << ", " << tmp_point.time << ", "
                      << std::endl;
        }
        printf("\n");
    }
}

void showActionsPlan(std::vector<std::vector<Action>>& plans) {
    for (size_t i = 0; i < plans.size(); i++) {
        printf("Path of agent: %lu\n", i);
        for (auto &action: plans[i]) {
            std::cout << "        {" << action.robot_id << ", " << action.time << ", "
                      << std::fixed << std::setprecision(1) << action.orientation << ", '"
                      << action.type << "', {" << action.start.first << ", " << action.start.second << "}, {"
                      << action.goal.first << ", " << action.goal.second << "}, " << action.nodeID << "}," << std::endl;
        }
        printf("\n");
    }
}

void raiseError(const string &msg) {
    item(
            val("type", "adg_error");
            key_log("info",
                msg
            );
        )
    // std::cerr << msg << std::endl;
    exit(-1);
}


bool parseEntirePlan(const std::string& input_file,
                     std::vector<std::vector<Action>>& plans,
                     double& raw_cost,
                     bool flipped_coord,
                     PlanType file_type){
    plans.clear();
    ifstream inFile(input_file);
    if (!inFile.is_open()) {
        cerr << "Failed to open the file." << endl;
        return false;
    }

    if (file_type == DISCRETE) {
        std::vector<std::vector<Step>> raw_plan;
        raw_cost = 0.0;
        string line;
        int agentId = 0;
        while (getline(inFile, line)) {
            std::vector<Step> tmp_plan;
            if (!line.empty()) {
                std::vector<Point> points = parseLine(line);
                raw_cost += static_cast<double> (points.size());
                processAgentActions(points, tmp_plan, flipped_coord);
                agentId++;
            }
            raw_plan.push_back(tmp_plan);
        }
        inFile.close();
        plans = processActions(raw_plan, flipped_coord);
        return true;
    } else if (file_type == CONTINUOUS) {
        std::vector<std::vector<Step>> raw_plan;
        raw_cost = 0.0;
        string line;
        int agentId = 0;
        while (getline(inFile, line)) {
            std::vector<Step> tmp_plan;
            if (!line.empty()) {
                vector<Point> points = parseLineContinuous(line);
                raw_cost += static_cast<double> (points.size());
                processAgentActionsContinuous(points, tmp_plan, flipped_coord);
                agentId++;
            }
            raw_plan.push_back(tmp_plan);
        }
        inFile.close();
        // showStepPoints(raw_plan);
        plans = processActions(raw_plan, flipped_coord);
        // showActionsPlan(plans);
        return true;
    } else {
        std::cerr << "Unsupported path format!" << std::endl;
    }
    exit(-1);
}

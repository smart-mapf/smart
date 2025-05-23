#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <iomanip>
#include "log.h"


using namespace std;

struct Point {
    int x, y;
    double time;
};

struct Step {
    int x, y, orientation;
    double time;
};

struct Action {
    int robot_id;
    double time;
    double orientation;
    char type;
    std::pair<double, double> start;
    std::pair<double, double> goal;
    int nodeID;
};

enum PlanType {
    DISCRETE,
    CONTINUOUS
};

int getOrientation(int x1, int y1, int x2, int y2);
void processAgentActions(const vector<Point>& points, vector<Step>& steps, int agentId);
vector<Point> parseLine(const string& line);
std::vector<std::vector<Action>> processActions(const std::vector<std::vector<Step>>& raw_steps, bool flipped_coord);
void processAgentActionsContinuous(const vector<Point>& points, vector<Step>& steps, bool flipped_coord);
bool parseEntirePlan(const std::string& input_file, std::vector<std::vector<Action>>& plans,
                     double& raw_cost, bool flipped_coord = true, PlanType file_type = PlanType::CONTINUOUS);
void raiseError(const string& msg);

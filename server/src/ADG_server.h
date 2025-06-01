#include <rpc/server.h>
#include <string>
#include <iostream>
#include "ADG.h"
#include <iostream>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <mutex>
#include <sstream>
#include <functional>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <map>
#include <numeric>
#include "json.hpp"
using json = nlohmann::json;

#include <mutex>

std::mutex globalMutex;

class ADG_Server{
public:
    ADG_Server(std::string& path_filename, std::string& target_output_filename,
        std::string map_name, std::string scen_name, std::string method_name, bool flip_coord);
    void saveStats();
    
    std::shared_ptr<ADG> adg;
    std::map<int, std::string> robotIDTOStartIndex;
    std::map<std::string, int> startIndexToRobotID;
    std::vector<std::vector<Action>> plans;
    std::vector<std::vector<int>> outgoingEdgesByRobot;
    std::vector<double> agent_finish_time;
    std::vector<int> agent_finish_sim_step;
    int latest_arr_sim_step = 0;
    bool all_agents_finished = false;
    std::vector<bool> agents_finish;
    std::string output_filename;
    std::string curr_map_name;
    std::string curr_scen_name;
    std::string curr_method_name;
    int numRobots = 0;
    int step_cnt = 0;
    double raw_plan_cost = -1.0;

private:
    std::string path_filename_;
};

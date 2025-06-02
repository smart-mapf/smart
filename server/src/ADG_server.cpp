#include "ADG_server.h"
#include "log.h"

std::vector<std::chrono::steady_clock::time_point> startTimers; // Start times for each robot
    // =======================

ADG_Server::ADG_Server(std::string &path_filename, 
    std::string& target_output_filename, 
    std::string map_name, 
    std::string scen_name,
    std::string method_name,
    bool flip_coord):
path_filename_(path_filename), curr_map_name(map_name), curr_scen_name(scen_name), curr_method_name(method_name)
 {
    if (path_filename == "none") {
        std::cerr << "No path file provided, exiting ..." << std::endl;
        exit(-1);
    } else {
        bool success = parseEntirePlan(path_filename, plans, raw_plan_cost, flip_coord);
        if (not success){
            std::cerr << "Incorrect path, no ADG constructed! exiting ..." << std::endl;
            exit(-1);
        }
    }

    adg = std::make_shared<ADG> (plans);
    output_filename = target_output_filename;
    numRobots = adg->numRobots();
    agent_finish_time.resize(numRobots, -1);
    agents_finish.resize(numRobots, false);
    agent_finish_sim_step.resize(numRobots, -1);
    auto name_mapping = adg->createRobotIDToStartIndexMaps();
    robotIDTOStartIndex = name_mapping.first;
    startIndexToRobotID = name_mapping.second;

    outgoingEdgesByRobot.resize(numRobots);
    startTimers.resize(numRobots);
}

void ADG_Server::saveStats() {
    std::ifstream infile(output_filename);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(output_filename);
        addHeads << "steps finish sim,sum of steps finish sim,time finish sim,sum finish time,original plan cost," <<
                 "#type-2 edges,#type-1 edges,#Nodes,"
                 "#Move,#Rotate,#Consecutive Move,#Agent pair," <<
                 "instance name,number of agent" << endl;
        addHeads.close();
    }
    ofstream stats(output_filename, std::ios::app);
    stats << latest_arr_sim_step << "," << std::accumulate(agent_finish_sim_step.begin(), agent_finish_sim_step.end(), 0) << "," <<
        *(std::max_element(agent_finish_time.begin(), agent_finish_time.end())) << "," <<
        std::accumulate(agent_finish_time.begin(), agent_finish_time.end(), 0.0) << "," <<
        raw_plan_cost << "," << adg->adg_stats.type2EdgeCount << "," <<
        adg->adg_stats.type1EdgeCount << "," << adg->adg_stats.totalNodes << "," <<
        adg->adg_stats.moveActionCount << "," << adg->adg_stats.rotateActionCount << "," <<
        adg->adg_stats.consecutiveMoveSequences << "," << static_cast<int>(adg->adg_stats.conflict_pairs.size()) << "," <<
        path_filename_ << "," << numRobots << endl;
    stats.close();
    // std::cout << "ADG statistics written to " << output_filename << std::endl;
    json result = {
        {"steps finish sim", latest_arr_sim_step},
        {"sum of steps finish sim", std::accumulate(agent_finish_sim_step.begin(), agent_finish_sim_step.end(), 0)},
        {"time finish sim", *(std::max_element(agent_finish_time.begin(), agent_finish_time.end()))},
        {"sum finish time", std::accumulate(agent_finish_time.begin(), agent_finish_time.end(), 0.0)},
        {"original plan cost", raw_plan_cost},
        {"#type-2 edges", adg->adg_stats.type2EdgeCount},
        {"#type-1 edges", adg->adg_stats.type1EdgeCount},
        {"#Nodes", adg->adg_stats.totalNodes},
        {"#Move", adg->adg_stats.moveActionCount},
        {"#Rotate", adg->adg_stats.rotateActionCount},
        {"#Consecutive Move", adg->adg_stats.consecutiveMoveSequences},
        {"#Agent pair", static_cast<int>(adg->adg_stats.conflict_pairs.size())},
        {"instance name", path_filename_},
        {"number of agent", numRobots}
    };

    // Output the JSON
    std::cout << result.dump() << std::endl; 
}


std::shared_ptr<ADG_Server> server_ptr = nullptr;

std::string receive_update(std::string& RobotID, int node_ID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    int Robot_ID = server_ptr->startIndexToRobotID[RobotID];
    bool status_update = server_ptr->adg->updateFinishedNode(Robot_ID, node_ID);
    server_ptr->adg->logAgentProgress(Robot_ID);
    server_ptr->adg->returnProgress();
    if (server_ptr->adg->isAgentFinished(Robot_ID)) {
        auto endTime = std::chrono::steady_clock::now();
        auto diff = endTime - startTimers[Robot_ID];
        double duration = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
        if (server_ptr->agent_finish_time[Robot_ID] < 0) {
            server_ptr->agent_finish_time[Robot_ID] = duration;
            server_ptr->agents_finish[Robot_ID] = true;
            logStatusChange(std::to_string(Robot_ID), "finished");
            server_ptr->adg->agent_act_status[Robot_ID] = FINISHED;
        }
    }

    server_ptr->all_agents_finished = true;
    for (auto status: server_ptr->agents_finish) {
        if (not status) {
            server_ptr->all_agents_finished = false;
        }
    }
    if (server_ptr->all_agents_finished) {
        return "exit";
    }
    if (server_ptr->agents_finish[Robot_ID]) {
        return "end";
    }
    return "None";
}

std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>> init(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    int Robot_ID = server_ptr->startIndexToRobotID[RobotID];
#ifdef DEBUG
    std::cerr << "TMP::Receive init request from agent " << RobotID << " with: " << Robot_ID << std::endl;
    if (Robot_ID == DEBUG_AGENT) {
        std::cerr << "Receive init request from agent " << Robot_ID << std::endl;
        exit(0);
    }
#endif
    startTimers[Robot_ID] = std::chrono::steady_clock::now();
    return server_ptr->adg->getPlan(Robot_ID);
}

int getRobotIdx(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    int Robot_ID = server_ptr->startIndexToRobotID[RobotID];
    return Robot_ID;
}

std::string getScenConfigName()
{
    std::string target_path = server_ptr->curr_method_name + "/" + server_ptr->curr_map_name + "/" + std::to_string(server_ptr->numRobots) + "/" + server_ptr->curr_scen_name;
    return target_path;
}


std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>> update(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);


    int Robot_ID = server_ptr->startIndexToRobotID[RobotID];
    // if (server_ptr->step_cnt % 20 == 0 and Robot_ID == 0) {
    //     server_ptr->adg->printProgress();
    // }
    server_ptr->step_cnt++;
#ifdef DEBUG
    if (Robot_ID == DEBUG_AGENT)
        std::cerr << "Receive update request from agent " << Robot_ID << std::endl;
#endif
    return server_ptr->adg->getPlan(Robot_ID);
}

void updateSimFinishTime(std::string& robot_id_str, int sim_step)
{
    int robot_id = server_ptr->startIndexToRobotID[robot_id_str];
    if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
        server_ptr->agent_finish_sim_step[robot_id] = sim_step;
        server_ptr->latest_arr_sim_step = sim_step;
    }
}

void closeServer(rpc::server& srv)
{
    // if (server_ptr->all_agents_finished) {
    //     std::cout << "Finish all actions, exiting..." << std::endl;
    // } else {
    //     std::cout << "BUG::exiting without finish all actions" << std::endl;
    // }
    item(
            val("type", "stats");
            str_log("mapf_plan_cost",
                server_ptr->raw_plan_cost
            );
            key_log("agent_exec_cost",
                obj_log(
                    for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
                        str_log(agent_id,
                            server_ptr->agent_finish_sim_step[agent_id]
                        );
                    }
                )
            );
        )
    server_ptr->saveStats();
    srv.stop();
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("path_file,p", po::value<string>(), "input file for path")
//            ("path_file,p", po::value<string>()->default_value("../data/maze-32-32-4_paths.txt"), "input file for path")
            ("port_number,n", po::value<int>()->default_value(8080), "rpc port number")
            ("flip_coord", po::value<bool>()->default_value(true), "input format of the mapf planner, 0 if xy, 1 if yx")
            ("output_file,o", po::value<string>()->default_value("stats.csv"), "output statistic filename")
            ("map_file,m", po::value<string>()->default_value("empty-8-8"), "map filename")
            ("scen_file,s", po::value<string>()->default_value("empty-8-8-random-1"), "scen filename")
            ("method_name", po::value<string>()->default_value("PBS"), "method we used")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }
    po::notify(vm);
    std::string filename = "none";

    if (vm.count("path_file")) {
        filename = vm["path_file"].as<string>();
    }
    // std::cout << "Solving for path name: " << filename << std::endl;
    std::string out_filename = vm["output_file"].as<string>();
    server_ptr = std::make_shared<ADG_Server>(filename, out_filename, vm["map_file"].as<string>(), 
        vm["scen_file"].as<string>(), vm["method_name"].as<string>(), vm["flip_coord"].as<bool>());
    logStatusChange("null", "initialized");

    int port_number = vm["port_number"].as<int>();
    try {
        rpc::server srv(port_number);  // Setup the server to listen on the specified port number
        srv.bind("receive_update", &receive_update);  // Bind the function to the server
        srv.bind("init", &init);
        srv.bind("update", &update);
        srv.bind("get_config", &getScenConfigName);
        srv.bind("update_finish_agent", &updateSimFinishTime);
        srv.bind("get_robot_idx", &getRobotIdx);
        srv.bind("closeServer", [&srv]() {
            closeServer(srv);
        });
        srv.run();  // Start the server, blocking call
    } catch (...) {
        // Catch any other exceptions
        std::cerr << "Fail to starting the server for scen "<< vm["scen_file"].as<string>() << " at port number: " << port_number << std::endl;
    }
    return 0;
}
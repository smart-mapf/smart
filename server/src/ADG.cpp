#include "ADG.h"

ADG::ADG(const std::vector<std::vector<Action>> &plans) {
  std::map<int, int> lastActionIndexByRobot;
  num_robots = static_cast<int>(plans.size());
  finished_node_idx.resize(num_robots, -1);
  enqueue_nodes_idx.resize(num_robots);
  graph.resize(num_robots);
  for (int i = 0; i < num_robots; i++) {
    int j = 0;
    for (const auto &action : plans[i]) {
      ADGNode node{action, j, {}, {}};
      graph[i].push_back(node);
      j++;
      total_nodes_cnt++;
    }
  }
  std::vector<std::vector<bool>> conflict_pair_table;
  conflict_pair_table.resize(plans.size());
  for (auto &tmp_entry : conflict_pair_table) {
    tmp_entry.resize(plans.size(), false);
  }

  for (int i = 0; i < plans.size(); i++) {
    adg_stats.type1EdgeCount += static_cast<int>(plans[i].size()) - 1;
    adg_stats.totalNodes += static_cast<int>(plans[i].size());
    bool consecutive_move = false;
    for (int j = 0; j < plans[i].size(); j++) {
      if (plans[i][j].type == 'M') {
        adg_stats.moveActionCount++;
        if (not consecutive_move) {
          consecutive_move = true;
          adg_stats.consecutiveMoveSequences++;
        }
      } else if (plans[i][j].type == 'T') {
        adg_stats.rotateActionCount++;
        consecutive_move = false;
      } else {
        std::cerr << "Invalid type!\n" << std::endl;
        consecutive_move = false;
      }
      for (int k = i + 1; k < plans.size(); k++) {
        for (int l = 0; l < plans[k].size(); l++) {
          bool found_conflict = false;
          if (plans[i][j].start == plans[k][l].goal &&
              plans[i][j].time <= plans[k][l].time) {
            std::shared_ptr<Edge> tmp_edge = std::make_shared<Edge>(i, k, j, l);
            graph[i][j].outEdges.push_back(tmp_edge);
            graph[k][l].incomeEdges.push_back(tmp_edge);
            found_conflict = true;
          } else if (plans[k][l].start == plans[i][j].goal &&
                     plans[k][l].time <= plans[i][j].time) {
            std::shared_ptr<Edge> tmp_edge = std::make_shared<Edge>(k, i, l, j);
            graph[i][j].incomeEdges.push_back(tmp_edge);
            graph[k][l].outEdges.push_back(tmp_edge);
            found_conflict = true;
          }
          if (found_conflict) {
            // if (i == 0 and k == 2) {
            //   printf("Agents %d and %d collide at action [(%f, %f)->(%f, %f)] "
            //          "and action [(%f, %f)->(%f, %f)]\n",
            //          i, k, plans[i][j].start.first, plans[i][j].start.second,
            //          plans[i][j].goal.first, plans[i][j].goal.second,
            //          plans[k][l].start.first, plans[k][l].start.second,
            //          plans[k][l].goal.first, plans[k][l].goal.second);
            // }
            adg_stats.type2EdgeCount++;
            if (not conflict_pair_table[i][k]) {
              conflict_pair_table[i][k] = true;
              conflict_pair_table[k][i] = true;
              adg_stats.conflict_pairs.emplace(i, k);
            }
          }
        }
      }
    }
  }

  if (hasCycle()) {
    raiseError("Find cycle!");
    // std::cerr << "Find cycle!" << std::endl;
  }
}

void printEdge() {}

std::pair<std::map<int, std::string>, std::map<std::string, int>>
ADG::createRobotIDToStartIndexMaps() {
  for (int robot_id = 0; robot_id < num_robots; robot_id++) {
    auto start = graph[robot_id][0].action.start;
    std::ostringstream oss;
    oss << static_cast<int>(start.first) << "_"
        << static_cast<int>(start.second);
    std::string startStr = oss.str();

    robotIDToStartIndex[robot_id] = startStr;
    startIndexToRobotID[startStr] = robot_id;
  }

  return {robotIDToStartIndex, startIndexToRobotID};
}

void inline updateADGNode(ADGNode &tmp_node) {
  bool valid = false;
  for (auto &tmp_in_edge : tmp_node.incomeEdges) {
    if (tmp_in_edge->valid) {
      valid = true;
      break;
    }
  }
  tmp_node.has_valid_in_edge = valid;
}

bool ADG::getAvailableNodes(int robot_id, std::vector<int> &available_nodes) {
  auto &curr_agent_plan = graph[robot_id];
  int latest_finished_idx = finished_node_idx[robot_id];
  int next_node_idx = latest_finished_idx + 1;
  for (int i = next_node_idx; i < curr_agent_plan.size(); i++) {
    if (curr_agent_plan[i].has_valid_in_edge) {
      updateADGNode(curr_agent_plan[i]);
    }

    if (curr_agent_plan[i].has_valid_in_edge) {
      break;
    }

    if (enqueue_nodes_idx[robot_id].empty() or
        enqueue_nodes_idx[robot_id].back() < i) {
      available_nodes.push_back(i);
    }
  }
  return true;
}

bool ADG::updateFinishedNode(int robot_id, int node_id) {
  int latest_finished_idx = finished_node_idx[robot_id];
  if (node_id <= latest_finished_idx) {
    std::cerr << "Reconfirming nodes!" << std::endl;
    return true;
  } else {
    if (not enqueue_nodes_idx[robot_id].empty() and
        node_id > enqueue_nodes_idx[robot_id].back()) {
      std::cerr << "Confirm for nodes never enqueue!" << std::endl;
      return false;
    } else {
      for (int tmp_idx = latest_finished_idx + 1; tmp_idx <= node_id;
           tmp_idx++) {
        for (auto &tmp_edge : graph[robot_id][tmp_idx].incomeEdges) {
          assert(not tmp_edge->valid);
        }

        for (auto &tmp_edge : graph[robot_id][tmp_idx].outEdges) {
          tmp_edge->valid = false;
        }
      }
      finished_node_idx[robot_id] = node_id;
      while (not enqueue_nodes_idx[robot_id].empty()) {
        if (enqueue_nodes_idx[robot_id].front() <= node_id) {
          enqueue_nodes_idx[robot_id].pop_front();
        } else {
          break;
        }
      }
      return true;
    }
  }
}

void ADG::setEnqueueNodes(int robot_id, std::vector<int> &enqueue_nodes) {
  auto &curr_enqueue = enqueue_nodes_idx[robot_id];
  if (curr_enqueue.empty()) {
    curr_enqueue.insert(curr_enqueue.end(), enqueue_nodes.begin(),
                        enqueue_nodes.end());
  } else {
    int i = 0;
    for (i = 0; i < enqueue_nodes.size(); i++) {
      if (enqueue_nodes[i] > curr_enqueue.back()) {
        break;
      }
    }
    curr_enqueue.insert(curr_enqueue.end(), enqueue_nodes.begin() + i,
                        enqueue_nodes.end());
  }
}

void ADG::findConstraining(int robot_id) {
  val("agent_id", std::to_string(robot_id));
  auto &curr_agent_plan = graph[robot_id];
  int latest_finished_idx = finished_node_idx[robot_id];
  int next_node_idx = latest_finished_idx + 1;
  key_log("constraining_agent",
  list(
      for (int i = next_node_idx; i < curr_agent_plan.size(); i++) {
        if (curr_agent_plan[i].has_valid_in_edge) {
          updateADGNode(curr_agent_plan[i]);
        }
        if (curr_agent_plan[i].has_valid_in_edge) {
          // std::cout << "Constraining idx: " << i << ";";
          for (auto tmp_edge : curr_agent_plan[i].incomeEdges) {
            if (tmp_edge->valid) {
              // std::cout << " Constraint Agent " << tmp_edge->from_agent_id
              //           << " at node " << tmp_edge->from_node_id << ";";
              obj_log(
                val("id", tmp_edge->from_agent_id);
              );
            }
          }
          break;
        }

      }
    );
  );
}

void ADG::printActions(
    const std::vector<std::tuple<std::string, int, double, std::string,
                                 std::pair<double, double>,
                                 std::pair<double, double>>> &actions) {
  for (const auto &action : actions) {
    std::string robot_id = std::get<0>(action);
    int time = std::get<1>(action);
    double orientation = std::get<2>(action);
    std::string type = std::get<3>(action);
    auto start = std::get<4>(action);
    auto goal = std::get<5>(action);

    std::cout << "Robot ID: " << robot_id << ", nodeID: " << time
              << ", Orientation: " << orientation << ", Type: " << type
              << ", Start: (" << start.first << ", " << start.second << ")"
              << ", Goal: (" << goal.first << ", " << goal.second << ")"
              << std::endl;
  }
}

SIM_PLAN ADG::getPlan(int agent_id) {
  SIM_PLAN sim_plan;
  std::vector<int> enque_acts;
  getAvailableNodes(agent_id, enque_acts);
  for (int enque_id : enque_acts) {
    const Action &action = graph[agent_id][enque_id].action;
    sim_plan.emplace_back(robotIDToStartIndex[action.robot_id], enque_id,
                          action.orientation, std::string(1, action.type),
                          action.start, action.goal);
    enqueue_nodes_idx[agent_id].push_back(enque_id);
  }
  return sim_plan;
}

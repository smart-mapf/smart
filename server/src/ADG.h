#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <cassert>
#include <string>
#include <sstream>
#include <iomanip>
#include <unordered_set>

#include "parser.h"

typedef std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>> SIM_PLAN;

enum AgentStatus {
    UNINITIALIZED = 0,
    ACTIVE = 1,
    IDLE = 2,
    FINISHED = 3,
};

struct Edge {
    int from_agent_id;
    int to_agent_id;
    int from_node_id;
    int to_node_id;
    bool valid = true;
    Edge(int from_agent, int to_agent, int from_node, int to_node): from_agent_id(from_agent), to_agent_id(to_agent),
    from_node_id(from_node), to_node_id(to_node) {}
};

struct ADG_STATS {
    int type1EdgeCount = 0;
    int type2EdgeCount = 0;
    int moveActionCount = 0;
    int rotateActionCount = 0;
    int consecutiveMoveSequences = 0;
    int totalNodes = 0;
    std::set<std::pair<int, int>> conflict_pairs;
};

struct ADGNode {
    Action action;
    int node_id;
    std::vector<std::shared_ptr<Edge>> incomeEdges;
    std::vector<std::shared_ptr<Edge>> outEdges;
    bool has_valid_in_edge = true;
};

class ADG {
public:
    ADG(const std::vector<std::vector<Action>>& plans);
    [[nodiscard]] int numRobots() const {
        return num_robots;
    }

    [[nodiscard]] int numNodes() const {
        return total_nodes_cnt;
    }

    std::pair<std::map<int, std::string>, std::map<std::string, int>> createRobotIDToStartIndexMaps();
    bool getAvailableNodes(int robot_id, std::vector<int>& available_nodes);
    bool updateFinishedNode(int robot_id, int node_id);
    void setEnqueueNodes(int robot_id, std::vector<int>& enqueue_nodes);
    void logAgentProgress(int robot_id) {
        logFinishPercentage(robot_id, finished_node_idx[robot_id]+1, static_cast<int>(graph[robot_id].size()) );
    }
    SIM_PLAN getPlan(int agent_id);
    bool isAgentFinished(int robot_id) {
        return finished_node_idx[robot_id] >= graph[robot_id].size()-1;
    }

    void printProgress()
    {
        for (int agent_id = 0; agent_id < num_robots; agent_id++) {
            std::cout << "Agent " << agent_id << ", ID: " << robotIDToStartIndex[agent_id] << " with plan size " << graph[agent_id].size() << ": ";
            findConstraining(agent_id);
            for (int i = 0; i <= finished_node_idx[agent_id]; i++) {
                std::cout << "#";
            }
            for (auto elem: enqueue_nodes_idx[agent_id]) {
                std::cout << '0';
            }
            int unstart;
            if (enqueue_nodes_idx[agent_id].empty()) {
                unstart = finished_node_idx[agent_id];
            } else {
                unstart = enqueue_nodes_idx[agent_id].back();
            }
            for (int i = unstart+1; i < graph[agent_id].size(); i++) {
                std::cout << "*";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void returnProgress() {
        item(
            val("type", "adg_progress");
            key_log("constraints", 
                list(
                    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
                        obj_log(findConstraining(agent_id));
                    }
                )
            );
        )
    }

        // Helper DFS function
    using loopNode = std::pair<int, int>;
    struct NodeHash {
        std::size_t operator()(const loopNode& n) const noexcept {
            return std::hash<int>()(n.first) ^ (std::hash<int>()(n.second) << 1);
        }
    };
    bool dfs(int agent_id, int node_id,
             std::unordered_map<int, std::unordered_set<int>>& visited,
             std::unordered_map<int, std::unordered_set<int>>& recStack,
             const std::vector<std::vector<ADGNode>>& graph,
             std::unordered_map<loopNode, loopNode, NodeHash>& parent,
             std::vector<loopNode>& cycle_path) {
        visited[agent_id].insert(node_id);
        recStack[agent_id].insert(node_id);

        const ADGNode& node = graph[agent_id][node_id];
        for (const auto& edge : node.outEdges) {
            if (!edge->valid) continue;
            int next_agent = edge->to_agent_id;
            int next_node = edge->to_node_id;

            if (visited[next_agent].count(next_node) == 0) {
                parent[{next_agent, next_node}] = {agent_id, node_id};
                if (dfs(next_agent, next_node, visited, recStack, graph, parent, cycle_path)) {
                    return true;
                }
            } else if (recStack[next_agent].count(next_node)) {
                // Cycle detected!
                // Reconstruct cycle path
                loopNode current = {agent_id, node_id};
                cycle_path.push_back({next_agent, next_node});
                while (current != loopNode{next_agent, next_node}) {
                    cycle_path.push_back(current);
                    current = parent[current];
                }
                cycle_path.push_back({next_agent, next_node}); // close the loop
                std::reverse(cycle_path.begin(), cycle_path.end());
                return true;
            }
        }
        // Visit its type-1 neighbor
        if (node_id + 1 < graph[agent_id].size()) {
            int next_agent = agent_id;
            int next_node = node_id + 1;

            if (visited[next_agent].count(next_node) == 0) {
                parent[{next_agent, next_node}] = {agent_id, node_id};
                if (dfs(next_agent, next_node, visited, recStack, graph, parent, cycle_path)) {
                    return true;
                }
            } else if (recStack[next_agent].count(next_node)) {
                // Cycle detected!
                // Reconstruct cycle path
                loopNode current = {agent_id, node_id};
                cycle_path.push_back({next_agent, next_node});
                while (current != loopNode{next_agent, next_node}) {
                    cycle_path.push_back(current);
                    current = parent[current];
                }
                cycle_path.push_back({next_agent, next_node}); // close the loop
                std::reverse(cycle_path.begin(), cycle_path.end());
                return true;
            }
        }
        recStack[agent_id].erase(node_id);
        return false;
    }



    bool hasCycle() {
        std::unordered_map<int, std::unordered_set<int>> visited;
        std::unordered_map<int, std::unordered_set<int>> recStack;
        std::unordered_map<loopNode, loopNode, NodeHash> parent;
        std::vector<loopNode> cycle_path;

        for (int agent_id = 0; agent_id < graph.size(); ++agent_id) {
            for (const auto& node : graph[agent_id]) {
                if (visited[agent_id].count(node.node_id) == 0) {
                    if (dfs(agent_id, node.node_id, visited, recStack, graph, parent, cycle_path)) {
                        // Print the cycle path
                        std::cout << "Cycle detected:\n";
                        for (const auto& [aid, nid] : cycle_path) {
                            std::cout << "(Agent " << aid << ", Node " << nid << ") -> ";
                        }
                        std::cout << "(back to start)\n";
                        return true;
                    }
                }
            }
        }

        // std::cout << "No cycle detected.\n";
        return false;
    }

private:
    void printActions(const std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>>& actions);
    void findConstraining(int robot_id);

public:
    std::vector< int > finished_node_idx;
    std::vector< std::deque<int> > enqueue_nodes_idx;
    ADG_STATS adg_stats;
    std::vector< AgentStatus > agent_act_status;

private:
    std::vector<std::vector<ADGNode>> graph;
    int num_robots = 0;
    int total_nodes_cnt = 0;
    std::map<int, std::string> robotIDToStartIndex;
    std::map<std::string, int> startIndexToRobotID;
};

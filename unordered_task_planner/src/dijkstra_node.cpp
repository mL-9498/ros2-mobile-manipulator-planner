#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planner_interfaces/msg/task_visit.hpp"
#include "planner_interfaces/srv/plan_unordered_tasks.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unordered_task_planner
{
using Service = planner_interfaces::srv::PlanUnorderedTasks;

struct State
{
  std::uint64_t finished_mask{0};
  int current_index{-1};

  bool operator==(const State & other) const
  {
    return finished_mask == other.finished_mask && current_index == other.current_index;
  }
};

struct StateHash
{
  std::size_t operator()(const State & state) const
  {
    const auto first = std::hash<std::uint64_t>{}(state.finished_mask);
    const auto second = std::hash<int>{}(state.current_index);
    return first ^ (second + 0x9e3779b9 + (first << 6) + (first >> 2));
  }
};

struct QueueItem
{
  double cost;
  State state;

  bool operator>(const QueueItem & other) const
  {
    if (std::abs(cost - other.cost) > 1e-12) {
      return cost > other.cost;
    }
    if (state.current_index != other.state.current_index) {
      return state.current_index > other.state.current_index;
    }
    return state.finished_mask > other.state.finished_mask;
  }
};

struct Parent
{
  State previous;
  std::uint64_t newly_completed{0};
  double move_cost{0.0};
  double accumulated_cost{0.0};
};

class DijkstraPlanner
{
public:
  bool solve(
    const Service::Request & request,
    std::vector<planner_interfaces::msg::TaskVisit> & visits,
    double & total_cost, int & expanded_states, std::string & error)
  {
    const int task_count = request.task_count;
    const int candidate_count = static_cast<int>(request.candidates.size());
    if (task_count <= 0 || task_count > 63) {
      error = "task_count must be in [1, 63]";
      return false;
    }
    if (candidate_count == 0) {
      error = "at least one candidate base pose is required";
      return false;
    }
    if (request.travel_cost_matrix.size() !=
      static_cast<std::size_t>(candidate_count * candidate_count))
    {
      error = "travel_cost_matrix must contain N*N row-major values";
      return false;
    }

    std::vector<std::uint64_t> coverage(candidate_count, 0);
    int start_index = -1;
    for (int i = 0; i < candidate_count; ++i) {
      if (request.candidates[i].base_id == request.start_base_id) {
        start_index = i;
      }
      for (const auto task_id : request.candidates[i].covered_task_ids) {
        if (task_id < 0 || task_id >= task_count) {
          error = "a covered task ID is outside [0, task_count)";
          return false;
        }
        coverage[i] |= (std::uint64_t{1} << task_id);
      }
    }
    if (request.start_base_id >= 0 && start_index < 0) {
      error = "start_base_id does not match any candidate, or use -1 for a free first action";
      return false;
    }

    for (const double cost : request.travel_cost_matrix) {
      if (std::isnan(cost) || cost < 0.0) {
        error = "travel costs must be non-negative or positive infinity";
        return false;
      }
    }

    const std::uint64_t goal_mask = (std::uint64_t{1} << task_count) - 1;
    const State start{0, start_index};
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    std::unordered_map<State, double, StateHash> best_cost;
    std::unordered_map<State, Parent, StateHash> parents;
    best_cost[start] = 0.0;
    open.push({0.0, start});
    State goal;
    bool found = false;
    expanded_states = 0;

    while (!open.empty()) {
      const QueueItem current = open.top();
      open.pop();
      const auto best_it = best_cost.find(current.state);
      if (best_it == best_cost.end() || current.cost > best_it->second + 1e-12) {
        continue;
      }
      ++expanded_states;
      if (current.state.finished_mask == goal_mask) {
        goal = current.state;
        found = true;
        break;
      }

      for (int next = 0; next < candidate_count; ++next) {
        const std::uint64_t newly_completed =
          coverage[next] & ~current.state.finished_mask;
        if (newly_completed == 0) {
          continue;
        }

        double move_cost = 0.0;
        if (current.state.current_index >= 0 && current.state.current_index != next) {
          move_cost = request.travel_cost_matrix[
            current.state.current_index * candidate_count + next];
        }
        if (!std::isfinite(move_cost)) {
          continue;
        }

        const State next_state{
          current.state.finished_mask | newly_completed,
          next};
        const double next_cost = current.cost + move_cost;
        const auto old = best_cost.find(next_state);
        if (old != best_cost.end() && old->second <= next_cost + 1e-12) {
          continue;
        }

        best_cost[next_state] = next_cost;
        parents[next_state] = Parent{
          current.state, newly_completed, move_cost, next_cost};
        open.push({next_cost, next_state});
      }
    }

    if (!found) {
      error = "no sequence can complete every task with finite travel costs";
      return false;
    }

    std::vector<planner_interfaces::msg::TaskVisit> reversed;
    for (State state = goal; !(state == start); ) {
      const auto parent_it = parents.find(state);
      if (parent_it == parents.end()) {
        error = "internal parent-chain error";
        return false;
      }
      planner_interfaces::msg::TaskVisit visit;
      visit.base_id = request.candidates[state.current_index].base_id;
      visit.move_cost = parent_it->second.move_cost;
      visit.accumulated_cost = parent_it->second.accumulated_cost;
      for (int task = 0; task < task_count; ++task) {
        if (parent_it->second.newly_completed & (std::uint64_t{1} << task)) {
          visit.newly_completed_task_ids.push_back(task);
        }
      }
      reversed.push_back(std::move(visit));
      state = parent_it->second.previous;
    }
    std::reverse(reversed.begin(), reversed.end());
    visits = std::move(reversed);
    total_cost = best_cost.at(goal);
    return true;
  }
};

class DijkstraNode : public rclcpp::Node
{
public:
  DijkstraNode() : Node("dijkstra_task_planner")
  {
    service_ = create_service<Service>(
      "plan_unordered_tasks",
      [this](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        int expanded = 0;
        response->success = planner_.solve(
          *request, response->visits, response->total_cost, expanded, response->message);
        response->expanded_states = expanded;
        if (response->success) {
          response->message = "minimum-cost unordered task sequence found";
        }
      });
    RCLCPP_INFO(get_logger(), "Dijkstra unordered-task service is ready");
  }

private:
  DijkstraPlanner planner_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace unordered_task_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unordered_task_planner::DijkstraNode>());
  rclcpp::shutdown();
  return 0;
}

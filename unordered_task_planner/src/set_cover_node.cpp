#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "planner_interfaces/srv/compute_set_cover.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unordered_task_planner
{
using Service = planner_interfaces::srv::ComputeSetCover;

struct Candidate
{
  int id;
  std::vector<bool> covers;
};

class ExactSetCover
{
public:
  std::vector<int> solve(int task_count, std::vector<Candidate> candidates)
  {
    task_count_ = task_count;
    candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(), [](const Candidate & c) {
        return std::none_of(c.covers.begin(), c.covers.end(), [](bool value) {return value;});
      }),
      candidates.end());
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return std::count(a.covers.begin(), a.covers.end(), true) >
             std::count(b.covers.begin(), b.covers.end(), true);
    });
    candidates_ = std::move(candidates);
    best_.clear();
    std::vector<int> selected;
    std::vector<bool> covered(task_count_, false);
    search(0, selected, covered);
    return best_;
  }

private:
  bool complete(const std::vector<bool> & covered) const
  {
    return std::all_of(covered.begin(), covered.end(), [](bool value) {return value;});
  }

  bool remaining_can_cover(std::size_t index, const std::vector<bool> & covered) const
  {
    std::vector<bool> possible = covered;
    for (std::size_t i = index; i < candidates_.size(); ++i) {
      for (int task = 0; task < task_count_; ++task) {
        possible[task] = possible[task] || candidates_[i].covers[task];
      }
    }
    return complete(possible);
  }

  void search(
    std::size_t index, std::vector<int> & selected,
    const std::vector<bool> & covered)
  {
    if (complete(covered)) {
      if (best_.empty() || selected.size() < best_.size()) {
        best_ = selected;
      }
      return;
    }
    if (index >= candidates_.size() || (!best_.empty() && selected.size() >= best_.size()) ||
      !remaining_can_cover(index, covered))
    {
      return;
    }

    auto with_current = covered;
    bool adds_coverage = false;
    for (int task = 0; task < task_count_; ++task) {
      if (!with_current[task] && candidates_[index].covers[task]) {
        adds_coverage = true;
      }
      with_current[task] = with_current[task] || candidates_[index].covers[task];
    }
    if (adds_coverage) {
      selected.push_back(candidates_[index].id);
      search(index + 1, selected, with_current);
      selected.pop_back();
    }
    search(index + 1, selected, covered);
  }

  int task_count_{0};
  std::vector<Candidate> candidates_;
  std::vector<int> best_;
};

class SetCoverNode : public rclcpp::Node
{
public:
  SetCoverNode() : Node("set_cover_node")
  {
    service_ = create_service<Service>(
      "compute_set_cover",
      [this](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        handle_request(*request, *response);
      });
    RCLCPP_INFO(get_logger(), "Exact Set Cover service is ready");
  }

private:
  void handle_request(const Service::Request & request, Service::Response & response)
  {
    if (request.task_count <= 0) {
      response.message = "task_count must be positive";
      return;
    }

    std::vector<Candidate> candidates;
    candidates.reserve(request.candidates.size());
    for (const auto & input : request.candidates) {
      Candidate candidate{input.base_id, std::vector<bool>(request.task_count, false)};
      for (const auto task_id : input.covered_task_ids) {
        if (task_id < 0 || task_id >= request.task_count) {
          response.message = "covered_task_ids contains an out-of-range task";
          return;
        }
        candidate.covers[task_id] = true;
      }
      candidates.push_back(std::move(candidate));
    }

    response.selected_base_ids = solver_.solve(request.task_count, std::move(candidates));
    response.success = !response.selected_base_ids.empty();
    response.message = response.success ? "minimum-cardinality cover found" :
      "the candidates cannot cover every task";
  }

  ExactSetCover solver_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace unordered_task_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unordered_task_planner::SetCoverNode>());
  rclcpp::shutdown();
  return 0;
}

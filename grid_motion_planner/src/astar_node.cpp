#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_interfaces/srv/plan_grid_path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace grid_motion_planner
{
using Service = planner_interfaces::srv::PlanGridPath;

struct QueueEntry
{
  double f;
  int index;
  bool operator>(const QueueEntry & other) const {return f > other.f;}
};

class AStarNode : public rclcpp::Node
{
public:
  AStarNode() : Node("astar_node")
  {
    service_ = create_service<Service>(
      "plan_grid_path",
      [this](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        plan(*request, *response);
      });
    RCLCPP_INFO(get_logger(), "Grid A* service is ready");
  }

private:
  static double heuristic(int ax, int ay, int bx, int by)
  {
    return std::hypot(static_cast<double>(ax - bx), static_cast<double>(ay - by));
  }

  void plan(const Service::Request & request, Service::Response & response)
  {
    const auto cell_count = static_cast<std::size_t>(request.width) * request.height;
    if (request.width == 0 || request.height == 0 || request.resolution <= 0.0 ||
      request.occupancy.size() != cell_count)
    {
      response.message = "invalid grid dimensions, resolution or occupancy data";
      return;
    }

    const auto to_x = [&](double world_x) {
        return static_cast<int>(std::floor((world_x - request.origin_x) / request.resolution));
      };
    const auto to_y = [&](double world_y) {
        return static_cast<int>(std::floor((world_y - request.origin_y) / request.resolution));
      };
    const int sx = to_x(request.start.x);
    const int sy = to_y(request.start.y);
    const int gx = to_x(request.goal.x);
    const int gy = to_y(request.goal.y);
    const auto inside = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < static_cast<int>(request.width) &&
               y < static_cast<int>(request.height);
      };
    const auto index = [&](int x, int y) {
        return y * static_cast<int>(request.width) + x;
      };
    if (!inside(sx, sy) || !inside(gx, gy) ||
      request.occupancy[index(sx, sy)] != 0 || request.occupancy[index(gx, gy)] != 0)
    {
      response.message = "start or goal is outside the free grid";
      return;
    }

    constexpr double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> g(cell_count, infinity);
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    const int start = index(sx, sy);
    const int goal = index(gx, gy);
    g[start] = 0.0;
    open.push({heuristic(sx, sy, gx, gy), start});

    const std::array<std::pair<int, int>, 8> moves{{
      {-1, 0}, {1, 0}, {0, -1}, {0, 1},
      {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};

    while (!open.empty()) {
      const auto current = open.top();
      open.pop();
      if (current.index == goal) {
        break;
      }
      const int cx = current.index % static_cast<int>(request.width);
      const int cy = current.index / static_cast<int>(request.width);
      if (current.f > g[current.index] + heuristic(cx, cy, gx, gy) + 1e-12) {
        continue;
      }

      for (const auto & [dx, dy] : moves) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!inside(nx, ny)) {
          continue;
        }
        const int next = index(nx, ny);
        if (request.occupancy[next] != 0) {
          continue;
        }
        if (dx != 0 && dy != 0 &&
          (request.occupancy[index(cx + dx, cy)] != 0 ||
          request.occupancy[index(cx, cy + dy)] != 0))
        {
          continue;
        }
        const double step = (dx == 0 || dy == 0) ? 1.0 : std::sqrt(2.0);
        const double tentative = g[current.index] + step;
        if (tentative < g[next]) {
          g[next] = tentative;
          parent[next] = current.index;
          open.push({tentative + heuristic(nx, ny, gx, gy), next});
        }
      }
    }

    if (!std::isfinite(g[goal])) {
      response.message = "no collision-free path exists";
      return;
    }

    std::vector<int> cells;
    for (int current = goal; current >= 0; current = parent[current]) {
      cells.push_back(current);
      if (current == start) {
        break;
      }
    }
    std::reverse(cells.begin(), cells.end());
    response.path.header.frame_id = "map";
    response.path.header.stamp = now();
    for (const int cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = response.path.header;
      const int x = cell % static_cast<int>(request.width);
      const int y = cell / static_cast<int>(request.width);
      pose.pose.position.x = request.origin_x + (x + 0.5) * request.resolution;
      pose.pose.position.y = request.origin_y + (y + 0.5) * request.resolution;
      pose.pose.orientation.w = 1.0;
      response.path.poses.push_back(pose);
    }
    response.path_length = g[goal] * request.resolution;
    response.success = true;
    response.message = "path found";
  }

  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace grid_motion_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grid_motion_planner::AStarNode>());
  rclcpp::shutdown();
  return 0;
}

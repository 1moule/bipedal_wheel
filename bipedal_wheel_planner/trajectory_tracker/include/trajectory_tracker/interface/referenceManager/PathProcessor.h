//
// Created by guanlin on 25-12-8.
//

#pragma once

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf2/utils.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace trajectory_tracker
{
// 定义轻量级结果结构体
struct LookAheadResult
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  bool valid = false;
};

class PathProcessor
{
public:
  /**
     * @brief 高效剪裁路径：找到最近点，并返回从最近点开始的子路径
     * @note 性能优化：使用平方距离比较，减少 sqrt 调用
     */
  static nav_msgs::Path prunePath(
    const nav_msgs::Path & global_path, const geometry_msgs::Point & robot_pos)
  {
    if (global_path.poses.empty()) return global_path;

    double min_dist_sq = std::numeric_limits<double>::max();
    size_t min_idx = 0;

    // 1. 寻找最近点索引 (使用平方距离)
    // 优化：通常机器人就在路径开头附近，可以设置搜索窗口，这里为了通用性搜全长
    for (size_t i = 0; i < global_path.poses.size(); ++i) {
      double dx = global_path.poses[i].pose.position.x - robot_pos.x;
      double dy = global_path.poses[i].pose.position.y - robot_pos.y;
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        min_idx = i;
      }
    }

    // 2. 构造剪裁后的路径
    nav_msgs::Path pruned_path;
    pruned_path.header = global_path.header;

    // 预分配内存，避免 push_back 时的扩容开销
    size_t remaining_size = global_path.poses.size() - min_idx;
    if (remaining_size > 0) {
      pruned_path.poses.reserve(remaining_size);
      // 批量插入，比循环 push_back 更快
      pruned_path.poses.insert(
        pruned_path.poses.end(), global_path.poses.begin() + min_idx, global_path.poses.end());
    }

    return pruned_path;
  }

  /**
     * @brief 计算前瞻点和曲率
     * @note 核心优化：使用一元二次方程直接解析线段与圆的交点，无需迭代
     */
  static LookAheadResult computeLookAheadPoint(
    const nav_msgs::Path & path, const geometry_msgs::Point & robot_pos, double lookahead_dist)
  {
    LookAheadResult result;
    if (path.poses.empty()) return result;

    double L_sq = lookahead_dist * lookahead_dist;

    // 1. 寻找刚超出前瞻距离的线段 (P_in -> P_out)
    // 假设路径已经剪裁过，起点在机器人附近
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & pt = path.poses[i].pose.position;
      double dx = pt.x - robot_pos.x;
      double dy = pt.y - robot_pos.y;
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq >= L_sq) {
        // 找到了圆外的第一个点 path[i]
        if (i == 0) {
          // 如果第一个点就在圆外，直接用第一个点（或根据需求报错）
          result.x = pt.x;
          result.y = pt.y;
          result.theta = tf2::getYaw(path.poses[i].pose.orientation);
          result.kappa = 0.0;
          result.valid = true;
          return result;
        }

        // 线段起点 P1 (圆内), 终点 P2 (圆外)
        const auto & p1 = path.poses[i - 1].pose.position;
        const auto & p2 = path.poses[i].pose.position;

        // 2. 解析几何求交点 (求解 t)
        // 设 P(t) = P1 + t * (P2 - P1), 寻找 t 使得 ||P(t) - Robot|| = L
        // 定义向量 d = P2 - P1, f = P1 - Robot
        double d_x = p2.x - p1.x;
        double d_y = p2.y - p1.y;
        double f_x = p1.x - robot_pos.x;
        double f_y = p1.y - robot_pos.y;

        // 构造方程: (d_x^2 + d_y^2)t^2 + 2(f_x*d_x + f_y*d_y)t + (f_x^2 + f_y^2 - L^2) = 0
        // 即 a*t^2 + b*t + c = 0
        double a = d_x * d_x + d_y * d_y;
        double b = 2.0 * (f_x * d_x + f_y * d_y);
        double c = (f_x * f_x + f_y * f_y) - L_sq;

        double discriminant = b * b - 4 * a * c;
        double t = 0.0;

        if (discriminant < 0 || a == 0) {
          // 异常情况：无解（理论不应发生，因为P1在内P2在外）或两点重合
          t = 0.0;
        } else {
          // 我们需要正根，且由于从圆内射向圆外，取较大的那个根通常更稳健
          // 但标准求根公式中，因为 a>0 且 c<0 (P1在圆内), 必有一正一负，取正根
          t = (-b + std::sqrt(discriminant)) / (2.0 * a);
          // 限制 t 在 [0, 1] 范围内
          t = std::max(0.0, std::min(1.0, t));
        }

        // 计算交点坐标
        result.x = p1.x + t * d_x;
        result.y = p1.y + t * d_y;

        // 计算朝向 (使用线段切线方向)
        result.theta = std::atan2(d_y, d_x);

        // 3. 计算曲率 (使用 Menger Curvature - 三点圆拟合)
        // 需要 P1, P2 和 P3 (如果有)
        if (i + 1 < path.poses.size()) {
          const auto & p3 = path.poses[i + 1].pose.position;
          result.kappa = calculateMengerCurvature(p1, p2, p3);
        } else {
          result.kappa = 0.0;
        }

        result.valid = true;
        return result;
      }
    }

    // 如果跑完了循环都没找到圆外的点，取终点
    const auto & end_pose = path.poses.back();
    result.x = end_pose.pose.position.x;
    result.y = end_pose.pose.position.y;
    result.theta = tf2::getYaw(end_pose.pose.orientation);
    result.kappa = 0.0;
    result.valid = true;

    return result;
  }

private:
  /**
     * @brief 计算三点构成的圆的曲率 (Menger Curvature)
     * k = 4 * Area / (|AB| * |BC| * |AC|)
     */
  static double calculateMengerCurvature(
    const geometry_msgs::Point & a, const geometry_msgs::Point & b, const geometry_msgs::Point & c)
  {
    // 三角形面积 (叉积的一半)
    double area = 0.5 * std::abs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

    // 边长
    double ab = std::hypot(a.x - b.x, a.y - b.y);
    double bc = std::hypot(b.x - c.x, b.y - c.y);
    double ac = std::hypot(a.x - c.x, a.y - c.y);

    if (ab < 1e-4 || bc < 1e-4 || ac < 1e-4) return 0.0;  // 防止除零

    return (4.0 * area) / (ab * bc * ac);
  }
};
}

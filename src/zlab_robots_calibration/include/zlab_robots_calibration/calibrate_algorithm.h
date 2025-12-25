#pragma once

#include <vector>
#include <tf2/LinearMath/Transform.h>

/**
 * @brief
 * 稳定位姿采集与统计工具类，用于静态标定场景。
 *
 * 本类用于在相机和机械臂均保持静止的前提下，
 * 连续采集多帧位姿测量结果，并通过简单的统计方法
 * 获得一个稳定、鲁棒的代表位姿。
 *
 * 算法流程说明：
 *  1. 连续接收位姿输入（tf2::Transform）。
 *  2. 若当前帧与上一帧的位姿变化超过设定阈值，则丢弃该帧。
 *  3. 对满足稳定性条件的帧进行累计，直到达到指定帧数。
 *  4. 对平移分量（x, y, z）分别取中值（median）作为最终结果。
 *  5. 姿态分量使用中值帧对应的旋转作为代表姿态。
 *  本类不适用于运动过程中或实时位姿跟踪场景
 */
class CalibrateAlgorithm
{
public:
  CalibrateAlgorithm(int target_count);

  // 输入一帧数据
  // 返回 true 表示这一帧被接受
  bool addSample(const tf2::Transform& T1,
                 const tf2::Transform& T2,
                 const tf2::Transform& T3);

  // 是否已经收集完成
  bool isReady() const;

  // 获取 median 结果（调用前必须 isReady() == true）
  void getMedian(tf2::Transform& T1_out,
                 tf2::Transform& T2_out,
                 tf2::Transform& T3_out) const;

  // 清空所有状态
  void reset();

private:
  int target_count_;

  std::vector<tf2::Transform> buf1_;
  std::vector<tf2::Transform> buf2_;
  std::vector<tf2::Transform> buf3_;

  tf2::Transform last_T1_;
  tf2::Transform last_T2_;
  tf2::Transform last_T3_;
  bool has_last_;

  double trans_thresh_; // meters
  double rot_thresh_;   // radians

  bool isClose(const tf2::Transform& A,
               const tf2::Transform& B) const;
};

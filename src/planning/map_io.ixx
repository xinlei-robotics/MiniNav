module;

#include <string>

export module mininav.planning.map_io;

import mininav.planning.occupancy_grid;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // load_occupancy_grid: 从 ROS map_server 风格的 "image + yaml" 描述加载地图。
    //
    // map.yaml 字段(对齐 ROS map_server):
    //   image           同目录灰度图(本实现支持 PGM P2/P5,暗=占据 亮=free)
    //   resolution      米/cell
    //   origin          [x, y, yaw];取 x/y 作为栅格左下角的 world 坐标,yaw 忽略
    //   occupied_thresh occ 概率 > 此值 → kOccupied(可选,默认 0.65)
    //   free_thresh     occ 概率 < 此值 → kFree;之间 → kUnknown(可选,默认 0.25)
    //   negate          非 0 时反转明暗(可选,默认 0)
    //
    // 加载流程:解析 yaml → 读 PGM(自写解析,零依赖)→ 灰度按阈值三值化 →
    //   y 轴翻转(图像行序自上而下,world 原点在左下角)→ OccupancyGrid。
    //
    // 错误处理:yaml/图像打不开、PGM 头损坏、像素数与声明尺寸不符、缺少必需字段
    //   (image/resolution/origin)均抛 std::runtime_error,消息带文件路径。
    //
    // image 路径相对 map.yaml 所在目录解析(绝对路径原样使用)。
    // ---------------------------------------------------------------------------
    [[nodiscard]] OccupancyGrid load_occupancy_grid(const std::string& map_yaml_path);
}

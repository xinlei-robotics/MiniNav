module;

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

export module mininav.planning.occupancy_grid;

import mininav.planning.grid_types;

export namespace mininav::planning
{
    // ---------------------------------------------------------------------------
    // 占据值约定(对齐 ROS map_server):每 cell 一个 int8。
    //   kFree=0、kOccupied=100、kUnknown=-1。
    // PR2 从 PGM 灰度三值化时产出这三类值;PR1 由调用方在内存里直接给。
    // ---------------------------------------------------------------------------
    inline constexpr std::int8_t kFree = 0;
    inline constexpr std::int8_t kOccupied = 100;
    inline constexpr std::int8_t kUnknown = -1;

    // ---------------------------------------------------------------------------
    // OccupancyGrid: 二维占据栅格 + world↔grid 坐标变换。
    //
    // 坐标系约定:
    //   - world 系:连续坐标,单位米,与 V0–V2 的 Pose2D 同一坐标系。
    //   - grid 系:离散 cell 索引 (col, row)。
    //   - origin:栅格**左下角**(cell (0,0) 的角点,非中心)对应的 world 坐标,
    //     沿用 ROS map_server 约定;PR2 加载图像时即图像左下角。
    //
    // 存储:data 行优先(row-major),data[row * width + col];row(grid y)随
    //   world y 增大而增大(底行 row=0 = 最小 world y)。图像是上→下行序,
    //   PR2 的加载流程负责 y 轴翻转后再喂进这里,PR1 不涉及。
    //
    // 边界/未知策略(保守):
    //   - at() 对越界返回 kOccupied —— 让 A* 邻居生成不必到处写边界判断。
    //   - is_free() 仅对真正的 free cell 为真(排除 occupied / unknown / 越界);
    //     unknown 是否可走由规划器配置(allow_unknown)决定,不在本类放宽。
    // ---------------------------------------------------------------------------
    class OccupancyGrid
    {
    public:
        // 从内存构造。约束:width>0、height>0、resolution>0、
        //   data.size() == width*height,否则抛 std::invalid_argument。
        OccupancyGrid(int width, int height, double resolution,
                      Eigen::Vector2d origin, std::vector<std::int8_t> data);

        [[nodiscard]] int width() const noexcept { return width_; }
        [[nodiscard]] int height() const noexcept { return height_; }
        [[nodiscard]] double resolution() const noexcept { return resolution_; }
        [[nodiscard]] Eigen::Vector2d origin() const noexcept { return origin_; }

        [[nodiscard]] bool in_bounds(GridCoord c) const noexcept;
        [[nodiscard]] std::int8_t at(GridCoord c) const noexcept;   // 越界 → kOccupied
        [[nodiscard]] bool is_free(GridCoord c) const noexcept;

        [[nodiscard]] GridCoord world_to_grid(const Eigen::Vector2d& p) const noexcept;
        [[nodiscard]] Eigen::Vector2d grid_to_world(GridCoord c) const noexcept;

    private:
        // 调用前必须 in_bounds(c) 为真。
        [[nodiscard]] std::size_t index(GridCoord c) const noexcept;

        int width_{0};
        int height_{0};
        double resolution_{1.0};
        Eigen::Vector2d origin_{Eigen::Vector2d::Zero()};
        std::vector<std::int8_t> data_;
    };
}

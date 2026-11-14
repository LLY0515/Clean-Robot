#include "ego_nav2_controller/quiet.hpp"
#include "grid_map.h"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <iomanip>

// 构造函数：初始化地图（世界尺寸→栅格数量，初始化原始/膨胀网格）
GridMap2D::GridMap2D(double resolution, const Eigen::Vector2i& world_size)
    : resolution_(resolution), inflate_radius_(0.5),world_size_(world_size) {  // 默认膨胀半径0.5米
   
}

void GridMap2D::setCurPose(double x,double y)
{
    // 1. 校验输入合法性
    if (resolution_ <= 0) {
        throw std::invalid_argument("地图分辨率必须为正数（当前：" + std::to_string(resolution_) + "）");
    }
    if (world_size_.x() <= 0 || world_size_.y() <= 0) {
        throw std::invalid_argument("世界尺寸（x,y）必须为正数（当前：(" + 
            std::to_string(world_size_.x()) + "," + std::to_string(world_size_.y()) + ")）");
    }

    // 2. 计算栅格数量（世界尺寸 / 分辨率，向上取整确保覆盖完整世界范围）
    int grid_cols = static_cast<int>(std::ceil(static_cast<double>(world_size_.x()) / resolution_));  // x方向→列数
    int grid_rows = static_cast<int>(std::ceil(static_cast<double>(world_size_.y()) / resolution_));  // y方向→行数
    map_size_.x() = grid_cols;
    map_size_.y() = grid_rows;

    grid_cols_ = grid_cols;
    grid_rows_ = grid_rows;

    // 3. 设置地图原点（世界坐标）：让地图中心对齐世界坐标原点
    origin_.x() = x -static_cast<double>(world_size_.x()) / 2.0;  // 世界x范围：[origin.x(), origin.x()+world_size.x()]
    origin_.y() = y -static_cast<double>(world_size_.y()) / 2.0;  // 世界y范围：[origin.y(), origin.y()+world_size.y()]

    // 4. 初始化网格（原始障碍物网格 + 膨胀后网格）：默认全为自由空间（false）
    original_grid_.assign(grid_rows, std::vector<bool>(grid_cols, false));  // 存储原始障碍物
    grid_.assign(grid_rows, std::vector<bool>(grid_cols, false));          // 存储原始+膨胀障碍物

    // 打印初始化信息
    ego_quiet::stream() << "[GridMap2D] 地图初始化完成：" << std::endl;
    ego_quiet::stream() << "  - 世界尺寸：" << world_size_.x() << "m × " << world_size_.y() << "m" << std::endl;
    ego_quiet::stream() << "  - 栅格分辨率：" << resolution_ << "m/栅格" << std::endl;
    ego_quiet::stream() << "  - 栅格数量：" << grid_cols << "列 × " << grid_rows << "行" << std::endl;
    ego_quiet::stream() << "  - 世界坐标范围：x∈[" << std::fixed << std::setprecision(2) << origin_.x() << "," 
              << origin_.x() + world_size_.x() << "], y∈[" << origin_.y() << "," 
              << origin_.y() + world_size_.y() << "]" << std::endl;
    ego_quiet::stream() << "  - 默认膨胀半径：" << inflate_radius_ << "m" << std::endl;

}

void GridMap2D::resetGrids() {
    for (auto& row : original_grid_) {
        std::fill(row.begin(), row.end(), false);
    }
    for (auto& row : grid_) {
        std::fill(row.begin(), row.end(), false);
    }
    ego_quiet::stream() << "[resetGrids] 所有栅格已重置为自由空间。" << std::endl;
}


// 设置原始障碍物（同时更新原始网格和初始膨胀网格）
void GridMap2D::setObstacle(const Eigen::Vector2i& grid_index, bool is_obstacle) {
    // 1. 校验栅格索引合法性
    if (!isIndexValid(grid_index)) {
        ego_quiet::stream() << "[setObstacle] 警告：栅格索引（" << grid_index.x() << "," << grid_index.y() 
                  << "）超出范围（最大：(" << map_size_.x()-1 << "," << map_size_.y()-1 << ")），不执行设置" << std::endl;
        return;
    }

    // 2. 更新原始障碍物网格（永久存储原始障碍物）
    original_grid_[grid_index.y()][grid_index.x()] = is_obstacle;

    // 3. 更新膨胀后网格（初始状态 = 原始网格，膨胀后会叠加膨胀区域）
    grid_[grid_index.y()][grid_index.x()] = is_obstacle;

    ego_quiet::stream() << "[setObstacle] 栅格（" << grid_index.x() << "," << grid_index.y() 
              << "）设置为" << (is_obstacle ? "障碍物" : "自由空间") << std::endl;
}

// 原始接口：检查点是否为【原始障碍物】（不包含膨胀区域）
bool GridMap2D::isObstacle(const Eigen::Vector2d& world_pos) const {
    // 1. 世界坐标→栅格索引
    Eigen::Vector2i grid_index = worldToGrid(world_pos);

    // 2. 超出地图范围视为障碍物（避免越界）
    if (!isIndexValid(grid_index)) {
        // ego_quiet::stream() << "[isObstacle] 点（" << world_pos.x() << "," << world_pos.y() << "）超出地图范围，视为原始障碍物" << std::endl;
        return true;
    }

    // 3. 读取原始网格状态
    bool is_obs = original_grid_[grid_index.y()][grid_index.x()];
    // ego_quiet::stream() << "[isObstacle] 点（" << world_pos.x() << "," << world_pos.y() << "）→ 栅格（" 
    //           << grid_index.x() << "," << grid_index.y() << "），原始障碍物状态：" << is_obs << std::endl;
    return is_obs;
}

// 获取障碍物点云并转化为世界坐标
std::vector<Eigen::Vector2d> GridMap2D::getObstaclePointCloud(bool return_inflated_map) const {
    std::vector<Eigen::Vector2d> point_cloud;

    // 预估一下大小以减少内存重分配（可选优化，假设10%是障碍物）
    // point_cloud.reserve(map_size_.x() * map_size_.y() * 0.1);

    // 遍历整个地图
    for (int row = 0; row < map_size_.y(); ++row) {
        for (int col = 0; col < map_size_.x(); ++col) {
            
            // 根据参数决定检查 膨胀网格 还是 原始网格
            bool is_obs = return_inflated_map ? grid_[row][col] : original_grid_[row][col];

            if (is_obs) {
                // 将栅格索引转换为世界坐标
                // 逻辑与 gridToWorld 保持一致：原点 + 索引 * 分辨率
                double world_x = origin_.x() + col * resolution_;
                double world_y = origin_.y() + row * resolution_;

                point_cloud.emplace_back(world_x, world_y);
            }
        }
    }

    // ego_quiet::stream() << "[getObstaclePointCloud] 生成点云数量：" << point_cloud.size() 
    //           << " (模式：" << (return_inflated_map ? "膨胀后" : "原始") << ")" << std::endl;

    return point_cloud;
}

// 新增接口：检查点是否处于【膨胀后的障碍物区域】（原始障碍物+膨胀区）
bool GridMap2D::getInflateOccupancy(const Eigen::Vector2d& world_pos) const {
    // 1. 世界坐标→栅格索引
    Eigen::Vector2i grid_index = worldToGrid(world_pos);

    // 2. 超出地图范围视为占用（膨胀区延伸到地图边界外，避免路径超出地图）
    if (!isIndexValid(grid_index)) {
        ego_quiet::stream() << "[getInflateOccupancy] 点（" << world_pos.x() << "," << world_pos.y() << "）超出地图范围，视为膨胀占用" << std::endl;
        return true;
    }

    // 3. 读取膨胀后网格状态（原始+膨胀障碍物）
    bool is_occupied = grid_[grid_index.y()][grid_index.x()];
    // ego_quiet::stream() << "[getInflateOccupancy] 点（" << world_pos.x() << "," << world_pos.y() << "）→ 栅格（" 
    //           << grid_index.x() << "," << grid_index.y() << "），膨胀占用状态：" << is_occupied << std::endl;
    return is_occupied;
}

// 带参膨胀接口：按指定半径膨胀障碍物（核心膨胀逻辑）
// Squared Euclidean distance transform: linear in number of grid cells.
// This replaces per-obstacle disk stamping, which exceeded the planning budget
// with a 70 cm center clearance and the occupied outside-map boundary.
static std::vector<double> squaredDistance1D(const std::vector<double> &f) {
    const int n=static_cast<int>(f.size());
    std::vector<int> v(n);std::vector<double> z(n+1),d(n);
    int k=0;v[0]=0;z[0]=-1e30;z[1]=1e30;
    for(int q=1;q<n;++q) {
        double cross;
        do {
            const int p=v[k];
            cross=((f[q]+double(q)*q)-(f[p]+double(p)*p))/(2.0*(q-p));
            if(cross>z[k])break;
            --k;
        } while(k>=0);
        ++k;v[k]=q;z[k]=cross;z[k+1]=1e30;
    }
    k=0;
    for(int q=0;q<n;++q) {
        while(z[k+1]<q)++k;
        d[q]=double(q-v[k])*(q-v[k])+f[v[k]];
    }
    return d;
}
void GridMap2D::inflateObstacles(double radius) {
    if(radius<0 || !std::isfinite(radius))throw std::invalid_argument("Invalid inflation radius");
    if(grid_rows_<=0||grid_cols_<=0)return;
    std::vector<std::vector<double>> horizontal(grid_rows_);
    for(int row=0;row<grid_rows_;++row) {
        std::vector<double> f(grid_cols_,1e12);
        for(int col=0;col<grid_cols_;++col)if(original_grid_[row][col])f[col]=0;
        horizontal[row]=squaredDistance1D(f);
    }
    const double r=std::ceil(radius/resolution_);
    for(int col=0;col<grid_cols_;++col) {
        std::vector<double> f(grid_rows_);
        for(int row=0;row<grid_rows_;++row)f[row]=horizontal[row][col];
        auto d=squaredDistance1D(f);
        for(int row=0;row<grid_rows_;++row)grid_[row][col]=(d[row]<=r*r);
    }
}

// // 新增接口：设置默认膨胀半径（支持动态调整）
void GridMap2D::setInflateRadius(double radius) {
    if (radius <= 0) {
        ego_quiet::stream() << "[setInflateRadius] 警告：膨胀半径必须为正数（当前输入：" << radius << "），不修改默认半径" << std::endl;
        return;
    }
    inflate_radius_ = radius;
    ego_quiet::stream() << "[setInflateRadius] 成功更新默认膨胀半径：" << radius << "m（原：" << inflate_radius_ << "m）" << std::endl;
}

// 新增接口：使用默认膨胀半径膨胀障碍物（无参调用）
void GridMap2D::inflate() {
    ego_quiet::stream() << "\n[inflate] 开始使用默认半径膨胀障碍物" << std::endl;
    inflateObstacles(inflate_radius_);  // 复用带参膨胀逻辑
}

// 世界坐标 → 栅格索引（四舍五入到最近栅格）
Eigen::Vector2i GridMap2D::worldToGrid(const Eigen::Vector2d& world_pos) const {
    Eigen::Vector2i grid_index;
    // 计算逻辑：(世界坐标 - 原点坐标) / 分辨率 → 栅格偏移，四舍五入到整数
    grid_index.x() = static_cast<int>(std::round((world_pos.x() - origin_.x()) / resolution_));
    grid_index.y() = static_cast<int>(std::round((world_pos.y() - origin_.y()) / resolution_));
    // ego_quiet::stream() << "[worldToGrid] 世界坐标（" << world_pos.x() << "," << world_pos.y() << "）→ 栅格索引（" 
    //           << grid_index.x() << "," << grid_index.y() << "）" << std::endl;
    return grid_index;
}

// 栅格索引 → 世界坐标（返回栅格中心的世界坐标）
Eigen::Vector2d GridMap2D::gridToWorld(const Eigen::Vector2i& grid_index) const {
    // 校验索引合法性
    if (!isIndexValid(grid_index)) {
        ego_quiet::stream() << "[gridToWorld] 警告：栅格索引（" << grid_index.x() << "," << grid_index.y() 
                  << "）超出范围，返回原点坐标" << std::endl;
        return origin_;
    }

    // 计算逻辑：原点坐标 + 栅格索引 × 分辨率
    Eigen::Vector2d world_pos;
    world_pos.x() = origin_.x() + grid_index.x() * resolution_;
    world_pos.y() = origin_.y() + grid_index.y() * resolution_;
    // ego_quiet::stream() << "[gridToWorld] 栅格索引（" << grid_index.x() << "," << grid_index.y() << "）→ 世界坐标（" 
    //           << world_pos.x() << "," << world_pos.y() << "）" << std::endl;
    return world_pos;
}

// 校验栅格索引是否在地图范围内
bool GridMap2D::isIndexValid(const Eigen::Vector2i& grid_index) const {
    // 列索引（x）：[0, 列数-1]；行索引（y）：[0, 行数-1]
    bool valid = (grid_index.x() >= 0 && grid_index.x() < map_size_.x()) &&
                 (grid_index.y() >= 0 && grid_index.y() < map_size_.y());
    if (!valid) {
        ego_quiet::stream() << "[isIndexValid] 栅格索引（" << grid_index.x() << "," << grid_index.y() 
                  << "）无效！地图范围：列[0," << map_size_.x()-1 << "], 行[0," << map_size_.y()-1 << "]" << std::endl;
    }
    return valid;
}

// 控制台打印地图（可视化原始/膨胀障碍物）
// void GridMap2D::print(bool print_inflated /*= true*/) const {
//     ego_quiet::stream() << "\n======================= 地图可视化 =======================" << std::endl;
//     ego_quiet::stream() << "  地图类型：" << (print_inflated ? "膨胀后障碍物（#=原始/膨胀，.=自由）" : "原始障碍物（#=原始，.=自由/膨胀）") << std::endl;
//     ego_quiet::stream() << "  栅格尺寸：" << map_size_.x() << "列 × " << map_size_.y() << "行" << std::endl;
//     ego_quiet::stream() << "==========================================================" << std::endl;

//     // 从下到上打印（符合视觉习惯：y轴向上为正）
//     for (int row = map_size_.y() - 1; row >= 0; --row) {
//         ego_quiet::stream() << "| ";  // 左边界
//         for (int col = 0; col < map_size_.x(); ++col) {
//             // 选择要打印的网格（原始/膨胀后）
//             bool is_occupied = print_inflated ? grid_[row][col] : original_grid_[row][col];
//             ego_quiet::stream() << (is_occupied ? "# " : ". ");  // 障碍物→#，自由→.
//         }
//         ego_quiet::stream() << "|" << std::endl;  // 右边界
//     }

//     ego_quiet::stream() << "==========================================================" << std::endl;
// }

// Getter：获取分辨率
double GridMap2D::resolution() const {
    return resolution_;
}

// Getter：获取栅格尺寸（列数x行数）
const Eigen::Vector2i& GridMap2D::mapSize() const {
    return map_size_;
}

// Getter：获取世界坐标原点
const Eigen::Vector2d& GridMap2D::origin() const {
    return origin_;
}

double GridMap2D::getResolution()
{
    return resolution_;
}

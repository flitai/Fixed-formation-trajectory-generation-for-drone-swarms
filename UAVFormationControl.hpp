// UAVFormationControl.hpp
// 固定翼无人机编队参考轨迹生成与控制算法
// 基于路径同步 + 曲率约束 + 自适应阵形的统一模型

#ifndef UAV_FORMATION_CONTROL_HPP
#define UAV_FORMATION_CONTROL_HPP

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

namespace UAVFormation {

// ==================== 基础数据结构 ====================

// 二维点
struct Point2D {
    double x, y;
    
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    
    Point2D operator+(const Point2D& p) const { return Point2D(x + p.x, y + p.y); }
    Point2D operator-(const Point2D& p) const { return Point2D(x - p.x, y - p.y); }
    Point2D operator*(double s) const { return Point2D(x * s, y * s); }
    double dot(const Point2D& p) const { return x * p.x + y * p.y; }
    double norm() const { return std::sqrt(x * x + y * y); }
    Point2D normalized() const { 
        double n = norm();
        return n > 1e-10 ? Point2D(x/n, y/n) : Point2D(0, 0);
    }
};

// 中心线采样点
struct CenterlineSample {
    Point2D position;      // 位置 γ(s)
    double psi;           // 切向角 ψ(s)
    double kappa;         // 曲率 κ(s)
    double s;             // 弧长坐标
};

// 中心线类
class Centerline {
private:
    std::vector<CenterlineSample> samples;
    double total_length;
    
public:
    Centerline() : total_length(0) {}
    
    // 从离散点构建中心线（需要样条拟合）
    void buildFromPoints(const std::vector<Point2D>& points, int resample_count = 100);
    
    // 按弧长插值查询
    CenterlineSample interpolate(double s) const;
    
    // 获取切向量和法向量
    Point2D getTangent(double s) const;
    Point2D getNormal(double s) const;  // 左法向
    
    double getLength() const { return total_length; }
    const std::vector<CenterlineSample>& getSamples() const { return samples; }
};

// 槽位定义
struct Slot {
    double b_lat;      // 横向侧距 (右正左负) [m]
    double ds_long;    // 纵向弧长错位 [m]
    
    Slot(double b = 0, double ds = 0) : b_lat(b), ds_long(ds) {}
};

// 飞行器参数
struct AircraftParams {
    double phi_max_rad;    // 最大滚转角 [rad]
    double v_nom;          // 名义速度 [m/s]
    double L0;             // LOS基础前视距离 [m]
    double kv;             // LOS速度相关系数
    double tau_n;          // 侧距平滑时间常数 [s]
    
    AircraftParams() : phi_max_rad(0.5236), v_nom(20), L0(10), kv(0.5, tau_n(2.0) {}
};

// 参考输出
struct ReferenceOutput {
    Point2D position;      // 参考位置 P*
    double psi_path;       // 参考航向角
    double kappa_ref;      // 参考曲率
    double v_cmd;          // 速度指令
    double s;              // 当前弧长坐标
    double n;              // 当前侧距
    double phi_ff;         // 前馈滚转角
};

// 无人机状态缓存
struct UAVStateCache {
    double s;              // 弧长坐标
    double n;              // 侧距
    double v;              // 速度
    double last_alpha;     // 上一时刻的收拢系数
    
    UAVStateCache() : s(0), n(0), v(20), last_alpha(1.0) {}
};

// ==================== 阵型模板生成器 ====================

class FormationTemplate {
public:
    enum FormationType {
        COLUMN,        // 纵队
        LINE_ABREAST,  // 线列
        WEDGE          // 楔形
    };
    
    // 生成阵型槽位
    static std::vector<Slot> generateFormation(
        FormationType type,
        int num_uavs,
        double d_s,    // 纵向间隔
        double b       // 横向间隔
    );
};

// ==================== 主控制器类 ====================

class FormationController {
private:
    Centerline centerline;
    std::vector<Slot> slots;
    std::map<std::string, int> slotOf;  // UAV_ID -> slot_index
    std::map<std::string, UAVStateCache> state_cache;
    std::map<std::string, AircraftParams> aircraft_params;
    
    const double g = 9.81;  // 重力加速度
    const double epsilon = 0.05;  // 退化防护阈值
    
    // 计算自适应收拢系数
    double computeAlpha(double kappa, double b, double v, const AircraftParams& params) const;
    
    // 计算最大可飞曲率
    double computeMaxCurvature(double v, double phi_max) const;
    
    // 速度规划
    double computeSpeedLimit(double kappa_ref, double phi_max) const;
    
public:
    FormationController() {}
    
    // 初始化中心线
    void initCenterline(const std::vector<Point2D>& leader_path);
    
    // 设置阵型
    void setFormation(const std::vector<Slot>& formation_slots);
    void setFormation(FormationTemplate::FormationType type, int num_uavs, 
                      double d_s, double b);
    
    // 分配UAV到槽位
    void assignSlot(const std::string& uav_id, int slot_index);
    
    // 设置飞行器参数
    void setAircraftParams(const std::string& uav_id, const AircraftParams& params);
    
    // 获取参考轨迹（主接口）
    ReferenceOutput computeReference(
        const std::string& uav_id,
        double leader_s,        // 领航当前弧长
        double dt               // 时间步长
    );
    
    // 引导控制接口
    double computeHeadingCommand(
        const Point2D& current_pos,
        const ReferenceOutput& ref,
        double current_v
    ) const;
    
    double computeRollCommand(
        double psi_cmd,
        double psi_current,
        double v,
        double kappa_ref,
        double phi_max
    ) const;
    
    // 计算横向误差
    double computeCrossTrackError(
        const Point2D& current_pos,
        const ReferenceOutput& ref
    ) const;
    
    // 获取诊断信息
    std::map<std::string, double> getDiagnostics(const std::string& uav_id) const;
};

// ==================== 实现部分 ====================

// Centerline实现
void Centerline::buildFromPoints(const std::vector<Point2D>& points, int resample_count) {
    if (points.size() < 2) return;
    
    samples.clear();
    
    // 简化实现：使用线性插值（实际应用中应使用样条）
    // 计算累积弧长
    std::vector<double> cumulative_s;
    cumulative_s.push_back(0);
    
    for (size_t i = 1; i < points.size(); ++i) {
        double dist = (points[i] - points[i-1]).norm();
        cumulative_s.push_back(cumulative_s.back() + dist);
    }
    
    total_length = cumulative_s.back();
    
    // 等弧长重采样
    for (int i = 0; i <= resample_count; ++i) {
        double target_s = (i * total_length) / resample_count;
        
        // 找到对应段
        size_t seg = 0;
        for (size_t j = 1; j < cumulative_s.size(); ++j) {
            if (target_s <= cumulative_s[j]) {
                seg = j - 1;
                break;
            }
        }
        
        // 线性插值
        double t = (target_s - cumulative_s[seg]) / 
                   (cumulative_s[seg+1] - cumulative_s[seg]);
        
        CenterlineSample sample;
        sample.s = target_s;
        sample.position = points[seg] + (points[seg+1] - points[seg]) * t;
        
        // 计算切向角
        Point2D tangent = (points[seg+1] - points[seg]).normalized();
        sample.psi = std::atan2(tangent.y, tangent.x);
        
        // 简化曲率计算（实际应使用更精确的方法）
        if (i > 0 && i < resample_count) {
            double ds = total_length / resample_count;
            double psi_prev = samples.back().psi;
            sample.kappa = (sample.psi - psi_prev) / ds;
        } else {
            sample.kappa = 0;
        }
        
        samples.push_back(sample);
    }
}

CenterlineSample Centerline::interpolate(double s) const {
    if (samples.empty()) return CenterlineSample();
    
    // 边界处理
    if (s <= 0) return samples.front();
    if (s >= total_length) return samples.back();
    
    // 二分查找
    size_t left = 0, right = samples.size() - 1;
    while (right - left > 1) {
        size_t mid = (left + right) / 2;
        if (samples[mid].s <= s) left = mid;
        else right = mid;
    }
    
    // 线性插值
    double t = (s - samples[left].s) / (samples[right].s - samples[left].s);
    
    CenterlineSample result;
    result.s = s;
    result.position = samples[left].position + 
                     (samples[right].position - samples[left].position) * t;
    result.psi = samples[left].psi + t * (samples[right].psi - samples[left].psi);
    result.kappa = samples[left].kappa + t * (samples[right].kappa - samples[left].kappa);
    
    return result;
}

Point2D Centerline::getTangent(double s) const {
    CenterlineSample sample = interpolate(s);
    return Point2D(std::cos(sample.psi), std::sin(sample.psi));
}

Point2D Centerline::getNormal(double s) const {
    CenterlineSample sample = interpolate(s);
    return Point2D(-std::sin(sample.psi), std::cos(sample.psi));
}

// FormationTemplate实现
std::vector<Slot> FormationTemplate::generateFormation(
    FormationType type, int num_uavs, double d_s, double b) {
    
    std::vector<Slot> slots;
    
    switch (type) {
        case COLUMN:  // 纵队
            for (int i = 0; i < num_uavs; ++i) {
                slots.emplace_back(0, i * d_s);
            }
            break;
            
        case LINE_ABREAST:  // 线列
            for (int i = 0; i < num_uavs; ++i) {
                int offset = i - num_uavs / 2;
                slots.emplace_back(offset * b, 0);
            }
            break;
            
        case WEDGE:  // 楔形
            slots.emplace_back(0, 0);  // 领航者
            for (int i = 1; i < num_uavs; i += 2) {
                int level = (i + 1) / 2;
                slots.emplace_back(level * b, level * d_s);   // 右侧
                if (i + 1 < num_uavs) {
                    slots.emplace_back(-level * b, level * d_s);  // 左侧
                }
            }
            break;
    }
    
    return slots;
}

// FormationController实现
void FormationController::initCenterline(const std::vector<Point2D>& leader_path) {
    centerline.buildFromPoints(leader_path);
}

void FormationController::setFormation(const std::vector<Slot>& formation_slots) {
    slots = formation_slots;
}

void FormationController::setFormation(FormationTemplate::FormationType type, 
                                       int num_uavs, double d_s, double b) {
    slots = FormationTemplate::generateFormation(type, num_uavs, d_s, b);
}

void FormationController::assignSlot(const std::string& uav_id, int slot_index) {
    if (slot_index >= 0 && slot_index < slots.size()) {
        slotOf[uav_id] = slot_index;
        if (state_cache.find(uav_id) == state_cache.end()) {
            state_cache[uav_id] = UAVStateCache();
        }
    }
}

void FormationController::setAircraftParams(const std::string& uav_id, 
                                           const AircraftParams& params) {
    aircraft_params[uav_id] = params;
}

double FormationController::computeAlpha(double kappa, double b, double v, 
                                        const AircraftParams& params) const {
    // 外侧或直线
    if (kappa * b <= 0) return 1.0;
    
    // 内侧转弯 - 计算收拢系数
    double kappa_max = computeMaxCurvature(v, params.phi_max_rad);
    double alpha = (1.0 - std::abs(kappa) / kappa_max) / (kappa * b);
    
    return std::max(0.0, std::min(1.0, alpha));
}

double FormationController::computeMaxCurvature(double v, double phi_max) const {
    return g * std::tan(phi_max) / (v * v);
}

double FormationController::computeSpeedLimit(double kappa_ref, double phi_max) const {
    if (std::abs(kappa_ref) < 1e-6) return 1000.0;  // 无限制
    return std::sqrt(g * std::tan(phi_max) / std::abs(kappa_ref));
}

ReferenceOutput FormationController::computeReference(
    const std::string& uav_id,
    double leader_s,
    double dt) {
    
    ReferenceOutput output;
    
    // 获取槽位
    if (slotOf.find(uav_id) == slotOf.end()) {
        return output;  // UAV未分配槽位
    }
    
    int slot_idx = slotOf[uav_id];
    const Slot& slot = slots[slot_idx];
    
    // 获取飞行器参数
    AircraftParams params;
    if (aircraft_params.find(uav_id) != aircraft_params.end()) {
        params = aircraft_params[uav_id];
    }
    
    // 获取状态缓存
    UAVStateCache& cache = state_cache[uav_id];
    
    // 纵向同步：弧长错位
    cache.s = leader_s - slot.ds_long;
    
    // 限制在有效范围内
    cache.s = std::max(0.0, std::min(centerline.getLength(), cache.s));
    
    // 获取中心线信息
    CenterlineSample cl_sample = centerline.interpolate(cache.s);
    
    // 计算自适应收拢系数
    double alpha = computeAlpha(cl_sample.kappa, slot.b_lat, cache.v, params);
    
    // 侧距平滑
    double target_n = alpha * slot.b_lat;
    cache.n += (target_n - cache.n) * dt / params.tau_n;
    
    // 退化防护
    if (std::abs(cl_sample.kappa) > 1e-6) {
        double denominator = 1.0 - cl_sample.kappa * cache.n;
        if (denominator < epsilon) {
            cache.n = (1.0 - epsilon) / cl_sample.kappa;
        }
    }
    
    // 计算参考位置
    Point2D normal = centerline.getNormal(cache.s);
    output.position = cl_sample.position + normal * cache.n;
    
    // 计算参考曲率
    double denominator = 1.0 - cl_sample.kappa * cache.n;
    output.kappa_ref = cl_sample.kappa / denominator;
    
    // 计算参考航向
    output.psi_path = cl_sample.psi;
    
    // 速度规划
    double v_limit = computeSpeedLimit(output.kappa_ref, params.phi_max_rad);
    output.v_cmd = std::min(params.v_nom, v_limit);
    cache.v = output.v_cmd;
    
    // 前馈滚转角
    output.phi_ff = std::atan(cache.v * cache.v * output.kappa_ref / g);
    output.phi_ff = std::max(-params.phi_max_rad, 
                             std::min(params.phi_max_rad, output.phi_ff));
    
    // 诊断信息
    output.s = cache.s;
    output.n = cache.n;
    
    return output;
}

double FormationController::computeHeadingCommand(
    const Point2D& current_pos,
    const ReferenceOutput& ref,
    double current_v) const {
    
    // 默认LOS参数
    AircraftParams default_params;
    
    // 计算横向误差
    double e_y = computeCrossTrackError(current_pos, ref);
    
    // LOS前视距离
    double L = default_params.L0 + default_params.kv * current_v;
    
    // LOS航向指令
    double psi_cmd = ref.psi_path + std::atan(e_y / L);
    
    return psi_cmd;
}

double FormationController::computeRollCommand(
    double psi_cmd,
    double psi_current,
    double v,
    double kappa_ref,
    double phi_max) const {
    
    // 前馈项
    double phi_ff = std::atan(v * v * kappa_ref / g);
    
    // 反馈项（简化的P控制）
    double psi_error = psi_cmd - psi_current;
    // 角度归一化到[-π, π]
    while (psi_error > M_PI) psi_error -= 2 * M_PI;
    while (psi_error < -M_PI) psi_error += 2 * M_PI;
    
    double Kp = 1.0;  // 比例增益
    double phi_fb = Kp * psi_error;
    
    // 合成滚转指令
    double phi_cmd = phi_ff + phi_fb;
    
    // 饱和限制
    phi_cmd = std::max(-phi_max, std::min(phi_max, phi_cmd));
    
    return phi_cmd;
}

double FormationController::computeCrossTrackError(
    const Point2D& current_pos,
    const ReferenceOutput& ref) const {
    
    // 获取路径法向量
    Point2D normal = centerline.getNormal(ref.s);
    
    // 计算横向误差
    Point2D error = current_pos - ref.position;
    double e_y = error.dot(normal);
    
    return e_y;
}

std::map<std::string, double> FormationController::getDiagnostics(
    const std::string& uav_id) const {
    
    std::map<std::string, double> diag;
    
    if (state_cache.find(uav_id) != state_cache.end()) {
        const UAVStateCache& cache = state_cache.at(uav_id);
        diag["arc_length_s"] = cache.s;
        diag["lateral_offset_n"] = cache.n;
        diag["velocity"] = cache.v;
        diag["alpha"] = cache.last_alpha;
    }
    
    return diag;
}

} // namespace UAVFormation

#endif // UAV_FORMATION_CONTROL_HPP
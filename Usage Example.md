# 示例程序 UAVFormationExample.cpp 

## 主要功能模块

### 1. **FormationSimulator** - 完整仿真环境
- 两个预设场景（圆形路径、8字形路径）
- 简化的无人机动力学模型
- CSV日志记录功能
- 实时状态输出

### 2. **FormationTransition** - 阵型变换演示
- 展示从纵队→线列→楔形的动态切换
- 平滑过渡控制

### 3. **PerformanceTest** - 性能基准测试
- 测试大规模路径（1000点）
- 测试大规模编队（20架UAV）
- 内存使用估算

### 4. **SafetyValidator** - 安全性验证
- 急转弯场景测试
- 动力学约束检查
- 最小间距验证
- 自适应速度和侧距调整

### 5. **DiagnosticTool** - 诊断工具（新增）
- S形曲线路径测试
- 关键点状态输出
- 详细的诊断信息

## 编译和运行

```bash
# 编译
g++ -std=c++11 -O2 UAVFormationExample.cpp -o formation_sim

# 运行不同场景
./formation_sim 1  # 圆形路径楔形编队
./formation_sim 2  # 8字形路径纵队编队
./formation_sim 3  # 阵型变换演示
./formation_sim 4  # 性能测试
./formation_sim 5  # 安全性验证
./formation_sim 6  # 诊断工具
```

## 输出文件

- **formation_log.csv** - 包含时间序列数据，可用于后处理和可视化
- 控制台输出 - 实时状态和诊断信息

## 集成建议

1. **与现有仿真系统集成**
```cpp
// 示例：集成到您的仿真框架
class YourSimulator {
    UAVFormation::FormationController formation_controller;
    
    void initFormation() {
        // 从您的路径规划器获取路径
        std::vector<Point2D> path = your_path_planner.getPath();
        formation_controller.initCenterline(path);
        
        // 设置编队
        formation_controller.setFormation(
            FormationTemplate::WEDGE, 
            num_uavs, 
            longitudinal_spacing, 
            lateral_spacing
        );
    }
    
    void updateControl() {
        for (auto& uav : your_uav_list) {
            // 获取参考
            auto ref = formation_controller.computeReference(
                uav.id, 
                leader_progress, 
                dt
            );
            
            // 发送到飞控
            uav.sendCommand(ref.position, ref.v_cmd, ref.phi_ff);
        }
    }
};
```

2. **ROS集成示例**
```cpp
// ROS节点封装
class FormationControlNode {
    ros::NodeHandle nh;
    FormationController controller;
    ros::Publisher ref_pub;
    ros::Subscriber path_sub;
    
    void pathCallback(const nav_msgs::Path& msg) {
        std::vector<Point2D> path;
        for (const auto& pose : msg.poses) {
            path.emplace_back(
                pose.pose.position.x,
                pose.pose.position.y
            );
        }
        controller.initCenterline(path);
    }
    
    void publishReferences() {
        // 发布参考轨迹到各UAV
    }
};
```

3. **实时监控接口**
```cpp
// 添加实时监控功能
class FormationMonitor {
    FormationController* controller;
    
    struct FormationMetrics {
        double max_position_error;
        double min_separation;
        double max_roll_angle;
        bool safety_violation;
    };
    
    FormationMetrics computeMetrics() {
        FormationMetrics metrics;
        // 计算编队性能指标
        return metrics;
    }
    
    void exportTelemetry(const std::string& filename) {
        // 导出遥测数据
    }
};
```

## 扩展功能建议

### 1. **高级路径处理**
```cpp
// 添加到 Centerline 类
class Centerline {
    // ... 现有代码 ...
    
    // 添加样条插值
    void buildFromPointsWithSpline(const std::vector<Point2D>& points) {
        // 使用 Catmull-Rom 或 B-样条
    }
    
    // 添加圆角处理
    void addFilletAtCorners(double min_radius) {
        // 在尖角处添加圆弧过渡
    }
    
    // 添加 Clothoid 过渡
    void addClothoidTransitions() {
        // 使用回旋线过渡
    }
};
```

### 2. **通信延迟和故障处理**
```cpp
class RobustFormationController : public FormationController {
    std::map<std::string, double> last_update_time;
    std::map<std::string, ReferenceOutput> predicted_refs;
    
    ReferenceOutput computeReferenceWithDelay(
        const std::string& uav_id,
        double leader_s,
        double dt,
        double comm_delay
    ) {
        // 考虑通信延迟的预测补偿
        double predicted_s = leader_s + comm_delay * nominal_speed;
        return computeReference(uav_id, predicted_s, dt);
    }
    
    void handleUAVFailure(const std::string& failed_uav_id) {
        // 重新分配槽位，填补空缺
        redistributeSlots(failed_uav_id);
    }
};
```

### 3. **3D扩展**
```cpp
struct Point3D {
    double x, y, z;
};

class Formation3DController {
    // 扩展到3D空间
    struct Centerline3D {
        std::vector<Point3D> positions;
        std::vector<double> pitch_angles;  // 俯仰角
        std::vector<double> climb_rates;   // 爬升率
    };
    
    struct Reference3D : ReferenceOutput {
        double altitude;
        double pitch_ref;
        double climb_rate;
    };
};
```

### 4. **优化算法**
```cpp
class OptimalSlotAssignment {
    // 使用匈牙利算法优化槽位分配
    std::map<std::string, int> computeOptimalAssignment(
        const std::vector<Point2D>& current_positions,
        const std::vector<Slot>& target_slots
    ) {
        // 最小化总移动距离
        return optimized_assignment;
    }
};
```

## 测试和验证

### 单元测试示例
```cpp
#include <cassert>

void testCenterline() {
    Centerline cl;
    std::vector<Point2D> path = {
        {0, 0}, {10, 0}, {20, 0}
    };
    cl.buildFromPoints(path);
    
    auto sample = cl.interpolate(5.0);
    assert(std::abs(sample.position.x - 5.0) < 0.01);
    assert(std::abs(sample.position.y - 0.0) < 0.01);
    
    std::cout << "Centerline test passed!\n";
}

void testFormationGeneration() {
    auto slots = FormationTemplate::generateFormation(
        FormationTemplate::WEDGE, 5, 10.0, 5.0
    );
    assert(slots.size() == 5);
    assert(slots[0].b_lat == 0);  // 领航者在中心
    
    std::cout << "Formation generation test passed!\n";
}

void runAllTests() {
    testCenterline();
    testFormationGeneration();
    // 添加更多测试...
}
```

## 使用建议总结

1. **初始集成**：先在仿真环境中测试，验证算法正确性
2. **参数调优**：根据具体UAV性能调整 `phi_max`, `tau_n`, `L0`, `kv` 等参数
3. **路径规划**：确保输入路径满足最小转弯半径要求
4. **安全边界**：设置合理的最小间距和动力学限制
5. **渐进式部署**：先测试简单阵型，再尝试复杂场景和阵型变换

这个完整的实现可以根据具体需求进行定制和扩展。

// UAVFormationExample.cpp
// 无人机编队控制算法使用示例

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include "UAVFormationControl.hpp"

using namespace UAVFormation;

// ==================== 仿真接口类 ====================

class FormationSimulator {
private:
    FormationController controller;
    
    // 无人机状态
    struct UAVState {
        std::string id;
        Point2D position;
        double psi;      // 航向角
        double v;        // 速度
        double phi;      // 滚转角
    };
    
    std::vector<UAVState> uav_states;
    double leader_s;  // 领航弧长
    double sim_time;
    double dt;
    
    // 日志文件
    std::ofstream log_file;
    
public:
    FormationSimulator(double time_step = 0.1) 
        : leader_s(0), sim_time(0), dt(time_step) {
        log_file.open("formation_log.csv");
        log_file << "time,uav_id,x,y,psi,v,phi,ref_x,ref_y,error\n";
    }
    
    ~FormationSimulator() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }
    
    // 初始化场景1：圆形路径
    void setupScenario1_CircularPath() {
        // 创建圆形路径
        std::vector<Point2D> path;
        double radius = 100.0;
        int num_points = 100;
        
        for (int i = 0; i <= num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            path.emplace_back(
                radius * std::cos(angle),
                radius * std::sin(angle)
            );
        }
        
        controller.initCenterline(path);
        
        // 设置5架无人机的楔形编队
        controller.setFormation(FormationTemplate::WEDGE, 5, 20.0, 15.0);
        
        // 初始化无人机
        for (int i = 0; i < 5; ++i) {
            UAVState uav;
            uav.id = "UAV_" + std::to_string(i);
            uav.position = Point2D(radius + i * 5, 0);
            uav.psi = 0;
            uav.v = 20.0;
            uav.phi = 0;
            
            uav_states.push_back(uav);
            
            // 分配槽位
            controller.assignSlot(uav.id, i);
            
            // 设置飞行器参数
            AircraftParams params;
            params.phi_max_rad = 0.5236;  // 30度
            params.v_nom = 20.0;
            params.L0 = 10.0;
            params.kv = 0.5;
            params.tau_n = 2.0;
            
            controller.setAircraftParams(uav.id, params);
        }
        
        std::cout << "Scenario 1 initialized: Circular path with wedge formation\n";
    }
    
    // 初始化场景2：8字形路径
    void setupScenario2_FigureEight() {
        // 创建8字形路径
        std::vector<Point2D> path;
        double a = 100.0;  // 8字形尺寸
        int num_points = 200;
        
        for (int i = 0; i <= num_points; ++i) {
            double t = 4 * M_PI * i / num_points;
            double x = a * std::sin(t);
            double y = a * std::sin(t) * std::cos(t);
            path.emplace_back(x, y);
        }
        
        controller.initCenterline(path);
        
        // 设置纵队编队
        controller.setFormation(FormationTemplate::COLUMN, 4, 25.0, 0);
        
        // 初始化4架无人机
        for (int i = 0; i < 4; ++i) {
            UAVState uav;
            uav.id = "UAV_" + std::to_string(i);
            uav.position = Point2D(0, -i * 25);
            uav.psi = M_PI / 2;
            uav.v = 15.0;
            uav.phi = 0;
            
            uav_states.push_back(uav);
            controller.assignSlot(uav.id, i);
            
            AircraftParams params;
            params.phi_max_rad = 0.4363;  // 25度
            params.v_nom = 15.0;
            params.L0 = 8.0;
            params.kv = 0.6;
            params.tau_n = 1.5;
            
            controller.setAircraftParams(uav.id, params);
        }
        
        std::cout << "Scenario 2 initialized: Figure-8 path with column formation\n";
    }
    
    // 仿真步进
    void step() {
        // 更新领航弧长（简单的匀速前进）
        double leader_speed = 15.0;
        leader_s += leader_speed * dt;
        
        // 对每架无人机进行控制
        for (auto& uav : uav_states) {
            // 获取参考轨迹
            ReferenceOutput ref = controller.computeReference(
                uav.id, leader_s, dt
            );
            
            // 计算控制指令
            double psi_cmd = controller.computeHeadingCommand(
                uav.position, ref, uav.v
            );
            
            double phi_cmd = controller.computeRollCommand(
                psi_cmd, uav.psi, uav.v, ref.kappa_ref, 0.5236
            );
            
            // 简化的无人机动力学模型
            updateUAVDynamics(uav, phi_cmd, ref.v_cmd);
            
            // 计算跟踪误差
            double error = (uav.position - ref.position).norm();
            
            // 记录日志
            logState(uav, ref, error);
            
            // 输出状态（每10步输出一次）
            if (static_cast<int>(sim_time / dt) % 10 == 0) {
                printStatus(uav, ref, error);
            }
        }
        
        sim_time += dt;
    }
    
    // 运行仿真
    void run(double duration) {
        std::cout << "\n===== Starting Formation Simulation =====\n";
        std::cout << "Duration: " << duration << " seconds\n";
        std::cout << "Time step: " << dt << " seconds\n\n";
        
        while (sim_time < duration) {
            step();
        }
        
        std::cout << "\n===== Simulation Complete =====\n";
        std::cout << "Log saved to: formation_log.csv\n";
    }
    
private:
    // 简化的无人机动力学更新
    void updateUAVDynamics(UAVState& uav, double phi_cmd, double v_cmd) {
        const double g = 9.81;
        
        // 滚转角响应（一阶滤波）
        double tau_phi = 0.3;  // 滚转时间常数
        uav.phi += (phi_cmd - uav.phi) * dt / tau_phi;
        
        // 转弯率
        double omega = g * std::tan(uav.phi) / uav.v;
        
        // 更新航向
        uav.psi += omega * dt;
        
        // 速度响应
        double tau_v = 1.0;  // 速度时间常数
        uav.v += (v_cmd - uav.v) * dt / tau_v;
        
        // 更新位置
        uav.position.x += uav.v * std::cos(uav.psi) * dt;
        uav.position.y += uav.v * std::sin(uav.psi) * dt;
    }
    
    // 记录状态到日志
    void logState(const UAVState& uav, const ReferenceOutput& ref, double error) {
        log_file << std::fixed << std::setprecision(3)
                 << sim_time << ","
                 << uav.id << ","
                 << uav.position.x << ","
                 << uav.position.y << ","
                 << uav.psi << ","
                 << uav.v << ","
                 << uav.phi << ","
                 << ref.position.x << ","
                 << ref.position.y << ","
                 << error << "\n";
    }
    
    // 打印状态信息
    void printStatus(const UAVState& uav, const ReferenceOutput& ref, double error) {
        std::cout << std::fixed << std::setprecision(2)
                  << "T=" << sim_time << "s | "
                  << uav.id << " | "
                  << "Pos=(" << uav.position.x << ", " << uav.position.y << ") | "
                  << "Ref=(" << ref.position.x << ", " << ref.position.y << ") | "
                  << "Error=" << error << "m | "
                  << "V=" << uav.v << "m/s | "
                  << "φ=" << uav.phi * 180/M_PI << "°\n";
    }
};

// ==================== 阵型变换示例 ====================

class FormationTransition {
private:
    FormationController controller;
    
public:
    // 演示阵型变换
    void demonstrateTransition() {
        std::cout << "\n===== Formation Transition Demo =====\n";
        
        // 创建直线路径
        std::vector<Point2D> path;
        for (int i = 0; i <= 200; ++i) {
            path.emplace_back(i * 5.0, 0);
        }
        controller.initCenterline(path);
        
        // 初始：纵队编队
        std::cout << "Initial: Column Formation\n";
        controller.setFormation(FormationTemplate::COLUMN, 5, 20.0, 0);
        assignUAVs();
        simulateFormation(10.0);
        
        // 变换为线列
        std::cout << "\nTransition to: Line Abreast Formation\n";
        controller.setFormation(FormationTemplate::LINE_ABREAST, 5, 0, 15.0);
        reassignUAVs();
        simulateFormation(10.0);
        
        // 变换为楔形
        std::cout << "\nTransition to: Wedge Formation\n";
        controller.setFormation(FormationTemplate::WEDGE, 5, 15.0, 12.0);
        reassignUAVs();
        simulateFormation(10.0);
        
        std::cout << "\nFormation transitions completed successfully!\n";
    }
    
private:
    void assignUAVs() {
        for (int i = 0; i < 5; ++i) {
            std::string id = "UAV_" + std::to_string(i);
            controller.assignSlot(id, i);
            
            AircraftParams params;
            params.phi_max_rad = 0.5236;
            params.v_nom = 20.0;
            params.L0 = 10.0;
            params.kv = 0.5;
            params.tau_n = 3.0;  // 较大的时间常数用于平滑过渡
            controller.setAircraftParams(id, params);
        }
    }
    
    void reassignUAVs() {
        // 可以实现更复杂的重分配逻辑
        // 例如：最小化总移动距离的优化分配
        assignUAVs();
    }
    
    void simulateFormation(double duration) {
        double dt = 0.1;
        double leader_s = 0;
        
        for (double t = 0; t < duration; t += dt) {
            leader_s += 20.0 * dt;  // 领航速度20m/s
            
            for (int i = 0; i < 5; ++i) {
                std::string id = "UAV_" + std::to_string(i);
                ReferenceOutput ref = controller.computeReference(id, leader_s, dt);
                
                if (static_cast<int>(t / dt) % 20 == 0) {  // 每2秒输出一次
                    std::cout << "  " << id << " -> Ref: ("
                              << std::fixed << std::setprecision(1)
                              << ref.position.x << ", " << ref.position.y << "), "
                              << "V: " << ref.v_cmd << " m/s\n";
                }
            }
        }
    }
};

// ==================== 性能测试 ====================

class PerformanceTest {
public:
    static void runBenchmark() {
        std::cout << "\n===== Performance Benchmark =====\n";
        
        FormationController controller;
        
        // 创建复杂路径（1000个点）
        std::vector<Point2D> path;
        for (int i = 0; i < 1000; ++i) {
            double t = 2 * M_PI * i / 100.0;
            path.emplace_back(
                100 * std::cos(t) + 50 * std::cos(3*t),
                100 * std::sin(t) + 50 * std::sin(2*t)
            );
        }
        
        auto start = std::chrono::high_resolution_clock::now();
        controller.initCenterline(path);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Centerline initialization (1000 points): " 
                  << duration.count() << " μs\n";
        
        // 设置大规模编队（20架）
        controller.setFormation(FormationTemplate::WEDGE, 20, 15.0, 10.0);
        
        // 分配所有UAV
        for (int i = 0; i < 20; ++i) {
            std::string id = "UAV_" + std::to_string(i);
            controller.assignSlot(id, i);
            
            AircraftParams params;
            params.phi_max_rad = 0.5236;
            params.v_nom = 25.0;
            controller.setAircraftParams(id, params);
        }
        
        // 测试参考生成性能
        start = std::chrono::high_resolution_clock::now();
        
        for (int step = 0; step < 1000; ++step) {
            double leader_s = step * 0.1;
            for (int i = 0; i < 20; ++i) {
                controller.computeReference("UAV_" + std::to_string(i), leader_s, 0.01);
            }
        }
        
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "1000 steps × 20 UAVs reference generation: " 
                  << duration.count() << " ms\n";
        std::cout << "Average per UAV per step: " 
                  << std::fixed << std::setprecision(3)
                  << duration.count() / (1000.0 * 20.0) << " ms\n";
        
        // 测试内存使用
        std::cout << "\nMemory usage estimate:\n";
        std::cout << "  Centerline samples: " << 100 * sizeof(CenterlineSample) << " bytes\n";
        std::cout << "  Per UAV state cache: " << sizeof(UAVStateCache) << " bytes\n";
        std::cout << "  Total for 20 UAVs: ~" 
                  << (100 * sizeof(CenterlineSample) + 20 * sizeof(UAVStateCache)) / 1024 
                  << " KB\n";
    }
};

// ==================== 安全性测试 ====================

class SafetyValidator {
public:
    static void validateSafety() {
        std::cout << "\n===== Safety Validation =====\n";
        
        FormationController controller;
        
        // 创建急转弯路径
        std::vector<Point2D> path;
        
        // 直线段
        for (int i = 0; i <= 50; ++i) {
            path.emplace_back(i * 2.0, 0);
        }
        
        // 急转弯（小半径）
        double radius = 30.0;
        for (int i = 1; i <= 50; ++i) {
            double angle = M_PI * i / 50.0;
            path.emplace_back(
                100 + radius * std::cos(angle),
                radius * std::sin(angle)
            );
        }
        
        // 第二个直线段
        for (int i = 1; i <= 50; ++i) {
            path.emplace_back(100 - i * 2.0, radius * 2);
        }
        
        controller.initCenterline(path);
        controller.setFormation(FormationTemplate::LINE_ABREAST, 3, 0, 20.0);
        
        // 测试不同速度下的安全性
        std::vector<double> test_speeds = {10.0, 20.0, 30.0};
        
        for (double v_nom : test_speeds) {
            std::cout << "\nTesting with nominal speed: " << v_nom << " m/s\n";
            std::cout << "Maximum safe turn radius: " 
                      << v_nom * v_nom / (9.81 * std::tan(0.5236)) 
                      << " m\n";
            
            for (int i = 0; i < 3; ++i) {
                std::string id = "UAV_" + std::to_string(i);
                controller.assignSlot(id, i);
                
                AircraftParams params;
                params.phi_max_rad = 0.5236;  // 30度
                params.v_nom = v_nom;
                params.L0 = 10.0;
                params.kv = 0.5;
                params.tau_n = 2.0;
                controller.setAircraftParams(id, params);
            }
            
            // 测试关键点
            std::vector<double> test_points = {50, 75, 100, 125, 150};  // 弧长位置
            
            for (double s : test_points) {
                bool all_safe = true;
                
                for (int i = 0; i < 3; ++i) {
                    std::string id = "UAV_" + std::to_string(i);
                    ReferenceOutput ref = controller.computeReference(id, s, 0.1);
                    
                    // 检查滚转角限制
                    if (std::abs(ref.phi_ff) > 0.5236 + 0.01) {  // 小容差
                        std::cout << "  ⚠ WARNING: " << id 
                                  << " exceeds roll limit at s=" << s 
                                  << " (φ=" << ref.phi_ff * 180/M_PI << "°)\n";
                        all_safe = false;
                    }
                    
                    // 检查速度调整
                    if (ref.v_cmd < v_nom * 0.9) {
                        std::cout << "  ℹ INFO: " << id 
                                  << " speed reduced at s=" << s 
                                  << " (v=" << ref.v_cmd << " m/s, "
                                  << (100 * ref.v_cmd / v_nom) << "% of nominal)\n";
                    }
                    
                    // 检查侧距收拢
                    if (std::abs(ref.n) < 19.0 && i != 0) {  // 中心机除外
                        std::cout << "  ℹ INFO: " << id 
                                  << " lateral offset adjusted at s=" << s 
                                  << " (n=" << ref.n << " m)\n";
                    }
                }
                
                if (all_safe && s == 100) {
                    std::cout << "  ✓ All UAVs safe at turn apex (s=" << s << ")\n";
                }
            }
            
            // 检查最小间距
            std::cout << "  Checking minimum separation distances...\n";
            double min_separation = 1000.0;
            for (int i = 0; i < 3; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    ReferenceOutput ref_i = controller.computeReference(
                        "UAV_" + std::to_string(i), 100, 0.1);
                    ReferenceOutput ref_j = controller.computeReference(
                        "UAV_" + std::to_string(j), 100, 0.1);
                    
                    double separation = (ref_i.position - ref_j.position).norm();
                    min_separation = std::min(min_separation, separation);
                }
            }
            std::cout << "  Minimum separation at turn: " 
                      << std::fixed << std::setprecision(1)
                      << min_separation << " m\n";
        }
        
        std::cout << "\nSafety validation complete!\n";
    }
};

// ==================== 诊断工具 ====================

class DiagnosticTool {
public:
    static void runDiagnostics() {
        std::cout << "\n===== Formation Diagnostics =====\n";
        
        FormationController controller;
        
        // 创建S形曲线路径
        std::vector<Point2D> path;
        for (int i = 0; i <= 200; ++i) {
            double x = i * 2.0;
            double y = 50 * std::sin(x * 0.02);
            path.emplace_back(x, y);
        }
        
        controller.initCenterline(path);
        controller.setFormation(FormationTemplate::WEDGE, 3, 15.0, 10.0);
        
        // 设置UAV
        for (int i = 0; i < 3; ++i) {
            std::string id = "UAV_" + std::to_string(i);
            controller.assignSlot(id, i);
            
            AircraftParams params;
            params.phi_max_rad = 0.4363;  // 25度
            params.v_nom = 18.0;
            params.L0 = 12.0;
            params.kv = 0.4;
            params.tau_n = 2.5;
            controller.setAircraftParams(id, params);
        }
        
        // 输出诊断信息
        std::cout << "\nFormation Configuration:\n";
        std::cout << "  Formation type: Wedge\n";
        std::cout << "  Number of UAVs: 3\n";
        std::cout << "  Longitudinal spacing: 15.0 m\n";
        std::cout << "  Lateral spacing: 10.0 m\n";
        
        std::cout << "\nPath Characteristics:\n";
        std::cout << "  Path type: S-curve\n";
        std::cout << "  Total length: ~400 m\n";
        
        std::cout << "\nSimulation at key points:\n";
        std::vector<double> key_points = {0, 50, 100, 150, 200};
        
        for (double s : key_points) {
            std::cout << "\n  At s = " << s << " m:\n";
            
            for (int i = 0; i < 3; ++i) {
                std::string id = "UAV_" + std::to_string(i);
                ReferenceOutput ref = controller.computeReference(id, s, 0.1);
                auto diag = controller.getDiagnostics(id);
                
                std::cout << "    " << id << ":\n";
                std::cout << "      Position: (" 
                          << std::fixed << std::setprecision(1)
                          << ref.position.x << ", " << ref.position.y << ")\n";
                std::cout << "      Speed: " << ref.v_cmd << " m/s\n";
                std::cout << "      Curvature: " << ref.kappa_ref << " 1/m\n";
                std::cout << "      Roll angle: " << ref.phi_ff * 180/M_PI << "°\n";
                
                if (!diag.empty()) {
                    std::cout << "      Arc length: " << diag["arc_length_s"] << " m\n";
                    std::cout << "      Lateral offset: " << diag["lateral_offset_n"] << " m\n";
                }
            }
        }
        
        std::cout << "\nDiagnostics complete!\n";
    }
};

// ==================== 主函数 ====================

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "   UAV Formation Control Simulation    \n";
    std::cout << "========================================\n";
    
    // 选择测试场景
    int choice = 1;
    if (argc > 1) {
        choice = std::atoi(argv[1]);
    }
    
    switch (choice) {
        case 1: {
            // 场景1：圆形路径楔形编队
            FormationSimulator sim(0.1);
            sim.setupScenario1_CircularPath();
            sim.run(60.0);  // 运行60秒
            break;
        }
        
        case 2: {
            // 场景2：8字形路径纵队编队
            FormationSimulator sim(0.1);
            sim.setupScenario2_FigureEight();
            sim.run(90.0);  // 运行90秒
            break;
        }
        
        case 3: {
            // 阵型变换演示
            FormationTransition transition;
            transition.demonstrateTransition();
            break;
        }
        
        case 4: {
            // 性能测试
            PerformanceTest::runBenchmark();
            break;
        }
        
        case 5: {
            // 安全性验证
            SafetyValidator::validateSafety();
            break;
        }
        
        case 6: {
            // 诊断工具
            DiagnosticTool::runDiagnostics();
            break;
        }
        
        default:
            std::cout << "\nUsage: " << argv[0] << " [scenario]\n";
            std::cout << "Scenarios:\n";
            std::cout << "  1 - Circular path with wedge formation (default)\n";
            std::cout << "  2 - Figure-8 path with column formation\n";
            std::cout << "  3 - Formation transition demo\n";
            std::cout << "  4 - Performance benchmark\n";
            std::cout << "  5 - Safety validation\n";
            std::cout << "  6 - Diagnostic tool\n\n";
            
            std::cout << "Running default scenario (1)...\n";
            FormationSimulator sim(0.1);
            sim.setupScenario1_CircularPath();
            sim.run(30.0);
    }
    
    return 0;
}
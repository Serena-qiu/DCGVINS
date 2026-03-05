/**
 * @file timing_statistics.h
 * @brief DCGVINS时间统计模块 - 用于论文性能对比
 * 
 * 功能：
 * 1. 模块级时间统计（类似ORB-SLAM3论文表格）
 * 2. DD约束开/关对比
 * 3. DCGVINS vs GVINS对比
 * 4. 支持CSV/LaTeX导出
 * 
 * 使用方法：
 *   TIME_MODULE("ModuleName");  // RAII自动计时
 *   TIME_START(name); ... TIME_END(name);  // 手动计时
 */

#pragma once

#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <numeric>
#include "tic_toc.h"

// ============================================================
//                    开关配置
// ============================================================
// 设为0可完全关闭计时功能（发布版本）
#define TIMING_ENABLED 1

// 是否启用详细子模块计时
#define TIMING_DETAILED 1

// ============================================================
//                    TimingStatistics 类
// ============================================================
class TimingStatistics {
public:
    /**
     * @brief 获取单例实例
     */
    static TimingStatistics& getInstance() {
        static TimingStatistics instance;
        return instance;
    }
    
    /**
     * @brief 记录一次耗时
     * @param module 模块名称
     * @param time_ms 耗时（毫秒）
     */
    void record(const std::string& module, double time_ms) {
        if (!enabled_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto& stats = stats_[module];
        stats.times.push_back(time_ms);
        stats.sum += time_ms;
        stats.count++;
        stats.min_val = std::min(stats.min_val, time_ms);
        stats.max_val = std::max(stats.max_val, time_ms);
    }
    
    /**
     * @brief 获取均值
     */
    double getMean(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = stats_.find(module);
        if (it == stats_.end() || it->second.times.empty()) return 0;
        return it->second.sum / it->second.times.size();
    }
    
    /**
     * @brief 获取标准差
     */
    double getStdDev(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = stats_.find(module);
        if (it == stats_.end() || it->second.times.size() < 2) return 0;
        
        const auto& times = it->second.times;
        double mean = it->second.sum / times.size();
        double sq_sum = 0;
        for (double t : times) {
            sq_sum += (t - mean) * (t - mean);
        }
        return std::sqrt(sq_sum / times.size());
    }
    
    /**
     * @brief 获取样本数
     */
    size_t getCount(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = stats_.find(module);
        if (it == stats_.end()) return 0;
        return it->second.times.size();
    }
    
    /**
     * @brief 获取最小值
     */
    double getMin(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = stats_.find(module);
        if (it == stats_.end()) return 0;
        return it->second.min_val;
    }
    
    /**
     * @brief 获取最大值
     */
    double getMax(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = stats_.find(module);
        if (it == stats_.end()) return 0;
        return it->second.max_val;
    }
    
    /**
     * @brief 打印论文风格表格到终端
     * @param title 表格标题（如 "DCGVINS" 或 "GVINS"）
     * @param dd_enabled 是否启用DD约束
     */
    void printTable(const std::string& title = "DCGVINS", bool dd_enabled = true) const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║             TIMING STATISTICS: " << std::left << std::setw(20) << title;
        std::cout << (dd_enabled ? "(DD ON)" : "(DD OFF)") << "      ║\n";
        std::cout << "║                    (Mean ± Std in ms)                            ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ " << std::left << std::setw(28) << "Module" 
                  << std::right << std::setw(18) << "Time (ms)" 
                  << std::setw(12) << "Count"
                  << std::setw(8) << "Min"
                  << std::setw(8) << "Max" << " ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        
        // 按类别组织输出 - 参考ORB-SLAM3论文表格结构
        std::vector<std::pair<std::string, std::vector<std::string>>> categories = {
            {"Frontend (Thread 1)", {
                "IMU_Integration",
                "Feature_Processing", 
                "FT_ImageConvert",      // Feature Tracker: 图像转换
                "FT_Tracking",          // Feature Tracker: 特征跟踪
                "FT_IDUpdate",          // Feature Tracker: ID更新
                "Feature_Tracking"      // 兼容原有的计时
            }},
            {"Backend (Thread 2)", {
                "Optimization_Total",
                "Opt_Prepare",
                "Opt_Visual_Factor",
                "Opt_GNSS_Factor",
                "Opt_DD_Factor",        // DD相关 - 关键对比项
                "Opt_Ceres_Solve",
                "Marginalization"
            }},
            {"GNSS Processing", {
                "GNSS_Process",
                "DD_Computation",       // DD计算 - 关键对比项
                "DD_Factor_Create",
                "DD_Quality_Check"
            }},
            {"Total", {
                "ProcessImage_Total",
                "SolveOdometry_Total",
                "Frame_Total"
            }}
        };
        
        for (const auto& cat : categories) {
            std::cout << "║ --- " << std::left << std::setw(61) << cat.first << "--- ║\n";
            
            for (const auto& name : cat.second) {
                auto it = stats_.find(name);
                if (it != stats_.end() && !it->second.times.empty()) {
                    double mean = it->second.sum / it->second.times.size();
                    double std_dev = computeStdDev(it->second.times, mean);
                    
                    std::ostringstream time_str;
                    time_str << std::fixed << std::setprecision(2) 
                             << mean << " ± " << std_dev;
                    
                    std::cout << "║ " << std::left << std::setw(28) << name
                              << std::right << std::setw(18) << time_str.str()
                              << std::setw(12) << it->second.times.size()
                              << std::setw(8) << std::fixed << std::setprecision(1) << it->second.min_val
                              << std::setw(8) << it->second.max_val << " ║\n";
                }
            }
        }
        
        // 输出未分类的模块
        std::cout << "║ --- " << std::left << std::setw(61) << "Other" << "--- ║\n";
        for (const auto& kv : stats_) {
            bool in_category = false;
            for (const auto& cat : categories) {
                if (std::find(cat.second.begin(), cat.second.end(), kv.first) != cat.second.end()) {
                    in_category = true;
                    break;
                }
            }
            
            if (!in_category && !kv.second.times.empty()) {
                double mean = kv.second.sum / kv.second.times.size();
                double std_dev = computeStdDev(kv.second.times, mean);
                
                std::ostringstream time_str;
                time_str << std::fixed << std::setprecision(2) 
                         << mean << " ± " << std_dev;
                
                std::cout << "║ " << std::left << std::setw(28) << kv.first
                          << std::right << std::setw(18) << time_str.str()
                          << std::setw(12) << kv.second.times.size()
                          << std::setw(8) << std::fixed << std::setprecision(1) << kv.second.min_val
                          << std::setw(8) << kv.second.max_val << " ║\n";
            }
        }
        
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";
    }
    
    /**
     * @brief 导出CSV文件（方便Python/MATLAB绘图）
     * @param filename 文件路径
     */
    void exportCSV(const std::string& filename) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "[Timing] Failed to open file: " << filename << std::endl;
            return;
        }
        
        // CSV Header
        file << "Module,Mean_ms,StdDev_ms,Min_ms,Max_ms,Count\n";
        
        for (const auto& kv : stats_) {
            if (kv.second.times.empty()) continue;
            
            double mean = kv.second.sum / kv.second.times.size();
            double std_dev = computeStdDev(kv.second.times, mean);
            
            file << kv.first << ","
                 << std::fixed << std::setprecision(4)
                 << mean << ","
                 << std_dev << ","
                 << kv.second.min_val << ","
                 << kv.second.max_val << ","
                 << kv.second.times.size() << "\n";
        }
        
        file.close();
        std::cout << "[Timing] Exported CSV to: " << filename << std::endl;
    }
    
    /**
     * @brief 导出所有原始数据（用于详细分析）
     * @param filename 文件路径
     */
    void exportRawData(const std::string& filename) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "[Timing] Failed to open file: " << filename << std::endl;
            return;
        }
        
        for (const auto& kv : stats_) {
            file << kv.first;
            for (double t : kv.second.times) {
                file << "," << std::fixed << std::setprecision(4) << t;
            }
            file << "\n";
        }
        
        file.close();
        std::cout << "[Timing] Exported raw data to: " << filename << std::endl;
    }
    
    /**
     * @brief 导出LaTeX表格格式（直接用于论文）
     * @param filename 文件路径
     * @param caption 表格标题
     */
    void exportLatex(const std::string& filename, 
                     const std::string& caption = "Runtime of DCGVINS") const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "[Timing] Failed to open file: " << filename << std::endl;
            return;
        }
        
        file << "% Auto-generated timing table\n";
        file << "\\begin{table}[h]\n";
        file << "\\centering\n";
        file << "\\caption{" << caption << " (Mean $\\pm$ Std in ms)}\n";
        file << "\\label{tab:timing}\n";
        file << "\\begin{tabular}{lcc}\n";
        file << "\\toprule\n";
        file << "Module & Time (ms) & Count \\\\\n";
        file << "\\midrule\n";
        
        // 主要模块
        std::vector<std::string> main_modules = {
            "IMU_Integration",
            "Feature_Processing",
            "Optimization_Total",
            "Opt_DD_Factor",
            "DD_Computation",
            "Marginalization",
            "ProcessImage_Total"
        };
        
        for (const auto& name : main_modules) {
            auto it = stats_.find(name);
            if (it != stats_.end() && !it->second.times.empty()) {
                double mean = it->second.sum / it->second.times.size();
                double std_dev = computeStdDev(it->second.times, mean);
                
                // 格式化模块名（下划线转空格）
                std::string display_name = name;
                std::replace(display_name.begin(), display_name.end(), '_', ' ');
                
                file << display_name << " & $" 
                     << std::fixed << std::setprecision(2) << mean 
                     << " \\pm " << std_dev << "$ & "
                     << it->second.times.size() << " \\\\\n";
            }
        }
        
        file << "\\bottomrule\n";
        file << "\\end{tabular}\n";
        file << "\\end{table}\n";
        
        file.close();
        std::cout << "[Timing] Exported LaTeX to: " << filename << std::endl;
    }
    
    /**
     * @brief 导出对比表格（GVINS vs DCGVINS 或 DD ON vs DD OFF）
     * @param other 另一组统计数据
     * @param filename 文件路径
     * @param label1 第一组标签
     * @param label2 第二组标签
     */
    void exportComparisonLatex(const TimingStatistics& other,
                               const std::string& filename,
                               const std::string& label1 = "GVINS",
                               const std::string& label2 = "DCGVINS") const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "[Timing] Failed to open file: " << filename << std::endl;
            return;
        }
        
        file << "% Comparison table: " << label1 << " vs " << label2 << "\n";
        file << "\\begin{table}[h]\n";
        file << "\\centering\n";
        file << "\\caption{Runtime Comparison: " << label1 << " vs " << label2 << "}\n";
        file << "\\begin{tabular}{lccc}\n";
        file << "\\toprule\n";
        file << "Module & " << label1 << " (ms) & " << label2 << " (ms) & Overhead (\\%) \\\\\n";
        file << "\\midrule\n";
        
        std::vector<std::string> compare_modules = {
            "Optimization_Total",
            "Opt_DD_Factor",
            "DD_Computation",
            "ProcessImage_Total"
        };
        
        for (const auto& name : compare_modules) {
            double mean1 = getMeanInternal(name);
            double mean2 = other.getMean(name);
            
            if (mean1 > 0 || mean2 > 0) {
                double overhead = (mean1 > 0) ? ((mean2 - mean1) / mean1 * 100.0) : 0;
                
                std::string display_name = name;
                std::replace(display_name.begin(), display_name.end(), '_', ' ');
                
                file << display_name << " & "
                     << std::fixed << std::setprecision(2) << mean1 << " & "
                     << mean2 << " & ";
                if (overhead > 0) {
                    file << "+" << std::setprecision(1) << overhead;
                } else {
                    file << std::setprecision(1) << overhead;
                }
                file << " \\\\\n";
            }
        }
        
        file << "\\bottomrule\n";
        file << "\\end{tabular}\n";
        file << "\\end{table}\n";
        
        file.close();
        std::cout << "[Timing] Exported comparison to: " << filename << std::endl;
    }
    
    /**
     * @brief 重置所有统计
     */
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.clear();
    }
    
    /**
     * @brief 启用/禁用计时
     */
    void setEnabled(bool enable) { enabled_ = enable; }
    bool isEnabled() const { return enabled_; }
    
    /**
     * @brief 设置实验名称（用于日志）
     */
    void setExperimentName(const std::string& name) { experiment_name_ = name; }
    std::string getExperimentName() const { return experiment_name_; }

private:
    TimingStatistics() : enabled_(true), experiment_name_("default") {}
    
    // 禁止拷贝
    TimingStatistics(const TimingStatistics&) = delete;
    TimingStatistics& operator=(const TimingStatistics&) = delete;
    
    struct Stats {
        std::vector<double> times;
        double sum = 0;
        double min_val = 1e9;
        double max_val = 0;
        size_t count = 0;
    };
    
    double computeStdDev(const std::vector<double>& times, double mean) const {
        if (times.size() < 2) return 0;
        double sq_sum = 0;
        for (double t : times) {
            sq_sum += (t - mean) * (t - mean);
        }
        return std::sqrt(sq_sum / times.size());
    }
    
    double getMeanInternal(const std::string& module) const {
        auto it = stats_.find(module);
        if (it == stats_.end() || it->second.times.empty()) return 0;
        return it->second.sum / it->second.times.size();
    }
    
    mutable std::mutex mutex_;
    std::map<std::string, Stats> stats_;
    bool enabled_;
    std::string experiment_name_;
};


// ============================================================
//                    ScopedTimer - RAII计时器
// ============================================================
class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name) : name_(name), enabled_(true) {
        if (TimingStatistics::getInstance().isEnabled()) {
            timer_.tic();
        } else {
            enabled_ = false;
        }
    }
    
    ~ScopedTimer() {
        if (enabled_) {
            TimingStatistics::getInstance().record(name_, timer_.toc());
        }
    }
    
    // 禁止拷贝
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    std::string name_;
    TicToc timer_;
    bool enabled_;
};


// ============================================================
//                    ManualTimer - 手动计时器
// ============================================================
class ManualTimer {
public:
    ManualTimer() : running_(false) {}
    
    void start() {
        if (TimingStatistics::getInstance().isEnabled()) {
            timer_.tic();
            running_ = true;
        }
    }
    
    double stop(const std::string& name) {
        if (running_) {
            double elapsed = timer_.toc();
            TimingStatistics::getInstance().record(name, elapsed);
            running_ = false;
            return elapsed;
        }
        return 0;
    }
    
    double elapsed() const {
        if (running_) {
            TicToc temp;
            // 获取已过时间但不停止
            return const_cast<TicToc&>(timer_).toc();
        }
        return 0;
    }

private:
    TicToc timer_;
    bool running_;
};


// ============================================================
//                    便捷宏定义
// ============================================================
#if TIMING_ENABLED
    // RAII 自动计时 - 作用域结束时自动记录
    #define TIME_MODULE(name) ScopedTimer _scoped_timer_##__LINE__(name)
    
    // 手动计时 - 需要显式调用开始和结束
    #define TIME_START(var_name) TicToc _timer_##var_name; _timer_##var_name.tic()
    #define TIME_END(var_name, module_name) \
        TimingStatistics::getInstance().record(module_name, _timer_##var_name.toc())
    #define TIME_END_PRINT(var_name, module_name) \
        do { \
            double _elapsed = _timer_##var_name.toc(); \
            TimingStatistics::getInstance().record(module_name, _elapsed); \
            ROS_DEBUG("[Timing] %s: %.2f ms", module_name, _elapsed); \
        } while(0)
    
    // 条件计时
    #define TIME_MODULE_IF(condition, name) \
        std::unique_ptr<ScopedTimer> _cond_timer_##__LINE__; \
        if (condition) _cond_timer_##__LINE__ = std::make_unique<ScopedTimer>(name)
    
    // 打印统计
    #define TIMING_PRINT_STATS(title) \
        TimingStatistics::getInstance().printTable(title)
    
    // 导出
    #define TIMING_EXPORT_CSV(path) \
        TimingStatistics::getInstance().exportCSV(path)
    #define TIMING_EXPORT_LATEX(path, caption) \
        TimingStatistics::getInstance().exportLatex(path, caption)
        
#else
    #define TIME_MODULE(name) ((void)0)
    #define TIME_START(var_name) ((void)0)
    #define TIME_END(var_name, module_name) ((void)0)
    #define TIME_END_PRINT(var_name, module_name) ((void)0)
    #define TIME_MODULE_IF(condition, name) ((void)0)
    #define TIMING_PRINT_STATS(title) ((void)0)
    #define TIMING_EXPORT_CSV(path) ((void)0)
    #define TIMING_EXPORT_LATEX(path, caption) ((void)0)
#endif


// ============================================================
//              TimingExporter - 便捷导出类
// ============================================================
class TimingExporter {
public:
    /**
     * @brief 导出所有格式
     * @param base_path 基础路径（不含扩展名）
     * @param experiment_name 实验名称
     * @param dd_enabled 是否启用DD
     */
    static void exportAll(const std::string& base_path,
                         const std::string& experiment_name = "dcgvins",
                         bool dd_enabled = true) {
        auto& stats = TimingStatistics::getInstance();
        
        // 打印到终端
        stats.printTable(experiment_name, dd_enabled);
        
        // 生成文件名后缀
        std::string suffix = dd_enabled ? "_dd_on" : "_dd_off";
        
        // CSV
        stats.exportCSV(base_path + suffix + ".csv");
        
        // Raw data
        stats.exportRawData(base_path + suffix + "_raw.csv");
        
        // LaTeX
        std::string caption = experiment_name + (dd_enabled ? " (DD Enabled)" : " (DD Disabled)");
        stats.exportLatex(base_path + suffix + ".tex", caption);
        
        std::cout << "[TimingExporter] All files exported to: " << base_path << suffix << ".*" << std::endl;
    }
};
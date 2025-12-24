#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>

// 定义点类型（可以根据需要修改）
struct LivoxPointXYZI {
    PCL_ADD_POINT4D;      // XYZ position
    float intensity;      // 强度
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 注册点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZI,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

class LivoxCloudCollector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber livox_sub_;
    
    pcl::PointCloud<LivoxPointXYZI>::Ptr accumulated_cloud_;
    std::mutex cloud_mutex_;
    
    ros::Time start_time_;
    bool collecting_;
    bool finished_;
    
    std::string output_filename_;
    double collect_duration_;  // 收集持续时间（秒）
    
public:
    LivoxCloudCollector() : 
        collecting_(false),
        finished_(false),
        collect_duration_(5.0) {
        
        // 初始化点云
        accumulated_cloud_.reset(new pcl::PointCloud<LivoxPointXYZI>());
        
        // 从参数服务器获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param("output_filename", output_filename_, std::string("livox_collected.pcd"));
        private_nh.param("collect_duration", collect_duration_, 5.0);
        
        ROS_INFO("Livox Cloud Collector initialized");
        ROS_INFO("Will collect point cloud for %.1f seconds", collect_duration_);
        ROS_INFO("Output file: %s", output_filename_.c_str());
        
        // 订阅Livox话题
        livox_sub_ = nh_.subscribe("/livox/lidar", 1000, 
                                  &LivoxCloudCollector::livoxCallback, this);
    }
    
    ~LivoxCloudCollector() {
        if (!finished_ && !accumulated_cloud_->empty()) {
            savePointCloud();
        }
    }
    
    void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        // 如果是第一次接收消息，开始计时
        if (!collecting_) {
            start_time_ = ros::Time::now();
            collecting_ = true;
            ROS_INFO("Start collecting point cloud...");
        }
        
        // 检查是否超过收集时间
        ros::Time current_time = ros::Time::now();
        double elapsed = (current_time - start_time_).toSec();
        
        if (elapsed > collect_duration_) {
            if (!finished_) {
                finished_ = true;
                ROS_INFO("Collection finished. Collected %.1f seconds of data.", elapsed);
                savePointCloud();
                ros::shutdown();
            }
            return;
        }
        
        // 处理Livox点云数据
        processLivoxMessage(msg);
        
        // 显示进度
        if (std::fmod(elapsed, 1.0) < 0.1) {  // 每秒显示一次
            ROS_INFO("Collecting: %.1f/%.1f seconds, Points: %lu", 
                    elapsed, collect_duration_, accumulated_cloud_->size());
        }
    }
    
    void processLivoxMessage(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        // 将Livox点云数据转换为PCL格式
        for (size_t i = 0; i < msg->point_num; ++i) {
            const auto& livox_point = msg->points[i];
            
            // 只添加有效的点（根据您的需要进行调整）
            if (!std::isnan(livox_point.x) && 
                !std::isnan(livox_point.y) && 
                !std::isnan(livox_point.z)) {
                
                LivoxPointXYZI pcl_point;
                pcl_point.x = livox_point.x;
                pcl_point.y = livox_point.y;
                pcl_point.z = livox_point.z;
                pcl_point.intensity = livox_point.reflectivity;
                
                accumulated_cloud_->push_back(pcl_point);
            }
        }
    }
    
    void savePointCloud() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        if (accumulated_cloud_->empty()) {
            ROS_WARN("No point cloud data collected!");
            return;
        }
        
        // 设置点云属性
        accumulated_cloud_->width = accumulated_cloud_->size();
        accumulated_cloud_->height = 1;
        accumulated_cloud_->is_dense = false;
        
        try {
            // 保存为PCD文件
            if (pcl::io::savePCDFileBinary(output_filename_, *accumulated_cloud_) == 0) {
                ROS_INFO("Successfully saved %lu points to %s", 
                        accumulated_cloud_->size(), output_filename_.c_str());
                
                // 打印点云信息
                ROS_INFO("Point cloud info:");
                ROS_INFO("  Size: %lu points", accumulated_cloud_->size());
                ROS_INFO("  Bounding box:");
                
                // // 计算边界框
                // LivoxPointXYZI min_pt, max_pt;
                // pcl::getMinMax3D(*accumulated_cloud_, min_pt, max_pt);
                // ROS_INFO("    X: [%.3f, %.3f]", min_pt.x, max_pt.x);
                // ROS_INFO("    Y: [%.3f, %.3f]", min_pt.y, max_pt.y);
                // ROS_INFO("    Z: [%.3f, %.3f]", min_pt.z, max_pt.z);
            } else {
                ROS_ERROR("Failed to save point cloud to %s", output_filename_.c_str());
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while saving PCD file: %s", e.what());
        }
    }
    
    bool isFinished() const {
        return finished_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_cloud_collector");
    
    ROS_INFO("=== Livox Point Cloud Collector ===");
    ROS_INFO("This node will:");
    ROS_INFO("1. Subscribe to /livox/lidar topic");
    ROS_INFO("2. Collect point cloud data for 5 seconds");
    ROS_INFO("3. Save all collected points to a PCD file");
    ROS_INFO("Waiting for Livox data...");
    
    LivoxCloudCollector collector;
    
    // 设置超时（如果超过30秒没有收到数据，则退出）
    ros::Time start_wait = ros::Time::now();
    while (ros::ok() && !collector.isFinished()) {
        ros::spinOnce();
        
        // 检查是否长时间没有收到数据
        if ((ros::Time::now() - start_wait).toSec() > 30.0) {
            ROS_WARN("No Livox data received for 30 seconds. Exiting...");
            break;
        }
        
        ros::Duration(0.01).sleep();  // 10ms sleep
    }
    
    if (collector.isFinished()) {
        ROS_INFO("Collection completed successfully!");
    } else {
        ROS_WARN("Collection terminated early!");
    }
    
    return 0;
}
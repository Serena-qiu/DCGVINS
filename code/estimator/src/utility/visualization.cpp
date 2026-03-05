#include "visualization.h"

using namespace ros;
using namespace Eigen;

ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher pub_gnss_lla;
ros::Publisher pub_enu_path, pub_rtk_enu_path;
nav_msgs::Path enu_path, rtk_enu_path;
ros::Publisher pub_anc_lla;
ros::Publisher pub_enu_pose;
ros::Publisher pub_sat_info;
ros::Publisher pub_yaw_enu_local;

// 🆕 双差可视化发布器
ros::Publisher pub_secondary_path;
ros::Publisher pub_dd_constraints;
ros::Publisher pub_satellite_markers;
ros::Publisher pub_dd_info_text;
nav_msgs::Path secondary_gnss_path;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_gnss_lla = n.advertise<sensor_msgs::NavSatFix>("gnss_fused_lla", 1000);
    pub_enu_path = n.advertise<nav_msgs::Path>("gnss_enu_path", 1000);
    pub_anc_lla = n.advertise<sensor_msgs::NavSatFix>("gnss_anchor_lla", 1000);
    pub_enu_pose = n.advertise<geometry_msgs::PoseStamped>("enu_pose", 1000);

    // 🆕 双差可视化发布器注册
    if (ENABLE_REAL_TIME_DD) {
        pub_secondary_path = n.advertise<nav_msgs::Path>("secondary_gnss_path", 1000);
        pub_dd_constraints = n.advertise<visualization_msgs::MarkerArray>("dd_constraints", 1000);
        pub_satellite_markers = n.advertise<visualization_msgs::MarkerArray>("satellite_markers", 1000);
        pub_dd_info_text = n.advertise<visualization_msgs::MarkerArray>("dd_info_text", 1000);
        ROS_INFO("✅ Double Difference visualization publishers registered");
    }

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
                       const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("ts: %f\n", estimator.Headers[WINDOW_SIZE].stamp.toSec());
    printf("position: %f, %f, %f\n", estimator.Ps[WINDOW_SIZE].x(), 
           estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();

        pubGnssResult(estimator, header);
    }
}

void pubGnssResult(const Estimator &estimator, const std_msgs::Header &header)
{
    if (!estimator.gnss_ready) return;
    
    const double gnss_ts = estimator.Headers[WINDOW_SIZE].stamp.toSec() + 
        estimator.diff_t_gnss_local;
    Eigen::Vector3d lla_pos = ecef2geo(estimator.ecef_pos);
    printf("global time: %f\n", gnss_ts);
    printf("latitude longitude altitude: %f, %f, %f\n", lla_pos.x(), lla_pos.y(), lla_pos.z());
    
    sensor_msgs::NavSatFix gnss_lla_msg;
    gnss_lla_msg.header.stamp = ros::Time(gnss_ts);
    gnss_lla_msg.header.frame_id = "geodetic";
    gnss_lla_msg.latitude = lla_pos.x();
    gnss_lla_msg.longitude = lla_pos.y();
    gnss_lla_msg.altitude = lla_pos.z();
    pub_gnss_lla.publish(gnss_lla_msg);

    const Eigen::Vector3d anc_lla = ecef2geo(estimator.anc_ecef);
    sensor_msgs::NavSatFix anc_lla_msg;
    anc_lla_msg.header = gnss_lla_msg.header;
    anc_lla_msg.latitude = anc_lla.x();
    anc_lla_msg.longitude = anc_lla.y();
    anc_lla_msg.altitude = anc_lla.z();
    pub_anc_lla.publish(anc_lla_msg);

    geometry_msgs::PoseStamped enu_pose_msg;
    Eigen::Matrix3d R_s_c;
    R_s_c <<  0,  0,  1,
             -1,  0,  0,
              0, -1,  0;
    Eigen::Matrix3d R_w_sensor = estimator.Rs[WINDOW_SIZE] * estimator.ric[0] * R_s_c.transpose();
    Eigen::Quaterniond enu_ori(estimator.R_enu_local * R_w_sensor);
    enu_pose_msg.header.stamp = header.stamp;
    enu_pose_msg.header.frame_id = "world";
    enu_pose_msg.pose.position.x = estimator.enu_pos.x();
    enu_pose_msg.pose.position.y = estimator.enu_pos.y();
    enu_pose_msg.pose.position.z = estimator.enu_pos.z();
    enu_pose_msg.pose.orientation.x = enu_ori.x();
    enu_pose_msg.pose.orientation.y = enu_ori.y();
    enu_pose_msg.pose.orientation.z = enu_ori.z();
    enu_pose_msg.pose.orientation.w = enu_ori.w();
    pub_enu_pose.publish(enu_pose_msg);

    enu_path.header = enu_pose_msg.header;
    enu_path.poses.push_back(enu_pose_msg);
    pub_enu_path.publish(enu_path);

    Eigen::Quaterniond q_enu_world(estimator.R_enu_local);
    static tf::TransformBroadcaster br;
    tf::Transform transform_enu_world;
    transform_enu_world.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion tf_q;
    tf_q.setW(q_enu_world.w());
    tf_q.setX(q_enu_world.x());
    tf_q.setY(q_enu_world.y());
    tf_q.setZ(q_enu_world.z());
    transform_enu_world.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform_enu_world, header.stamp, "enu", "world"));

    ofstream gnss_output(GNSS_RESULT_PATH, ios::app);
    gnss_output.setf(ios::fixed, ios::floatfield);
    gnss_output.precision(0);
    gnss_output << header.stamp.toSec() * 1e9 << ',';
    gnss_output << gnss_ts * 1e9 << ',';
    gnss_output.precision(5);
    gnss_output << estimator.ecef_pos(0) << ','
                << estimator.ecef_pos(1) << ','
                << estimator.ecef_pos(2) << ','
                << estimator.yaw_enu_local << ','
                << estimator.para_rcv_dt[(WINDOW_SIZE)*4+0] << ','
                << estimator.para_rcv_dt[(WINDOW_SIZE)*4+1] << ','
                << estimator.para_rcv_dt[(WINDOW_SIZE)*4+2] << ','
                << estimator.para_rcv_dt[(WINDOW_SIZE)*4+3] << ','
                << estimator.para_rcv_ddt[WINDOW_SIZE] << ','
                << estimator.anc_ecef(0) << ','
                << estimator.anc_ecef(1) << ','
                << estimator.anc_ecef(2) << '\n';
    gnss_output.close();
}

// 🆕 ========== 双差可视化函数实现 ==========

void pubSecondaryGNSSPath(const Eigen::Vector3d& secondary_pos_ecef, 
                          const std_msgs::Header &header,
                          const Eigen::Vector3d& anchor_ecef,
                          const Eigen::Matrix3d& R_ecef_enu)
{
    // ECEF → ENU 转换
    Eigen::Vector3d enu_pos = R_ecef_enu.transpose() * (secondary_pos_ecef - anchor_ecef);
    
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.header.frame_id = "world";
    pose.pose.position.x = enu_pos.x();
    pose.pose.position.y = enu_pos.y();
    pose.pose.position.z = enu_pos.z();
    pose.pose.orientation.w = 1.0;
    
    secondary_gnss_path.header = header;
    secondary_gnss_path.header.frame_id = "world";
    secondary_gnss_path.poses.push_back(pose);
    
    // 限制路径长度
    if (secondary_gnss_path.poses.size() > 1000) {
        secondary_gnss_path.poses.erase(secondary_gnss_path.poses.begin());
    }
    
    pub_secondary_path.publish(secondary_gnss_path);
}

void pubDDConstraints(const Estimator &estimator, 
                     const std::vector<DoubleDifferenceObs>& dd_obs_list,
                     const std_msgs::Header &header)
{
    if (!ENABLE_REAL_TIME_DD || dd_obs_list.empty()) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < dd_obs_list.size() && i < 30; ++i) {
        const auto& dd = dd_obs_list[i];
        
        // 获取主设备ENU位置
        Eigen::Vector3d primary_enu = estimator.R_enu_local * estimator.Ps[WINDOW_SIZE];
        
        // 获取副设备位置
        Eigen::Vector3d secondary_ecef;
        if (!const_cast<Estimator&>(estimator).getSecondaryPositionAtTime(dd.timestamp, secondary_ecef)) {
            continue;
        }
        
        // 转换副设备到ENU
        Eigen::Vector3d secondary_enu = estimator.R_ecef_enu.transpose() * 
                                        (secondary_ecef - estimator.anc_ecef);
        
        // 创建连线marker
        visualization_msgs::Marker line;
        line.header = header;
        line.header.frame_id = "world";
        line.ns = "dd_constraints";
        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.1;  // 线宽
        line.lifetime = ros::Duration(2.0);
        
        // 根据质量设置颜色
        line.color.r = 1.0 - dd.quality_score;
        line.color.g = dd.quality_score;
        line.color.b = 0.2;
        line.color.a = 0.7;
        
        geometry_msgs::Point p1, p2;
        p1.x = primary_enu.x();
        p1.y = primary_enu.y();
        p1.z = primary_enu.z();
        
        p2.x = secondary_enu.x();
        p2.y = secondary_enu.y();
        p2.z = secondary_enu.z();
        
        line.points.push_back(p1);
        line.points.push_back(p2);
        
        marker_array.markers.push_back(line);
    }
    
    pub_dd_constraints.publish(marker_array);
}

void pubSatelliteMarkers(const Estimator &estimator,
                        const std::vector<DoubleDifferenceObs>& dd_obs_list,
                        const std_msgs::Header &header)
{
    if (!ENABLE_REAL_TIME_DD || dd_obs_list.empty()) return;
    
    visualization_msgs::MarkerArray marker_array;
    std::set<int> displayed_sats;
    
    for (size_t i = 0; i < dd_obs_list.size() && displayed_sats.size() < 15; ++i) {
        const auto& dd = dd_obs_list[i];
        
        // 显示参考卫星
        if (displayed_sats.find(dd.sat1_id) == displayed_sats.end()) {
            visualization_msgs::Marker sat_marker;
            sat_marker.header = header;
            sat_marker.header.frame_id = "world";
            sat_marker.ns = "satellites";
            sat_marker.id = dd.sat1_id;
            sat_marker.type = visualization_msgs::Marker::SPHERE;
            sat_marker.action = visualization_msgs::Marker::ADD;
            sat_marker.lifetime = ros::Duration(2.0);
            
            // 缩放卫星位置便于显示
            Eigen::Vector3d sat_enu = estimator.R_ecef_enu.transpose() * 
                                     (dd.sat1_pos_ecef - estimator.anc_ecef);
            sat_enu = sat_enu / 50000.0;  // 大幅缩放
            
            sat_marker.pose.position.x = sat_enu.x();
            sat_marker.pose.position.y = sat_enu.y();
            sat_marker.pose.position.z = sat_enu.z() + 200;  // 抬高显示
            
            sat_marker.scale.x = sat_marker.scale.y = sat_marker.scale.z = 8.0;
            sat_marker.color.r = 1.0;
            sat_marker.color.g = 0.8;
            sat_marker.color.b = 0.0;
            sat_marker.color.a = 0.9;
            
            marker_array.markers.push_back(sat_marker);
            displayed_sats.insert(dd.sat1_id);
        }
    }
    
    pub_satellite_markers.publish(marker_array);
}

void pubDDInfoText(const Estimator &estimator,
                  const std::vector<DoubleDifferenceObs>& dd_obs_list,
                  const std_msgs::Header &header)
{
    if (!ENABLE_REAL_TIME_DD) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    // 统计信息
    int total_dd = dd_obs_list.size();
    double avg_quality = 0.0;
    for (const auto& dd : dd_obs_list) {
        avg_quality += dd.quality_score;
    }
    if (total_dd > 0) avg_quality /= total_dd;
    
    // 创建文本marker
    visualization_msgs::Marker text;
    text.header = header;
    text.header.frame_id = "world";
    text.ns = "dd_info";
    text.id = 0;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.lifetime = ros::Duration(2.0);
    
    Eigen::Vector3d primary_enu = estimator.R_enu_local * estimator.Ps[WINDOW_SIZE];
    text.pose.position.x = primary_enu.x();
    text.pose.position.y = primary_enu.y();
    text.pose.position.z = primary_enu.z() + 10.0;
    
    text.scale.z = 3.0;  // 文字大小
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    
    std::stringstream ss;
    ss << "DD: " << total_dd << " | Q: " << std::fixed << std::setprecision(2) << avg_quality;
    text.text = ss.str();
    
    marker_array.markers.push_back(text);
    pub_dd_info_text.publish(marker_array);
}

// ========== 原有函数保持不变 ==========

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    key_poses.id = 0;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);

    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_keyframe_pose.publish(odometry);

        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point.publish(point_cloud);
    }
}
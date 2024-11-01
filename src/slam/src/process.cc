#include <process.h>

namespace slam_czc
{
    // 仿真不考虑运动畸变
    bool Process::dataAlign()
    {
        if (pc_deque_.empty() || imu_deque_.empty())
        {
            return false;
        }

        if (!pc_pushed_)
        {
            sorted_data_.pc_ = pc_deque_.front();
            sorted_data_.pc_begin_time_ = static_cast<double>(sorted_data_.pc_->header.stamp) / 1e6;
            pc_pushed_ = true;
        }

        if (last_timestamp_imu_ <= sorted_data_.pc_begin_time_)
        {
            return false;
        }

        sorted_data_.imu_.clear();

        double imu_time = imu_deque_.front()->timestamp_;
        // 遍历imu，放入早于lidar的数据
        while ((!imu_deque_.empty()) && imu_time < sorted_data_.pc_begin_time_)
        {
            imu_time = imu_deque_.front()->timestamp_;
            sorted_data_.imu_.push_back(imu_deque_.front());
            imu_deque_.pop_front();
        }

        pc_pushed_ = false;
        pc_deque_.pop_front();

        dataProcess();
        return true;
    }

    bool Process::dataProcess()
    {
        if (!imu_init_flag) // imu初始化
        {
            initIMU();
            return true; // 初始化成功之前的数据舍弃
        }

        predictByIMU();
        // 不考虑运动畸变

        alignByPC();

        return true;
    }

    bool Process::alignByPC()
    {
        PointType::Ptr pc_trans(new PointType);
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = TIL_;
        transform.linear() = QIL_.toRotationMatrix();
        pcl::transformPointCloud(*sorted_data_.pc_, *pc_trans, transform);

        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(0.5, 0.5, 0.5);
        voxel.setInputCloud(*pc_trans);

        PonitType::Ptr pc_trans_filter(new PointType);
        voxel.filter(*pc_trans_filter);

        if (pc_first_flag_)
        {
            ndt_.setInputTarget(*pc_trans);
            imu_pre_ = std::make_shared<IMUPreintegration>(imu_init_.init_bg_, imu_init_.init_ba_, imu_init_.cov_gyro_[0], imu_init_.cov_acce_[0]);
            pc_first_flag_ = false;

            // 发送一次位姿到rviz
            updatePath(last_state_);
            return true;
        }

        ndt_.setInputSource(*pc_trans_filter);

        // 预测当前位姿
        current_state_ = imu_pre_->Predict(last_state_, imu_init.gravity_);

        ndt_pose_ = current_state_.getTransform();
        ndt_pose_ = ndt.align(ndt_pose_);

        optimize();
    }

    bool Process::optimize()
    {
        ceres::Problem problem;
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
        // 优化变量定义
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        // using BlockSolverType = g2o::BlockSolverX;
        // using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

        // auto *solver = new g2o::OptimizationAlgorithmLevenberg(
        //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        // g2o::SparseOptimizer optimizer;
        // optimizer.setAlgorithm(solver);

        // 优化变量转成数组
        vecter2double();

        problem.AddParameterBlock(current_q_, 4, q_parameterization);
        problem.AddParameterBlock(last_q_, 4, q_parameterization);

        problem.AddParameterBlock(last_bg_, 3);
        problem.AddParameterBlock(last_ba_, 3);
        // 如果不需要优化外参就设置为fix
        problem.SetParameterBlockConstant(last_bg_);
        problem.SetParameterBlockConstant(last_ba_);

        ceres::CostFunction *imu_factor = IMUFactor::Create(
            imu_pre_, imu_init_.gravity_);
        problem.AddResidualBlock(imu_factor, loss_function, last_q_, last_p_, last_v_, last_bg_, last_ba_, current_q_, current_p_, current_v_, current_bg_, current_ba_);

        ceres::CostFunction *marg_factor = MargFactor::Create(
            imu_pre_, imu_init_.gravity_);
        problem.AddResidualBlock(marg_factor, loss_function, last_q_, last_p_, last_v_, last_bg_, last_ba_);

        // 残差：imu得到的位资、lidar的位资
        Eigen::Matrix<double, 6, 6> ndt_info_ = Eigen::Matrix<double, 6, 6>::Identity(); // 后续需要优化
        ceres::CostFunction *pc_factor = PointCloudFactor::Create(
            ndt_pose_, ndt_info_);
        problem.AddResidualBlock(pc_factor, loss_function, current_q_, current_p_);
        options.linear_solver_type = ceres::DENSE_SCHUR;
        // options.num_threads = 2;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = NUM_ITERATIONS;
        // options.use_explicit_schur_complement = true;
        // options.minimizer_progress_to_stdout = true;
        // options.use_nonmonotonic_steps = true;

        options.max_solver_time_in_seconds = SOLVER_TIME;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary); // ceres优化求解

        // 更新状态
        Eigen::Map<Eigen::Quaterniond> current_q(current_q_);
        current_state_.q_ = current_q;
        current_state_.p_ = Eigen::Map<Eigen::Vector3d>(current_p_);
        current_state_.v_ = Eigen::Map<Eigen::Vector3d>(current_v_);
        current_state_.bg_ = Eigen::Map<Eigen::Vector3d>(current_bg_);
        current_state_.ba_ = Eigen::Map<Eigen::Vector3d>(current_ba_);
        last_state_ = current_state_;

        // 更新imu预积分
        imu_pre_ = std::make_shared<IMUPreintegration>(current_state_.bg_, current_state_.ba_, imu_init_.cov_gyro_[0], imu_init_.cov_acce_[0]);

        // 发送位姿到rviz
        updatePath(last_state_);
        pubPath_.publish(global_path);

        return true;
    }

}
void Process::updatePath(const State &state)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(state.timestamp_);
    pose_stamped.header.frame_id = odom_frame_id;
    pose_stamped.pose.position.x = state.p_.x();
    pose_stamped.pose.position.y = state.p_.y();
    pose_stamped.pose.position.z = state.p_.z();

    pose_stamped.pose.orientation.x = state.q_.x();
    pose_stamped.pose.orientation.y = state.q_.y();
    pose_stamped.pose.orientation.z = state.q_.z();
    pose_stamped.pose.orientation.w = state.q_.w();

    global_path.poses.push_back(pose_stamped);
}
bool Process::vecter2double()
{
    current_q_[0] = current_state_.q_.w();
    current_q_[1] = current_state_.q_.x();
    current_q_[2] = current_state_.q_.y();
    current_q_[3] = current_state_.q_.z();
    current_p_[0] = current_state_.p_.x();
    current_p_[1] = current_state_.p_.y();
    current_p_[2] = current_state_.p_.z();
    current_v_[0] = current_state_.v_.x();
    current_v_[1] = current_state_.v_.y();
    current_v_[2] = current_state_.v_.z();
    current_bg_[0] = current_state_.bg_.x();
    current_bg_[1] = current_state_.bg_.y();
    current_bg_[2] = current_state_.bg_.z();
    current_ba_[0] = current_state_.ba_.x();
    current_ba_[1] = current_state_.ba_.y();
    current_ba_[2] = current_state_.ba_.z();

    last_q_[0] = last_state_.q_.w();
    last_q_[1] = last_state_.q_.x();
    last_q_[2] = last_state_.q_.y();
    last_q_[3] = last_state_.q_.z();
    last_p_[0] = last_state_.p_.x();
    last_p_[1] = last_state_.p_.y();
    last_p_[2] = last_state_.p_.z();
    last_v_[0] = last_state_.v_.x();
    last_v_[1] = last_state_.v_.y();
    last_v_[2] = last_state_.v_.z();
    last_bg_[0] = last_state_.bg_.x();
    last_bg_[1] = last_state_.bg_.y();
    last_bg_[2] = last_state_.bg_.z();
    last_ba_[0] = last_state_.ba_.x();
    last_ba_[1] = last_state_.ba_.y();
    last_ba_[2] = last_state_.ba_.z();
    return true;
}

bool Process::predictByIMU()
{
    for (auto &imu : sorted_data_.imu_)
    {
        if (last_imu_ != nullptr)
        {
            imu_pre_->integrate(*imu, imu->timestamp_ - last_imu_->timestamp_);
        }
        last_imu_ = imu;
    }
    return true;
}

bool Process::initIMU()
{
    for (auto imu : sorted_data_.imu_)
    {
        imu_init_.addIMU(*imu);
    }
    if (imu_init_.init_success_)
    {
        ROS_INFO("IMU初始化成功");

        imu_pre_ = std::make_shared<IMUPreintegration>(imu_init_.init_bg_, imu_init_.init_ba_, imu_init_.cov_gyro_[0], imu_init_.cov_acce_[0]);

        imu_init_flag = false;
        current_state_.timestamp_ = sorted_data_.imu.back().timestamp_;
        current_state_.g_ = imu_init_.gravity_;
        last_state_ = current_state_;
        last_imu_ = sorted_data_.imu.back();
    }
    return true;
}

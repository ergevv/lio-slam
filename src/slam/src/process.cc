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
            last_key_state_.p_.x() = -p_thresh_ - 0.1; // 确保第一帧点云是关键帧
            return true;                               // 初始化成功之前的数据舍弃
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
        pcl::transformPointCloud(*sorted_data_.pc_, *pc_trans, transform); // 转到imu坐标系下

        pcl::VoxelGrid<Point> voxel; // 注意这里模板应该是点的类型
        voxel.setLeafSize(0.5, 0.5, 0.5);
        voxel.setInputCloud(pc_trans);

        PointType::Ptr pc_trans_filter(new PointType);
        voxel.filter(*pc_trans_filter);

        if (pc_first_flag_)
        {
            *pc_key_ += *pc_trans_filter;
            ndt_.setInputTarget(pc_key_);
            imu_pre_ = std::make_shared<IntegrationBase>(last_imu_->acce_, last_imu_->gyro_, imu_init_.init_ba_, imu_init_.init_bg_, imu_init_.cov_acce_n_, imu_init_.cov_gyro_n_, imu_init_.cov_acce_w_, imu_init_.cov_gyro_w_);
            pc_first_flag_ = false;

            // 发送一次位姿到rviz
            updatePath(last_state_);
            return true;
        }

        ndt_.setInputSource(pc_trans_filter);

        // 预测当前位姿
        current_state_ = imu_pre_->predict(last_state_, imu_init_.gravity_);

        PointType::Ptr output(new PointType);
        ndt_pose_ = current_state_.getTransform();
        ndt_.align(*output, ndt_pose_);
        ndt_pose_ = ndt_.getFinalTransformation(); // 这里都是面点，导致匹配不好，后续更改

        optimize();

        // 更新imu预积分

        imu_pre_ = std::make_shared<IntegrationBase>(last_imu_->acce_, last_imu_->gyro_, current_state_.ba_, current_state_.bg_, imu_init_.cov_acce_n_, imu_init_.cov_gyro_n_, imu_init_.cov_acce_w_, imu_init_.cov_gyro_w_);
        last_state_ = current_state_;
        // 发送位姿到rviz
        updatePath(last_state_);

        ros::Time time(current_state_.timestamp_);
        global_path_.header.stamp = time;
        global_path_.header.frame_id = map_frame_id_;
        pub_path_.publish(global_path_);

        // 发布map和base_link坐标系
        static tf::TransformBroadcaster br;
        tf::Transform send_tf;
        tf::Quaternion q;
        // body frame

        send_tf.setOrigin(tf::Vector3(current_state_.p_.x(),
                                      current_state_.p_.y(),
                                      current_state_.p_.z()));
        q.setW(current_state_.q_.w());
        q.setX(current_state_.q_.x());
        q.setY(current_state_.q_.y());
        q.setZ(current_state_.q_.z());
        send_tf.setRotation(q);

        br.sendTransform(tf::StampedTransform(send_tf, time, map_frame_id_, "base_link"));

        // 关键帧判断，添加点云
        Eigen::Vector3d delta_p = current_state_.p_ - last_key_state_.p_;
        Eigen::Vector3d delta_theta = Q2theta(last_key_state_.q_.inverse() * current_state_.q_);

        if (delta_p.norm() > p_thresh_ || delta_theta.norm() > q_thresh_ * 3.14 / 180.0)
        {
            ndt_pose_ = current_state_.getTransform();
            pcl::transformPointCloud(*pc_trans_filter, *output, ndt_pose_);
            *pc_key_ += *output;
            pcl::VoxelGrid<Point> voxel_key; // 注意这里模板应该是点的类型
            if (num_key_frame_ > 10)
            {
                leaf_size_ = 2;
            }

            voxel_key.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            voxel_key.setInputCloud(pc_key_);

            voxel_key.filter(*pc_key_);
            ndt_.setInputTarget(pc_key_);
            num_key_frame_++;
            last_key_state_ = current_state_;
        }

        return true;
    }

    bool Process::optimize()
    {
        ceres::Problem problem;
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(3.0);
        // 优化变量定义
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); // 虚数在前，实部在后
        // using BlockSolverType = g2o::BlockSolverX;
        // using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

        // auto *solver = new g2o::OptimizationAlgorithmLevenberg(
        //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        // g2o::SparseOptimizer optimizer;
        // optimizer.setAlgorithm(solver);

        // 优化变量转成数组
        vecter2double();

        problem.AddParameterBlock(last_p_, 3);
        problem.AddParameterBlock(last_q_, 4, q_parameterization);
        problem.AddParameterBlock(last_v_, 3);
        problem.AddParameterBlock(last_bg_, 3);
        problem.AddParameterBlock(last_ba_, 3);

        problem.AddParameterBlock(current_p_, 3);
        problem.AddParameterBlock(current_q_, 4, q_parameterization);
        problem.AddParameterBlock(current_v_, 3);
        problem.AddParameterBlock(current_bg_, 3);
        problem.AddParameterBlock(current_ba_, 3);

        if (marg_success_)
        {
            auto *marg_factor = new MargFactor(
                last_state_, Jr_, Er_, marg_weight_);
            problem.AddResidualBlock(marg_factor, loss_function, last_p_, last_q_, last_v_, last_bg_, last_ba_);
        }
        else
        {
            problem.SetParameterBlockConstant(last_p_);
            problem.SetParameterBlockConstant(last_q_);
            problem.SetParameterBlockConstant(last_v_);
            // 如果不需要优化外参就设置为fix
            problem.SetParameterBlockConstant(last_bg_);
            problem.SetParameterBlockConstant(last_ba_);
        }

        ceres::CostFunction *imu_factor = IMUFactor::Create(
            imu_pre_, imu_init_.gravity_);
        problem.AddResidualBlock(imu_factor, loss_function, last_q_, last_p_, last_v_, last_bg_, last_ba_, current_q_, current_p_, current_v_, current_bg_, current_ba_);

        // 残差：imu得到的位资、lidar的位资
        double gp2 = 0.1 * 0.1;
        double ga2 = (2.0 * 3.14 / 180) * (2.0 * 3.14 / 180);
        Eigen::Matrix<double, 6, 6> ndt_info_ = Eigen::Matrix<double, 6, 6>::Identity(); // 后续需要优化
        ndt_info_.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gp2;

        ceres::CostFunction *pc_factor = PointCloudFactor::Create(
            ndt_pose_.cast<double>(), ndt_info_);
        problem.AddResidualBlock(pc_factor, loss_function, current_q_, current_p_);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR; // 适用于稠密矩阵的 Schur 分解。
        // options.num_threads = 2;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 10;
        options.parameter_tolerance = 1e-6; // 参数变化小于1e-6时终止
        // options.use_explicit_schur_complement = true;
        // options.minimizer_progress_to_stdout = true;
        // options.use_nonmonotonic_steps = true;

        options.max_solver_time_in_seconds = 0.4; // 总体优化时间

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary); // ceres优化求解

        // 更新状态
        Eigen::Map<Eigen::Quaterniond> current_q(current_q_);
        current_state_.q_ = current_q;
        current_state_.p_ = Eigen::Map<Eigen::Vector3d>(current_p_);
        current_state_.v_ = Eigen::Map<Eigen::Vector3d>(current_v_);
        current_state_.bg_ = Eigen::Map<Eigen::Vector3d>(current_bg_);
        current_state_.ba_ = Eigen::Map<Eigen::Vector3d>(current_ba_);

        // 边缘化
        ceres::Problem::EvaluateOptions eval_options;
        ceres::CRSMatrix jacobian_crs_matrix;
        // eval_options.residuals = true; // 请求计算残差
        std::vector<double> residuals_std;

        problem.Evaluate(eval_options, nullptr, &residuals_std, nullptr, &jacobian_crs_matrix);

        Eigen::VectorXd residuals = Eigen::Map<Eigen::VectorXd>(residuals_std.data(), residuals_std.size());
        // 转换为Eigen稀疏矩阵并打印相关信息

        Eigen::SparseMatrix<double> J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);

        // 计算海森矩阵
        Eigen::VectorXd b = -J.transpose() * residuals;
        Eigen::MatrixXd H = J.transpose() * J;

        // 通过海森矩阵求取雅可比矩阵和残差
        marginalize(H, b, Jr_, Er_, 15);
        marg_success_ = true;
        return true;
    }

    void Process::updatePath(const State &state)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(state.timestamp_);
        pose_stamped.header.frame_id = map_frame_id_;
        pose_stamped.pose.position.x = state.p_.x();
        pose_stamped.pose.position.y = state.p_.y();
        pose_stamped.pose.position.z = state.p_.z();

        pose_stamped.pose.orientation.x = state.q_.x();
        pose_stamped.pose.orientation.y = state.q_.y();
        pose_stamped.pose.orientation.z = state.q_.z();
        pose_stamped.pose.orientation.w = state.q_.w();

        global_path_.poses.push_back(pose_stamped);
    }
    bool Process::vecter2double()
    {
        current_state_.q_.normalize();
        current_q_[0] = current_state_.q_.x();
        current_q_[1] = current_state_.q_.y();
        current_q_[2] = current_state_.q_.z();
        current_q_[3] = current_state_.q_.w();

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

        last_state_.q_.normalize();
        last_q_[0] = last_state_.q_.x();
        last_q_[1] = last_state_.q_.y();
        last_q_[2] = last_state_.q_.z();
        last_q_[3] = last_state_.q_.w();

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
                imu_pre_->propagate(imu->timestamp_ - last_imu_->timestamp_, imu->acce_, imu->gyro_);
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
            last_imu_ = sorted_data_.imu_.back();
            imu_pre_ = std::make_shared<IntegrationBase>(last_imu_->acce_, last_imu_->gyro_, imu_init_.init_ba_, imu_init_.init_bg_, imu_init_.cov_acce_n_, imu_init_.cov_gyro_n_, imu_init_.cov_acce_w_, imu_init_.cov_gyro_w_);

            imu_init_flag = true;
            current_state_.timestamp_ = sorted_data_.imu_.back()->timestamp_;
            current_state_.g_ = imu_init_.gravity_;
            last_state_ = current_state_;
        }
        return true;
    }

    Process::Process(std::string imu_topic, std::string pc_topic) : pc_key_(new PointType)
    {
        nh = ros::NodeHandle("~");
        sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 2000, &Process::processIMU, this, ros::TransportHints().tcpNoDelay()); // 频率100

        sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(pc_topic, 10, &Process::processPointCloud, this, ros::TransportHints().tcpNoDelay()); // 频率10

        pub_path_ = nh.advertise<nav_msgs::Path>("/path", 1);

        readParameters(nh);
        TIL_ = TIC;
        QIL_ = Eigen::Quaterniond(RIC);
        acc_n_ = ACC_N;
        acc_w_ = ACC_W;
        gyr_n_ = GYR_N;
        gyr_w_ = GYR_W;
        g_ = G;

        imu_init_.cov_acce_w_ = Eigen::Matrix3d::Identity() * acc_w_ * acc_w_;
        imu_init_.cov_gyro_w_ = Eigen::Matrix3d::Identity() * gyr_w_ * gyr_w_;
        imu_init_.options_.gravity_norm_ = g_.z();

        ndt_.setResolution(1.0);             // NDT网格分辨率
        ndt_.setMaximumIterations(35);       // 最大迭代次数
        ndt_.setTransformationEpsilon(0.01); // 收敛阈值
        ndt_.setStepSize(0.1);               // 梯度下降步长
        // ndt_.setNumThreads(4);               // 使用多线程加速
    }

}

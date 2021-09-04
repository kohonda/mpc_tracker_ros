#include "mpc_tracker/course_manager.hpp"

namespace pathtrack_tools
{
    CourseManager::CourseManager(/* args */)
    {
        nearest_index_ = 0;
        nearest_ratio_ = 1.0;
        second_nearest_index_ = 0;
        second_nearest_ratio_ = 0.0;
        current_pose_x_f_ = 0.0;
        hash_xf2index_ = {{0.0, 0}};
    }

    CourseManager::~CourseManager()
    {
    }

    int CourseManager::get_path_size() const
    {
        return mpc_course_.size();
    }

    MPCCourse CourseManager::get_mpc_course() const
    {
        return mpc_course_;
    }

    // Read the course information from the csv and set them to a member variable.
    void CourseManager::set_course_from_csv(const std::string &csv_path)
    {
        // csv read
        const rapidcsv::Document csv(csv_path);

        // get value from csv
        const std::vector<double> reference_x = csv.GetColumn<double>("reference.x");
        const std::vector<double> reference_y = csv.GetColumn<double>("reference.y");
        const std::vector<double> reference_speed = csv.GetColumn<double>("reference.speed");   // [m/s]
        const std::vector<double> drivable_delta_y_f = csv.GetColumn<double>("drivable_width"); // Mainly, cource width

        // Check all vectors have the same size and the size is not zero
        try
        {
            if (reference_x.size() == 0)
            {
                throw std::range_error("Course data size is zero!");
            }
            else if (reference_x.size() != reference_y.size() || reference_x.size() != reference_speed.size() || reference_x.size() != drivable_delta_y_f.size())
            {
                throw std::range_error("All course data size are not same!");
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // set value to reference_course_ with index
        mpc_course_.resize(reference_x.size());
        mpc_course_.x = reference_x;
        mpc_course_.y = reference_y;
        mpc_course_.speed = reference_speed;
        mpc_course_.drivable_width = drivable_delta_y_f;

        // set accumulated path length, which is nearly equal to x_f
        // offset = 0.0, which must be set previous xf when mpc_course is given sequentially.
        set_accumulated_path_length(0.0, &mpc_course_);

        // set curvature, which is nearly equal to x_f
        set_path_curvature(curvature_smoothing_num_, &mpc_course_);

        // set path yaw_g for using frenet2global
        set_path_yaw(&mpc_course_);

        // Filtering curvature and reference_speed based on curvature change rate
        filtering_based_curvature_rate(mpc_course_.accumulated_path_length, &mpc_course_.curvature, &mpc_course_.speed);

        // set lookup table covert from xf to nearest index
        set_hash_xf2index(mpc_course_, &hash_xf2index_);
    }

    void CourseManager::set_course_from_nav_msgs(const nav_msgs::Path &path, const double &reference_speed)
    {
        mpc_course_.clear();

        /*convert nav_msgs::path to mpc_course*/
        for (const auto &pose : path.poses)
        {
            mpc_course_.x.push_back(pose.pose.position.x);
            mpc_course_.y.push_back(pose.pose.position.y);
            mpc_course_.z.push_back(pose.pose.position.z);
            mpc_course_.yaw.push_back(tf2::getYaw(pose.pose.orientation));
            mpc_course_.speed.push_back(reference_speed);
        }

        // set accumulated path length, which is nearly equal to x_f
        // offset = 0.0, which must be set previous xf when mpc_course is given sequentially.
        // TODO: ここをどうするか．
        set_accumulated_path_length(0.0, &mpc_course_);

        // set curvature, which is nearly equal to x_f
        set_path_curvature(curvature_smoothing_num_, &mpc_course_);

        // set path yaw_g for using frent2global
        set_path_yaw(&mpc_course_);

        // Filtering curvature and reference_speed based on curvature change rate
        filtering_based_curvature_rate(mpc_course_.accumulated_path_length, &mpc_course_.curvature, &mpc_course_.speed);

        // set lookup table covert from xf to nearest index
        set_hash_xf2index(mpc_course_, &hash_xf2index_);
    }

    // offset = Updated first reference x_f
    void CourseManager::set_accumulated_path_length(const double &offset, MPCCourse *mpc_course)
    {
        if (mpc_course->size() == 0)
        {
            std::cerr << "[Warning] Reference Course size is zero!" << std::endl;
        }

        mpc_course->accumulated_path_length = calc_accumulated_path_length(offset, *mpc_course);
    }

    // Calculate the distance to the reference point by interpolating line segments between each reference point.
    std::vector<double> CourseManager::calc_accumulated_path_length(const double &offset, const MPCCourse &mpc_course) const
    {
        std::vector<double> accumulated_path_length(mpc_course.x.size());
        for (size_t i = 0; i < accumulated_path_length.size(); i++)
        {
            // calculate accumulated path length, which is nearly equal to x_f
            if (i == 0)
            {
                accumulated_path_length[0] = offset;
            }
            else
            {
                const double delta_length = std::hypot(mpc_course.x[i] - mpc_course.x[i - 1], mpc_course.y[i] - mpc_course.y[i - 1]);
                accumulated_path_length[i] = accumulated_path_length[i - 1] + delta_length;
            }
        }

        return accumulated_path_length;
    }

    void CourseManager::set_path_curvature(const int &smoothing_num, MPCCourse *mpc_course)
    {
        if (mpc_course->size() == 0)
        {
            std::cerr << "[Warning] Reference Course size is zero!" << std::endl;
        }
        mpc_course->curvature = calc_path_curvature(smoothing_num, *mpc_course);
    }

    // Now, Only used for frenet2global coordinate
    void CourseManager::set_path_yaw(MPCCourse *mpc_course)
    {
        for (int i = 0; i < mpc_course->size() - 2; i++)
        {
            Eigen::Vector2d p(mpc_course->x.at(i), mpc_course->y.at(i));
            Eigen::Vector2d p_next(mpc_course->x.at(i + 1), mpc_course->y.at(i + 1));

            const auto delta_vec = p_next - p;

            mpc_course->yaw.at(i) = std::atan2(delta_vec(1), delta_vec(0));
        }
        mpc_course->yaw.at(mpc_course->size() - 1) = mpc_course->yaw.at(mpc_course->size() - 2);
    }

    // Calculate the curvature at each reference point.
    // Fitted with a circle using three points around a few points to make the curvature smooth.
    std::vector<double> CourseManager::calc_path_curvature(const int &smoothing_num, const MPCCourse &mpc_course) const
    {
        const int path_size = mpc_course.x.size();
        std::vector<double> curvature_vec(path_size);

        const int max_smoothing_num = std::floor(0.5 * (path_size - 1));
        const int L = std::min(max_smoothing_num, smoothing_num);

        // calculate the curvature outside the edge of the reference path
        // when counterclockwise, curvature ρ > 0 ,
        // when the path is clockwise, curvature ρ < 0 ,
        // when the path is straight, curvature ρ = 0
        for (int i = L; i < path_size - L; i++)
        {
            const Eigen::Vector2d p1(mpc_course.x[i - L], mpc_course.y[i - L]);
            const Eigen::Vector2d p2(mpc_course.x[i], mpc_course.y[i]);
            const Eigen::Vector2d p3(mpc_course.x[i + L], mpc_course.y[i + L]);
            const double denominator = std::max((p1 - p2).norm() * (p2 - p3).norm() * (p3 - p1).norm(), 0.0001);

            const double curvature = 2.0 * ((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])) / denominator;
            curvature_vec[i] = curvature;
        }

        // Caluculate the curvature except the edge
        for (int i = 0; i < std::min(path_size, L); i++)
        {
            curvature_vec[i] = curvature_vec[std::min(L, path_size - 1)];
            curvature_vec[path_size - i - 1] = curvature_vec[std::max(path_size - L - 1, 0)];
        }

        return curvature_vec;
    }

    double CourseManager::calc_distance(const std::array<double, 2> &p1, const std::array<double, 2> &p2) const
    {
        return std::hypot(p1[0] - p2[0], p1[1] - p2[1]);
    }

    // Saturate the rate of curvature change when curvature_change_rate > max_curvature_change_rate
    void CourseManager::filtering_based_curvature_rate(const std::vector<double> &accumulated_path_length, std::vector<double> *path_curvature, std::vector<double> *ref_speed)
    {
        bool is_filtered = false;

        // return (+1 or -1 or 0) according to the sign of x value
        auto sign = [](const double &x) { return (x > 0) - (x < 0); };

        for (size_t i = 1; i < path_curvature->size(); i++)
        {
            const double delta_xf = accumulated_path_length[i] - accumulated_path_length[i - 1];
            const double delta_curvature = path_curvature->at(i) - path_curvature->at(i - 1);
            const double curvature_change_rate = delta_curvature / std::max(0.00001, delta_xf);

            // filtering path curvature
            if (std::abs(curvature_change_rate) > max_curvature_change_rate_)
            {
                path_curvature->at(i) = path_curvature->at(i - 1) + sign(delta_curvature) * max_curvature_change_rate_ * delta_xf;

                is_filtered = true;
            }

            // filtering reference speed
            // v_ref_filtered = v_ref * exp (-speed_reduction_rate * curvature_rate^2)
            ref_speed->at(i) = ref_speed->at(i) * std::exp(-speed_reduction_rate_ * curvature_change_rate * curvature_change_rate);
        }
        if (is_filtered)
        {
            std::cout << "Filtered mpc_course curvature because path curvature change rate is too big." << std::endl;
        }
    }

    void CourseManager::set_hash_xf2index(const MPCCourse &mpc_course, std::unordered_map<double, int> *hash_xf2index)
    {
        hash_xf2index->clear();
        const double start_xf = round_resolution(mpc_course.accumulated_path_length.front() - redundant_xf_);
        const double end_xf = round_resolution(mpc_course.accumulated_path_length.back());

        int last_nearest_index = 0;
        for (double xf = start_xf; xf < end_xf; xf += hash_resolution_)
        {
            const int nearest_index = search_nearest_index(mpc_course, xf, last_nearest_index);
            last_nearest_index = nearest_index;
            const double round_xf = round_resolution(xf);
            hash_xf2index->emplace(round_xf, nearest_index);
        }
    }

    double CourseManager::round_resolution(const double &raw_x_f) const
    {
        // return the x_f value rounded by the resolution
        const double round_x_f = hash_resolution_ * std::round(raw_x_f / hash_resolution_);
        return round_x_f;
    }

    double CourseManager::get_curvature(const double &pose_x_f)
    {
        // To avoid the same calculation many times in the same prediction step
        if (pose_x_f != current_pose_x_f_)
        {
            current_pose_x_f_ = pose_x_f;

            nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

            // Calculate internal ratio of pose_x_f between nearest and second_nearest
            set_internal_ratio(mpc_course_, pose_x_f, nearest_index_);
        }

        const double curvature_interpolated = linear_interpolate(mpc_course_.curvature, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);

        return curvature_interpolated;
    }

    double CourseManager::get_speed(const double &pose_x_f)
    {
        // To avoid the same calculation many times in the same prediction step
        if (pose_x_f != current_pose_x_f_)
        {
            current_pose_x_f_ = pose_x_f;

            nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

            // Calculate internal ratio of pose_x_f between nearest and second_nearest
            set_internal_ratio(mpc_course_, pose_x_f, nearest_index_);
        }

        const double reference_speed_interpolated = linear_interpolate(mpc_course_.speed, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);
        return reference_speed_interpolated;
    }

    double CourseManager::get_drivable_width(const double &pose_x_f)
    {
        // To avoid the same calculation many times in the same prediction step
        if (pose_x_f != current_pose_x_f_)
        {
            current_pose_x_f_ = pose_x_f;

            nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

            // Calculate internal ratio of pose_x_f between nearest and second_nearest
            set_internal_ratio(mpc_course_, pose_x_f, nearest_index_);
        }

        const double drivable_width_interpolated = linear_interpolate(mpc_course_.drivable_width, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);
        return drivable_width_interpolated;
    }

    int CourseManager::search_nearest_index(const MPCCourse &mpc_course, const double &pose_x_f, const int &last_nearest_index) const noexcept
    {
        // NOTE : Not consider z and yaw
        // If the reference path is ever crossed, consider z and yaw
        // For example, compare yaw and reference_yaw, and ignore if they're too big or too close.
        int tmp_nearest_index = last_nearest_index;

        for (int i = 0; i < mpc_course.size(); i++)
        {
            const int prev_index = std::max(0, tmp_nearest_index - 1);
            const int next_index = std::min(tmp_nearest_index + 1, mpc_course.size() - 1);

            const auto dist_to_prev = std::abs(pose_x_f - mpc_course.accumulated_path_length[prev_index]);
            const auto dist_to_last = std::abs(pose_x_f - mpc_course.accumulated_path_length[tmp_nearest_index]);
            const auto dist_to_next = std::abs(pose_x_f - mpc_course.accumulated_path_length[next_index]);

            if (dist_to_last <= dist_to_prev && dist_to_last <= dist_to_next)
            {
                const int nearest_index = tmp_nearest_index;
                return nearest_index;
            }
            else
            {
                if (dist_to_prev < dist_to_next)
                {
                    tmp_nearest_index = prev_index;
                }
                else if (dist_to_prev > dist_to_next)
                {
                    tmp_nearest_index = next_index;
                }
            }
        }

        return -1;
    }

    // Not used now
    int CourseManager::search_nearest_index(const MPCCourse &mpc_course, const Pose &pose) const
    {
        if (mpc_course.size() == 0)
        {
            std::cerr << "[Warning] Reference Cource size is zero!" << std::endl;
            return -1;
        }

        int nearest_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < mpc_course.size(); i++)
        {
            const auto dx = mpc_course.x[i] - pose.x;
            const auto dy = mpc_course.y[i] - pose.y;
            const auto dist = dx * dx + dy * dy;

            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_index = i;
            }
        }

        return nearest_index;
    }

    int CourseManager::lookup_xf_to_nearest_index(const std::unordered_map<double, int> &hash_xf2index, const double &pose_x_f) const
    {
        const double round_x_f = round_resolution(pose_x_f);

        int nearest_index = 0;

        try
        {
            nearest_index = hash_xf2index.at(round_x_f);
        }
        catch (const std::out_of_range &ex)
        {
            std::cerr << "[Warning] Out of range of Reference cource. Plsease set longer mpc_course. " << std::endl;
            nearest_index = mpc_course_.size() - 1;
        }

        return nearest_index;
    }

    void CourseManager::set_internal_ratio(const MPCCourse &mpc_course, const double &pose_x_f, const int &nearest_index)
    {
        // calculate second nearest index based difference from x_f
        const int prev_index = std::max(0, nearest_index - 1);
        const int next_index = std::min(nearest_index + 1, mpc_course.size() - 1);
        const auto diff_nearest = pose_x_f - mpc_course_.accumulated_path_length[nearest_index];
        if (diff_nearest < 0)
        {
            // vehicle is between nearest and prev point
            second_nearest_index_ = prev_index;
        }
        else if (diff_nearest > 0)
        {
            // vehicle is between nearest and next point
            second_nearest_index_ = next_index;
        }
        else if (diff_nearest == 0)
        {
            // vehicle is on the nearest point
            second_nearest_index_ = nearest_index_;
        }

        // Calculate internal ratio of pose_x_f between nearest and second_nearest
        const double dist_nearest_and_second = std::abs(mpc_course_.accumulated_path_length[second_nearest_index_] - mpc_course_.accumulated_path_length[nearest_index_]);
        // nearest と second nearestが近すぎるときのnanを防ぐy
        if (dist_nearest_and_second < 1.0e-5)
        {
            nearest_ratio_ = 0.0;
            second_nearest_ratio_ = 1.0;
        }
        else
        {
            const double dist_to_nearest = std::abs(mpc_course_.accumulated_path_length[nearest_index] - pose_x_f);
            const double dist_to_second_nearest = std::abs(mpc_course_.accumulated_path_length[second_nearest_index_] - pose_x_f);
            nearest_ratio_ = dist_to_nearest / dist_nearest_and_second;
            second_nearest_ratio_ = dist_to_second_nearest / dist_nearest_and_second;
        }
    }

    double CourseManager::linear_interpolate(const std::vector<double> &reference_vec, const int &nearest_index, const double &nearest_ratio, const int &second_nearest_index,
                                             const double &second_nearest_ratio) const noexcept
    {
        return second_nearest_ratio * reference_vec[nearest_index] + nearest_ratio * reference_vec[second_nearest_index];
    }
} // namespace pathtrack_tools

#include "main.hpp"
#include <iomanip>
#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#if defined(_WIN32)
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#define GETCWD _getcwd
#else
#include <sys/stat.h>
#include <sys/types.h>
#define MKDIR(path) mkdir(path, 0755)
#define GETCWD getcwd
#endif

namespace
{
    std::string get_current_time()
    {
        std::time_t now = std::time(nullptr);
        std::tm *local_time = std::localtime(&now);

        std::ostringstream stream;
        stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S");
        return stream.str();
    }

    std::string get_current_working_directory()
    {
        char buffer[FILENAME_MAX];
        std::string res = " ";
        if (GETCWD(buffer, FILENAME_MAX))
        {
            res = std::string(buffer);
        }
        else
        {
            throw std::runtime_error("Failed to get current working directory");
        }

        return res;
    }
}

template <typename T>
void SensorTracker<T>::reset()
{
    curr_timestamp = 0.0;
    predict_cov_ts = 0.0;
    info_count = 0;
    lidar_frames = 0;
    imu_frames = 0;
}

// Print function
template <typename T>
void SensorTracker<T>::print() const
{
    std::string separator(30, '-');
    std::cout << separator << std::endl;
    std::cout << "| " << std::setw(25) << std::left << "Attribute" << " | " << std::setw(8) << "Value" << " |" << std::endl;
    std::cout << separator << std::endl;

    std::cout << "| " << std::setw(25) << "Current Timestamp" << " | " << std::setw(8) << std::fixed << std::setprecision(2) << curr_timestamp << " |" << std::endl;
    std::cout << "| " << std::setw(25) << "Predict Covariance Timestamp" << " | " << std::setw(8) << predict_cov_ts << " |" << std::endl;
    std::cout << "| " << std::setw(25) << "Info Count" << " | " << std::setw(8) << info_count << " |" << std::endl;
    std::cout << "| " << std::setw(25) << "Lidar Frames" << " | " << std::setw(8) << lidar_frames << " |" << std::endl;
    std::cout << "| " << std::setw(25) << "IMU Frames" << " | " << std::setw(8) << imu_frames << " |" << std::endl;

    std::cout << separator << std::endl;
}

template struct SensorTracker<double>;
template class SensorTracker<float>;

template <typename T>
CFLIO<T>::CFLIO()
{
    // initialize parameters
    init.init_params(nh, params<T>);

    acq = std::make_shared<DataAcquisition<T>>(nh);

    kf_interface = std::make_shared<EstimationInterface<T>>();

    viz = std::make_shared<Visualizer<T>>(nh);
}

template <typename T>
void CFLIO<T>::predict_stage(Frame::BasePtr<T> &frame, bool imu_data)
{
    kf_tracker.curr_timestamp = frame->timestamp;
    T dt_pred = kf_tracker.curr_timestamp - kf_tracker.predict_cov_ts;

    if (dt_pred > 0.0)
    {
        kf_interface->kf->process(dt_pred, true, true, imu_data);
        kf_tracker.predict_cov_ts = kf_tracker.curr_timestamp;
    }
}

template <typename T>
void CFLIO<T>::run_imu(Frame::BasePtr<T> &frame)
{
    ++kf_tracker.imu_frames;
    predict_stage(frame, true);

    auto ptr = Frame::Imu<T>::cast(frame);
    kf_interface->imu_run(ptr->acc, ptr->gyro);

    if (params<T>.verbose)
    {
        std::cout << "\nMeasured Acc: " << Point3d<T>::eig_to_string(ptr->acc) << std::endl;
        std::cout << "Measured Gyro: " << Point3d<T>::eig_to_string(ptr->gyro) << std::endl;
    }
}

template <typename T>
void CFLIO<T>::run_lidar(Frame::BasePtr<T> frame, bool run_ops)
{
    if (frame)
    {
        ++kf_tracker.lidar_frames;

        if (first_subframe) // typically marks first subframe frame in lidar frame
        {
            first_subframe = false;
            kf_interface->update_base_timestamp(frame->timestamp);
            kf_tracker.curr_timestamp = frame->timestamp;
        }

        auto ptr = Frame::Lidar<T>::cast(frame);

        // stacks the lidar frames
        kf_interface->lidar_frame_stack(ptr);

        if (!run_ops)
            return;

        if (!init_frame)
        {
            init_frame = kf_interface->build_init();
            ROS_WARN("Lidar Frames Initialized");

            kf_tracker.predict_cov_ts = frame->timestamp;
            first_subframe = true;
            return;
        }

        // predicting step
        predict_stage(frame, false);
    }

    first_subframe = true;

    // run predict, update, visualization if necessary steps
    bool full_visualization = false;
    if (params<T>.visualize_map)
        full_visualization = main_frame_track <= 1 || main_frame_track % params<T>.full_map_interval == 0;

    if (!kf_interface->lidar_run(full_visualization))
        return; // nothing to run or visualize

    // visualize the data
    if (params<T>.visualize_subframes || full_visualization)
        viz->new_data(kf_interface->cycle_tracker, true);
}

template <typename T>
void CFLIO<T>::run_with_imu(Frame::BasePtr<T> &frame, bool run_ops)
{
    if (frame->type == SensorType::Imu)
    {
        // run if any lidar has been stacked
        if (init_frame)
            run_lidar(nullptr, true);

        // Updating time parameters
        run_imu(frame);
        return;
    }

    // regular run
    if (frame->type == SensorType::Lidar)
        run_lidar(frame, run_ops);
}

template <typename T>
void CFLIO<T>::establish_path()
{

    const std::string directory_name = params<T>.result_folder.empty() ? get_current_working_directory() + "/result" : params<T>.result_folder;
    struct stat info;

    if (stat(directory_name.c_str(), &info) != 0)
    {
        if (MKDIR(directory_name.c_str()) == 0)
            std::cout << "Directory created: " << directory_name << std::endl;
        else
            std::cerr << "Failed to create directory: " << directory_name << std::endl;
    }
    else if (info.st_mode & S_IFDIR)
    {
        std::cout << "Directory already exists: " << directory_name << std::endl;
    }
    else
    {
        std::cerr << "A file with the name 'result' exists but is not a directory." << std::endl;
    }

    std::string current_time = get_current_time();
    file_path = directory_name + "/stored_pose_info_" + current_time + ".txt";
    std::cout << "Pose Results stored at: " << file_path << std::endl;
}

template <typename T>
void CFLIO<T>::store_state_info()
{
    std::ofstream file;
    file.open(file_path, std::ios_base::app);

    std::ostringstream content;
    content << "\nFrame_id: " << main_frame_track << std::endl;
    content << kf_interface->state->to_yaml() << std::endl;

    if (file.is_open())
    {
        file << content.str();
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }
}

template <typename T>
void CFLIO<T>::run_script()
{
    ros::Rate rate(params<T>.imu.freq);

    while (ros::ok())
    {
        ros::spinOnce();
        auto data_frame = acq->data_sync();
        if (!data_frame.valid)
        {
            rate.sleep();
            continue;
        }

        ++main_frame_track;
        auto subframe = data_frame.get_next();

        if (first_frame)
        {
            kf_interface->init_orientation();
            viz->set_id_info(acq->imu_header_id, acq->pc_header_id);
            kf_tracker.predict_cov_ts = subframe->timestamp;
            first_frame = false;

            if (params<T>.store_state)
                establish_path();
        }

        while (subframe)
        {
            if ((subframe->timestamp - kf_tracker.curr_timestamp) < 0)
            {
                subframe = data_frame.get_next();
                continue;
            }

            if (!params<T>.imu_en)
                run_lidar(subframe, data_frame.empty());
            else
                run_with_imu(subframe, data_frame.empty());

            // next frame
            subframe = data_frame.get_next();
        }

        kf_interface->end_of_cycle_ops();
        if (params<T>.store_state)
            store_state_info();

        if (params<T>.max_runs > 0) // assumed testing
        {
            std::cout << "\nCurrent state at iteration: " << main_frame_track << std::endl;
            kf_interface->state->print();

            // for testing limited number of runs
            if (main_frame_track == params<T>.max_runs)
                break;
        }

        rate.sleep();
    }
}

template class CFLIO<double>;
template class CFLIO<float>;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coarse_fine_slam");

    CFLIO<double> op;
    op.run_script();

    std::cout << "End of Run" << std::endl;
    ros::shutdown();
    return 0;
}
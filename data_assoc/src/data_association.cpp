#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <vector>
#include <map>
#include "kinematics/Transformation.hpp"

typedef double TimeStamp;

/**
 * @brief       Struct storing the ground truth transformation of the vicon object
 *
 * @param[in]   tx ty tz           The translation of the vicon object
 * @param[in]   qx qy qz qw        The rotation of the vicon object expressed as a quaternoin
 *    
 */
struct GroundTruthRecord
{
    GroundTruthRecord() {};
    GroundTruthRecord(double tx, double ty, double tz, double qx, double qy, double qz, double qw) : tx(tx), ty(ty),
                                                                                                     tz(tz), qx(qx),
                                                                                                     qy(qy), qz(qz),
                                                                                                     qw(qw) { }

    GroundTruthRecord(Eigen::Vector3d translation, Eigen::Quaterniond rotation) :   tx(translation.x()), ty(translation.y()), 
                                                                                    tz(translation.z()),
                                                                                    qx(rotation.x()), qy(rotation.y()), 
                                                                                    qz(rotation.z()), qw(rotation.w()) {  }
    double tx, ty, tz, qx, qy, qz, qw;
};

typedef std::map<TimeStamp, std::string> TimeStampDataMap;
typedef std::map<TimeStamp, GroundTruthRecord> GroundTruthDataMap;
typedef std::map<std::string, GroundTruthRecord> RGBGroundTruthAssociationMap;

/**
 * @brief       Calculates the fraction a given sample is located between the closest previous and next sample
 *
 * @param[in]   previous_closest_diff       Closest difference of the previous sample to a given sample
 * @param[in]   next_closest_diff           Closest difference of the next sample to a given sample
 *    
 * @return      Fraction
 */
inline double calculateAlpha(   double previous_closest_diff, 
                                double next_closest_diff) 
{
    double alpha = previous_closest_diff / (previous_closest_diff - next_closest_diff);
    return alpha;
};

/**
 * @brief      Linear 3D interpolation
 *
 * @param[in]  previous_vector3D  The previous vector 3D
 * @param[in]  next_vector3D      The next vector 3D
 * @param[in]  alpha              Fraction
 *
 * @return     Interpolated translation
 */
inline Eigen::Vector3d interpolateTranslation(  const Eigen::Vector3d  &previous_vector3D, 
                                                const Eigen::Vector3d  &next_vector3D, 
                                                double                 alpha) {
    return previous_vector3D + alpha * (next_vector3D - previous_vector3D);
};

/**
 * @brief      Slerp interpolation for quaterniond
 *
 * @param[in]  previous_orientation  The previous orientation
 * @param[in]  next_orientation      The next orientation
 * @param[in]  alpha                 Fraction
 *
 * @return     Interpolated orientation
 */
inline Eigen::Quaterniond interpolateOrientation( const Eigen::Quaterniond    &previous_orientation, 
                                                  const Eigen::Quaterniond    &next_orientation, 
                                                  double                      alpha) {
    return previous_orientation.slerp(alpha, next_orientation);
};

/**
 * @brief      Interpolation for transformations
 *
 * @param[in]  previous_transformation  The previous transformation
 * @param[in]  next_transformation      The next transformation
 * @param[in]  alpha                    Fraction
 *
 * @return     Interpolated transformation
 */
inline GroundTruthRecord interpolateTransformation( const GroundTruthRecord     &previous_transformation,
                                                    const GroundTruthRecord     &next_transformation,
                                                    double                      alpha){
    Eigen::Vector3d previous_translation(previous_transformation.tx, previous_transformation.ty, previous_transformation.tz);
    Eigen::Vector3d next_translation(next_transformation.tx, next_transformation.ty, next_transformation.tz);

    Eigen::Quaterniond previous_rotation(previous_transformation.qx, previous_transformation.qy, previous_transformation.qz, previous_transformation.qw);
    Eigen::Quaterniond next_rotation(next_transformation.qx, next_transformation.qy, next_transformation.qz, next_transformation.qw);


    Eigen::Vector3d interpolated_translation = interpolateTranslation(previous_translation, next_translation, alpha);
    Eigen::Quaterniond interpolated_rotation = interpolateOrientation(previous_rotation, next_rotation, alpha);

    return GroundTruthRecord(interpolated_translation, interpolated_rotation);
};

/**
 * @brief      Computes the associated translation to a given rgb image based on the closest previous and next translation sample
 *
 * @param[in]  map                      The mapping between time stamps and ground truth transformations
 * @param[in]  val                      The time stamp of the given rgb image
 * @param[in]  rgb_image_path           The path rgb image
 * @param[in]  offset                   The const time added the ground truth translation time stamp
 * @param[in]  max_diff                 The max alloed time difference between the closest ground truth transformation and the rgb image time stamp  
 *
 * @return     Pair with rgb images with their associated transformation and a valid state
 */
std::pair<bool, std::pair<std::string, GroundTruthRecord>>  computeAssociatedTranslation( GroundTruthDataMap  map, 
                                                                                          TimeStamp           val, 
                                                                                          std::string         rgb_image_path, 
                                                                                          TimeStamp           offset, 
                                                                                          TimeStamp           max_diff) {
    double previous_closest_diff = max_diff;
    double next_closest_diff = -max_diff;
    bool has_pair = false;
    bool has_previous = false;
    bool has_next = false;
    GroundTruthRecord previous_closest;
    GroundTruthRecord next_closest;
    TimeStamp previous_time_stamp(0), next_time_stamp(0);

    // std::cout << "==============================" << std::endl;
    // std::cout << "RGB time stamp: " << std::fixed << std::setprecision(6) << val << std::endl;

    for (auto i : map) {
        double diff = val - (i.first + offset);

        // Search closest previous ground truth sample (positive value)
        if (diff < previous_closest_diff && diff >= 0) {
            has_previous = true;
            previous_closest = i.second;
            previous_closest_diff = diff;
            previous_time_stamp = i.first;
        }

        // Search closest next ground truth sample (negative value)
        if (diff > next_closest_diff && diff <= 0) {
            has_next = true;
            next_closest = i.second;
            next_closest_diff = diff;
            next_time_stamp = i.first;
        }
    }

    GroundTruthRecord interpolated_ground_truth(0,0,0,0,0,0,1);

    if (has_previous && has_next) {
        has_pair = true;
        double alpha = calculateAlpha(previous_closest_diff, next_closest_diff);
        interpolated_ground_truth = interpolateTransformation(previous_closest, next_closest, alpha);
    } else {
        std::cout << "Skipped image " << rgb_image_path << std::endl;
    } 



    return std::pair<bool, std::pair<std::string, GroundTruthRecord>>(has_pair, std::pair<std::string, GroundTruthRecord>(rgb_image_path, interpolated_ground_truth));
}

/*
 * Find the best pairing of rgb and depth images, with max_diff.
 */
RGBGroundTruthAssociationMap associate( TimeStampDataMap    &rgb_images, 
                                        GroundTruthDataMap  &ground_truth, 
                                        double max_diff) {
    RGBGroundTruthAssociationMap associations;
    double offset = 0;

    for (auto rgb_image : rgb_images) {
        std::pair<bool, std::pair<std::string, GroundTruthRecord>> interpolated_ground_truth = computeAssociatedTranslation(ground_truth, rgb_image.first, rgb_image.second, offset, max_diff);

        if(interpolated_ground_truth.first)
            associations.insert(interpolated_ground_truth.second);
    }

    return associations;
}

int main(int argc, char** argv) {

    double max_diff;
    cv::Mat dist_param(4,1,CV_32FC1,0.0f);
    cv::Mat camera_param = cv::Mat::eye(3, 3, CV_32FC1);  

    bool undistort_rgb_images = false; 

    // Check input arguments from terminal
    boost::program_options::options_description description("Usage:");
    description.add_options()   ("help,h", "Display this help message")
                                ("input,i", boost::program_options::value<std::string>(), "Input Bag file")
                                ("output,o", boost::program_options::value<std::string>(), "Path for dataset output")
                                ("maxdiff,m", boost::program_options::value<double>(), "Input maximum time stamp difference between rgb image and ground truth")
                                ("distparam,d", boost::program_options::value<std::string>(), "Input distortion parameter d1,d2,d3,d4")
                                ("cameraparam,k", boost::program_options::value<std::string>(), "Input camera parameter k1,k2,k3,k4");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(description).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << description << std::endl;
    }

    if (!vm.count("input")) {
        std::cout << "Input Bag file not specified" << std::endl;
        return -1;
    }

    if (!vm.count("output")) {
        std::cout << "Path for dataset output not specified" << std::endl;
        return -1;
    }

    if (!vm.count("maxdiff")) {
        std::cout << "Set maximum sample difference to default value max_diff = 0.02" << std::endl;
        max_diff = 0.02;
    } else {
        max_diff = vm["maxdiff"].as<double>();
    }

    if (!vm.count("distparam") || !vm.count("cameraparam")) {
        std::cout << "Image undistortion deactivated" << std::endl;
    } else {
        undistort_rgb_images = true;
        std::string s;

        std::string dist_arg = vm["distparam"].as<std::string>();
        std::istringstream dist_arg_stream(dist_arg);
        if (getline(dist_arg_stream, s, ',')) {
            dist_param.at<float>(0,0) = atof(s.c_str());
            if (getline(dist_arg_stream, s, ',')) {
                dist_param.at<float>(1,0) = atof(s.c_str());
                if (getline(dist_arg_stream, s, ',')) {
                    dist_param.at<float>(2,0) = atof(s.c_str());
                    if (getline(dist_arg_stream, s, ',')) {
                        dist_param.at<float>(3,0) = atof(s.c_str());
                    }
                } 
            } 
        }
        std::cout << "Set distortion parameter to dist_param = " << "\n" << dist_param << std::endl;

        std::string camera_arg = vm["cameraparam"].as<std::string>();
        std::istringstream camera_arg_stream(camera_arg);
        if (getline(camera_arg_stream, s, ',')) {
            camera_param.at<float>(0,0) = atof(s.c_str());
            if (getline(camera_arg_stream, s, ',')) {
                camera_param.at<float>(1,1) = atof(s.c_str());
                if (getline(camera_arg_stream, s, ',')) {
                    camera_param.at<float>(0,2) = atof(s.c_str());
                    if (getline(camera_arg_stream, s, ',')) {
                        camera_param.at<float>(1,2) = atof(s.c_str());
                    }
                } 
            } 
        }
        std::cout << "Set camera parameter to camera_param = " << "\n" << camera_param << std::endl;
    }

    // Bag filename and dataset folder
    const std::string rosbag_path = vm["input"].as<std::string>();
    const std::string dataset_folder = vm["output"].as<std::string>();

    // Topic names
    const std::string topic_rgb = "/camera/color/image_raw";
    const std::string topic_vicon_camera = "/vicon/d435/d435";

    // Filenames
    const std::string rgb_image_save_path = dataset_folder + "rgb/";
    const std::string rgb_ground_truth_association_filename = dataset_folder + "rgb_ground_truth_association.txt";

    // Create Dataset folder
    boost::filesystem::path directory_to_be_created(dataset_folder.c_str());
    if (!boost::filesystem::exists(directory_to_be_created)) {
        boost::filesystem::create_directory(directory_to_be_created);
    }

    // Create RGB folder
    directory_to_be_created = boost::filesystem::path(rgb_image_save_path.c_str());
    if (!boost::filesystem::exists(directory_to_be_created)) {
        boost::filesystem::create_directory(directory_to_be_created);
    }

    std::cout << "Opening " << rosbag_path << " file" << std::endl;
    rosbag::Bag bag_read;

    // Make sure that path exists, hardcode it for now
    bag_read.open(rosbag_path, rosbag::bagmode::Read);
    std::cout << "Opening " << rosbag_path << " file -> Done!" << std::endl;

    // Create a view on a bag
    rosbag::View view_all(bag_read);
    std::cout << "Converting rgb data to .png" << std::endl;

    // Open files
    std::ofstream rgb_ground_truth_association_file;
    rgb_ground_truth_association_file.open(rgb_ground_truth_association_filename);

    // Vicon object - Camera Transformation
    const okvis::kinematics::Transformation T_V_C(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

    TimeStampDataMap rgb_map;
    GroundTruthDataMap ground_truth_map;

    // Convert ROS data to TUM format
    for (const rosbag::MessageInstance m : view_all) {

        // Process rgb data
        if (m.getTopic() == topic_rgb) {
            const sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
            const double timestamp = ros::Time(image_msg->header.stamp).toSec();

            // Concert ROS message to cv image
            cv_bridge::CvImagePtr bridge;
            try {
                bridge = cv_bridge::toCvCopy(image_msg, "bgr8");
            } catch (cv_bridge::Exception& e) {
                std::cout << "Failed to transform rgb image." << std::endl;
                return -1;
            }

            std::stringstream rgb_image_filename;
            rgb_image_filename << std::fixed << std::setprecision(6) << timestamp << ".png";
            cv::Mat imageUndistorted;
            if (undistort_rgb_images) {
                cv::undistort(bridge->image, imageUndistorted, camera_param, dist_param);
                cv::imwrite(rgb_image_save_path + rgb_image_filename.str(), imageUndistorted);
            } else {
                cv::imwrite(rgb_image_save_path + rgb_image_filename.str(), bridge->image); 
            }

            std::string rgb_file_path = "rgb/" + rgb_image_filename.str();
            rgb_map.insert(std::pair<double, std::string>(timestamp, rgb_file_path));
            
        }

        // Process Vicon data
        if (m.getTopic() == topic_vicon_camera) {
            const geometry_msgs::TransformStampedPtr vicon_msg = m.instantiate<geometry_msgs::TransformStamped>();
            const double timestamp = ros::Time(vicon_msg->header.stamp).toSec();
            const Eigen::Vector3d position = Eigen::Vector3d(vicon_msg->transform.translation.x, vicon_msg->transform.translation.y, vicon_msg->transform.translation.z);
            const Eigen::Quaterniond orientation =
                Eigen::Quaterniond(vicon_msg->transform.rotation.w, vicon_msg->transform.rotation.x, vicon_msg->transform.rotation.y, vicon_msg->transform.rotation.z);
            const okvis::kinematics::Transformation T_W_V = okvis::kinematics::Transformation(position, orientation);
            const okvis::kinematics::Transformation T_W_C = T_W_V * T_V_C;

            const Eigen::Vector3d position_C = T_W_C.r();
            const Eigen::Quaterniond orientation_C = T_W_C.q();

            ground_truth_map.insert(std::pair<double, GroundTruthRecord>(timestamp, GroundTruthRecord(position_C(0), position_C(1), position_C(2), orientation_C.x(), orientation_C.y(), orientation_C.z(), orientation_C.w())));
        }
    }

    std::cout << "Converting rgb data to .png -> Done!" << std::endl;

    std::cout << "Associate rgb data and ground truth" << std::endl;

    RGBGroundTruthAssociationMap rgb_ground_truth_associations = associate(rgb_map, ground_truth_map, max_diff);

    for (auto rgb_ground_truth_association : rgb_ground_truth_associations) {
        // Save the Depth timestamps in a separate file
       rgb_ground_truth_association_file << rgb_ground_truth_association.first << " "
                   << rgb_ground_truth_association.second.tx << " "
                   << rgb_ground_truth_association.second.ty << " "
                   << rgb_ground_truth_association.second.tz << " "
                   << rgb_ground_truth_association.second.qx << " "
                   << rgb_ground_truth_association.second.qy << " "
                   << rgb_ground_truth_association.second.qz << " "
                   << rgb_ground_truth_association.second.qw << std::endl;
    }

    rgb_ground_truth_association_file.close();
    bag_read.close();

    std::cout << "Associate rgb data and ground truth -> Done!" << std::endl;   
    return 0;
}





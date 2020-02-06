#include <iostream>
#include <ros/ros.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>
#include<string>
#include<cmath>
#include<Eigen/Dense>
#include<pcl_conversions/pcl_conversions.h>
#include<ctime>
#include<cstdlib>


/* obb_generator header */
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <obb_generator_msgs/cloudArray.h>

using namespace std::chrono_literals;

#define MODE 0 // 1 is AABB mode
                // 0 is OBB mode

class ObbGenerator{
public:
    ObbGenerator(){
        obb_sub = nh.subscribe("/obb_cluster", 1, &ObbGenerator::obb_callback, this);
        obbArr_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obb_boxes", 1);
    }
    
    ~ObbGenerator(){

    }

    inline bool _init(){
        
#if MODE /* AABB mode */
        feature_extractor.getAABB (min_point_AABB, max_point_AABB);

#else /* OBB mode */
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
#endif
        return true;
    }

    void obb_callback(const obb_generator_msgs::cloudArray::ConstPtr& in_obb){
        pcl::PointCloud<pcl::PointXYZ> scan;
        jsk_recognition_msgs::BoundingBox obb;
        jsk_recognition_msgs::BoundingBoxArray obb_arr;

        obb_arr.header = in_obb->header;
        int z_idx = 0;
        for (auto& cloudarr : in_obb->cloudArray){
            pcl::fromROSMsg(cloudarr, scan);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (scan));
            feature_extractor.setInputCloud (cloud);
            feature_extractor.compute ();
            if(!_init()){
                ROS_ERROR("value initialize error!");
                exit(1);
            } 
#if MODE
            obb.header = in_obb->header;
            obb.pose.position.x = min_point_AABB.x + (max_point_AABB.x - min_point_AABB.x) / 2;
            obb.pose.position.y = min_point_AABB.y + (max_point_AABB.y - min_point_AABB.y) / 2;
            obb.pose.position.z = min_point_AABB.z + (max_point_AABB.z - min_point_AABB.z) / 2;
            obb.dimensions.x = max_point_AABB.x - min_point_AABB.x;
            obb.dimensions.y = max_point_AABB.y - min_point_AABB.y;
            obb.dimensions.z = in_obb->zArray[z_idx];


#else    
            obb.header = in_obb->header;
            Eigen::Quaternionf quat (rotational_matrix_OBB);
            obb.pose.orientation.x = quat.x();
            obb.pose.orientation.y = quat.y();
            obb.pose.orientation.z = quat.z();
            obb.pose.orientation.w = quat.w();
            obb.pose.position.x = position_OBB.x;
            obb.pose.position.y = position_OBB.y;
            obb.pose.position.z = position_OBB.z + (in_obb->zArray[z_idx] / 2) ;
            obb.dimensions.x = max_point_OBB.x - min_point_OBB.x;
            obb.dimensions.y = max_point_OBB.y - min_point_OBB.y;
            obb.dimensions.z = in_obb->zArray[z_idx];
#endif
            ++z_idx;
            obb_arr.boxes.emplace_back(obb);
        }
        obbArr_pub.publish(obb_arr);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber obb_sub;
    ros::Publisher obbArr_pub;
    
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    
#if MODE
    /* AABB mode */
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
#else
    /* OBB mode */
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
#endif
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "obb_generator");
    ObbGenerator obbgen;
    ros::spin();
    return (0);
}
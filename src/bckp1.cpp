//segmantation fault 27/09/2019

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

// Needed to return a Pose message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


class Perception
{
public:
    Perception(ros::NodeHandle* nodehandle);


    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    // ros::ServiceServer minimal_service_;
    ros::Publisher cropped_pub_;
    ros::Publisher table_pub_;
    ros::Publisher object_pub_;
    ros::Publisher cluster_pub_;

// private:    
    std::string cloud_topic, world_frame, camera_frame;
    float voxel_leaf_size;
    float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
    int plane_max_iter;
    float plane_dist_thresh;
    float cluster_tol;
    int cluster_min_size;
    int cluster_max_size;

    void initializeSubscribers();
    void initializePublishers();
    // void initializeServices();

    void subscriberCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
};

Perception::Perception(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{   // constructor
    initializeSubscribers();
    initializePublishers();
    // initializeServices();

    //initialize variables here, as needed

    cloud_topic = nh_.param<std::string>("cloud_topic", "/camera/depht/points");
    world_frame = nh_.param<std::string>("world_frame", "camera_link");
    camera_frame = nh_.param<std::string>("camera_frame", "camera_link");
    voxel_leaf_size = nh_.param<float>("voxel_leaf_size", 0.002);
    x_filter_min = nh_.param<float>("x_filter_min", -5);
    x_filter_max = nh_.param<float>("x_filter_max",  5);
    y_filter_min = nh_.param<float>("y_filter_min", -5);
    y_filter_max = nh_.param<float>("y_filter_max",  5);
    z_filter_min = nh_.param<float>("z_filter_min", -5);
    z_filter_max = nh_.param<float>("z_filter_max",  5);
    plane_max_iter = nh_.param<int>("plane_max_iterations", 50);
    plane_dist_thresh = nh_.param<float>("plane_distance_threshold", 0.05);
    cluster_tol = nh_.param<float>("cluster_tolerance", 0.01);
    cluster_min_size = nh_.param<int>("cluster_min_size", 100);
    cluster_max_size = nh_.param<int>("cluster_max_size", 50000);


}

void Perception::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pcl_sub_ = nh_.subscribe("/camera/depth/points", 3, &Perception::subscriberCallback,this);

}

void Perception::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    cropped_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    table_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_cluster", 1, true);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1, true);

}

void Perception::subscriberCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
{


    // TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME

    tf::TransformListener listener;
    tf::StampedTransform stransform;
    try
    {
        listener.waitForTransform(world_frame, pcl_msg->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
        listener.lookupTransform(world_frame, pcl_msg->header.frame_id,  ros::Time(0), stransform);
    }
    catch (tf::TransformException ex)
    {
        // ROS_ERROR("%s",ex.what());
        ROS_ERROR("erroorrr");
    }
    sensor_msgs::PointCloud2 transformed_cloud;
    //  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
    //               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
    pcl_ros::transformPointCloud(world_frame, stransform, *pcl_msg, transformed_cloud);

    // CONVERT POINTCLOUD ROS->PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);
    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*pcl_msg,pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::fromPCLPointCloud2(pcl_pc2,cloud);

    ROS_INFO_STREAM("PointCloud originalt: " << cloud.points.size() << " data points." );

    // Voxel Grid Downsampling

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud (cloud_ptr);
    voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter (*cloud_voxel_filtered);

    // PassThrough Filter

    // pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    // pcl::PassThrough<pcl::PointXYZ> pass_x;
    // pass_x.setInputCloud(cloud_voxel_filtered);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(x_filter_min, x_filter_max);
    // pass_x.filter(xf_cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    // pcl::PassThrough<pcl::PointXYZ> pass_y;
    // pass_y.setInputCloud(xf_cloud_ptr);
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(y_filter_min, y_filter_max);
    // pass_y.filter(yf_cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    // pcl::PassThrough<pcl::PointXYZ> pass_z;
    // pass_z.setInputCloud(yf_cloud_ptr);
    // pass_z.setFilterFieldName("z");
    // pass_z.setFilterLimits(z_filter_min, z_filter_max);
    // pass_z.filter(zf_cloud);


    // // CROPBOX 

    // pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
    // pcl::CropBox<pcl::PointXYZ> crop;
    // crop.setInputCloud(cloud_voxel_filtered);
    // Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    // Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
    // crop.setMin(min_point);
    // crop.setMax(max_point);
    // crop.filter(xyz_filtered_cloud);


    // ROS_INFO_STREAM("PointCloud cropped: " << xyz_filtered_cloud.points.size() << " data points." );

    // RANSAC Plane Segmentation

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (plane_max_iter);
    seg.setDistanceThreshold (plane_dist_thresh);
    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud (cropped_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
    }

    // Extract inliers and outliers

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);

    // Clustering -> EUCLIDEAN

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    *cloud_filtered = *cloud_f;
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (cluster_min_size);
    ec.setMaxClusterSize (cluster_max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
        clusters.push_back(cloud_cluster);
        sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
        pc2_clusters.push_back(tempROSMsg);
    }

    // Statistical Outlier Filtering

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cluster_cloud_ptr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*sor_cloud_filtered);

    // BROADCAST TRANSFORM 
    static tf::TransformBroadcaster br;
    tf::Transform part_transform;

    //Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
    part_transform.setOrigin( tf::Vector3(sor_cloud_filtered->at(1).x, sor_cloud_filtered->at(1).y, sor_cloud_filtered->at(1).z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    part_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));

    /* ========================================
    * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
    * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr asdvjsahdbajkd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    pcl::ExtractIndices<pcl::PointXYZ> extract_ind;
    prism.setInputCloud(sensor_cloud_ptr);
    pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());
    // create prism surface
    double box_length=0.25;
    double box_width=0.25;
    pick_surface_cloud_ptr->width = 5;
    pick_surface_cloud_ptr->height = 1;
    pick_surface_cloud_ptr->points.resize(5);

    pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[0].z = 0;

    pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
    pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[1].z = 0;

    pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
    pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
    pick_surface_cloud_ptr->points[2].z = 0;

    pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
    pick_surface_cloud_ptr->points[3].z = 0;

    pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[4].z = 0;

    Eigen::Affine3d eigen3d;
    tf::transformTFToEigen(part_transform,eigen3d);
    pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

    prism.setInputPlanarHull( pick_surface_cloud_ptr);
    prism.setHeightLimits(-10,10);

    prism.segment(*pt_inliers);

    extract_ind.setInputCloud(sensor_cloud_ptr);
    extract_ind.setIndices(pt_inliers);

    extract_ind.setNegative(true);
    extract_ind.filter(*prism_filtered_cloud);



    // Convert PCL data to ROS messages

    sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*sor_cloud_filtered, *pc2_cloud);

    // Publish ROS messages
    pc2_cloud->header.frame_id=world_frame;
    pc2_cloud->header.stamp=ros::Time::now();
    object_pub_.publish(pc2_cloud);

    ROS_INFO_STREAM("ok");
}

int main(int argc, char** argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "percetion_node");

    ros::NodeHandle nh;

    Perception perception(&nh);

    ROS_INFO("spinning");
//    ros::Rate r(10);
//    while (ros::ok())
//    {
//        ros::spinOnce();
//        r.sleep();
//    }
    ros::spin();
    return 0;
} 
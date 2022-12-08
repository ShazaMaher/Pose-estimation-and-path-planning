#include "string"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

void alignment(PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, const size_t globalite, const size_t localite);

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("I heard Height[%d] Width[%d", cloud_msg->height, cloud_msg->width);

    // Load
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *scene);

    PointCloud<PointT>::Ptr input_object(new PointCloud<PointT>);
    loadPCDFile("/home/student/catkin_ws/src/pose_est/solaire.pcd", *input_object);

    // Scale object down as it is way to big
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    int N=1000;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,0) = transform (0,0) / N;
    transform (1,1) = transform (1,1) / N;
    transform (2,2) = transform (2,2) / N;
    pcl::transformPointCloud (*input_object, *object, transform);


    // Write the scene pointcloud to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("/home/student/catkin_ws/src/pose_est/scene_full.pcd", *scene, false);


    /***************************** filters **************************************/
    /*
    // PassThrough filter to remove background points
    //PointCloud<PointT>::Ptr pass_filtered(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene_filtered(new PointCloud<PointT>);
    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud (scene);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.7);
    //pass.setFilterLimitsNegative (true);
    //pass.filter (*pass_filtered);
    pass.filter (*scene_filtered);

    // Voxel grid to reduce amount of points (only object until new CAD model)
    PointCloud<PointT>::Ptr object_filtered(new PointCloud<PointT>);
    //PointCloud<PointT>::Ptr scene_filtered(new PointCloud<PointT>);
    pcl::VoxelGrid<pcl::PointNormal> sor;
    sor.setLeafSize (0.003f, 0.003f, 0.003f);  //Object parameters
    sor.setInputCloud (object);
    sor.filter (*object_filtered);
    //sor.setLeafSize (0.001f, 0.001f, 0.001f);  //scene parameters
    //sor.setInputCloud (pass_filtered);
    //sor.filter (*scene_filtered);

    //Planefilter to remove table
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointNormal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.009);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointNormal> extract;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_p (new pcl::PointCloud<pcl::PointNormal>), cloud_f (new pcl::PointCloud<pcl::PointNormal>);
    int i = 0, nr_points = (int) scene_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (scene_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (scene_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (scene_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        scene_filtered.swap (cloud_f);
        i++;
    } */

    //alignment(scene_filtered, object_filtered, 1000, 40);

    // Convert to ROS data type
    //sensor_msgs::PointCloud2 output;
    //pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    //pub.publish (output);
}


int main(int argc, char**argv) {



    // Initialize ROS
    ros::init (argc, argv, "pose_est_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();

    return 0;

}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

void alignment(PointCloud<PointT>::Ptr scene, PointCloud<PointT>::Ptr object, const size_t globalite, const size_t localite){

    // Compute surface normals
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(10);

        ne.setInputCloud(object);
        ne.compute(*object);

        ne.setInputCloud(scene);
        ne.compute(*scene);
    }

    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");

        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(0.03);

        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);

        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }

    // Find feature matches
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }

    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);

    // Set RANSAC parameters
    const size_t iter = globalite;
    const float thressq = 0.01 * 0.01;

    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            if((i + 1) % 100 == 0)
                cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }

            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);

            // Apply pose
            transformPointCloud(*object, *object_aligned, T);

            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);

            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;

            // Update result
            if(penaltyi < penalty) {
                cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
            }
        }

        transformPointCloud(*object, *object_aligned, pose);

        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);

        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing

    //**************************local alignment ******************************

    // Create a k-d tree for scene
    search::KdTree<PointNormal> l_tree;
    tree.setInputCloud(scene);

    // Set ICP parameters
    const size_t l_iter = localite;
    const float l_thressq = 0.01 * 0.001;

    // Start ICP
    Matrix4f l_pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_align(new PointCloud<PointNormal>(*object_aligned));
    {
        ScopeTime t("ICP");
        cout << "Starting ICP..." << endl;
        for(size_t i = 0; i < l_iter; ++i) {
            // 1) Find closest points
            vector<vector<int> > l_idx;
            vector<vector<float> > l_distsq;
            tree.nearestKSearch(*object_align, std::vector<int>(), 1, l_idx, l_distsq);

            // Threshold and create indices for object/scene and compute RMSE
            vector<int> l_idxobj;
            vector<int> l_idxscn;
            for(size_t j = 0; j < l_idx.size(); ++j) {
                if(l_distsq[j][0] <= l_thressq) {
                    l_idxobj.push_back(j);
                    l_idxscn.push_back(l_idx[j][0]);
                }
            }

            // 2) Estimate transformation
            Matrix4f localT;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_align, l_idxobj, *scene, l_idxscn, localT);

            // 3) Apply pose
            transformPointCloud(*object_align, *object_align, localT);

            // 4) Update result
            l_pose = localT * l_pose;

        }

        // Compute inliers and RMSE
        vector<vector<int> > l_idx;
        vector<vector<float> > l_distsq;
        tree.nearestKSearch(*object_align, std::vector<int>(), 1, l_idx, l_distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < l_distsq.size(); ++i)
            if(l_distsq[i][0] <= thressq)
                ++inliers, rmse += l_distsq[i][0];
        rmse = sqrtf(rmse / inliers);

        // Print pose
        cout << "Got the following pose:" << endl << l_pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing
}

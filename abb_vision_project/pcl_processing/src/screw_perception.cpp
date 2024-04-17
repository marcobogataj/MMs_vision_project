#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/colors.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>

std::string removeTrailingZerosFromDecimal(const std::string &numStr)
{
    if (numStr.find('.') != std::string::npos)
    { // Check if the number has a decimal point
        std::string resultString = numStr;
        // Remove trailing zeros
        while (!resultString.empty() && resultString.back() == '0')
        {
            resultString.pop_back();
        }
        // If the last character is a dot, remove it
        if (!resultString.empty() && resultString.back() == '.')
        {
            resultString.pop_back();
        }
        return resultString;
    }
    // Return the original string if there's no decimal point
    return numStr;
}

int countAcquisitions = 0;

double init_secs_total;

void write_timeData_to_log_file(const std::vector<double> &timeVector)
{   
    std::string timeStr;
    std::ofstream log_file("/home/abb/catkin_ws/src/abb_vision_project/pcl_processing/log/ProcessingTimeDataLog.txt", std::ios_base::out | std::ios_base::app );
    log_file << countAcquisitions;
    for(int i=0; i<timeVector.size(); i++)
    {
    //timeStr = removeTrailingZerosFromDecimal(rounded2string(timeVector[i]));
    timeStr = std::to_string(round(timeVector[i]*1000) / 1000);

    //remove three zeroes
    timeStr.pop_back();
    timeStr.pop_back();
    timeStr.pop_back();

    log_file << "," << timeStr;
    }
    log_file << std::endl;
    countAcquisitions++;
}

std::vector<double> timeData(6);

using namespace message_filters;

class CylinderSegment
{
public:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  CylinderSegment() : sync(cloud_subscriber_, normals_subscriber_, 10)
  {
    cloud_subscriber_.subscribe(nh_,"/zivid_camera/points/xyz", 1);
    normals_subscriber_.subscribe(nh_,"/zivid_camera/normals/xyz",1);

    sync.registerCallback(boost::bind(&CylinderSegment::cloudCB,this, _1, _2));

    visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
    cylinder_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("processing_marker_array", 0);
    processed_cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/pcl_processing/clusters/xyzrgb", 1);
    cylinder_cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pcl_processing/cylinder/xyz", 1);
  }

  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene. */
  void addCylinder()
  {
    // BEGIN_SUB_TUTORIAL add_cylinder
    //
    // Adding Cylinder to Planning Scene
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    // Setting height of cylinder. 
    primitive.dimensions[0] = cylinder_params.height;
    // Setting radius of cylinder. 
    primitive.dimensions[1] = cylinder_params.radius; 
                                                        
    geometry_msgs::Quaternion q; //quaternion CAM to ROB [ 0.9830354, -0.0165454, 0.1252577, -0.1329591 ]
    q.x = 0.9830354;
    q.y = -0.0165454;
    q.z = 0.1252577;
    q.w = -0.1329591;

    geometry_msgs::Vector3 axis_cam;
    axis_cam.x = cylinder_params.direction_vec[0];
    axis_cam.y = cylinder_params.direction_vec[1];
    axis_cam.z = cylinder_params.direction_vec[2];

    geometry_msgs::Vector3 center_cam;
    center_cam.x = cylinder_params.center_pt[0];
    center_cam.y = cylinder_params.center_pt[1];
    center_cam.z = cylinder_params.center_pt[2];

    geometry_msgs::Vector3 axis_rob;

    geometry_msgs::Vector3 t; 
    t.x = 0; 
    t.y = 0;
    t.z = 0;

    axis_rob = quat_trasl_rotate(axis_cam,q,t); //rotate and translate using camera calibration

    t.x = +0.037611496; 
    t.y = -0.595347046;
    t.z = +0.677256409;

    geometry_msgs::Vector3 point_rob;

    point_rob = quat_trasl_rotate(center_cam,q,t); //rotate and translate using camera calibration

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(axis_rob.x, axis_rob.y, axis_rob.z);

    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();

    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder (translate using camera calibration)
    cylinder_pose.position.x = point_rob.x;
    cylinder_pose.position.y = point_rob.y;
    cylinder_pose.position.z = point_rob.z;

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object); 

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2);

    marker_array.markers[0].header.frame_id = "base_link"; //
    marker_array.markers[0].header.stamp = ros::Time();
    //marker.ns = "my_namespace";
    marker_array.markers[0].id = 0;
    marker_array.markers[0].type = visualization_msgs::Marker::CYLINDER;
    marker_array.markers[0].action = visualization_msgs::Marker::ADD;
    marker_array.markers[0].pose.orientation.x = cylinder_pose.orientation.x;
    marker_array.markers[0].pose.orientation.y = cylinder_pose.orientation.y;
    marker_array.markers[0].pose.orientation.z = cylinder_pose.orientation.z;
    marker_array.markers[0].pose.orientation.w = cylinder_pose.orientation.w;
    marker_array.markers[0].pose.position.x = cylinder_pose.position.x;
    marker_array.markers[0].pose.position.y = cylinder_pose.position.y;
    marker_array.markers[0].pose.position.z = cylinder_pose.position.z;

    marker_array.markers[0].scale.x = cylinder_params.radius*2;
    marker_array.markers[0].scale.y = cylinder_params.radius*2;
    marker_array.markers[0].scale.z = cylinder_params.height;
    marker_array.markers[0].color.a = 0.8; // Don't forget to set the alpha!
    marker_array.markers[0].color.r = 0.0;
    marker_array.markers[0].color.g = 1.0;
    marker_array.markers[0].color.b = 0.0;

    marker_array.markers[1].header.frame_id = "base_link";
    marker_array.markers[1].header.stamp = ros::Time();  
    marker_array.markers[1].id = 1;
    marker_array.markers[1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array.markers[1].action = visualization_msgs::Marker::ADD;

    std::string title, s_diam, s_height,s_posx,s_posy,s_posz;
    s_diam = std::to_string(round(cylinder_params.radius*2*1000 * 10) / 10);
    s_height = std::to_string(round(cylinder_params.height*1000 * 10) / 10);
    s_posx = std::to_string(round(cylinder_pose.position.x*1000 * 10) / 10);
    s_posy = std::to_string(round(cylinder_pose.position.y*1000 * 10) / 10);
    s_posz = std::to_string(round(cylinder_pose.position.z*1000 * 10) / 10);

    s_diam = removeTrailingZerosFromDecimal(s_diam);
    s_height = removeTrailingZerosFromDecimal(s_height);
    s_posx = removeTrailingZerosFromDecimal(s_posx);
    s_posy = removeTrailingZerosFromDecimal(s_posy);
    s_posz = removeTrailingZerosFromDecimal(s_posz);
    
    title = "Cylinder:[mm] h=" + s_height + " d=" + s_diam + " pos=[" + s_posx + "," + s_posy + "," + s_posz + "]";
    marker_array.markers[1].text = title;

    marker_array.markers[1].pose.position.x = cylinder_pose.position.x;
    marker_array.markers[1].pose.position.y = cylinder_pose.position.y;
    marker_array.markers[1].pose.position.z = cylinder_pose.position.z-cylinder_params.radius*2 - cylinder_params.height;
    marker_array.markers[1].scale.z = 0.02; //font size

    marker_array.markers[1].color.a = 0.8; // Don't forget to set the alpha!
    marker_array.markers[1].color.r = 0.0;
    marker_array.markers[1].color.g = 1.0;
    marker_array.markers[1].color.b = 0.0;

    visualization_publisher_.publish( marker_array );
    timeData[5] = ros::Time::now().toSec() - init_secs_total;

    
    do 
    {}
    while (cylinder_publisher_.getNumSubscribers() == 0);

    ros::Duration(0.1).sleep(); // sleep for 0.1 seconds
    cylinder_publisher_.publish( marker_array );
  }

  geometry_msgs::Vector3 quat_trasl_rotate(geometry_msgs::Vector3 const &point,
                                          geometry_msgs::Quaternion const &quat,
                                          geometry_msgs::Vector3 const &trasl) { 
    tf2::Vector3 const tf2_point(point.x, point.y, point.z);
    tf2::Quaternion const tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Vector3 const tf2_output = tf2::quatRotate(tf2_quat, tf2_point);
    geometry_msgs::Vector3 output;
    output.x = tf2_output.getX()+trasl.x;
    output.y = tf2_output.getY()+trasl.y;
    output.z = tf2_output.getZ()+trasl.z;
    return output;
  }

  void estimateLocationHeight (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                               const pcl::ModelCoefficients::Ptr& coefficients_cylinder)
  { 
    // To estimate cylinder height: https://math.stackexchange.com/questions/3324579/sorting-collinear-points-on-a-3d-line
    // 1-> Project cylinder inliers onto the cylinder axis 𝑣. (https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html)
    // 2-> Select first and last points in the cylinder point cloud (ONLY with ordered point clouds file types)

    pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients ());
    coefficients_line->values.resize (6);
      
    for(int k=0; k<=5; k++) coefficients_line->values[k] = coefficients_cylinder->values[k];

    pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients_line);
    proj.filter (*line_proj);

    //compute height by computing the segment connecting first and last points of the cylinder inliers
    //since the point cloud is organized and saved in a ordered fashion, these results in the two extremes on the cylinder axis.

    cylinder_params.height = pcl::euclideanDistance(line_proj->points[0],line_proj->points[line_proj->size()-1]); //cylinder height estimation

    cylinder_params.center_pt[0] = (line_proj->points[line_proj->size()-1].x + line_proj->points[0].x)/2;
    cylinder_params.center_pt[1] = (line_proj->points[line_proj->size()-1].y + line_proj->points[0].y)/2;
    cylinder_params.center_pt[2] = (line_proj->points[line_proj->size()-1].z + line_proj->points[0].z)/2;
  }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_filtered)
  {
    ROS_INFO("Apply passthrough filter.");
    pcl::PassThrough<pcl::PointXYZ> pass(true);
    //pass.setKeepOrganized(true);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 0.64);
    pass.filter(*cloud);
    pass.getRemovedIndices(*inliers_filtered);
  }

  void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    ROS_INFO("Apply voxelization.");
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.0005f, 0.0005f, 0.0005f);
    vox.filter (*cloud);
  }

  void uniformSamplingFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_filtered)
  {
    ROS_INFO("Apply uniform sampling.");
    pcl::UniformSampling<pcl::PointXYZ> us(true);
    us.setInputCloud (cloud);
    us.setRadiusSearch (0.0004f);
    us.filter (*cloud);
    us.getRemovedIndices(*inliers_filtered);
  }

  void statisticalOutliersFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_filtered)
  {
    ROS_INFO("Remove outliers.");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
    //sor.setKeepOrganized(true);
    sor.setInputCloud (cloud);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud); 
    sor.getRemovedIndices(*inliers_filtered);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    //extract_normals.setKeepOrganized(true);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);
  }

  /** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane)
  {
    ROS_INFO("Remove plane...");
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(10000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.0025); // good is 0.005
    segmentor.setInputCloud(cloud);
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
  }

  std::vector<pcl::PointIndices> extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)                     
  {
    ROS_INFO("Start cluster extraction...");
    std::vector<pcl::PointIndices> inliers_clusters;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    
    ec.setClusterTolerance (0.0005); //good is 0.0005
    ec.setMinClusterSize (50); //for smaller screws 200
    ec.setMaxClusterSize (5500); //for smaller screws 700
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (inliers_clusters);
    
    std::cout<< inliers_clusters.size() <<" clusters found!"<<std::endl;

    return inliers_clusters;
  }

  void labelClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                      std::vector<pcl::PointIndices> inliers_clusters,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_clusters)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters_temp (new pcl::PointCloud<pcl::PointXYZRGB>);

    int count_c = 0;

    for (const auto& cluster : inliers_clusters)
    {
      pcl::copyPointCloud(*cloud, cluster, *cloud_clusters_temp);
      pcl::RGB rgb = pcl::GlasbeyLUT::at(count_c);

      for (auto& point : cloud_clusters_temp->points)
      {
        point.rgb = *reinterpret_cast<float*>(&rgb);
      }

      *cloud_clusters = *cloud_clusters + *cloud_clusters_temp;

      count_c++;
      std::cout<<"Cluster "<<count_c<<": "<<cloud_clusters_temp->size() <<" points"<<std::endl;
    }
  }
  
  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the cylinder from the
     pointcloud and store the cylinder parameters in coefficients_cylinder.
      @param cloud - Pointcloud whose plane is removed.
      @param coefficients_cylinder - Cylinder parameters used to define an infinite cylinder will be stored here.
      @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
  void extractCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.2); //0.3 before
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(10000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.01); //0.05 before
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.0009, 0.008);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    //extract.setKeepOrganized(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);

    std::cout<<"Cylinder inliers: "<<cloud->size() <<" points"<<std::endl;
  }

  
  void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                  const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,const pcl::PointCloud<pcl::Normal>::ConstPtr& zivid_normals)
  {
    // Visualization using PCLVisualizer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText ("PCL normals", 10, 10, "v1 text", v1);
    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);
    viewer->addText ("Zivid normals", 10, 10, "v2 text", v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color, "cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color, "cloud2", v2);
    
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.004, "normals",v1);
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, zivid_normals, 10, 0.004, "zivid_normals",v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud",v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2",v2);
    while (!viewer->wasStopped ()) { // Display the visualiser 
    viewer->spinOnce ();
    }
  }
  

  void cloudCB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
               const pcl::PointCloud<pcl::Normal>::ConstPtr& normals)
  {
    init_secs_total = ros::Time::now().toSec();
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // This section uses a standard PCL-based processing pipeline to estimate a cylinder's pose in the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *input;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cylinder(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_zivid(new pcl::PointCloud<pcl::Normal>);

    *cloud_normals = *normals;

    pcl::PointIndices::Ptr inliers_filtered(new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_cluster(new pcl::PointIndices);

    ROS_INFO("Segmentation.");
    // Use a passthrough filter to get the region of interest.
    // The filter removes points outside the specified range.
    ROS_INFO("Apply filtering...");
    

    double init_secs = ros::Time::now().toSec();
    passThroughFilter(cloud,inliers_filtered);
    timeData[0] = ros::Time::now().toSec() - init_secs;

    //cout<<"passThroughFiltering time: "<<ros::Time::now().toSec() - init_secs<<" s"<<std::endl;
    
    //extractNormals(cloud_normals,inliers_filtered);

    //std::cout<< cloud->size() <<" cloud points!"<<std::endl;
    //std::cout<< cloud_normals->size() <<" normal points!"<<std::endl;

    //statisticalOutliersFilter(cloud);
    //extractNormals(cloud_normals,inliers_filtered);

    init_secs = ros::Time::now().toSec();
    voxelFilter(cloud);
    timeData[1] = ros::Time::now().toSec() - init_secs;

    //cout<<"Voxelization time: "<<ros::Time::now().toSec() - init_secs<<" s"<<std::endl;

    //uniformSamplingFilter(cloud,inliers_filtered);
    //extractNormals(cloud_normals,inliers_filtered);

    //std::cout<< cloud->size() <<" cloud points!"<<std::endl;
    //std::cout<< cloud_normals->size() <<" normal points!"<<std::endl;

    //ROS_INFO("Compute normals...");
    //computeNormals(cloud, cloud_normals);
    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    // Detect and remove points on the (planar) surface on which the cylinder is resting.
    ROS_INFO("Remove plane...");
    
    init_secs = ros::Time::now().toSec();
    removePlaneSurface(cloud, inliers_plane);
    timeData[2] = ros::Time::now().toSec() - init_secs;

    //cout<<"Plane removal time: "<<ros::Time::now().toSec() - init_secs<<" s"<<std::endl;
    // Remove surface points from normals as well
    //ROS_INFO("Extract normals...");
    //extractNormals(cloud_normals, inliers_plane);

    //std::cout<< cloud->size() <<" cloud points!"<<std::endl;
    //std::cout<< cloud_normals->size() <<" normal points!"<<std::endl;

    //processed_cloud_publisher_.publish(*cloud); //publish filtered point cloud

    //Extract clusters
    init_secs = ros::Time::now().toSec();
    std::vector<pcl::PointIndices> inliers_clusters = extractClusters(cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);

    labelClusters(cloud,inliers_clusters,cloud_clusters);
    
    sensor_msgs::PointCloud2 cloud_clusters_msg;
    cloud_clusters_msg.header.frame_id = "zivid_optical_frame";
    cloud_clusters_msg.header.stamp = ros::Time::now();

    pcl_conversions::toPCL(cloud_clusters_msg.header,cloud_clusters->header);

    processed_cloud_publisher_.publish(cloud_clusters); //publish labeled clusters point cloud

    timeData[3] = ros::Time::now().toSec() - init_secs;
    
    //cout<<"Clusterization time: "<<ros::Time::now().toSec() - init_secs<<" s"<<std::endl;

    init_secs = ros::Time::now().toSec();
    // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length.
    // It has a public attribute |code_start| values\ |code_end| of type |code_start| std::vector<float>\ |code_end|\ .
    // |br|
    // |code_start| values[0-2]\ |code_end| hold a point on the center line of the cylinder. |br|
    // |code_start| values[3-5]\ |code_end| hold direction vector of the z-axis. |br|
    // |code_start| values[6]\ |code_end| is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    /* Extract the cylinder using SACSegmentation. */
    ROS_INFO("Extract cylinder...");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    
    int count_j = 0;

    // initialize cylinder params
    cylinder_params.radius = 0; 
    cylinder_params.direction_vec[0] = 0;
    cylinder_params.direction_vec[1] = 0;
    cylinder_params.direction_vec[2] = 0;
    cylinder_params.height = 0;
    cylinder_params.center_pt[0] = 0;
    cylinder_params.center_pt[1] = 0;
    cylinder_params.center_pt[2] = 0;

    for (const auto& segment_inliers : inliers_clusters)
    {
      count_j++;
      *inliers_cluster = segment_inliers;
      pcl::copyPointCloud(*cloud, *inliers_cluster, *cloud_cylinder);
      //pcl::copyPointCloud(*cloud_normals, *inliers_cluster, *cloud_normals_zivid);

      computeNormals(cloud_cylinder, cloud_normals);
      //std::cout<< cloud_cylinder->size() <<" cluster points!"<<std::endl;
      //std::cout<< cloud_normals_zivid->size() <<" normal points!"<<std::endl;

      //visualizeNormals(cloud_cylinder,cloud_normals,cloud_normals_zivid);

      //extractNormals(cloud_normals_cylinder, inliers_cluster);
      extractCylinder(cloud_cylinder, coefficients_cylinder, cloud_normals);

      if (cloud_cylinder->points.empty() || 
          coefficients_cylinder->values.size() != 7) 
          //|| cloud_cylinder->size() < cloud_cluster->size()*0.7)
      {
        ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
      }
      else 
      {
        ROS_INFO("Cylinder found in cluster %d!",count_j);
        ROS_INFO("Adding cilindrical CollisionObject to PlanningScene");

        std::cout<<"Publish cloud cylinder "<<std::endl;
        cylinder_cloud_publisher_.publish(cloud_cylinder); //publish labeled clusters point cloud

        cylinder_params.radius = coefficients_cylinder->values[6];

        /* Store direction vector of z-axis of cylinder. */
        cylinder_params.direction_vec[0] = coefficients_cylinder->values[3];
        cylinder_params.direction_vec[1] = coefficients_cylinder->values[4];
        cylinder_params.direction_vec[2] = coefficients_cylinder->values[5];

        // Compute center point and height of the cylinder using point cloud projection on the cylinder axis
        std::cout<<"Estimate location "<<std::endl;
        estimateLocationHeight(cloud_cylinder, coefficients_cylinder);

        break;
      }
    }
    //cout<<"Cylinder identification time: "<<ros::Time::now().toSec() - init_secs<<" s"<<std::endl;
    timeData[4] = ros::Time::now().toSec() - init_secs;
    
    // Use the parameters extracted to add the cylinder to the planning scene as a collision object.
    std::cout<<"Add cylinder: "<<std::endl;
    addCylinder();

    write_timeData_to_log_file(timeData);
  }

private:
  // BEGIN_SUB_TUTORIAL 
  ros::NodeHandle nh_;

  message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> cloud_subscriber_;
  message_filters::Subscriber<pcl::PointCloud<pcl::Normal>> normals_subscriber_;

  ros::Publisher visualization_publisher_;
  ros::Publisher processed_cloud_publisher_;
  ros::Publisher cylinder_cloud_publisher_;
  ros::Publisher cylinder_publisher_;

  TimeSynchronizer<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::Normal>> sync;

  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector along the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  struct AddClusterParams
  {
    /* centroids */
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator <pcl::PointXYZ> > centroids;
  };

  // Declare a variable of type AddCylinderParams and store relevant values from ModelCoefficients.
  AddCylinderParams cylinder_params;
  AddClusterParams cluster_params;
  // END_SUB_TUTORIAL
};

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "screw_perception");

  //TO DO: implement condition on service callback/result before starting to segment.

  // Start the segmentor
  ROS_INFO("Ready to segment.");
  CylinderSegment segmentor;

  // Spin
  ros::spin();
}


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class CylinderSegment
{
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::Subscriber cloud_subscriber_;

  ros::Publisher visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  ros::Publisher processed_cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pcl_processing/filtered/xyz", 1);

public:
  CylinderSegment()
    : cloud_subscriber_(nh_.subscribe("/zivid_camera/points/xyz", 1, &CylinderSegment::cloudCB, this))  
  {
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
    collision_object.header.frame_id = "zivid_optical_frame";
    collision_object.id = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = cylinder_params.height;
    /* Setting radius of cylinder. */
    primitive.dimensions[1] = cylinder_params.radius;

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params.direction_vec[0], cylinder_params.direction_vec[1],
                                         cylinder_params.direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params.center_pt[0];
    cylinder_pose.position.y = cylinder_params.center_pt[1];
    cylinder_pose.position.z = cylinder_params.center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2);

    marker_array.markers[0].header.frame_id = "zivid_optical_frame";
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
    marker_array.markers[0].color.g = 0.0;
    marker_array.markers[0].color.b = 1.0;

    marker_array.markers[1].header.frame_id = "zivid_optical_frame";
    marker_array.markers[1].header.stamp = ros::Time();  
    marker_array.markers[1].id = 1;
    marker_array.markers[1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array.markers[1].action = visualization_msgs::Marker::ADD;

    std::string title, s_diam, s_height,s_posx,s_posy,s_posz;
    s_diam = std::to_string(cylinder_params.radius*2*1000);
    s_height = std::to_string(cylinder_params.height*1000);
    s_posx = std::to_string(cylinder_pose.position.x*1000);
    s_posy = std::to_string(cylinder_pose.position.y*1000);
    s_posz = std::to_string(cylinder_pose.position.z*1000);
    title = "Cylinder:[mm] h=" + s_height + " d=" + s_diam + " pos=[" + s_posx + "," + s_posy + "," + s_posz + "]";
    marker_array.markers[1].text = title;

    marker_array.markers[1].pose.position.x = cylinder_pose.position.x;
    marker_array.markers[1].pose.position.y = cylinder_pose.position.y;
    marker_array.markers[1].pose.position.z = cylinder_pose.position.z-cylinder_params.radius*2 - cylinder_params.height;
    marker_array.markers[1].scale.z = 0.007; //font size

    marker_array.markers[1].color.a = 0.8; // Don't forget to set the alpha!
    marker_array.markers[1].color.r = 0.0;
    marker_array.markers[1].color.g = 1.0;
    marker_array.markers[1].color.b = 0.0;

    visualization_publisher_.publish( marker_array );
    // END_SUB_TUTORIAL
  }

  /// @brief Estimate height and center point of the cylinder by axis projection method.
  /// @param cloud Cylinder point cloud.
  /// @param coefficients_cylinder Model coefficients of the cylinder.
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
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    ROS_INFO("Apply passthrough filter.");
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 0.64);
    pass.filter(*cloud);
  }

  void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    ROS_INFO("Apply voxelization.");
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.001f, 0.001f, 0.001f);
    vox.filter (*cloud);
  }

  void statisticalOutliersFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    ROS_INFO("Remove outliers.");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud); 
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
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
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
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
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
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(5000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.015);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.005, 0.085);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // This section uses a standard PCL-based processing pipeline to estimate a cylinder's pose in the point cloud.
    //
    // First, we convert from sensor_msgs to pcl::PointXYZ which is needed for most of the processing.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    ROS_INFO("Segmentation.");
    // Use a passthrough filter to get the region of interest.
    // The filter removes points outside the specified range.
    ROS_INFO("Apply filtering...");
    
    passThroughFilter(cloud);
    //statisticalOutliersFilter(cloud);
    //voxelFilter(cloud);

    // Compute point normals for later sample consensus step.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ROS_INFO("Compute normals...");
    computeNormals(cloud, cloud_normals);
    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    // Detect and remove points on the (planar) surface on which the cylinder is resting.
    ROS_INFO("Remove plane...");
    removePlaneSurface(cloud, inliers_plane);
    // Remove surface points from normals as well
    ROS_INFO("Extract normals...");
    extractNormals(cloud_normals, inliers_plane);

    processed_cloud_publisher_.publish(*cloud); //publish filtered point cloud

    // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length.
    // It has a public attribute |code_start| values\ |code_end| of type |code_start| std::vector<float>\ |code_end|\ .
    // |br|
    // |code_start| values[0-2]\ |code_end| hold a point on the center line of the cylinder. |br|
    // |code_start| values[3-5]\ |code_end| hold direction vector of the z-axis. |br|
    // |code_start| values[6]\ |code_end| is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    /* Extract the cylinder using SACSegmentation. */
    ROS_INFO("Extract cylinder...");
    extractCylinder(cloud, coefficients_cylinder, cloud_normals);
    // END_SUB_TUTORIAL
    if (cloud->points.empty() || coefficients_cylinder->values.size() != 7)
    {
      ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
      return;
    }

    ROS_INFO("Detected Cylinder - Adding CollisionObject to PlanningScene");

    // BEGIN_TUTORIAL
    // CALL_SUB_TUTORIAL callback
    //
    // Storing Relevant Cylinder Values
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The information that we have in |code_start| coefficients_cylinder\ |code_end| is not enough to define our
    // cylinder.
    // It does not have the actual location of the cylinder nor the actual height. |br|
    // We define a struct to hold the parameters that are actually needed for defining a collision object completely.
    // |br|
    // CALL_SUB_TUTORIAL param_struct
    /* Store the radius of the cylinder. */
    cylinder_params.radius = coefficients_cylinder->values[6];
    /* Store direction vector of z-axis of cylinder. */
    cylinder_params.direction_vec[0] = coefficients_cylinder->values[3];
    cylinder_params.direction_vec[1] = coefficients_cylinder->values[4];
    cylinder_params.direction_vec[2] = coefficients_cylinder->values[5];
    //
    // Extracting Location and Height
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Compute the center point of the cylinder using standard geometry
    // extractLocationHeight(cloud);
    // CALL_SUB_TUTORIAL extract_location_height

    // Compute center point and height of the cylinder using point cloud projection on the cylinder axis
    estimateLocationHeight(cloud, coefficients_cylinder);
    // Use the parameters extracted to add the cylinder to the planning scene as a collision object.
    addCylinder();
    // CALL_SUB_TUTORIAL add_cylinder
    // END_TUTORIAL
  }

private:
  // BEGIN_SUB_TUTORIAL param_struct
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
  // Declare a variable of type AddCylinderParams and store relevant values from ModelCoefficients.
  AddCylinderParams cylinder_params;
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
  ros::init(argc, argv, "cylinder_segment");

  //TO DO: implement condition on service callback/result before starting to segment.

  // Start the segmentor
  ROS_INFO("Ready to segment.");
  CylinderSegment segmentor;

  // Spin
  ros::spin();
}


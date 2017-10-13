#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
// PCL specific includes
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// for clustering
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
ros::Publisher coord_pub;
static int pixel_index[2] = {0, 0};
static bool published = false;
static bool sent = false;
geometry_msgs::Point output;
static int codown = 20;


static double focal = 621.7;
static double baseline = 0.024;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	if (codown != 0)
	{
		codown--;
		return;
		}
  codown = 20;
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //sor.setInputCloud (cloudPtr);
  //sor.setLeafSize (0.03, 0.03, 0.015);
  //sor.filter (cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *pt_cloud);

  int number = pt_cloud->width * pt_cloud->height;
  
  int i = pixel_index[0] + 640*pixel_index[1] ;
  
  ROS_INFO("direct value! %d ", i); 

	// search for the correct corresponding pixel !

  double temp1;
  int temp_index;
  double tempz;
  double disparity;
  for(int tt = 0; tt < 640 ; tt++)
  {
  	// ROS_INFO("tt %d", tt);
  	temp_index = 640*pixel_index[1] + tt;
    if( !(isnan(pt_cloud->points[temp_index].x) || isnan(pt_cloud->points[temp_index].y) || isnan(pt_cloud->points[temp_index].z)) )
    {
	  	tempz = pt_cloud->points[temp_index].z;
	  	disparity = (focal * baseline) / tempz;
	  	temp1 = disparity + tt;
	  	
	  	// ROS_INFO("depth %f", tempz);

	  	if ( temp1 - pixel_index[0] < 3 && temp1 - pixel_index[0] > -3 )
	  	{
	  		ROS_INFO("Eureka!!! tt %d tempindex %d", tt, temp_index);
	  		i = temp_index; 
	  	}
	  	
		// ROS_INFO("temp1 %d; pixel index %d", (int)temp1, pixel_index[0]);
  	}
  }
  
  int count_loop = 0;
  if(i != 0 && i != -1)
  	{ 
  		while(count_loop < 5)
  		{
  			if ( isnan(pt_cloud->points[i].x) || isnan(pt_cloud->points[i].y) || isnan(pt_cloud->points[i].z) )
  			{
  				ROS_INFO("nan value!"); 
  				i = i+1;
  				count_loop++;
  			}
  		else{
  			ROS_INFO("point %d out of %d : x %f, y %f, z%f ", i, number, pt_cloud->points[i].x, pt_cloud->points[i].y, pt_cloud->points[i].z);
  			break;
  			}
  		}
  	}
 
  
  if (published == false) {
  if ( (isnan(pt_cloud->points[i].x) || isnan(pt_cloud->points[i].y) || isnan(pt_cloud->points[i].z)) )
  { // ROS_INFO("waiting or done!"); 
  }
  else
  {	
  	output.x = pt_cloud->points[i].x;
  	output.y = pt_cloud->points[i].y;
  	output.z = pt_cloud->points[i].z;
  	published = true;
  }
  }
  /*
  for (int i = 0; i < number; i++)
  {
      ROS_INFO("point %d : x %f, y %f, z%f", i, pt_cloud->points[i].x, pt_cloud->points[i].y, pt_cloud->points[i].z);
  }
  */

  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  if (published == true)
  { 
    // ROS_INFO("publish! x %f y %f z %f", output.x, output.y, output.z);
  	coord_pub.publish (output);
  	sent = true;
  	}
}


void pixel_readin(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	ROS_INFO("this is what i got %d %d", msg->data[0], msg->data[1]);
	pixel_index[0] = msg->data[0];
	pixel_index[1] = msg->data[1];
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  
  ros::Subscriber pixel_sub = nh.subscribe ("pixel_index", 1, pixel_readin);

  // Create a ROS publisher for the output point cloud 
  // Publish to Coords
  coord_pub = nh.advertise<geometry_msgs::Point>("target_coord", 100); 
  ros::Rate loop_rate(10);
  int count = 0;

  // Spin
  ros::spin ();
}




/*
ros::Publisher pub;
static int saved = 1;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
// which is also something we can get from pcd
  pcl::PointCloud<pcl::PointXYZ>* cloud_obj = new pcl::PointCloud<pcl::PointXYZ>;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (cloud_obj), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *cloud_obj);

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  ROS_INFO("PointCloud after filtering has: %d", (int)cloud_filtered->points.size () );

  // set the segmentation object, which uses RANSAC
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);


  int i=0;
  int nr_points = (int) cloud_filtered->points.size ();
  // iteratively segment out some planes
    while (cloud_filtered->points.size () > 0.1 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        ROS_INFO("Could not estimate a planar model for the given dataset.");
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      ROS_INFO("PointCloud representing the planar component: %d   %d", i, (int)cloud_plane->points.size () );

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // very careful on this, as too small make one object seperated
  // too high value makes multiple objects becoming one
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // get each entry of cluster_indices, which is one cluster
  pcl::PCDWriter writer;
  int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      ROS_INFO("PointCloud representing the Cluster: %d", (int)cloud_cluster->points.size () );

      std::stringstream ss;
      ss << "cluster_" << j << ".pcd";
      pcl::io::savePCDFileASCII(ss.str (), *cloud_cluster); //*

      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(*cloud_cluster, output);

      pub.publish(output)

      j++;
    }
    saved--;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
     ros::spin();

}
*/

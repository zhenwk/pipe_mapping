/// @brief:	Pipe Scan Visualization
/// @author:	Weikun Zhen
/// @date:	Feb. 17, 2015
/// @detail:	This *.cpp file is used for animate the scanning motion
///		of snake robot SNASER 


#include <iostream>
#include <vector>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

#define XMAX 600.0
#define XMIN 100.0
#define MAX_SCAN 200

struct ColorMap{
	uint8_t r;
	uint8_t g;
	uint8_t b;	
};

/// Generate a color map based on a given range
ColorMap colorBlend( pcl::PointXYZ point, float max, float min )
{
	ColorMap  c;
	float range = max - min;
	float seg   = range/4.0;

	float x  = abs(point.x);
	float y  = abs(point.y);
	float z  = abs(point.z);
		
	if (min <= x && x <= min + seg){
		c.r = uint8_t(255);
		c.g = uint8_t(255.0*(x - min)/seg);
		c.b = uint8_t(0);
	}
	else if (min + 1.0*seg < x && x <= min + 2.0*seg){
		c.r = uint8_t(255.0*(1.0 - (x - min - 1.0*seg)/seg));
		c.g = uint8_t(255);
		c.b = uint8_t(0);
		}
	else if (min + 2.0*seg < x && x <= min + 3.0*seg){
		c.r = uint8_t(0);
		c.g = uint8_t(255);
		c.b = uint8_t(255.0*( (x - min - 2.0*seg)/seg));
		}
	else if (min + 3.0*seg < x && x <= min + 4.0*seg){
		c.r = uint8_t(0);
		c.g = uint8_t(255.0*(1.0 - (x - min - 3.0*seg)/seg));
		c.b = uint8_t(255.0);
	}

	return c;
}

/// main loop 
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr algCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  /// Load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../../2015-02-09/data/10/pcd_alg.pcd", *algCloud) == -1) 
  {
    PCL_ERROR ("Couldn't read file pcd_alg.pcd \n");
    return (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../../2015-02-09/data/10/pcd_raw.pcd", *rawCloud) == -1) 
  {
    PCL_ERROR ("Couldn't read file pcd_raw.pcd \n");
    return (-1);
  }
 
 
  /// Generate a pcl visualizer project
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
  	(new pcl::visualization::PCLVisualizer ("3D Local Map of Bent Pipe"));
  //viewer->addCoordinateSystem (50.0);
  viewer->initCameraParameters ();
 
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  //viewer->addText("Aligned Local Map", 10, 10, "v1 text", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, v2);
  //viewer->addText("Raw Local Map", 10, 10, "v2 text", v2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newScanAlg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newScanRaw (new pcl::PointCloud<pcl::PointXYZRGB>);

  int nPnt = algCloud->size();
  int iScanAlg = 0, iScanRaw = 0;
  for(int i=0; i < nPnt; i++){ 
     if (viewer->wasStopped () || iScanAlg > MAX_SCAN || iScanRaw > MAX_SCAN) {
	algCloud->clear();
	newScanAlg->clear();
	newScanRaw->clear();
	break;
     }

     /// Handle the aligned point cloud and visualize
     if (algCloud->points[i].x == 0 &&
	 algCloud->points[i].y == 0 &&
	 algCloud->points[i].z == 0){
	  if (newScanAlg->size() != 0){
	  	
		ostringstream conv;
		conv << "alg_" << iScanAlg;
			
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(newScanAlg);
		viewer->addPointCloud<pcl::PointXYZRGB> (newScanAlg, rgb, conv.str(), v1);
  		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, conv.str());
		cout << "scan: " << iScanAlg << "\t";
		newScanAlg->clear();
		iScanAlg++;

    		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (1000));
	  }
	}
     else{
	  pcl::PointXYZRGB pointRGB;
	  pointRGB.x = algCloud->points[i].x;
	  pointRGB.y = algCloud->points[i].y;
	  pointRGB.z = algCloud->points[i].z;

	  ColorMap c;
	  c = colorBlend( algCloud->points[i], XMAX, XMIN );
	  uint32_t rgb = (static_cast<uint32_t>(c.r) << 16 |
              static_cast<uint32_t>(c.g) << 8 | static_cast<uint32_t>(c.b));
	  pointRGB.rgb = *reinterpret_cast<float*>(&rgb);

	  newScanAlg->push_back( pointRGB );
	}
     
     /// Handle the raw point cloud and visualize
     if (rawCloud->points[i].x == 0 &&
	 rawCloud->points[i].y == 0 &&
	 rawCloud->points[i].z == 0){
	  if (newScanRaw->size() != 0){
	  	
		ostringstream conv;
		conv << "raw_" << iScanRaw;
			
  		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(newScanRaw);
		viewer->addPointCloud<pcl::PointXYZRGB> (newScanRaw, rgb, conv.str(), v2);
  		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, conv.str());
		cout << iScanRaw << endl;
		newScanAlg->clear();
		iScanRaw++;

    		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (1000));
	  }
	}
     else{
	  pcl::PointXYZRGB pointRGB;
	  pointRGB.x = rawCloud->points[i].x;
	  pointRGB.y = rawCloud->points[i].y;
	  pointRGB.z = rawCloud->points[i].z;

	  ColorMap c;
	  c = colorBlend( rawCloud->points[i], XMAX, XMIN );
	  uint32_t rgb = (static_cast<uint32_t>(c.r) << 16 |
              static_cast<uint32_t>(c.g) << 8 | static_cast<uint32_t>(c.b));
	  pointRGB.rgb = *reinterpret_cast<float*>(&rgb);

	  newScanRaw->push_back( pointRGB );
	}

  }
  algCloud->clear();


  /// While loop that keeps the viewer 
  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ostream>
#include <cstdlib>
#include <vector>
#include <ctime>

int main (int argc, char** argv)
{time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	char input[6][512];
	int switch_,max_iterations;
	float angle;
	int args_number = 0,string_args_number = 0;
	/*
	std::cout << "syntax: <input cloud> <output cloud> <output dir> <switch> <markers path>" << std::endl;
	std::cout << "switch 0 -> init transformation: Horn rotation + translation from nearest marker vector "
				"searchMethod: normals shooting with normal angle condition  <normals angle> \
				 <max iterations>" << std::endl;
	std::cout << "switch 1 -> init transformation: rotation&translation from Horn algorithm "
				"searchMethod: normals shooting with normal angle condition <normals angle> \
				 <max iterations>" << std::endl;
	std::cout << "switch 2 -> init transformation: identity matrix searchMethod: normals shooting  \
				<normals angle> <max iterations>" << std::endl;
	std::cout << "switch 3 -> nearest search with normal angle condition <normals angle> <max iterations>" << std::endl;
	std::cout << "switch 4 -> just nearest search <max iterations>" << std::endl;
	*/
	mclInitializeApplication(NULL,0);
	invMatrixInitialize();
	if(argc < 5)
	{
		std::cout << "Wrong number of arguments" << std::endl;
		return 1;
	}
	switch_ = atoi(argv[4]);
	if((switch_ < 4 && switch_ >= 0 && argc != 8) || (switch_ == 4 && argc != 7)) {
			std::cout << "Unrecognized switch value or wrong number of arguments" << std::endl;
			return 1;
	}
	char buf[512];
	double ALPHA_INIT[] = { 100};//,100,10,1,0.1,0.01};
	double ALPHA_STEP[] = { 0.1};
	double ALPHA_CONDITION[] = { 0.5};//,0.1,0.02,0.01};
	char spec[512];
	//for(int init = 0;init < 1;++init) {
	//for(int step = 0; step < 1;++step) {
	//for(int condition = 0;condition < 1;++condition) {
	for(int a = 12;a >=11;a--) {
	for(int i = 4;i <= 5;++i) {
		for(int j = 0; j < 1;++j) {
			if(a == 210 && i ==3)
				i = 4;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> Final; 
	
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		sprintf(buf,"..//test_%d_%d_%d_%d_switch_%d",timeinfo->tm_mday,timeinfo->tm_mon+1,
			timeinfo->tm_hour,timeinfo->tm_min,i);
		icp.setOutputDir(buf);
		mkdir(buf);
		sprintf(buf,"%s//info",buf);
		std::ofstream file(buf);
		icp.alpha_ = ALPHA_INIT[0];
		icp.alpha_step = ALPHA_STEP[0];


		sprintf(spec,"wyniki_nowe_2/wyniki200/marker_source_001_%d.pcd",a);
		//sprintf(spec,"wyniki300/marker_source_001.pcd");
		file << "Input cloud " << spec << std::endl;
		pcl::io::loadPCDFile(spec,*cloud_in);
		icp.setInputCloud(cloud_in);
		sprintf(spec,"wyniki_nowe_2/wyniki200/marker_target_001_%d.pcd",a);
		//sprintf(spec,"wyniki300/marker_target_001.pcd");
		pcl::io::loadPCDFile(spec,*cloud_out);
		icp.setInputTarget(cloud_out); 
		file << "Output cloud " << spec << std::endl;
		sprintf(spec,"wyniki_nowe_2/wyniki200/markersk_icp.txt",a);
		//sprintf(spec,"wyniki300/markersk_icp.txt");
		icp.setLandmarksPath(spec);
		double w_max=-10000000,w_min=1000000,h_max=-1000000,h_min=1000000;
		for(int i = 0;i < cloud_in->size();++i) {
			if(cloud_in->at(i).x < w_min) {
				w_min = cloud_in->at(i).x;
			}
			else if(cloud_in->at(i).x > w_max){
				w_max = cloud_in->at(i).x;
			}
			if(cloud_in->at(i).y < h_min) {
				h_min = cloud_in->at(i).y;
			}
			else if(cloud_in->at(i).y > h_max){
				h_max = cloud_in->at(i).y;
			}
		}
		float x = (((float)cloud_in->size())*0.05);
		//if(j == 0)
			icp.neighbors_= (int)(((float)cloud_in->size())*0.05);
		//else if(j == 1)
			
		//icp.neighbors_= (int)(((float)cloud_in->size())*0.1);
		//else
			
		//icp.neighbors_= (int)(((float)cloud_in->size())*0.15);
			
		if(j == 0)
		icp.radius_= 0.05*(w_max-w_min);
		else if(j == 1)
			
		icp.radius_= 0.1*(w_max-w_min);
		else
			
		icp.radius_= 0.15*(w_max-w_min);
		file << "Markers " << spec << std::endl;
		file << "Switch " << i << std::endl;
		file << "Alpha init value: " << icp.alpha_ << std::endl; 
		file << "Alpha step: " << icp.alpha_step << std::endl; 
		file << "Alpha change condition: " << ALPHA_CONDITION[0] << std::endl; 
		file.close();
		icp.setTransformationEpsilon(ALPHA_CONDITION[0]);
		icp.setSearchMethod(i);
		if(i !=  4)
			icp.setMaxAngle(atof(argv[argc-2]));
		else
			icp.setMaxAngle(360.0f);
		//else
		//	icp.setMaxAngle(360.0f);
		icp.setMaximumIterations(atoi(argv[argc-1]));
		//if(switch_ == 1 || switch_ == 0)
		//{
			icp.getInitTransformation();//argv[6]);
		//}
		//icp.getInitTransformation();
		/*
		else if(switch_ == 2)
		{
			icp.setLandmarksPath(argv[5]);
			icp.getMarkers(argv[6],argv[5]);
		}
		else
			icp.setLandmarksPath("landmarks1.txt");*/
		icp.setAlphaPath("alph.txt");
		icp.setTopologyPath("top.txt");
		icp.align(Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	}
	}
	}
	mclTerminateApplication();	
	invMatrixTerminate();
  /*char buf[512];
  //NEW - EDIT 21.05.2012
  FILE * file = fopen(input[0],"r");
  if(!file)
  {
	  sprintf(buf,"Could not open file: %s\n",input[0]);
	  return 1;
  }
  fscanf(file,"%d %d",&cloud_in->width,&cloud_in->height);
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
	if(fscanf(file,"%f %f %f",&cloud_in->points[i].x,&cloud_in->points[i].y,&cloud_in->points[i].z) != 3)
		return 1;
  }
  fclose(file);
  file = fopen(input[1],"r");
  if(!file)
  {
	  sprintf(buf,"Could not open file: %s\n",input[1]);
	  return 1;
  }
  fscanf(file,"%d %d",&cloud_out->width,&cloud_out->height);
  cloud_out->is_dense = false;
  cloud_out->points.resize (cloud_out->width * cloud_out->height);
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
  {
	if(fscanf(file,"%f %f %f",&cloud_out->points[i].x,&cloud_out->points[i].y,&cloud_out->points[i].z) != 3)
		return 1;
  }
  fclose(file);*/


  
	/*
  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + .7;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
  cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);*/
  //std::cout << "Matrix with marker vectors:" << std::endl;
  //icp.getMarkersMatrix(markers_matrix);
  //std::cout << markers_matrix;
	return (0);
}

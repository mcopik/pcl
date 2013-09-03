/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: registration.hpp 2532 2011-09-20 20:39:18Z bouffa $
 *
 */



 
//////////////////////////////////////////////////////////////////////////////////////////////
/*
*	Marcin Copik
*	Silesian University of Technology
*/

#include "RigidLandmarkRegistration.h"
template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getMarkers(const char * rectangles_path,const char * landmarks_path)
{	
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr markers_(new pcl::PointCloud<pcl::PointXYZ>);
	FILE* file = nullptr;
    double bufor1 = 0.0, bufor2 = 0.0, bufor3 = 0.0;
    int lba_elem = 0,temp = 0,input_size = 0,n = 0;
	double * marker_vectors_ = nullptr;
	// Read landmarks
    file = fopen(landmarks_path,"rb"); 
    fscanf (file, "%d", &n);
	marker_vectors_ = new double[3*n];
    markers_->width = n;
    markers_->height = 1;
    markers_->resize(markers_->width * markers_->height);
    for(int i = 0; i < n; ++i)
    {   
			fscanf(file,"%d",&temp);
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			markers_->points[i].x = bufor1;
            markers_->points[i].y = bufor2;
            markers_->points[i].z = bufor3;
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			
			//NEW - EDIT 21.05.2012
			marker_vectors_[3*i] = bufor1 - markers_->points[i].x;
			marker_vectors_[3*i+1] = bufor2 - markers_->points[i].y;
			marker_vectors_[3*i+2] = bufor3 - markers_->points[i].z;
    }
	fclose(file);
	double * marker_vectors_length = new double[n];
	for(int i =0;i < n;++i)
		marker_vectors_length[i] = pow(pow(marker_vectors_[3*i],2)+pow(marker_vectors_[3*i+1],2)+pow(marker_vectors_[3*i+2],2),1.0/2.0);
	input_size = (*input_).points.size();
    file = fopen(rectangles_path,"rb"); 
	fscanf(file,"%d",&lba_elem);
	float ax,ay,bx,by;
	int marker;
	int * mask = new int[input_size];
	double * sum = new double[input_size];
	for(int i =0;i< input_size;++i)
		sum[i] = 0;
	int * markers = new int[lba_elem];
	for(int i =0;i < input_size;++i)
		mask[i] = 0;
	for(int i = 0;i < lba_elem;++i)
	{
		fscanf(file,"%f %f %f %f %d",&ax,&ay,&bx,&by,&marker);
		markers[i] = marker;
		for(int j =0;j < input_size;++j)
		{
			if(input_->points[j].x >= ax && input_->points[j].x <= bx && (*input_).points[j].y >= by &&\
					(*input_).points[j].y <= ay)
			{
				mask[j] |= 1 << i;
				sum[j] += marker_vectors_length[marker];
			}
		}
	}
	marker_vectors.resize(input_size,3);
	float x,y,z;
	for (size_t i = 0; i < input_size; ++i)
	{
		x = 0;
		y = 0;
		z = 0;
		for(int j = 0;j < 32;++j)
		{
			if((mask[i] >> j) & 0x1)
			{
				x += marker_vectors_[3*markers[j]]*marker_vectors_length[markers[j]]/sum[i];
				y += marker_vectors_[3*markers[j]+1]*marker_vectors_length[markers[j]]/sum[i];
				z += marker_vectors_[3*markers[j]+2]*marker_vectors_length[markers[j]]/sum[i];
			}
		}
		marker_vectors.row(i) << x,y,z;
	}
	delete[] mask;
	delete[] sum;
	delete[] markers;
	delete[] marker_vectors_length;
	delete[] marker_vectors_;*/
} 

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getMarkers(const char * landmarks_path)
{
	initMatrixFlag = false;
	FILE* file = nullptr;
    double bufor1 = 0.0, bufor2 = 0.0, bufor3 = 0.0;
    int lba_elem = 0,temp = 0,input_size = 0,n = 0;
	double * marker_vectors_ = nullptr;
	KdTreePtr search_tree(new pcl::KdTreeFLANN<PointTarget>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr markers_(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::MatrixXf out_matrix;
	// Read landmarks
    file = fopen(landmarks_path,"rb"); 
    fscanf (file, "%d", &n);
	assert(n > 0);
	marker_vectors_ = new double[3*n];
    markers_->width = n;
    markers_->height = 1;
    markers_->resize(markers_->width * markers_->height);
    for(int i = 0; i < n; ++i)
    {   
			fscanf(file,"%d",&temp);
			assert(temp >= 0);
			fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			markers_->points[i].x = bufor1;
            markers_->points[i].y = bufor2;
            markers_->points[i].z = bufor3;
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			
			//NEW - EDIT 21.05.2012
			marker_vectors_[3*i] = bufor1 - markers_->points[i].x;
			marker_vectors_[3*i+1] = bufor2 - markers_->points[i].y;
			marker_vectors_[3*i+2] = bufor3 - markers_->points[i].z;
    }
	fclose(file);
	char path[512];
	sprintf(path,"%s\\marker_vectors.txt",output_dir);
	std::ofstream file1(path);
    for(int i = 0; i < n; ++i)
	file1 << marker_vectors_[3*i] << " " << marker_vectors_[3*i+1] << " " << marker_vectors_[3*i+2] << std::endl;
	file1 << "Assigning vectors to markers" << std::endl;
	// Find nearest neighbor for each element in input
	input_size = (*input_).points.size();
	initTransformation.resize(4*input_size,3);
	search_tree->setInputCloud(markers_);
	std::vector<int> indices(1);
	std::vector<float> distances(1);
	for (size_t i = 0; i < input_size; ++i)
	{
		search_tree->nearestKSearch(*input_,i,1,indices,distances);
		if(indices[0] < n)
		{
			initTransformation.row(4*i+3) << marker_vectors_[indices[0]*3],  marker_vectors_[indices[0]*3+1], marker_vectors_[indices[0]*3+2];
			file1 << "Point: " << (*input_).points[i].x << " " << (*input_).points[i].y << " "<< (*input_).points[i].z << std::endl;
			file1 << "Nearest marker:" <<  (*markers_).points[indices[0]].x << " " << (*markers_).points[indices[0]].y << " "<< (*markers_).points[indices[0]].z << std::endl;
			file1 << "Assigned vector: " << marker_vectors_[3*indices[0]] << " " << marker_vectors_[3*indices[0]+1] << " " << marker_vectors_[3*indices[0]+2] << std::endl;
			file1 << "------------------------------------------------" << std::endl;
		}
		else
		{
			std::cout << "Fatal error! \n Matched marker vector doesn't exist!" << std:: endl;
			delete[] marker_vectors_;
			return;
		}
	}
	delete[] marker_vectors_;
	file1.close();
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::findMarkersError(int iterationNumber, std::ofstream & fileOut,
pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_target) {
	
	FILE* file = nullptr;
    double bufor1 = 0.0, bufor2 = 0.0, bufor3 = 0.0;
    int lba_elem = 0,temp = 0,input_size = 0,n = 0;
	double * marker_vectors_ = nullptr;
	KdTreePtr searchSource(new pcl::KdTreeFLANN<PointTarget>);
	KdTreePtr searchTarget(new pcl::KdTreeFLANN<PointTarget>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr markersSource(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr markersTarget(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::MatrixXf out_matrix;

	// Read landmarks
    file = fopen(landmarks_path,"rb"); 
    fscanf (file, "%d", &n);
	assert(n > 0);

	//chmura na wspolrzedne x y z source markerow
    markersSource->width = n;
    markersSource->height = 1;
    markersSource->resize(markersSource->width * markersSource->height);

	//chmura na wspolrzedne x y z target markerow
    markersTarget->width = n;
    markersTarget->height = 1;
    markersTarget->resize(markersTarget->width * markersTarget->height);
    for(int i = 0; i < n; ++i)
    {   
			fscanf(file,"%d",&temp);
			assert(temp >= 0);
			fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			markersSource->points[i].x = bufor1;
            markersSource->points[i].y = bufor2;
            markersSource->points[i].z = bufor3;
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			markersTarget->points[i].x = bufor1;
            markersTarget->points[i].y = bufor2;
            markersTarget->points[i].z = bufor3;
    }
	fclose(file);


	// Find nearest neighbor for each element in input
    //dziala na chmurze output
	searchSource->setInputCloud(cloud_src);
    //dziala na chmurze target 
	searchTarget->setInputCloud(cloud_target);

	std::vector<int> indices(1);
	std::vector<float> distances(1);
	size_t sizeSource = cloud_src->size();
	size_t sizeTarget = cloud_target->size();

	//w petli po wszystkich markerach
	for (size_t i = 0; i < n; ++i)
	{
		//zmienne na indeksy znalezionych najblizszych elementow
		int indiceSource,indiceTarget;

		//DLA CHMURY OUTPUT
		//szukanie sasiada dla i-tego markera w chmurze output
		int res = searchSource->nearestKSearch(*markersSource,i,1,indices,distances);
	    //sprawdzenie czy nie wychodzimy z indeksem poza chmure
		if(indices[0] < sizeSource)
		{
			indiceSource = indices[0];
		}

		//DLA CHMURY TARGET
		//szukanie sasiada dla i-tego markera w chmurze target
		res = searchTarget->nearestKSearch(*markersTarget,i,1,indices,distances);
		//sprawdzenie czy nie wychodzimy z indeksem poza chmure
		if(indices[0] < sizeTarget)
		{
			indiceTarget = indices[0];
		}


		fileOut.width(15);
		fileOut.precision(5);
		fileOut << iterationNumber << " ";
		
		fileOut.width(25);
		fileOut.precision(5);
		//numer najblizszego punktu w source dla danego markera
		fileOut << indiceSource << " ";
		
		fileOut.width(15);
		fileOut.precision(5);
        //numer najblizszego punktu w target dla danego markera
		fileOut << indiceTarget << " ";

		fileOut.width(15);
		fileOut.precision(5);
		//roznca indeksow w source i target
		fileOut	<< std::abs(indiceTarget-indiceSource) << " ";

		fileOut.width(15);
		fileOut.precision(5);
		//odleglosc punktow odpowiadajacym markerom w source i target
		fileOut << std::sqrt(std::pow(cloud_target->points[indiceTarget].x-cloud_src->points[indiceSource].x,2)+
						std::pow(cloud_target->points[indiceTarget].y-cloud_src->points[indiceSource].y,2)+
						std::pow(cloud_target->points[indiceTarget].z-cloud_src->points[indiceSource].z,2)) << std::endl;
	}

	//oddzielenie graficzne danej iteracji w pliku wyniku
	fileOut << "-----------------------------------------------" << std::endl << std::endl;
}
     	 template <typename PointSource, typename PointTarget> inline  void pcl::Registration<PointSource, PointTarget>::znajdzWierszKolumnePktChmury(int width, int height, int index,int* wiersz, int* kolumna)
	   {   

		   //zalozenia:
		   // width - liczba elementow w wierszu - liczba kolumn 
		   // chmura jest przetwarzana wierszami

		   //ile wierszy ca³kowitych miesci sie w index
		   *wiersz= index / width;
		   *kolumna = index % width;


	   }




      template <typename PointSource, typename PointTarget> inline void
      pcl::Registration<PointSource, PointTarget>::znajdzPktNajblizszeLandmark(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_target, std::vector<int> &marker_source_indices, std::vector<int> &marker_target_indices)
	  {
	
		FILE* file = nullptr;
		double bufor1 = 0.0, bufor2 = 0.0, bufor3 = 0.0;
		int lba_elem = 0,temp = 0,input_size = 0,n = 0;
		double * marker_vectors_ = nullptr;

		//drzewa do wyszkiwan
		KdTreePtr searchSource(new pcl::KdTreeFLANN<PointTarget>);
		KdTreePtr searchTarget(new pcl::KdTreeFLANN<PointTarget>);

		//chmury na wspolrzedne markerow
		pcl::PointCloud<pcl::PointXYZ>::Ptr markersSource(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr markersTarget(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::MatrixXf out_matrix;

		// Read landmarks
		file = fopen(landmarks_path,"rb"); 
		fscanf (file, "%d", &n);
		assert(n > 0);

		//chmura na wspolrzedne x y z source markerow
		markersSource->width = n;
		markersSource->height = 1;
		markersSource->resize(markersSource->width * markersSource->height);

		//chmura na wspolrzedne x y z target markerow
		markersTarget->width = n;
		markersTarget->height = 1;
		markersTarget->resize(markersTarget->width * markersTarget->height);
		for(int i = 0; i < n; ++i)
		{   
				fscanf(file,"%d",&temp);
				assert(temp >= 0);
				fscanf (file, "%lf" ,&bufor1);
				fscanf (file, "%lf" ,&bufor2);
				fscanf (file, "%lf" ,&bufor3);
				markersSource->points[i].x = bufor1;
				markersSource->points[i].y = bufor2;
				markersSource->points[i].z = bufor3;
				fscanf (file, "%lf" ,&bufor1);
				fscanf (file, "%lf" ,&bufor2);
				fscanf (file, "%lf" ,&bufor3);
				markersTarget->points[i].x = bufor1;
				markersTarget->points[i].y = bufor2;
				markersTarget->points[i].z = bufor3;
		}
		fclose(file);


		// Find nearest neighbor for each element in input
		//dziala na chmurze output
		searchSource->setInputCloud(cloud_src);
		//dziala na chmurze target 
		searchTarget->setInputCloud(cloud_target);

		std::vector<int> indices(1);
		std::vector<float> distances(1);
		size_t sizeSource = cloud_src->size();
		size_t sizeTarget = cloud_target->size();

		//w petli po wszystkich markerach
		for (size_t i = 0; i < n; ++i)
		{
			

			//DLA CHMURY OUTPUT
			//szukanie sasiada dla i-tego markera w chmurze output
			int res = searchSource->nearestKSearch(*markersSource,i,1,indices,distances);
			//sprawdzenie czy nie wychodzimy z indeksem poza chmure
			if(indices[0] < sizeSource)
			{
				marker_source_indices[i] = indices[0];
			}

			//DLA CHMURY TARGET
			//szukanie sasiada dla i-tego markera w chmurze target
			res = searchTarget->nearestKSearch(*markersTarget,i,1,indices,distances);
			//sprawdzenie czy nie wychodzimy z indeksem poza chmure
			if(indices[0] < sizeTarget)
			{
				marker_target_indices[i] = indices[0];
			}


		//	fileOut.width(15);
		//	fileOut.precision(5);
		//	fileOut << iterationNumber << " ";
		//
		//	fileOut.width(25);
		//	fileOut.precision(5);
		//	//numer najblizszego punktu w source dla danego markera
		//	fileOut << indiceSource << " ";
		//
		//	fileOut.width(15);
		//	fileOut.precision(5);
		//	//numer najblizszego punktu w target dla danego markera
		//	fileOut << indiceTarget << " ";

		//	fileOut.width(15);
		//	fileOut.precision(5);
		//	//roznca indeksow w source i target
		//	fileOut	<< std::abs(indiceTarget-indiceSource) << " ";

		//	fileOut.width(15);
		//	fileOut.precision(5);
		//	//odleglosc punktow odpowiadajacym markerom w source i target
		//	fileOut << std::sqrt(std::pow(cloud_target->points[indiceTarget].x-cloud_src->points[indiceSource].x,2)+
		//					std::pow(cloud_target->points[indiceTarget].y-cloud_src->points[indiceSource].y,2)+
		//					std::pow(cloud_target->points[indiceTarget].z-cloud_src->points[indiceSource].z,2)) << std::endl;
		}

		////oddzielenie graficzne danej iteracji w pliku wyniku
		//fileOut << "-----------------------------------------------" << std::endl << std::endl;
}  




/*
template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getMarkersDominik(const char * landmarks_path)
{
	FILE* file = nullptr;
    double bufor1 = 0.0, bufor2 = 0.0, bufor3 = 0.0;
    int lba_elem = 0,temp = 0,input_size = 0,n = 0;
	double * marker_vectors_ = nullptr;
	KdTreePtr search_tree(new pcl::KdTreeFLANN<PointTarget>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr markers_(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::MatrixXf out_matrix;
	// Read landmarks
    file = fopen(landmarks_path,"rb"); 
    fscanf (file, "%d", &n);
	marker_vectors_ = new double[3*n];
    markers_->width = n;
    markers_->height = 1;
    markers_->resize(markers_->width * markers_->height);
    for(int i = 0; i < n; ++i)
    {   
			fscanf(file,"%d",&temp);
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			markers_->points[i].x = bufor1;
            markers_->points[i].y = bufor2;
            markers_->points[i].z = bufor3;
            fscanf (file, "%lf" ,&bufor1);
            fscanf (file, "%lf" ,&bufor2);
            fscanf (file, "%lf" ,&bufor3);
			
			//NEW - EDIT 21.05.2012
			marker_vectors_[3*i] = bufor1 - markers_->points[i].x;
			marker_vectors_[3*i+1] = bufor2 - markers_->points[i].y;
			marker_vectors_[3*i+2] = bufor3 - markers_->points[i].z;
    }
	fclose(file);
	
	// Find nearest neighbor for each element in input
	input_size = (*input_).points.size();
	initTransformation.resize(input_size,3);
	search_tree->setInputCloud(markers_);
	std::vector<int> indices(1);
	std::vector<float> distances(1);
	for (size_t i = 0; i < input_size; ++i)
	{
		search_tree->nearestKSearch(*input_,i,1,indices,distances);
		if(indices[0] < n)
		{
			initTransformation.row(i) << marker_vectors_[indices[0]*3],  marker_vectors_[indices[0]*3+1], marker_vectors_[indices[0]*3+2];
		}
		else
		{
			std::cout << "Fatal error! \n Matched marker vector doesn't exist!" << std:: endl;
			delete[] marker_vectors_;
			return;
		}
	}
	delete[] marker_vectors_;
	/*
	std::cout << "INPUT CLOUD: " << std::endl;
	for (size_t i = 0; i < input_->points.size (); ++i)
    std::cout << "    " << input_->points[i].x << " " << 
	input_->points[i].y << " " << input_->points[i].z << std::endl;
	std::cout << "MARKERS/LANDMARKS " << std::endl;
	for (size_t i = 0; i < markers_->points.size (); ++i)
    std::cout << "    " << markers_->points[i].x << " " << 
	markers_->points[i].y << " " << markers_->points[i].z << std::endl;
	std::cout << "MATRIX: "<< std::endl;
	std::cout << out_matrix;
}
*/

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getMarkersHorn(const char * landmarks_path)
{
	initMatrixFlag = true;
	FILE* file = nullptr;
    int lba_elem = 0,temp = 0,n = 0;
	// Read landmarks
    file = fopen(landmarks_path,"rb"); 
    fscanf (file, "%d", &n);
	assert(n > 0);
	double * tab = new double[6*n];
    for(int i = 0; i < n; ++i)
    {   
			fscanf(file,"%d",&temp);
			assert(temp >= 0);
            fscanf (file, "%lf" ,&tab[3*i]);
            fscanf (file, "%lf" ,&tab[3*i+1]);
            fscanf (file, "%lf" ,&tab[3*i+2]);
            fscanf (file, "%lf" ,&tab[3*(n+i)]);
            fscanf (file, "%lf" ,&tab[3*(n+i)+1]);
            fscanf (file, "%lf" ,&tab[3*(n+i)+2]);
			
    }
	
	double transform[4][4];
   LandmarkRegistration::RigidLandmarkRegistration::RejestracjaSztywna(n,tab,transform);
   char tab2[512];
   sprintf(tab2,"%s\\horn.txt",output_dir);
   std::ofstream file2(tab2);
   for(int i = 0;i < 4;++i){
	   for(int j = 0;j < 4;++j) {
		   file2 << transform[i][j] << " ";
	   }
	file2 << std::endl;
   }
	   file2.close();
	delete[] tab;
   int input_size = (*input_).points.size();
   initTransformation.resize(4*input_size,3);
   for(int k = 0;k < input_size;++k){
   for(int i = 0;i < 4;++i)
   initTransformation.row(4*k+i) << transform[0][i], transform[1][i], transform[2][i];
   }
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getInitTransformation()
{
	if(searchMethod == 0 || searchMethod == 1) {
		getMarkersHorn(landmarks_path);
		if(searchMethod == 0) {
			getMarkers(landmarks_path);
		}
	}
	else {
		Eigen::MatrixXf init_xi_transformation(4,3);
		init_xi_transformation << 1, 0, 0,   0, 1, 0,   0, 0, 1,   0,0,0;

		Eigen::MatrixXf init_temp_transformation(input_->size(),1);
		init_temp_transformation.fill(1);
		initTransformation.resize(4*input_->size(),3);
		initTransformation = computeinitXtransformation(init_temp_transformation, init_xi_transformation);
	}

}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getMarkersMatrix(Eigen::MatrixXf & out_matrix)
{
	if(initMatrixFlag)
		out_matrix = marker_vectors;
	else {
		int n = marker_vectors.rows();
		out_matrix.resize(4*n,3);
		for(int i = 0;i < n;++i)
		{
			out_matrix.row(i*4) << 1, 0, 0; ;
			out_matrix.row(i*4+1) << 0, 1, 0;
			out_matrix.row(i*4+2) << 0, 0, 1;
			///i have no idea why it has to be done this way...
			//out_matrix.row(i*4+3) << marker_vectors(i,2),  marker_vectors(i,0), marker_vectors(i,1);
			out_matrix.row(i*4+3) << marker_vectors(i,0),  marker_vectors(i,1), marker_vectors(i,2);
		}
	}
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getTopologyMatrix(Eigen::MatrixXf & topology,int width, int height,bool flag)
{
	int points = width*height;
	int edges = 0;
	if(flag)
		edges = (width-1)*(height-1)*3 + (height-1) + (width-1);
	else
		edges = (width-1)*(height-1)*2 + (height-1) + (width-1);
	int ones[] = {0,0,0};
	topology.resize(edges,points);
	int * Buffer = new int[points];
	for(int i = 0;i < points;++i)
		Buffer[i] = 0;
	int counter = 0;
	for(int i = 0;i < height-1;++i)
	{
		for(int j = 0;j < width-1;++j)
		{
			Buffer[i*width+j] = -1;
			ones[0] = i*width+j+1;
			ones[2] = ones[0] + width;
			ones[1] = ones[2] - 1;
			for(int z = 0;z < (2+flag);++z)
			{
				Buffer[ones[z]] = 1;
				for(int k = 0;k < points;++k)
					topology(counter,k) = Buffer[k];
				counter++;
				Buffer[ones[z]] = 0;
			}
			Buffer[i*width+j] = 0;
		}
		Buffer[(i+1)*width-1] = -1;
		Buffer[(i+2)*width-1] = 1;
		for(int k = 0;k < points;++k)
			topology(counter,k) = Buffer[k];
		counter++;
		Buffer[(i+1)*width-1] = 0;
		Buffer[(i+2)*width-1] = 0;
	}
	for(int i = 0;i < width-1;++i)
	{
		Buffer[(height-1)*width+i] = -1;
		Buffer[(height-1)*width+i+1] = 1;
		for(int k = 0;k < points;++k)
			topology(counter,k) = Buffer[k];
		counter++;
		Buffer[(height-1)*width+i] = 0;
		Buffer[(height-1)*width+i+1] = 0;
	}
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::getAlphaMatrix(Eigen::MatrixXf & topology,int width,int height,bool flag,double alpha_value)
{
	int points = width*height;
	int edges = 0;
	if(flag)
		edges = (width-1)*(height-1)*3 + (height-1) + (width-1);
	else
		edges = (width-1)*(height-1)*2 + (height-1) + (width-1);
	topology.resize(edges,points);
	for(int i = 0;i < edges;++i)
	{
		for(int j = 0;j < points;++j)
		{
			topology(i,j) = alpha_value;
		}
	}
}

template <typename PointSource, typename PointTarget> inline Eigen::Vector3d 
pcl::Registration<PointSource, PointTarget>::loadMarkerVector(char* path)
{
	FILE* file;
	file = fopen(path, "r");
	int m,n;
	float temp;    //m - liczba krawedzi  ; n- liczba wierzcholkow
			
	fscanf(file,"%d", &m);
	fscanf(file,"%d", &n);

	Eigen::Vector3d vec;

	for( int i = 0; i < 3; i++)
	{
			fscanf(file,"%f", &temp);
			vec(i) = temp;
		}

	fclose(file);
	
	return vec;
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::loadTopologyMatrxSize(char* path, int* m,int* n )
{
	int width = input_->width;
	int height = input_->height;
	int points = width*height;
	int edges = 0;
	//if(flag)
	//	edges = (width-1)*(height-1)*3 + (height-1) + (width-1);
	//else
		edges = (width-1)*(height-1)*2 + (height-1) + (width-1);
	*m = edges;
	*n = points;
}

template <typename PointSource, typename PointTarget> inline Eigen::MatrixXf
pcl::Registration<PointSource, PointTarget>::loadTopologyFile(char* path)
{/*
	FILE* file;
	file = fopen(path, "r");
	int m,n, temp;    //m - liczba krawedzi  ; n- liczba wierzcholkow
			
	fscanf(file,"%d", &m);
	fscanf(file,"%d", &n);


	for( int i = 0; i < m; i++)
		for( int j = 0; j < n; j++)
		{
			fscanf(file,"%d", &temp);
			M(i,j) = temp;
		}

	fclose(file);
	*/
	Eigen::MatrixXf M;
	if(!strcmp(path,topology_path))
		getTopologyMatrix(M,input_->width,input_->height,false);
	else
		getAlphaMatrix(M,input_->width,input_->height,false,5.0);
	return M;
}

template <typename PointSource, typename PointTarget> inline Eigen::MatrixXf
pcl::Registration<PointSource, PointTarget>::loadTopologyFile(char* path,double alpha_val)
{/*
	FILE* file;
	file = fopen(path, "r");
	int m,n, temp;    //m - liczba krawedzi  ; n- liczba wierzcholkow
			
	fscanf(file,"%d", &m);
	fscanf(file,"%d", &n);


	for( int i = 0; i < m; i++)
		for( int j = 0; j < n; j++)
		{
			fscanf(file,"%d", &temp);
			M(i,j) = temp;
		}

	fclose(file);
	*/
	Eigen::MatrixXf M;
	getAlphaMatrix(M,input_->width,input_->height,false,alpha_val);
	return M;
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setLandmarksPath(const char * path)
{
	strcpy(landmarks_path,path);
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setAlphaPath(const char * path)
{
	strcpy(alpha_path,path);
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setTopologyPath(const char * path)
{
	strcpy(topology_path,path);
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setOutputDir(const char * path)
{
	strcpy(output_dir,path);
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setMaxAngle(float angle)
{
	maxNormalsAngle = angle;
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setSearchMethod(int switch_)
{
	searchMethod = switch_;
}

template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  PointCloudTarget target = *cloud;
  // Set all the point.data[3] values to 1 to aid the rigid transformation
  for (size_t i = 0; i < target.points.size (); ++i)
    target.points[i].data[3] = 1.0;

  //target_ = cloud;
  target_ = target.makeShared ();

   tree_->setInputCloud (target_);
   int i =  target_->size();
   //DOMINIK
    pcl::PointCloud<pcl::Normal>::Ptr target_normals_init (new pcl::PointCloud<pcl::Normal>);
    target_normals_  = target_normals_init;
	ObliczNormalnePktChmury(target_, *target_normals_, 5);


	 char MarkerVectorpath[512] = "marker_vectors.txt";

	 Eigen::Vector3d marker_vectors;
	marker_vectors = loadMarkerVector(MarkerVectorpath);

	 //DOMINIK
  // inicjalizacja transformaty aficnzicnej dl a i-tego opunkt;
  Eigen::MatrixXf init_xi_transformation(4,3);
  init_xi_transformation << 1, 0, 0,   0, 1, 0,   0, 0, 1,  0,0,0; //marker_vectors(0),marker_vectors(1), marker_vectors(2);
  init_xi_transformation << 1, 0, 0,   0, 1, 0,   0, 0, 1, marker_vectors(0),marker_vectors(1), marker_vectors(2);

  //std::cout << init_xi_transformation;
  //std::cout << "\n";
  //std::cout << "marker_vectors \n";
  //std::cout << marker_vectors << "\n";

  //wykorzystywana do peoduktu Kroneckera
  Eigen::MatrixXf init_temp_transformation(input_->size(),1);
  init_temp_transformation.fill(1);
  //std::cout << init_temp_transformation;
  //std::cout << "\n";

  //wektor X poczatkowy tranformat afinicznych
  marker_transformation_matrix.resize(4*input_->size(),3);
  marker_transformation_matrix = computeinitXtransformation(init_temp_transformation, init_xi_transformation);
//  std::cout << marker_transformation_matrix;
  //std::cout << "\n";





}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
pcl::Registration<PointSource, PointTarget>::getFitnessScore (const std::vector<float> &distances_a, 
                                                              const std::vector<float> &distances_b)
{
  unsigned int nr_elem = std::min (distances_a.size (), distances_b.size ());
  Eigen::VectorXf map_a = Eigen::VectorXf::MapAligned (&distances_a[0], nr_elem);
  Eigen::VectorXf map_b = Eigen::VectorXf::MapAligned (&distances_b[0], nr_elem);
  return ((map_a - map_b).sum () / nr_elem);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
pcl::Registration<PointSource, PointTarget>::getFitnessScore (double max_range)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  transformPointCloud (*input_, input_transformed, transformation_,0,0);

  std::cout << "Macierz transformacji transformation transformation_";
  //std::cout << transformation_ ;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input_transformed.points[i].x,
                                          input_transformed.points[i].y,
                                          input_transformed.points[i].z, 0);
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    
    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target_->points[nn_indices[0]].x,
                                          target_->points[nn_indices[0]].y,
                                          target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
pcl::Registration<PointSource, PointTarget>::getFitnessScoreDominik (pcl::PointCloud<pcl::PointXYZ> &output, double max_range)
{
  double fitness_score = 0.0;


  //Nie musze przesuwac chmury bo juz jest przesunieta
  //// Transform the input dataset using the final transformation
  //PointCloudSource output_transformed;

  // // Tranform the data
  //transformPointCloud(output,  output, transformation_, 0, 0);
  //
  ////transformPointCloud (*input_, input_transformed, transformation_,0,0);

  std::cout << "Macierz transformacji transformation transformation_";
  //std::cout << transformation_ ;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < output.points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (output.points[i].x,
                                          output.points[i].y,
                                          output.points[i].z, 0);
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (output.points[i], 1, nn_indices, nn_dists);
    
    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target_->points[nn_indices[0]].x,
                                          target_->points[nn_indices[0]].y,
                                          target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}




//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::align (PointCloudSource &output)
{
  if (!initCompute ()) return;

  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = (int) indices_->size ();
    output.height   = 1;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  // Perform the actual transformation computation
  converged_ = false;
  char path[512];
  sprintf(path,"%s\\initTransformation.txt",output_dir);
  std::ofstream file(path);
  file << initTransformation << std::endl;
  final_transformation_ = transformation_ = previous_transformation_ = initTransformation;
  // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid 
  // transformation
  file.close();
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i].data[3] = 1.0;

  computeTransformation (output, initTransformation);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::Registration<PointSource, PointTarget>::align (PointCloudSource &output, const Eigen::MatrixXf& guess)
{
  if (!initCompute ()) return;

  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = indices_->size ();
    output.height   = 1;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  // Perform the actual transformation computation
  converged_ = false;


   //DOMINIK
  // inicjalizacja transformaty aficnzicnej dl a i-tego opunkt;
  Eigen::MatrixXf init_xi_transformation(4,3);
  init_xi_transformation << 1, 0, 0,   0, 1, 0,   0, 0, 1,   0,0,0;
 // std::cout << init_xi_transformation;

  //wykorzystywana do peoduktu Kroneckera
  Eigen::MatrixXf init_temp_transformation(input_->size(),1);
  init_temp_transformation.fill(1);
  //std::cout << init_temp_transformation;

  //wektor X poczatkowy tranformat afinicznych
  Eigen::MatrixXf init_X_transformation(4*input_->size(),3);
  init_X_transformation = computeinitXtransformation(init_temp_transformation, init_xi_transformation);
  //std::cout << init_X_transformation;



  //final_transformation_ = transformation_ = previous_transformation_ = init_X_transformation;

  // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid 
  // transformation
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i].data[3] = 1.0;

  computeTransformation (output, guess);

  deinitCompute ();
}


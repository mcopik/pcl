/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: registration.h 3041 2011-11-01 04:44:41Z rusu $
 *
 */

#ifndef PCL_REGISTRATION_H_
#define PCL_REGISTRATION_H_

#include <boost/function.hpp>
#include <boost/bind.hpp>

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/win32_macros.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/registration/transformation_estimation.h"

namespace pcl
{
  /** \brief @b Registration represents the base registration class. 
    * All 3D registration methods should inherit from this class.
    * \author Radu Bogdan Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget>
  class Registration : public PCLBase<PointSource>
  {
    public:
      using PCLBase<PointSource>::initCompute;
      using PCLBase<PointSource>::deinitCompute;
      using PCLBase<PointSource>::input_;
      using PCLBase<PointSource>::indices_;

      typedef boost::shared_ptr< Registration<PointSource, PointTarget> > Ptr;
      typedef boost::shared_ptr< const Registration<PointSource, PointTarget> > ConstPtr;

      typedef typename pcl::KdTree<PointTarget> KdTree;
      typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;
     
      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

	  //Dominik
	  



      typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;
      
      typedef typename pcl::registration::TransformationEstimation<PointSource, PointTarget> TransformationEstimation;
      typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;
      typedef typename TransformationEstimation::ConstPtr TransformationEstimationConstPtr;

      /** \brief Empty constructor. */
      Registration () : nr_iterations_(0),
                        max_iterations_(10),
                        target_ (),
                        //final_transformation_ (Eigen::Matrix4f::Identity ()),
                        //transformation_ (Eigen::Matrix4f::Identity ()),
                        //previous_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_epsilon_ (0.0), 
                        euclidean_fitness_epsilon_ (-std::numeric_limits<double>::max ()),
                        corr_dist_threshold_ (std::sqrt (std::numeric_limits<double>::max ())),
                        inlier_threshold_ (0.05),
                        converged_ (false), min_number_correspondences_ (3), 
                        transformation_estimation_ (),
                        point_representation_ ()
      {

		  ///copik
        //char path[512]  ="topology.txt ";
		//int m,n;
		//loadTopologyMatrxSize(path, &m, &n );
		//Eigen::MatrixXf M(m,n);
        //M = loadTopologyFile(path);
		//final_transformation_ (M),
        //transformation_ (M),
        //previous_transformation_ (M),



        tree_.reset (new pcl::KdTreeFLANN<PointTarget>);     // ANN tree for nearest neighbor search
		tree_for_normals_.reset (new pcl::KdTreeFLANN<PointTarget>); 
        update_visualizer_ = NULL;
      }


	    //DOMINIK
	  Eigen::MatrixXf computeinitXtransformation(Eigen::MatrixXf m1, Eigen::MatrixXf m2)
	  {
		  Eigen::MatrixXf m3(m1.rows()*m2.rows(), 3);

		  for (int i = 0; i < m1.rows(); i++)
		  {
				//for (int j = 0; j < m1.rows(); j++)

				{
					// parametry 1,2 stars rows ; start colums
					//parametry 3,4 - rozmiar bloku wynbika z rozmiaru m2
					
					m3.block(i*m2.rows(), 0, m2.rows(), m2.cols()) =  m1(i,0)*m2;
				}
		  }

		  return m3;

	  }

	  Eigen::MatrixXf computeinitXtrans()
	  {
		  Eigen::MatrixXf Xaff(3,4);
		  int N = input_->size();
		  
		  Eigen::MatrixXf A(N,3), B(N,4);

		  for(int i = 0; i < N; i++)
		  {
				A.block<1, 3> (i,0) = input_->points[i].getVector3fMap().transpose();
				B.block<1, 3> (i,0) = target_->points[i].getVector3fMap().transpose();
				B(i, 3) = 1;
		  }

		  Xaff = (A.transpose() * A).inverse() * A.transpose() * B ;
		  
		 // std::cout << "\n A:\n" << A << "\n B:\n" << B << "\n Xaff:\n" << Xaff << "\n A*Xaff\n" << A*Xaff << "\n";

		  std::ofstream plik1;

		  plik1.open("initX.txt");
		  plik1 << Xaff;
		  plik1.close();

		  return Xaff;
	  }

	   Eigen::MatrixXf computeCroneckerProduct(Eigen::MatrixXf m1, Eigen::MatrixXf m2)
	   {
		  

		   Eigen::MatrixXf m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());

			for (int i = 0; i < m1.cols(); i++)		
			{
				for (int j = 0; j < m1.rows(); j++) 
				{
					 m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
				}
			}

		  return m3;

	  }

	

	    void ObliczNormalnePktChmury(const pcl::PointCloud<PointSource> &cloud_src,  pcl::PointCloud<pcl::Normal> &cloud_normals, int liczba_sasiadow)
		{

		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

			//nieefektywna konwersja
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_Ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud_src));;
			ne.setInputCloud(cloud_src_Ptr);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree  (new pcl::search::KdTree<pcl::PointXYZ> ());
			ne.setSearchMethod(tree);
			ne.setIndices(0,0,cloud_src.height,cloud_src.width);
			
			//albo promien albo liczba sasiadow
			//ne.setRadiusSearch (0.3);
			// liczba s¹siadów
			ne.setKSearch(liczba_sasiadow);
	   
			ne.compute(cloud_normals);

			std::cout << "wypisanie normalnych \n";
			for(int i = 0; i < cloud_normals.size(); i++)
			{
			//	std::cout << cloud_normals.points[i];
			//	std::cout << "\n";
			}


			//	std::cout << "k: ";
			//	std::cout << ne.k_;
			//	std::cout << "\n";
			//	std::cout << "radius: ";
			//	std::cout << ne.getRadiusSearch();
			//	std::cout << "\n";
			//	std::cout << "SWearch Surface: ";
			//	ne.getSearchSurface();
			//	std::cout << "\n";




		}

	
		 void ObliczNormalnePktChmury(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,  pcl::PointCloud<pcl::Normal> &cloud_normals, int liczba_sasiadow)
		{

		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

			//nieefektywna konwersja
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_Ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud_src));;
			ne.setInputCloud(cloud_src);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree  (new pcl::search::KdTree<pcl::PointXYZ> ());
			ne.setSearchMethod(tree);
			ne.setIndices(0,0,cloud_src->height,cloud_src->width);
			
			//albo promien albo liczba sasiadow
			//ne.setRadiusSearch (0.3);
			// liczba s¹siadów
			ne.setKSearch(liczba_sasiadow);
	   
			ne.compute(cloud_normals);

			std::cout << "wypisanie normalnych \n";
			for(int i = 0; i < cloud_normals.size(); i++)
			{
				//std::cout << cloud_normals.points[i];
				//std::cout << "\n";
			}


				//std::cout << "k: ";
				//std::cout << ne.k_;
				//std::cout << "\n";
				//std::cout << "radius: ";
				//std::cout << ne.getRadiusSearch();
			//	std::cout << "\n";
				//std::cout << "SWearch Surface: ";
				//ne.getSearchSurface();
				//std::cout << "\n";




		}

		

         
     void ObliczOdlegloscOdNormalnej(float Pkt_source_x,float Pkt_source_y, float Pkt_source_z,  pcl::PointCloud<pcl::Normal>::Ptr output_normals, int index_pkt_output,const PointCloudTargetPtr cloud, Eigen::VectorXd &vecOdleglosci )
	 {
		 float A = output_normals->points[index_pkt_output].normal_x;
		 float B = output_normals->points[index_pkt_output].normal_y;
		 float C = output_normals->points[index_pkt_output].normal_z;

		 // float D = -A*Pkt_source_x - B*Pkt_source_y - C*Pkt_source_z;

		 Eigen::Vector3d r0, r1, v;

		 r0(0) = Pkt_source_x;
		 r0(1) = Pkt_source_y;
		 r0(2) = Pkt_source_z;

		 v(0) = A;
		 v(1) = B;
		 v(2) = C;

		 for(int i=0;	i <  cloud->size(); i++)
		 {
			 float Pkt_target_x = cloud->points[i].x;
			 float Pkt_target_y = cloud->points[i].y;
			 float Pkt_target_z = cloud->points[i].z;

			 r1(0) = Pkt_target_x;
			 r1(1) = Pkt_target_y;
		     r1(2) = Pkt_target_z;
			 
			 vecOdleglosci(i) = (v.cross(r0 - r1)).norm() / v.norm();

			 // odleg³oœæ punktu od p³aszczyzny
			 // vecOdleglosci(i) = abs(A*Pkt_target_x +B*Pkt_target_y + C*Pkt_target_z + D) / sqrt(A*A + B*B + C*C) ;
		 }


		//std::cout << '\n' << vecOdleglosci << '\n';
	 }

	
      // oblicz katy pomiedzy normalnymi dla wybranego pkt outputa i wszystkich puktow targeta
      void ObliczKatyNormalnychdoPkt( pcl::PointCloud<pcl::Normal>::Ptr output_normals, int index_pkt_output,  Eigen::VectorXd &vecKatow)
	 {
		 int ile = output_normals->size();
		 float a1, a2, a3, b1, b2, b3;

		 for(int i = 0; i < ile; i++ )
		 {
			 a1 = output_normals->points[index_pkt_output].normal_x;
			 a2 = output_normals->points[index_pkt_output].normal_y;
			 a3 = output_normals->points[index_pkt_output].normal_z;
			 b1 = target_normals_->points[i].normal_x;
			 b2 = target_normals_->points[i].normal_y;
			 b3 = target_normals_->points[i].normal_z;
			 
			 // alpha = arccos( (a.b) / (|a| |b|) ) 
			 // a.b - iloczyn skalarny
			 double temp = 180* ( acos((a1*b1 + a2*b2 + a3*b3) / (sqrt(a1*a1 + a2*a2 + a3*a3)*sqrt(b1*b1 + b2*b2 + b3*b3))) ) / 3.141592;
			 vecKatow(i) =  180* ( acos((a1*b1 + a2*b2 + a3*b3) / (sqrt(a1*a1 + a2*a2 + a3*a3)*sqrt(b1*b1 + b2*b2 + b3*b3))) ) / 3.141592;

		 }
    		 //std::cout << "\n"; 
		  //std::cout << "\n";
		  //std::cout << "\n";
		  //std::cout << "Katy pomiedzy normalnymi odpowiednikow: ";
		  //std::cout << vecKatow;
		  //std::cout << "\n";
		  //std::cout << "\n";
	 }          


		 	

     void ObliczKatyNormalnychwParachOdpowiednikow( pcl::PointCloud<pcl::Normal> &cloud_normals1, std::vector<int> &source_indices, pcl::PointCloud<pcl::Normal> &cloud_normals2, std::vector<int> &target_indices,  Eigen::VectorXd &vecKatow)
	 {
		 int ile = cloud_normals1.size();
		 double a1, a2, a3, b1, b2, b3;

		 for(int i = 0; i < ile; i++ )
		 {
			 a1 = cloud_normals1.points[source_indices[i]].normal_x;
			 a2 = cloud_normals1.points[source_indices[i]].normal_y;
			 a3 = cloud_normals1.points[source_indices[i]].normal_z;
			 b1 = cloud_normals2.points[target_indices[i]].normal_x;
			 b2 = cloud_normals2.points[target_indices[i]].normal_y;
			 b3 = cloud_normals2.points[target_indices[i]].normal_z;

			 // alpha = arccos( (a.b) / (|a| |b|) ) 
			 // a.b - iloczyn skalarny
			 vecKatow(i) =  180* ( acos((a1*b1 + a2*b2 + a3*b3) / (sqrt(a1*a1 + a2*a2 + a3*a3)*sqrt(b1*b1 + b2*b2 + b3*b3))) ) / 3.141592;

		 }
		 // std::cout << "\n";
		  //std::cout << "\n";
		  //std::cout << "Katy pomiedzy normalnymi odpowiednikow: ";
		  //std::cout << vecKatow;
		  //std::cout << "\n";
		 // std::cout << "\n";
	 }



      /** \brief destructor. */
      virtual ~Registration () {}
	  void loadTopologyMatrxSize(char* path, int* m,int* n);
	  Eigen::Vector3d loadMarkerVector(char* path);
	  Eigen::MatrixXf loadTopologyFile(char* path);
	  Eigen::MatrixXf loadTopologyFile(char* path,double alpha_val);
	  void getMarkersMatrix(Eigen::MatrixXf & out_matrix);
	  void getMarkersHorn(const char*);
	  void getMarkers(const char * rectangles_path,const char * landmarks_path);
	  void getMarkers(const char * landmarks_path);
	  void getMarkersDominik(const char * landmarks_path);
	  void getTopologyMatrix(Eigen::MatrixXf & topology,int width, int height,bool flag);
	  void getAlphaMatrix(Eigen::MatrixXf & topology,int width,int height,bool flag,double alpha_value);
	  void setLandmarksPath(const char * path);
	  void setTopologyPath(const char * path);
	  void setAlphaPath(const char * path);
	  void setOutputDir(const char * path);
	  void setMaxAngle(float angle);
	  void setSearchMethod(int switch_);
	  
	  void getInitTransformation();//const char * landmarks_path);
      void
      setTransformationEstimation (const TransformationEstimationPtr &te) { transformation_estimation_ = te; }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
        * \param cloud the input point cloud target
        */
      virtual inline void 
      setInputTarget (const PointCloudTargetConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset target. */
      inline PointCloudTargetConstPtr const 
      getInputTarget () { return (target_ ); }

      /** \brief Get the final transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f 
      getFinalTransformation () { return (final_transformation_); }

      /** \brief Get the last incremental transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f 
      getLastIncrementalTransformation () { return (transformation_); }

      /** \brief Set the maximum number of iterations the internal optimization should run for.
        * \param nr_iterations the maximum number of iterations the internal optimization should run for
        */
      inline void 
      setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }

      /** \brief Get the maximum number of iterations the internal optimization should run for, as set by the user. */
      inline int 
      getMaximumIterations () { return (max_iterations_); }

      /** \brief Set the inlier distance threshold for the internal RANSAC outlier rejection loop.
        * 
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed 
        * source index is smaller than the given inlier distance threshold. 
        * The value is set by default to 0.05m.
        * \param inlier_threshold the inlier distance threshold for the internal RANSAC outlier rejection loop
        */
      inline void 
      setRANSACOutlierRejectionThreshold (double inlier_threshold) { inlier_threshold_ = inlier_threshold; }

      /** \brief Get the inlier distance threshold for the internal outlier rejection loop as set by the user. */
      inline double 
      getRANSACOutlierRejectionThreshold () { return (inlier_threshold_); }

      /** \brief Set the maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        * \param distance_threshold the maximum distance threshold between a point and its nearest neighbor 
        * correspondent in order to be considered in the alignment process
        */
      inline void 
      setMaxCorrespondenceDistance (double distance_threshold) { corr_dist_threshold_ = distance_threshold; }

      /** \brief Get the maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        */
      inline double 
      getMaxCorrespondenceDistance () { return (corr_dist_threshold_); }

      /** \brief Set the transformation epsilon (maximum allowable difference between two consecutive 
        * transformations) in order for an optimization to be considered as having converged to the final 
        * solution.
        * \param epsilon the transformation epsilon in order for an optimization to be considered as having 
        * converged to the final solution.
        */
      inline void 
      setTransformationEpsilon (double epsilon) { transformation_epsilon_ = epsilon; }

      /** \brief Get the transformation epsilon (maximum allowable difference between two consecutive 
        * transformations) as set by the user.
        */
      inline double 
      getTransformationEpsilon () { return (transformation_epsilon_); }

      /** \brief Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before 
        * the algorithm is considered to have converged. 
        * The error is estimated as the sum of the differences between correspondences in an Euclidean sense, 
        * divided by the number of correspondences.
        * \param epsilon the maximum allowed distance error before the algorithm will be considered to have
        * converged
        */

      inline void 
      setEuclideanFitnessEpsilon (double epsilon) { euclidean_fitness_epsilon_ = epsilon; }

      /** \brief Get the maximum allowed distance error before the algorithm will be considered to have converged,
        * as set by the user. See \ref setEuclideanFitnessEpsilon
        */
      inline double 
      getEuclideanFitnessEpsilon () { return (euclidean_fitness_epsilon_); }

      /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
        * \param point_representation the PointRepresentation to be used by the k-D tree
        */
      inline void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation)
      {
        point_representation_ = point_representation;
      }

      /** \brief Register the user callback function which will be called from registration thread
       * in order to update point cloud obtained after each iteration
       * \param refference of the user callback function
       */
      template<typename FunctionSignature> inline bool
      registerVisualizationCallback (boost::function<FunctionSignature> &visualizerCallback)
      {
        if (visualizerCallback != NULL)
        {
          update_visualizer_ = visualizerCallback;
          return (true);
        }
        else
          return (false);
      }

      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * \param max_range maximum allowable distance between a point and its correspondence in the target 
        * (default: double::max)
        */
      inline double 
      getFitnessScore (double max_range = std::numeric_limits<double>::max ());


	  /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * \param max_range maximum allowable distance between a point and its correspondence in the target 
        * (default: double::max)
        */
      inline double 
      getFitnessScoreDominik (pcl::PointCloud<pcl::PointXYZ>& output, double max_range = std::numeric_limits<double>::max ());

	   /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * \param max_range maximum allowable distance between a point and its correspondence in the target 
        * (default: double::max)
        */
   


      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * from two sets of correspondence distances (distances between source and target points)
        * \param[in] distances_a the first set of distances between correspondences
        * \param[in] distances_b the second set of distances between correspondences
        */
      inline double 
      getFitnessScore (const std::vector<float> &distances_a, const std::vector<float> &distances_b);

      /** \brief Return the state of convergence after the last align run */
      inline bool 
      hasConverged () { return (converged_); }

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source 
        * (input) as \a output.
        * \param output the resultant input transfomed point cloud dataset
        */
      inline void 
      align (PointCloudSource &output);

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source 
        * (input) as \a output.
        * \param output the resultant input transfomed point cloud dataset
        * \param guess the initial gross estimation of the transformation
        */
      inline void 
      align (PointCloudSource &output, const Eigen::MatrixXf& guess);

      /** \brief Abstract class get name method. */
      inline const std::string&
      getClassName () const { return (reg_name_); }

    protected:
      /** \brief The registration method name. */
      std::string reg_name_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;
	  KdTreePtr tree_for_normals_;

      /** \brief The number of iterations the internal optimization ran for (used internally). */
      int nr_iterations_;

      /** \brief The maximum number of iterations the internal optimization should run for. */
      int max_iterations_;

      /** \brief The input point cloud dataset target. */
      PointCloudTargetConstPtr target_;
	  PointCloudTargetConstPtr target_good_;
	  pcl::PointCloud<pcl::Normal>::Ptr target_normals_; //(new pcl::PointCloud<pcl::Normal>);
	  //old marker_vectors
	  Eigen::MatrixXf initTransformation;
	  bool initMatrixFlag;
	  
      /** \brief The final transformation matrix estimated by the registration method after N iterations. */
      Eigen::MatrixXf final_transformation_;

      /** \brief The transformation matrix estimated by the registration method. */
      Eigen::MatrixXf transformation_;

      /** \brief The previous transformation matrix estimated by the registration method (used internally). */
      Eigen::MatrixXf previous_transformation_;

      /** \brief The maximum difference between two consecutive transformations in order to consider convergence 
        * (user defined). 
        */
      double transformation_epsilon_;

      /** \brief The maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the 
        * algorithm is considered to have converged. The error is estimated as the sum of the differences between 
        * correspondences in an Euclidean sense, divided by the number of correspondences.
        */
      double euclidean_fitness_epsilon_;

      /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will not be ignored in the alignement process.
        */
      double corr_dist_threshold_;

      /** \brief The inlier distance threshold for the internal RANSAC outlier rejection loop.
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed 
        * source index is smaller than the given inlier distance threshold. 
        */
      double inlier_threshold_;

      /** \brief Holds internal convergence state, given user parameters. */
      bool converged_;

      /** \brief The minimum number of correspondences that the algorithm needs before attempting to estimate the 
        * transformation. 
        */
      int min_number_correspondences_;

      /** \brief A set of distances between the points in the source cloud and their correspondences in the 
        * target.                                                                                           
        */       

	  //DOMINIK
	  Eigen::MatrixXf marker_transformation_matrix;
      std::vector<float> correspondence_distances_;   
	  //COPIK - 31.05
	  char landmarks_path[512];
	  char alpha_path[512];
	  char topology_path[512];
	  char output_dir[512];
	  float maxNormalsAngle;
	  int searchMethod;
      /** \brief A TransformationEstimation object, used to calculate the 4x4 rigid transformation. */
      TransformationEstimationPtr transformation_estimation_;

      /** \brief Callback function to update intermediate source point cloud position during it's registration
        * to the target point cloud.
        */
      boost::function<void(const pcl::PointCloud<PointSource> &cloud_src,
                           const std::vector<int> &indices_src,
                           const pcl::PointCloud<PointTarget> &cloud_tgt,
                           const std::vector<int> &indices_tgt)> update_visualizer_;

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param cloud the point cloud dataset to use for nearest neighbor search
        * \param index the index of the query point
        * \param indices the resultant vector of indices representing the k-nearest neighbors
        * \param distances the resultant distances from the query point to the k-nearest neighbors
        */
      inline bool
      searchForNeighbors (const PointCloudSource &cloud, int index, 
                          std::vector<int> &indices, std::vector<float> &distances)
      {
        int k = tree_->nearestKSearch (cloud, index, 1, indices, distances);
        if (k == 0)
          return (false);
        return (true);
      }


	  //searchForNeighborswithNormalAngleCondition(output, (*indices_)[idx], *output_normals, 45, nn_indices, nn_dists, normal_shooting)

	  //przelacznik
	  //0 - normal shoting  na chmurze spelniajacej warunek kata
	  //1 - marker_vectors_ na chmurze spelniajacej warunek kata
	  //2 - nearest search  na chmurze spelniajacej warunek kata 
	  //3 - nearest search  na pierwontej chmurze


	  inline bool
      searchForNeighborswithNormalAngleCondition(const PointCloudSource &cloud, int index,  pcl::PointCloud<pcl::Normal>::Ptr output_normals, float maxKatNormalnych, 
                          std::vector<int> &indices, std::vector<float> &distances, int przelacznik = 0)
      {

		//   ZNAJDUJE closest compotible point


		  
			PointCloudTargetPtr target_good_temp (new pcl::PointCloud<pcl::PointXYZ>);
		//  wektor na k¹ty normlanych do pkt
		  Eigen::VectorXd vecKatowNormalnychdoPkt (cloud.size());
		//  znalezienie katow wektorow normalnych do punktow w chmurze targetowej
		  ObliczKatyNormalnychdoPkt(output_normals, index,  vecKatowNormalnychdoPkt);
		//  //indeksy puktow targetu spelniajace warunek
		  int licznikPktspelniajacychWarunekMaxKataNormalnej = 0;
		  std::vector<int> target_indices_spelniajacychWarunekMaxKataNormalnej(cloud.size());

		//   wybranie tylko tych pkt targetu_ ktore spelniaja warunek
		  float min = 180.0;
		  int min_index = 0;
          for (size_t idx = 0; idx < cloud.size() ; ++idx)
          {
			  if(vecKatowNormalnychdoPkt(idx) >= 180.0) {
				  vecKatowNormalnychdoPkt(idx) -= 180.0;
			  }
			if (vecKatowNormalnychdoPkt(idx) < maxKatNormalnych ) 
			{
				target_indices_spelniajacychWarunekMaxKataNormalnej[licznikPktspelniajacychWarunekMaxKataNormalnej] = idx;
				licznikPktspelniajacychWarunekMaxKataNormalnej++;
				
			}
			//if(vecKatowNormalnychdoPkt(idx) < min) {
			//	min = vecKatowNormalnychdoPkt(idx);
			//	min_index = idx;
			//}
		}      
		   if ( licznikPktspelniajacychWarunekMaxKataNormalnej == 0 )
		   {
			   //target_indices_spelniajacychWarunekMaxKataNormalnej[licznikPktspelniajacychWarunekMaxKataNormalnej] = min_index;
			  // ++licznikPktspelniajacychWarunekMaxKataNormalnej;
			   PCL_ERROR("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), index);
		   }
		       //Przygotowanie nowej chmury tylko z punktzmi spelniajcymi warunek kata
			  // skrocenie wektora pkt spelniajacyh waruenk do faktycznej liczby pkt  
			/// do we need it?   
		   //target_indices_spelniajacychWarunekMaxKataNormalnej.resize(licznikPktspelniajacychWarunekMaxKataNormalnej);



			   //CHMURA NA KTOREJ WYSZUKUJEMY ODPOWIEDNIKOW ??PRZEFILTORWANY TARGER WARUNJKIEM KATA NORMALNYCH

			    //pcl::PointCloud<pcl::PointCloud<PointCloudTarget>>::Ptr 
			   target_good_temp->width = licznikPktspelniajacychWarunekMaxKataNormalnej;
               target_good_temp->height = 1;
			   target_good_temp->resize(licznikPktspelniajacychWarunekMaxKataNormalnej);


	   
			   ///
			   for(int i=0; i < licznikPktspelniajacychWarunekMaxKataNormalnej; i++)
			   {

				
				   target_good_temp->points[i].x = target_->points[ target_indices_spelniajacychWarunekMaxKataNormalnej[i]].x;
				   target_good_temp->points[i].y = target_->points[ target_indices_spelniajacychWarunekMaxKataNormalnej[i]].y;
				   target_good_temp->points[i].z = target_->points[ target_indices_spelniajacychWarunekMaxKataNormalnej[i]].z;


			   }


			   // w switchu nie mozna definiowac zmiennych

			   //case 0: 
			   Eigen::VectorXd vecOdleglosciodNormalnej (target_good_temp->size());
			   float Pkt_source_x = cloud.points[index].x;
			   float Pkt_source_y = cloud.points[index].y;
			   float Pkt_source_z = cloud.points[index].z;
			   int k;

			   //case 1:
			   //przesuwamy chmure zrodlowa o wektor przemieszczenia markera
			   //dopiero na tak przesunietej chmurze szukamy najblizszych sasiadow
			 //  pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Out_marker_vector_moved (new pcl::PointCloud<pcl::PointXYZ>);
			 //  Cloud_Out_marker_vector_moved->width = cloud.width;
			  // Cloud_Out_marker_vector_moved->height = cloud.height;
			   //Cloud_Out_marker_vector_moved->resize(cloud.width*cloud.height);
			   

			   /// we don't wanna do that - cloud is already moved!
			   //PRZESUWANIE CHMURY OUTPUT WG TRANFORAMTY MARKEROW
			   //transformPointCloud(cloud,*Cloud_Out_marker_vector_moved,marker_transformation_matrix,0,0);

			   //case 1,2:
			    PointCloudTarget target_temp = *target_good_temp;
				// Set all the point.data[3] values to 1 to aid the rigid transformation
				for (size_t i = 0; i < target_temp.points.size (); ++i)
				target_temp.points[i].data[3] = 1.0;

			 //target_ = cloud;
			  target_good_ = target_temp.makeShared ();

			  
			/*
			  
				pcl::PointCloud<pcl::Normal>::Ptr output_vectors(new pcl::PointCloud<pcl::Normal>);
				int n = marker_vectors.rows();
				output_vectors->width = n;
				output_vectors->height = 1;
				output_vectors->points.resize(n);
				for(int i = 0;i < n;++i)
				{
					output_vectors->points[i].normal_x = marker_vectors(i,2);
					output_vectors->points[i].normal_y = marker_vectors(i,0);
					output_vectors->points[i].normal_z = marker_vectors(i,1);
				}
				*/
						int index_elementu_min_w_nowej_chmurze;
			   switch ( przelacznik )
			   {

				//0 - normal shoting  na chmurze spelniajacej warunek kata
				   // znajduje odleglosc wszystkich punktow w "dober chmurze" od  normalnej 
				   // znajduje minimalna idleglosc
	  			case 0:
				case 1:
				case 2:
					
						ObliczOdlegloscOdNormalnej(Pkt_source_x, Pkt_source_y, Pkt_source_z, output_normals, index, target_good_temp, vecOdleglosciodNormalnej);
						
						//znalezienie w nowej chmurze indeksu elementu minimalnego
						vecOdleglosciodNormalnej.minCoeff(&index_elementu_min_w_nowej_chmurze);
						 
						 //przypisanie poprawnego numeru pkt targetowego z pierwotnej chmury target_
						indices[0] = target_indices_spelniajacychWarunekMaxKataNormalnej[index_elementu_min_w_nowej_chmurze];
						distances[0] = sqrt(float( (target_->points[indices[0]].x - cloud.points[index].x )* (target_->points[indices[0]].x - cloud.points[index].x ) + 
						+ ( target_->points[indices[0]].y - cloud.points[index].y )* (target_->points[indices[0]].y - cloud.points[index].y )  +
						(target_->points[indices[0]].z - cloud.points[index].z )* (target_->points[indices[0]].z - cloud.points[index].z ) ));
					   
					   
					   return (true);

					 break;

				//1 - marker_vectors shooting na chmurze spelniajacej warunek kata	  
		 	/*	case 1:
					    //SZUKAMY NIEREST SERCHEM PO TARGETGOODTEMP EUKLIDESOW0 (nereST SERACH) BEZ NORMALSCHOOTING
						ObliczOdlegloscOdNormalnej(Pkt_source_x, Pkt_source_y, Pkt_source_z, output_vectors, index, target_good_temp, vecOdleglosciodNormalnej);
						//ObliczOdlegloscOdNormalnej(Pkt_source_x, Pkt_source_y, Pkt_source_z, output_normals, index, target_good_temp, vecOdleglosciodNormalnej);
						
						std::cout << "vecOdleglosciodNormalnej: \n";
						std::cout << vecOdleglosciodNormalnej;
						//znalezienie w nowej chmurze indeksu elementu minimalnego
						vecOdleglosciodNormalnej.minCoeff(&index_elementu_min_w_nowej_chmurze);
						 
						 //przypisanie poprawnego numeru pkt targetowego z pierwotnej chmury target_
						indices[0] = target_indices_spelniajacychWarunekMaxKataNormalnej[index_elementu_min_w_nowej_chmurze];
						distances[0] = sqrt(float( (target_->points[indices[0]].x - cloud.points[index].x )* (target_->points[indices[0]].x - cloud.points[index].x ) + 
						+ ( target_->points[indices[0]].y - cloud.points[index].y )* (target_->points[indices[0]].y - cloud.points[index].y )  +
						(target_->points[indices[0]].z - cloud.points[index].z )* (target_->points[indices[0]].z - cloud.points[index].z ) ));
					   /*
					             
                    	tree_for_normals_->setInputCloud (target_good_);
						k = tree_for_normals_->nearestKSearch (*Cloud_Out_marker_vector_moved, index, 1, indices, distances);
						if (k == 0) return (false);
						return (true);
						break;
						
					
					break;*/
				//2 - rectangles shooting na chmurze spelniajacej warunek kata	  
		 /*		case 2:
					
						ObliczOdlegloscOdNormalnej(Pkt_source_x, Pkt_source_y, Pkt_source_z, output_vectors, index, target_good_temp, vecOdleglosciodNormalnej);
						//ObliczOdlegloscOdNormalnej(Pkt_source_x, Pkt_source_y, Pkt_source_z, output_normals, index, target_good_temp, vecOdleglosciodNormalnej);
						
						//znalezienie w nowej chmurze indeksu elementu minimalnego
						vecOdleglosciodNormalnej.minCoeff(&index_elementu_min_w_nowej_chmurze);
						 
						 //przypisanie poprawnego numeru pkt targetowego z pierwotnej chmury target_
						indices[0] = target_indices_spelniajacychWarunekMaxKataNormalnej[index_elementu_min_w_nowej_chmurze];
						distances[0] = sqrt(float( (target_->points[indices[0]].x - cloud.points[index].x )* (target_->points[indices[0]].x - cloud.points[index].x ) + 
						+ ( target_->points[indices[0]].y - cloud.points[index].y )* (target_->points[indices[0]].y - cloud.points[index].y )  +
						(target_->points[indices[0]].z - cloud.points[index].z )* (target_->points[indices[0]].z - cloud.points[index].z ) ));
					   /*

                    	tree_for_normals_->setInputCloud (target_good_);
						k = tree_for_normals_->nearestKSearch (*Cloud_Out_marker_vector_moved, index, 1, indices, distances);
						if (k == 0) return (false);
						return (true);
						break;
						
					
					break;*/

				//3 - nearest search  na chmurze spelniajacej warunek kata 
	  			case 3:
						tree_for_normals_->setInputCloud (target_good_temp);
						k = tree_for_normals_->nearestKSearch (cloud, index, 1, indices, distances);
						if (k == 0) return (false);
						return (true);
						break;


				//4 - nearest search  na pierwontej chmurze
				case 4:

	    
						k = tree_->nearestKSearch (cloud, index, 1, indices, distances);
						if (k == 0)
						  return (false);
						return (true);
						break;

	  }

			   return (true);
      }


	  void findMarkersError(int iterationNumber, std::ofstream & fileOut,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_target);
	  void znajdzPktNajblizszeLandmark(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_target, std::vector<int> &markers_source_indices, std::vector<int> &markers_target_indices);
	  void znajdzWierszKolumnePktChmury(int width, int height, int index,int* wiersz, int* kolumna); 
    private:
 
      /** \brief Abstract transformation computation method. */
      virtual void 
      computeTransformation (PointCloudSource &output) = 0;

      /** \brief Abstract transformation computation method with initial guess */
      virtual void 
      computeTransformation (PointCloudSource &output, const Eigen::MatrixXf& guess) {}

      /** \brief The point representation used (internal). */
      PointRepresentationConstPtr point_representation_;
   };
}

#include "pcl/registration/impl/registration.hpp"

#endif  //#ifndef PCL_REGISTRATION_H_

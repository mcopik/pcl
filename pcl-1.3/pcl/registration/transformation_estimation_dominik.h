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
 * $Id: transformation_estimation_svd.h 3041 2011-11-01 04:44:41Z rusu $
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DOMINIK_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DOMINIK_H_

#include <pcl/registration/transformation_estimation.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationSVD implements SVD-based estimation of
      * the transformation aligning the given correspondences.
      *
      * \author Dirk Holz, Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class TransformationEstimationDominik : public TransformationEstimation<PointSource, PointTarget>
    {
      public:
        TransformationEstimationDominik () {};

		// mno¿ymy macierz przez pole skalarne, w przypadku niepowodzenia zwracamy macierz
		Eigen::MatrixXf MnozenieTablicowe(Eigen::MatrixXf PoleSkalarne, Eigen::MatrixXf Macierz)
		{
			
			if((PoleSkalarne.rows() == Macierz.rows()) && (PoleSkalarne.cols() == Macierz.cols()))
			{
				Eigen::MatrixXf C(Macierz.rows(), Macierz.cols());

				for(int i = 0; i < Macierz.rows(); i++)
					for(int j = 0; j < Macierz.cols(); j++)
						C(i,j) = Macierz(i,j) * PoleSkalarne(i,j);

				return C;
			}
			else
				return Macierz;
		}

		Eigen::MatrixXf computeCroneckerProduct(Eigen::MatrixXf m1, Eigen::MatrixXf m2)
	   {
		  

		   Eigen::MatrixXf m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());
	//	   Eigen::MatrixXf temp(m2.rows(), m2.cols());

			for (int i = 0; i < m1.rows(); i++)		
			{
				for (int j = 0; j < m1.cols(); j++) 
				{
	//				 temp = m1(i,j)*m2; 
	//					 std::cout << '\n' << temp << '\n';
					 m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
	//					std::cout << '\n' << m3 << '\n';
				}
			}

		  return m3;

	  }

			  void loadTopologyMatrxSize(const char* path, int* m,int* n )
	   {
		    FILE* file;
			file = fopen(path, "r");

			
			fscanf(file,"%d", m);
			fscanf(file,"%d", n);

			fclose(file);
	

	   }



	   Eigen::MatrixXf loadTopologyFile(const char* path)
	   {
		    FILE* file;
			file = fopen(path, "r");
			int m,n, temp;    //m - liczba krawedzi  ; n- liczba wierzcholkow
			
			fscanf(file,"%d", &m);
			fscanf(file,"%d", &n);

			Eigen::MatrixXf M(m, n);

			for( int i = 0; i < m; i++)
				for( int j = 0; j < n; j++)
				{
					fscanf(file,"%d", &temp);
					M(i,j) = temp;
				}

			fclose(file);
	
			return M;

	   }



	   //////////////////////////////////////////////////////////////////////////////////////////
	     void loadLandmarksSize(const char* path, int* l )
	   {
		    FILE* file;
			file = fopen(path, "r");

			
			fscanf(file,"%d", l);
		

			fclose(file);
	

	   }



        void ObliczNormalnePktChmury(const pcl::PointCloud<PointSource> &cloud_src,  pcl::PointCloud<pcl::Normal> &cloud_normals, int liczba_sasiadow)
		{
		    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud (cloud_src);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree  (new pcl::search::KdTree<pcl::PointXYZ> ());
			ne.setSearchMethod(tree);
			ne.setIndices(0,0,cloud.height,cloud.width);
			
			//albo promien albo liczba sasiadow
			//ne.setRadiusSearch (0.3);
			// liczba s¹siadów
			ne.setKSearch(liczba_sasiadow);
	   
			ne.compute(cloud_normals);

			std::cout << "wypisanie normalnych \n";
			for(int i = 0; i < cloud_normals.size(); i++)
			{
				std::cout << cloud_normals->points[i];
				std::cout << "\n";
			}


				std::cout << "k: ";
				std::cout << ne.k_;
				std::cout << "\n";
				std::cout << "radius: ";
				std::cout << ne.getRadiusSearch();
				std::cout << "\n";
				std::cout << "SWearch Surface: ";
				ne.getSearchSurface();
				std::cout << "\n";




		}





	   	  void loadLandmarksFile(const char* path, Eigen::MatrixXf& DL, Eigen::MatrixXf& UL)
	      {
		    FILE* file;
			file = fopen(path, "r");
			int l, nr;    //l - liczba landmarków  
			float temp;
			
			fscanf(file,"%d", &l);
			
			DL.fill(0);
			UL.fill(0);


			for( int i = 0; i < l; i++)
				{
					fscanf(file,"%d", &nr);
					
					fscanf(file,"%f", &temp);
					DL(nr,4*nr) = temp;
					fscanf(file,"%f", &temp);
					DL(nr,4*nr+1) = temp;
					fscanf(file,"%f", &temp);
					DL(nr,4*nr+2) = temp;
					DL(nr,4*nr+3) = 1;

					fscanf(file,"%f", &temp);
					UL(nr,0) = temp;
					fscanf(file,"%f", &temp);
					UL(nr,1) = temp;
					fscanf(file,"%f", &temp);
					UL(nr,2) = temp;
				}

			fclose(file);
		   }



		   /////////////////////////////////////////////////////////////////////////
        virtual ~TransformationEstimationDominik () {};

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Eigen::MatrixXf &transformation_matrix);

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Eigen::MatrixXf &transformation_matrix);

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */

	 
		
        inline void
        estimateRigidTransformation (
             pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Eigen::MatrixXf &transformation_matrix,
			Eigen::MatrixXf &Alfa,
			int nr_iteracji);

        inline void
        estimateRigidTransformation (
             pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Eigen::MatrixXf &transformation_matrix,
			Eigen::MatrixXf &Alfa,
			int nr_iteracji,
			const char * landmarks_path,
			Eigen::MatrixXf & topology,
			const char * output_dir);
			

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const Correspondences &correspondences,
            Eigen::MatrixXf &transformation_matrix);

      protected:

        /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src * tgt'
          * \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen format
          * \param[in] centroid_src the input source centroid, in Eigen format
          * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
          * \param[in] centroid_tgt the input target cloud, in Eigen format
          * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
          */ 
        void
        getTransformationFromCorrelation (const Eigen::MatrixXf &cloud_src_demean,
                                          const Eigen::Vector4f &centroid_src,
                                          const Eigen::MatrixXf &cloud_tgt_demean,
                                          const Eigen::Vector4f &centroid_tgt,
                                          Eigen::MatrixXf &transformation_matrix);
		
    };

  }
}

#include <pcl/registration/impl/transformation_estimation_dominik.hpp>

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_H_ */

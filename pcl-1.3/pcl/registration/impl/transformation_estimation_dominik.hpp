/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DOMINIK_HPP_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_DOMINIK_HPP_
#include <ctime>
#include <iomanip>
#include "invMatrix.h"
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Eigen::MatrixXf &transformation_matrix)
{
  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, centroid_src);
  compute3DCentroid (cloud_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Eigen::MatrixXf &transformation_matrix)
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::TransformationDominik::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
}



void KonwertujNrRamki(int nr_ramki, char* z0, char* z1, char* z2)
{

            if (nr_ramki < 10)
			{
				* z0 ='0';
				* z1 ='0';
				itoa(nr_ramki, z2, 10);
				//nr[0] = nr[1] = '0';
				//itoa(nr_ramki, &nr[2], 10);//nr[2] = (char)(nr_ramki/10);
			}
            else if (nr_ramki < 100)
			{
				* z0 = '0';
				itoa(nr_ramki/10, z1, 10);
				itoa(nr_ramki%10,z2, 10);
			}
            else
			{
				itoa(nr_ramki/100, z0, 10);
				itoa(nr_ramki/10, z1, 10);
				itoa(nr_ramki%10, z2, 10);
			}
} 

template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::estimateRigidTransformation (
    pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::MatrixXf &transformation_matrix,
	Eigen::MatrixXf &Alfa,
	int nr_iteracji)
{
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::estimateRigidTransformation (
    pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Eigen::MatrixXf &transformation_matrix,
	Eigen::MatrixXf &Alfa,
	int nr_iteracji,
	const char * landmarks_path,
	Eigen::MatrixXf & topology,
	const char * output_dir)
{
	
	int t = clock();
	//==================================================================================================
	//1. Tworzenie macierzy D (pkt zrodlowych V_i) macierz punktow wejsciowych
	// kopiujemy z czmury wejœciowej punkty v do macierzy D (bez przestawiania)
	 int n = Alfa.cols();  //liczba pkt. w chmurze
	 int m = Alfa.rows();
	Eigen::MatrixXf D(n,4*n); 
	D.fill(0);

	// wype³niamy macierz D po przek¹tnej poziomo wektorami, tak ¿e wekotory s¹ na diagonalnej
	//dla wszystkich punktow zrodlowych
	/*for (int i =0 ; i < cloud_src.size(); i++)
	{		
		D.block<1, 3> (i,4*i) = cloud_src.points[i].getVector3fMap().transpose();
		D(i, 4*i +3) = 1;
	}
	*/
	//std::cout << "\n" << D << "\n";



	// wype³niamy macierz D po przek¹tnej poziomo wektorami, tak ¿e wekotory s¹ na diagonalnej
	//tylko dla punktow zrodlowych ktore maja odpowiedniki

	for (int i =0 ; i < indices_src.size(); i++)
	{		
		D.block<1, 3> (i,4*i) = cloud_src.points[indices_src[i]].getVector3fMap().transpose();
		D(i, 4*i +3) = 1;
	}

	//std::cout << "\n" << D << "\n";





	//=========================================
    //=========================================================
	//1.a Tworzenie macierzy DL (pkt zrodlowych V_i)  macierz punktow wejsciowych dla landmarkow n*n
	// kopiujemy z czmury wejœciowej punkty v do macierzy D (bez przestawiania)
	// pODMACIERZ UWZGKENIAJACA LANDMARKI ZAWSZE MA STALY ROZMIAR
	// natomiast rzeczywista liczba landmarkow wynika z zawartosci macierzy


	//char pathLandmarks[512]  ="landmarks_1.txt";//"landmarks.txt ";
	int l= 0;
	int beta = 1;														// !!!!!!!!!!! beta !!!!!!!!!!!!! 
	loadLandmarksSize(landmarks_path,&l);
	Eigen::MatrixXf DL(n,4*n);
	Eigen::MatrixXf UL(n,3);
	loadLandmarksFile(landmarks_path, DL, UL);
	DL=beta*DL;
  // std::cout << '\n' << DL  ; //<< '\n' << UL '\n';
  // std::cout << '\n' << UL  ;

   
	//=============================================================================================
	//2. Tworzenie macierzy U  - macierz odpowiadajcych punktow docelowych
	Eigen::MatrixXf U(n,3);
	U.fill(0);

	//kolejnosc w macierzy U powinna wynikac z wekroea odpowiednikow	
	// wype³niamy macierz D po przek¹tnej poziomo wektorami, tak ¿e wekotory s¹ na diagonalnej
	//dla wszykich punktow targetu
	/*for (int i =0 ; i < cloud_tgt.size(); i++)
	{		
		U.block<1, 3> (i,0) = cloud_tgt.points[indices_tgt[i]].getVector3fMap().transpose();
		std::cout << "\n" << indices_tgt[i] << "  ";
		std::cout << indices_src[i] << "\n";
	}
	
	std::cout << "\n" << U << "\n";*/
	

	//kolejnosc w macierzy U powinna wynikac z wekroea odpowiednikow	
	// wype³niamy macierz D po przek¹tnej poziomo wektorami, tak ¿e wekotory s¹ na diagonalnej
	//dla punktow targetu ktore maja odpowiednik
	for (int i =0 ; i < indices_tgt.size(); i++)
	{		
		U.block<1, 3> (i,0) = cloud_tgt.points[indices_tgt[i]].getVector3fMap().transpose();
		//std::cout << "\n" << indices_tgt[i] << "  ";
		//std::cout << indices_src[i] << "\n";
	}
	
	//std::cout << "\n" << U << "\n";






	//=============================================================================================
	//3. tWORZYMY MACIERZ G - macierz wmacniajaca trnalacje w skladniku sztywnosci
	Eigen::Matrix4f G = Eigen::Matrix4f::Identity();
	int gamma = 1;															// !!!!!!!!! TU WPISUJEMY GAMMA !!!!!!!!!!!!
	G(3,3) = gamma;

	//std::cout << "\n" << G ;

	
	//=============================================================================================
	//4. Macierz wag W   - macierz wskazujaca rzeczywiste pary (macierz wag ) 1 - oznacza jest para; 0 nie ma pary
	Eigen::MatrixXf W(n,n);
	W.fill(0);

	for(int i= 0; i <indices_src.size() ; i++)
	{
		W(indices_src[i],indices_src[i]) = 1;
	}

	//std::cout << "\n" << W ;

	//=============================================================================================
	//5. Macierz krawêdzi M
	 //char pathM[512]  ="topology.txt ";
	 
	 Eigen::MatrixXf & M = topology;

  //=============================================================================================
	//6. Macierz A i B   - macierz ukladu rownan
	Eigen::MatrixXf A(4*m+2*n,4*n);
	Eigen::MatrixXf B(4*m+2*n,3);

	A.fill(0);
	B.fill(0);

	//PODSTAWIANIE SKLADNIKA SZTYWNOSCI
	
	Eigen::MatrixXf AlfaM(m,n);
	AlfaM = MnozenieTablicowe(Alfa, M);

	Eigen::MatrixXf AlfaMG(4*m, 4*n);
	AlfaMG = computeCroneckerProduct(AlfaM, G);

	A.block(0,0, 4*m, 4*n) = AlfaMG; 


	//PODSTAWIANIE SKLADNIKA MACIERZY AFINICZNYCH
	//std::cout << W <<std::endl;
	//std::cout << D << std::endl;
	A.block(4*m, 0, n, 4*n) = W*D;


	//PODSTAWIANIE SKLADNIKA MACIERZY laNDARMOWYCH

	A.block(4*m+n, 0, n, 4*n) = beta*DL;



	//MACIERZ B

	B.block(4*m, 0, n, 3) = W * U;
	B.block(4*m+n, 0, n, 3) = UL;
	
 
	/*std::ofstream file("matrix_A");
	file << A.rows() << " " << A.cols() << std::endl;
	file << A;
	file.close();
	file.open("matrix_B");
	file << B.rows() << " " << B.cols() << std::endl;
	file << B;
	file.close();*/
	//std::cout << "A: \n" << A << "\n";
	//std::cout << "B: \n" << B << "\n";
	
	int time_1 = clock();
	//transformation_matrix =  ( A.transpose() * A ).inverse() * A.transpose() * B;
	//std::cout << "transformation_matrix : \n" << transformation_matrix << "\n";
	int time_2 = clock();
	int matlab_time1 = clock();
	mwArray matlab_a(A.rows(), A.cols(), mxDOUBLE_CLASS);
	matlab_a.SetData(A.data(), A.cols()*A.rows()); 
	mwArray matlab_b(B.rows(), B.cols(), mxDOUBLE_CLASS);
	matlab_b.SetData(B.data(), B.cols()*B.rows()); 
	mwArray ret;
	Amberg(1, ret, matlab_a, matlab_b);
	///TRANSFORMATION_MATRIX
	transformation_matrix.resize(4*n,3);
	for(int i = 0;i < 4*n;++i) { 
		for(int j = 0;j < 3;++j) {
			transformation_matrix(i,j) = ret(i+1,j+1);
		}
	}
	int matlab_time2 = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_przesuniete (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_src_przesuniete->width = cloud_src.width;
	cloud_src_przesuniete->height = cloud_src.height;
	cloud_src_przesuniete->resize(cloud_src_przesuniete->width * cloud_src_przesuniete->height);
	
	transformPointCloud(cloud_src,  *cloud_src_przesuniete, transformation_matrix,0, 0);



	//////////////////////////////////////////////////////////////////////////////
	//BLOK WYPISUJACY WYNIKI
	  //Przeksztalcenie 
  Eigen::MatrixXf temp;
  Eigen::MatrixXf aff;
  Eigen::Matrix3f Ai;
  Eigen::Vector3f b;
  
    std::ofstream plik;
	//char FileDir[512] = "..\\wyniki\\transforms";
	char sciezka[512];
	char nr_ramki[512];
	
	if(nr_iteracji < 10)
	{
		nr_ramki[2] = nr_iteracji + '0';
		nr_ramki[1] = '0';
		nr_ramki[0] = '0';
	}
	else if(nr_iteracji < 100)
	{
		nr_ramki[2] = nr_iteracji%10 + '0';
		nr_ramki[1] = nr_iteracji/10 + '0';
		nr_ramki[0] = '0';
	}
	else
	{
		int k = nr_iteracji/100;
		nr_ramki[2] = (nr_iteracji - k*100) % 10 + '0';
		nr_ramki[1] = (nr_iteracji - k*100)/10 + '0';
		nr_ramki[0] = k + '0';
	}
    //KonwertujNrRamki(nr_iteracji, &nr_ramki[0], &nr_ramki[1], &nr_ramki[2]);
	sprintf(sciezka,"%s\\transforms%c%c%c%s",output_dir, nr_ramki[0], nr_ramki[1],nr_ramki[2],".txt");
	plik.open(sciezka);
				 
	        FILE* file3;
			file3 = fopen(landmarks_path, "r");
			
			//float temp;

			int bufor, nr;
			float war;

			//pominac liczbe landmarkow
		    fscanf(file3,"%d", &bufor);
			
			fscanf(file3,"%d", &nr);


 
    for (size_t i = 0; i < indices_src.size (); ++i)
	{
		//zaczytanie numeru landmarka;
		


		temp = transformation_matrix.block<4, 3> (4*i, 0);
		aff = temp.transpose();
		Ai   = aff.block<3, 3> (0, 0);
		b   = aff.block<3, 1> (0, 3);
	    

		plik << cloud_src.points[indices_src[i]].getVector3fMap ().transpose();
		plik << "  source; nr: ";
		plik << indices_src[i];
		plik << "\n";
		plik << cloud_src_przesuniete->points[indices_src[i]].getVector3fMap ().transpose();
		plik << "  moved;"; //nr: ";
		//plik << indices_src[i];
		plik << "\n";
		plik << cloud_tgt.points[indices_tgt[i]].getVector3fMap ().transpose();
		plik << "  target; nr: ";
		plik << indices_tgt[i];

		if (nr == i)
		{
			plik << "\n";

			
			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";
			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";

			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";
			plik << " vi";


			plik << "\n";
			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";
			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";

			fscanf(file3,"%f", &war);
			plik << war;
			plik << " ";
			plik << " li";

			plik << "\n\n";

			fscanf(file3,"%d", &nr);

		}
		else
		{
			plik << "\n\n";
		}




		plik << aff;
		plik << "\n";
		plik << "===================================";
		plik << "\n\n\n";

		
	}
  
	plik << "Matlab matrix: " << std::endl;
	plik << std::setprecision(7) << ret << std::endl;
   plik << "Estimation time:" << std::endl;
   plik << "Time before (A'*A)^-1*A'*B: " << ((float)time_1-t)/CLOCKS_PER_SEC << std::endl;
   plik << "Time of (A'*A)^-1*A'*B: " << ((float)time_2-time_1)/CLOCKS_PER_SEC << std::endl;
   plik << "Time of matlab block: " << ((float)matlab_time2-matlab_time1)/CLOCKS_PER_SEC << std::endl;
   plik.close();
  



 /* if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimationDominik::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
    return;
  }*/

  // <cloud_src,cloud_src> is the source dataset
  //transformation_matrix.setIdentity ();

 // Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
//  compute3DCentroid (cloud_src, indices_src, centroid_src);
//  compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);

  // Subtract the centroids from source, target
 // Eigen::MatrixXf cloud_src_demean;
 // demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

 // Eigen::MatrixXf cloud_tgt_demean;
 // demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);

 //  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);


}







//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Eigen::MatrixXf &transformation_matrix)
{
  std::vector<int> indices_src, indices_tgt;
  pcl::registration::getQueryIndices (correspondences, indices_src);
  pcl::registration::getMatchIndices (correspondences, indices_tgt);

  // <cloud_src,cloud_src> is the source dataset
  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);

  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);

  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::registration::TransformationEstimationDominik<PointSource, PointTarget>::getTransformationFromCorrelation (
    const Eigen::MatrixXf &cloud_src_demean,
    const Eigen::Vector4f &centroid_src,
    const Eigen::MatrixXf &cloud_tgt_demean,
    const Eigen::Vector4f &centroid_tgt,
    Eigen::MatrixXf &transformation_matrix)
{
  transformation_matrix.setIdentity ();

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix3f R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner<3, 3> () = R;
  Eigen::Vector3f Rc = R * centroid_src.head<3> ();
  transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;
}

//#define PCL_INSTANTIATE_TransformationEstimationSVD(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationSVD<T,U>;

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_HPP_ */

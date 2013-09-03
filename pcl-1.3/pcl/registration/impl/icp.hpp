/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: icp.hpp 2844 2011-10-24 22:00:52Z rusu $
 *
 */

#include <boost/unordered_map.hpp>
#include <direct.h>
#include <windows.h>
#include <direct.h>
#include <string.h>
#include <tchar.h>
#include <strsafe.h>
void RunTest(TCHAR *AppName, TCHAR *CmdLine)
{
    wprintf(L"\nTest Running...\n");
    wprintf(L" AppName: %s\n", AppName);
    wprintf(L" CmdLine: %s\n", CmdLine);

    PROCESS_INFORMATION processInformation;
    STARTUPINFO startupInfo;
    memset(&processInformation, 0, sizeof(processInformation));
    memset(&startupInfo, 0, sizeof(startupInfo));
    startupInfo.cb = sizeof(startupInfo);

    BOOL result;
    TCHAR tempCmdLine[MAX_PATH * 2];  //Needed since CreateProcessW may change the contents of CmdLine
    if (CmdLine != NULL)
    {
        _tcscpy_s(tempCmdLine, MAX_PATH *2, CmdLine);
        result = ::CreateProcess(AppName, tempCmdLine, NULL, NULL, FALSE, NORMAL_PRIORITY_CLASS, NULL, NULL, &startupInfo, &processInformation);
		DWORD err = ::GetLastError();
    }
    else
    {
        result = ::CreateProcess(AppName, CmdLine, NULL, NULL, FALSE, NORMAL_PRIORITY_CLASS, NULL, NULL, &startupInfo, &processInformation);
    }

    if (result == 0)
    {
        wprintf(L"ERROR: CreateProcess failed!");
    }
    else
    {
        WaitForSingleObject( processInformation.hProcess, INFINITE );
        CloseHandle( processInformation.hProcess );
        CloseHandle( processInformation.hThread );
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output)
{
  pcl::IterativeClosestPoint<PointSource, PointTarget>::computeTransformation (output, Eigen::Matrix4f::Identity());
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output_, const Eigen::MatrixXf &guess)
{
	

  // Allocate enough space to hold the results
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  output->width = output_.width;
  output->height = output_.height;
  output->resize(output_.size());
  for(int i = 0;i < output->size();++i) {
	  output->points[i] = output_.points[i];
  }
  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudTarget input_corresp;
  input_corresp.points.resize (indices_->size ());

  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  int wierszPozadany, kolumnaPozadana, wierszRzeczywisty, kolumnaRzeczywista;



  
  char buf[512];
  sprintf(buf,"%s\\errors.txt",output_dir);
  std::ofstream err_out(buf);
  sprintf(buf,"%s\\transformat_norms.txt",output_dir);
  std::ofstream trans_norm(buf);
  err_out << "Srednia odleglosc najblizszych punktow output->target na starcie Dominik: " << getFitnessScoreDominik(*output) << std::endl;
  err_out << "Srednia odleglosc najblizszych punktow output->target na starcie Ogolna: " << getFitnessScore() << std::endl;

  std::vector<float> previous_correspondence_distances (indices_->size ());
  correspondence_distances_.resize (indices_->size ());
	  /**
	  * TODO:
	  * do we need this?
	  */
    //final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
	  transformPointCloud (*output, *output, guess, 0, 0);

  // Resize the vector of distances between correspondences
	 err_out << "Prog dopasowania korespondentow " << (-1.0)*dist_threshold << std::endl;
     err_out << "Srednia odleglosc najblizszych punktow output->target po poczatkowej transformacji: " << getFitnessScoreDominik(*output); //<< " " << getFitnessScore (correspondence_distances_, previous_correspondence_distances) << " "
		//	<< (transformation_ - previous_transformation_).norm() << std::endl;
   // err_out << std::endl << std::endl << "           ScoreDominik | ScoreOgolny | norma rozn. transf. | trans_eps_  |  Sr dyst. koresp.  |   Prog koresp_  "<< std::endl << std::endl;
	err_out << std::endl << std::endl << "           ScoreDominik |  Sr dyst. koresp.   |   norma rozn. transf. | trans_eps_  |  "<< std::endl << std::endl;  
  
	  
  
  ///number of vertices 
  int m = 0;
  ///number of edges
  int n = 0;
  int nr_iteracji_dla_danego_alfa = 0;
  char sciezka[512];


  loadTopologyMatrxSize(topology_path, &m, &n);

  Eigen::MatrixXf topology(m,n);
  topology = loadTopologyFile(topology_path);


  //Wartosc poczatkowa wspolczynnika alfa
  Eigen::MatrixXf Alpha(m,n);
  Alpha = loadTopologyFile(alpha_path);
  ///change output
  
  sprintf(buf,"%s\\diffs.txt",output_dir);
  std::ofstream diff_out(buf);
 // transformation_epsilon_ = 0.01;
	nr_iterations_ = 0;
	converged_ = false;
	nr_iteracji_dla_danego_alfa = 0;
	 diff_out << "Start with alpha: " << alpha_ << std::endl;
	diff_out << "Srednia odleglosc najblizszych punktow output->target na starcie Dominik: " << getFitnessScoreDominik(*output) << std::endl;
	  Alpha = loadTopologyFile(alpha_path,alpha_);
	  int numer_ = 0;
	  Eigen::MatrixXf final_calculate_temp;
	  Eigen::MatrixXf final_calculate_right(4,4);
	  Eigen::MatrixXf final_calculate_left(4,4);
	     char temp[512];
   sprintf(buf,"%s\\markers_errors.txt",output_dir);
   std::ofstream fileMarkersError(buf);
   fileMarkersError << "         Nr_iteracji     |    Indeks_output    |   Indeks_target     |    Odleglosc  |   lOdp / lPktChmury" << std::endl;
   sprintf(temp,"%s\\alpha%d",output_dir,numer_);
   int result = mkdir(temp);
	sprintf(sciezka,"%s\\%s",temp,"cloudFirstIterationSource.pcd");
	pcl::io::savePCDFile(sciezka, *input_); 
	sprintf(sciezka,"%s\\%s",temp,"cloudFirstIterationMovedSource.pcd");
	pcl::io::savePCDFile(sciezka, *output);  
	sprintf(sciezka,"%s\\%s",temp,"cloudFirstIterationTarget.pcd");
	pcl::io::savePCDFile(sciezka, *target_); 
   sprintf(temp,"%s\\alpha%d",output_dir,numer_);
   mkdir(temp);
   
      FILE* file = nullptr;
      int liczba_markerow = 0;
	    file = fopen(landmarks_path,"rb"); 
		fscanf (file, "%d", &liczba_markerow);
		assert(n > 0);
		
	std::vector<int> markers_source_indices (liczba_markerow);
    std::vector<int> markers_target_indices (liczba_markerow);
	
	znajdzPktNajblizszeLandmark(output,target_, markers_source_indices, markers_target_indices);






	int startTime = clock();
	//RunTest((_TCHAR*)"C:/VTK/1.1/Debug/pcl_visualizer_demo.exe ",(_TCHAR*) "C:/VTK/1.1/Debug/pcl_visualizer_demo.exe D:\\temp\\wyniki\\ cloudcloudFirstIteration  0 1 -i");
  while (!converged_)           // repeat until convergence
  {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;
    // And the previous set of distances
    previous_correspondence_distances = correspondence_distances_;
	
	std::cout << "Iteration number: " << nr_iterations_ << " Alpha value: " << alpha_ << std::endl;
    int cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());


	//DOMINIK
	//Przed petla po wszyskich punktach przesuwanych obliczamy wektor normalnych
	pcl::PointCloud<pcl::Normal>::Ptr output_normals (new pcl::PointCloud<pcl::Normal>);
	//output_normals.operator new (output.size());
    ObliczNormalnePktChmury(output, *output_normals, 10);
	//int przelacznik = 1;

	 



	//pêtla po punktach w chmurze Ÿród³owej
    // Iterating over the entire index vector and  find all correspondences
	int clock_ = clock();
	double sum = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr;
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {


      
      if (!searchForNeighborswithNormalAngleCondition(*output, (*indices_)[idx], output_normals, maxNormalsAngle, nn_indices, nn_dists, searchMethod))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[idx]);
        return;
      }
	   

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        source_indices[cnt] = (*indices_)[idx];    //nr_pkt_zrodlowego
        target_indices[cnt] = nn_indices[0];       //nr_pkt_doceloego
        cnt++;
      }

      // Save the nn_dists[0] to a global vector of distances
      correspondence_distances_[(*indices_)[idx]] = std::min (nn_dists[0], (float)dist_threshold);
	  sum += correspondence_distances_[(*indices_)[idx]];
	  //err_out << "Correspondent nr: " << idx << " distance to nearest neighbor " << correspondence_distances_[(*indices_)[idx]] << std::endl;
    }
	diff_out << nr_iterations_ << " " << ((float)(clock()-clock_))/CLOCKS_PER_SEC << std::endl;
	//diff_out << std::endl << "Nr iteration " << nr_iterations_ << std::endl << "Alpha nr iteration " << nr_iteracji_dla_danego_alfa << std::endl;
	//diff_out << "Number of correspondences found: " << cnt << std::endl;
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize (cnt); target_indices.resize (cnt);   //skrocenie wektorow odpowiedniosc i do porawnej
																//liczby par


	for(int i=0; i < liczba_markerow; i++)
	{

		
		    fileMarkersError.width(15);
			fileMarkersError.precision(5);
			fileMarkersError << nr_iterations_ << " ";
		
			fileMarkersError.width(25);
			fileMarkersError.precision(5);
			//numer najblizszego punktu w source dla danego markera
			fileMarkersError << markers_source_indices[i] << " ";
		
			fileMarkersError.width(15);
			fileMarkersError.precision(5);
			//numer najblizszego punktu w target dla danego markera
			fileMarkersError << markers_target_indices[i]<< " / ";

			/*fileMarkersError.width(15);
			fileMarkersError.precision(5);*/
			fileMarkersError	<<  target_indices[markers_source_indices[i]];

			znajdzWierszKolumnePktChmury(target_->width, target_->height, markers_target_indices[i], &wierszPozadany, &kolumnaPozadana);
		    znajdzWierszKolumnePktChmury(target_->width, target_->height, target_indices[markers_source_indices[i]], &wierszRzeczywisty, &kolumnaRzeczywista);

            fileMarkersError.width(15);
			fileMarkersError.precision(5);
			fileMarkersError  << abs(wierszRzeczywisty - wierszPozadany) << " / " << abs(kolumnaRzeczywista - kolumnaPozadana);

			//roznca indeksow w source i target
			//fileMarkersError	<< std::abs(indiceTarget-indiceSource) << " ";

			fileMarkersError.width(15);
			fileMarkersError.precision(5);
			////odleglosc punktow odpowiadajacym markerom w source i target
			//fileMarkersError << std::sqrt(std::pow(cloud_target->points[indiceTarget].x-cloud_src->points[indiceSource].x,2)+
			/*				std::pow(cloud_target->points[indiceTarget].y-cloud_src->points[indiceSource].y,2)+
							std::pow(cloud_target->points[indiceTa*/

		    fileMarkersError.width(15);
		    fileMarkersError.precision(5);
		    fileMarkersError << "     "  <<	cnt  <<  " / " << output_.size()  <<   std::endl;
			fileMarkersError << std::endl << std::endl;
			
  
	}
	fileMarkersError << "--------------------------------------------------------------------------------------------------" << std::endl << std::endl;

	//==========================================================================================================
	//Znajdowanie normalnej do powierzchni
	   
       
		//pcl::PointCloud<pcl::Normal>::Ptr output_normals (new pcl::PointCloud<pcl::Normal>);
		//ObliczNormalnePktChmury(output, *output_normals, 10);


		//


		///*pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
		//ObliczNormalnePktChmury(target_, *target_normals, 10);*/
  //      




		//Eigen::VectorXd vecKatow (cnt);


		//ObliczKatyNormalnychwParachOdpowiednikow( *output_normals, source_indices, *target_normals_, target_indices, vecKatow);

		//std::cout <<  vecKatow; 
		//int licznik_dobrych_odpowiednikow = 0;
		//std::vector<int> source_indices_war_normalne (cnt);
  //      std::vector<int> target_indices_war_normalne (cnt);

		//for (size_t idx = 0; idx < cnt; ++idx)
  //      {

		//	if (vecKatow(idx) < 30 ) 
		//	{
		//		source_indices_war_normalne[licznik_dobrych_odpowiednikow] = source_indices[idx];
		//		target_indices_war_normalne[licznik_dobrych_odpowiednikow] = target_indices[idx];
		//		licznik_dobrych_odpowiednikow++;
		//	
		//	}

		//}
		//source_indices_war_normalne.resize(licznik_dobrych_odpowiednikow);
		//target_indices_war_normalne.resize(licznik_dobrych_odpowiednikow);


	//=========================================================================================================
	//Blok RANSAC-a odpowiadajacy za wykluczanie zlych odopowiednikow


    //std::vector<int> source_indices_good;      //brak deklaracji dlugosci 
    //std::vector<int> target_indices_good;      //defacto dlugosc inliaers z RANSAC
    //{
    //  // From the set of correspondences found, attempt to remove outliers
    //  // Create the registration model
    //  typedef typename SampleConsensusModelRegistration<PointSource>::Ptr SampleConsensusModelRegistrationPtr;
    //  SampleConsensusModelRegistrationPtr model;
    //  model.reset (new SampleConsensusModelRegistration<PointSource> (output.makeShared (), source_indices));
    //  // Pass the target_indices
    //  model->setInputTarget (target_, target_indices);
    //  // Create a RANSAC model
    //  RandomSampleConsensus<PointSource> sac (model, inlier_threshold_);
    //  sac.setMaxIterations (1000);

    //  // Compute the set of inliers
    //  if (!sac.computeModel ())
    //  {
    //    source_indices_good = source_indices;
    //    target_indices_good = target_indices;
    //  }
    //  else
    //  {
    //    std::vector<int> inliers;
    //    // Get the inliers
    //    sac.getInliers (inliers);
    //    source_indices_good.resize (inliers.size ());     //
    //    target_indices_good.resize (inliers.size ());    // skrocenie wektorow do liczby odpowiednich par

    //    boost::unordered_map<int, int> source_to_target;
    //    for (unsigned int i = 0; i < source_indices.size(); ++i)
    //      source_to_target[source_indices[i]] = target_indices[i];

    //    // Copy just the inliers
    //    std::copy(inliers.begin(), inliers.end(), source_indices_good.begin());
    //    for (size_t i = 0; i < inliers.size (); ++i)
    //      target_indices_good[i] = source_to_target[inliers[i]];
    //  }
    //}

	//DOminik
	//source_indices_good - nr punktow wejsciowych, ktore maja odpowiedniki
	//target_indices_good - nr punktow wejsciowych, ktore maja odpowiedniki

    // Check whether we have enough correspondences
 /*     cnt = (int)source_indices_good.size ();
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    PCL_DEBUG ("[pcl::%s::computeTransformation] Number of correspondences %d [%f%%] out of %lu points [100.0%%], RANSAC rejected: %lu [%f%%].\n", getClassName ().c_str (), cnt, (cnt * 100.0) / indices_->size (), (unsigned long)indices_->size (), (unsigned long)source_indices.size () - cnt, (source_indices.size () - cnt) * 100.0 / source_indices.size ());*/
   
	//KOniec bloku RANSACA




    // Estimate the transform


	transformation_estimation_->estimateRigidTransformation (*output, source_indices, *target_, target_indices, transformation_, Alpha, nr_iterations_,landmarks_path,topology,temp);

	
    // Tranform the data
    transformPointCloud(*output,  *output, transformation_, 0, 0);
	
    char nr_ramki[512];
	if(nr_iterations_ < 10)
	{
		nr_ramki[2] = nr_iterations_ + '0';
		nr_ramki[1] = '0';
		nr_ramki[0] = '0';
	}
	else if(nr_iterations_ < 100)
	{
		nr_ramki[2] = nr_iterations_%10 + '0';
		nr_ramki[1] = nr_iterations_/10 + '0';
		nr_ramki[0] = '0';
	}
	else
	{
		int k = nr_iterations_/100;
		nr_ramki[2] = (nr_iterations_ - k*100) % 10 + '0';
		nr_ramki[1] = (nr_iterations_ - k*100)/10 + '0';
		nr_ramki[0] = k + '0';
	}
	PointCloudSource input_transformed;
	transformPointCloud (*input_, input_transformed, transformation_,0,0);
	sprintf(sciezka,"%s\\sourceTransformed%c%c%c%s",temp,nr_ramki[0], nr_ramki[1],nr_ramki[2],".pcd");
	pcl::io::savePCDFile(sciezka, input_transformed); 
	//zapis chmury do pliku
	sprintf(sciezka,"%s\\outputTransformed%c%c%c%s",temp,nr_ramki[0], nr_ramki[1],nr_ramki[2],".pcd");
	pcl::io::savePCDFile(sciezka, *output); 



    // Obtain the final transformation    
    //final_transformation_ = transformation_ * final_transformation_;
	for(int i = 0;i < indices_->size();++i) {
		//final
		final_calculate_temp = final_transformation_.block<4, 3> (4*i, 0);
		final_calculate_right.block<3,4>(0,0) = final_calculate_temp.transpose();
		final_calculate_right.block<1,4>(3,0) << 0,0,0,1;
		//current
		final_calculate_temp = transformation_.block<4, 3> (4*i, 0);
		final_calculate_left.block<3,4>(0,0) = final_calculate_temp.transpose();
		final_calculate_left.block<1,4>(3,0) << 0,0,0,1;
		//multiply and insert
		//std::cout <<final_transformation_.block<4, 3> (4*i, 0) << std::endl;
		//std::cout <<final_calculate_right << std::endl;
		//std::cout <<final_calculate_left << std::endl;
		final_transformation_.block<4,3>(4*i,0) = (final_calculate_left*final_calculate_right).block<3,4>(0,0).transpose();
		//std::cout << final_transformation_.block<4, 3> (4*i, 0) << std::endl;
	}
	/*
	///ustawienie epsilon
	if ( nr_iteracji_dla_danego_alfa == 0 ) 
	{
		transformation_epsilon_ = (transformation_ - previous_transformation_).norm();
	    std::cout << transformation_epsilon_;
	}
	*/
 //	trans_norm << nr_iterations_ << (transformation_ - previous_transformation_).norm() << std::endl;
 //	diff_out << "Norma roznicy transformaty i transformaty poprzedniej iteracji: " << (transformation_ - previous_transformation_).norm() << std::endl;
//	diff_out << nr_iterations_ << "  " << (transformation_ - previous_transformation_).norm() << std::endl;


//	diff_out << "Srednia z (odl pomiedzy odpowiednikami ) - (odl pomiedzy odpowiednikami z poprzedniej iteracji) (fitness euclidean): " <<	\
//		getFitnessScore (correspondence_distances_, previous_correspondence_distances) << std::endl;
	//upd_transformation = transformation_ * final_transformation_;
	//diff_out << "Global diff: " << (upd_transformation-final_transformation_).norm() << std::endl;
	//final_transformation_ = upd_transformation;
		//double score = getFitnessScore();
		double score = getFitnessScoreDominik(*output);
		//if(score <= 0.75)
		//	converged_=true;
//		diff_out << "Srednia odleglosc od najblizszego punktu w chmurze(fitness score): " << score << std::endl;
		err_out.width(3);
		err_out << nr_iterations_ << " ";
		err_out.width(15);
		err_out.precision(5);
		err_out << score << " ";
		err_out.width(15);
		err_out.precision(5);
		err_out << sum/cnt ;    //fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances));
		/*err_out.width(15);
		err_out.precision(5);
		err_out << getFitnessScore (correspondence_distances_, previous_correspondence_distances) << " ";*/
		err_out.width(15);
		err_out.precision(5);
		err_out		<< fabs ((transformation_ - previous_transformation_).sum ())<< " ";
		err_out.width(15);
		err_out.precision(5);
		err_out		<< "     "  <<  transformation_epsilon_/10  << std::endl;
		

		
		/*err_out.width(15);
		err_out.precision(5);
		err_out << euclidean_fitness_epsilon_ << std::endl; */
		

		//findMarkersError(nr_iterations_,fileMarkersError,output,target_);


		nr_iterations_++;
	    nr_iteracji_dla_danego_alfa++;
	
	//warunek zmiany alfa
	if ( (transformation_ - previous_transformation_).norm() <= transformation_epsilon_ )
	{
		/*float alfa_temp = Alpha(0,0) - alpha_step;
		if ( alfa_temp <= 0 ) alfa_temp =1; 
		Alpha.fill(alfa_temp);*/
		//ZERUJEMY LICZBE ODBYTYCH ITERACJI DLA NOWEGO ALPHA
		nr_iteracji_dla_danego_alfa = 0;
		err_out << std::endl << std::endl << "Change alpha! Old value: " << alpha_;
		alpha_ = alpha_ - alpha_step*alpha_;
		err_out << " New value: " << alpha_ << " Iteration: " << nr_iterations_-1 << std::endl ;
		Alpha = loadTopologyFile(alpha_path,alpha_);
		diff_out << "CHANGE ALPHA!!! NEW VALUE: " << alpha_ << std::endl << std::endl;
   sprintf(temp,"%s\\alpha%d",output_dir,numer_);
   mkdir(temp);
		numer_++;
		if(alpha_ <= 0 || numer_ >= 20)
			converged_ = true;
	}


    // Update the vizualization of icp convergence
    if (update_visualizer_ != 0)
    //  update_visualizer_(output, source_indices_good, *target_, target_indices_good );
	  update_visualizer_(*output, source_indices, *target_, target_indices );

    // Various/Different convergence termination criteria
    // 1. Number of iterations has reached the maximum user imposed number of iterations (via 
    //    setMaximumIterations)
    // 2. The epsilon (difference) between the previous transformation and the current estimated transformation 
    //    is smaller than an user imposed value (via setTransformationEpsilon)
    // 3. The sum of Euclidean squared errors is smaller than a user defined threshold (via 
    //    setEuclideanFitnessEpsilon)
	
	double roznica_transformat1 = fabs ((transformation_ - previous_transformation_).sum ());
	double roznica_transformat2 = fabs ((transformation_ - previous_transformation_).norm ());
	double roznica_transformat3 = (transformation_ - previous_transformation_).norm ();


    if (nr_iterations_ >= max_iterations_ ||
        fabs ((transformation_ - previous_transformation_).sum ()) < transformation_epsilon_/10 ||
        fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)) <= euclidean_fitness_epsilon_
       )
    {
      converged_ = true;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, fabs ((transformation_ - previous_transformation_).sum ()));

      PCL_DEBUG ("nr_iterations_ (%d) >= max_iterations_ (%d)\n", nr_iterations_, max_iterations_);
      PCL_DEBUG ("fabs ((transformation_ - previous_transformation_).sum ()) (%f) < transformation_epsilon_ (%f)\n",
                 fabs ((transformation_ - previous_transformation_).sum ()), transformation_epsilon_);
      PCL_DEBUG ("fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)) (%f) <= euclidean_fitness_epsilon_ (%f)\n",
                 fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)),
                 euclidean_fitness_epsilon_);

    }
	}
	
	diff_out << "Finished" << std::endl;
	diff_out << "Time till convergence: " << ((float)(clock()-startTime))/CLOCKS_PER_SEC << std::endl;
	err_out << std::endl << std::endl;
	err_out << "Nr iterations: " << nr_iterations_ << " / " << max_iterations_  << std::endl;
	err_out << "Transformation difference norm: " << fabs ((transformation_ - previous_transformation_).sum ()) ;
	err_out << " / transformation_epsilon_: " << transformation_epsilon_ /10 << std::endl;

	//err_out << "Correspondences' fitness score: " << getFitnessScore (correspondence_distances_, previous_correspondence_distances) << std::endl;
	err_out << "fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)): " ;
	err_out << fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances));
	err_out << " / euclidean_fitness_epsilon_: " << euclidean_fitness_epsilon_   << std::endl;  

	err_out << "Final transformation: "<< final_transformation_.rows() << "x" << final_transformation_.cols() << std::endl;
	err_out << final_transformation_ << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ> FinalCloud;
	  if (FinalCloud.points.size () != indices_->size ())
    FinalCloud.points.resize (indices_->size ());
  // Copy the header
  FinalCloud.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    FinalCloud.width    = indices_->size ();
    FinalCloud.height   = 1;
  }
  else
  {
    FinalCloud.width    = input_->width;
    FinalCloud.height   = input_->height;
  }
  FinalCloud.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    FinalCloud.points[i] = input_->points[(*indices_)[i]];
	  transformPointCloud (FinalCloud, FinalCloud, final_transformation_, 0, 0);
  sprintf(buf,"%s\\final.pcd",output_dir);
	  pcl::io::savePCDFile(buf,FinalCloud);
	err_out.close();
	diff_out.close();
  trans_norm.close();
  fileMarkersError.close();
}
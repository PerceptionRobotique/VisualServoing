/*!
 \file PGMPerspectiveVisualServoing.cpp
 \brief Photometric Gaussian Mixture (PGM) -based Visual Servoing. SSD compares current and desired PGMs for perpsective camera control (6 DOFs), exploiting PeR core, core_extended, io, features, estimation and sensor_pose_estimation modules
 * example command line :
./build/PGMPerspVS ./PGM_Perspective_VisualServoing_media/calibration/calib_640_512.xml 6 5.0 1000 10 -10
 \param xmlFic the perspective camera calibration xml file
 \param redFac the image scale reduction factor
 \param lambda_g the initial Gaussian expansion parameter
 \param metFac the factor to transform shiftX to meters
 \param shiftX the signed lateral shift (in coherent units regarding metFac)
 \param shiftZ the signed depth shift (in coherent units regarding metFac)
 \param rotY the signed vertical rotation (in degrees)
 *
 \author Guillaume CARON
 \version 0.1
 \date September 2020 ; December 2023 (camera genericity, applied to prOmni) ; February 2024 (prEqui)
 */

#define WITHROBOT
#define WITHCAMERA

//#define WITHFLIRCAM
#define WITHUVCTHETA

#ifdef WITHROBOT
  #include "src/C_UR.h"
#endif

#ifdef WITHCAMERA
  #ifdef WITHFLIRCAM
//  #include "src/CamFlir.hpp"
    #include "src/CamFlirSpinnaker.hpp"
  #elif defined(WITHUVCTHETA)
    #include "src/CamUVCtheta.hpp"
  #endif
#endif

#include <iostream>
#include <iomanip>

//#define O
#define E

#ifdef O
#include <per/prOmni.h>
#include <per/prOmniXML.h>
#elif defined(E)
#include <per/prEquirectangular.h>
#include <per/prEquirectangularXML.h>
#else //P
#include <per/prPerspective.h>
#include <per/prPerspectiveXML.h>
#endif

#include <per/prRegularlySampledCPImage.h>

#include <per/prPhotometricnnGMS.h>
#include <per/prFeaturesSet.h>

#include <per/prSSDCmp.h>

//#include <per/prPosePerspectiveEstim.h>
#include <per/prCameraPoseEstim.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

#define VERBOSE

#define INDICATORS
#define FILE_EXT "jpg"
#define OPT_DISP_MIN
#define OPT_DISP_MAX
//#define OPT_CLICK

/*!
 * \fn main()
 * \brief Main function of the PGM SSD perspective VS
 *
 * 1. Loading a perspective camera model from an XML file got from the MV calibration software
 * 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
 * 3. Successive computation of the current festures set for every acquired image that are used to control the camera toward the desired image
 * 4. Save the PGM-SSD for each image, processing times and computed velocities
 *
 * \return
 *          0 if the program ended without any issue
 *         -1 if no XML perspectrive camera is provided
 *         -2 if no reduction factor is provided
 *         -3 no initial lambda_g value
 */
int main(int argc, char **argv)
{
#ifdef INDICATORS
    std::ostringstream s;
    std::string filename;
#endif // INDICATORS
    
    //1. Loading a perpsective camera from an XML file got from the MV calibration software
    if(argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no XML perspective camera file given" << std::endl;
#endif
        return -1;
    }

    //Create a camera
    prSensorModel *_sensor = nullptr;
#ifdef O
    _sensor = new prOmni();
    prOmni *_camera = (prOmni *)_sensor;

    // Load the camera parameters from the XML file
    prOmniXML fromFile(argv[1]);
#elif defined(E)
    _sensor = new prEquirectangular();
    prEquirectangular *_camera = (prEquirectangular *)_sensor;

    // Load the camera parameters from the XML file
    prEquirectangularXML fromFile(argv[1]);
#else //including P for perspective camera, which is the default
    _sensor = new prPerspective();
    prPerspective *_camera = (prPerspective *)_sensor;

    // Load the camera parameters from the XML file
    prPerspectiveXML fromFile(argv[1]);
#endif

    fromFile >> (*_camera);

#ifdef VERBOSE
    std::cout << "Loading the XML file to an empty rig..." << std::endl;

    // If a sensor is loaded, print its parameters
    std::cout << "type " << _camera->getType() << " camera base intrinsic parameters are alpha_u = " << _camera->getau() << " ; alpha_v = " << _camera->getav() << " ; u_0 = " << _camera->getu0() << " ; v_0 = " << _camera->getv0() << std::endl;

#endif

    //Get the image size reduction factor
    if(argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no reduction factor" << std::endl;
#endif
        return -2;
    }
    unsigned int redFac = atoi(argv[2]);
    
    //Get the initial lambda_g value
    if(argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no initial lambda_g" << std::endl;
#endif
        return -3;
    }
    float lambda_g = atof(argv[3]);//0.35;//0.035;//
    
    //Loading the desired image with respect to which servo the camera
    vpImage<unsigned char> I_des, I_cur;
#ifndef WITHCAMERA
    if(argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    //Get filename thanks to boost
    char *desiredImageFilename = (char *)argv[4];

    try
    {
      vpImageIo::read(I_des, desiredImageFilename);
    }
    catch(vpException e)
    {
#ifdef VERBOSE
        std::cout << "unable to load the desired image file" << std::endl;
#endif
        return -5;
    }
#endif

#ifdef WITHROBOT       
	C_UR UR10("192.168.1.3", 30002, 2.0);//0.032);

  // with F0.95 lens
	vpColVector j_init(6);
/*	j_init[0] = vpMath::rad(-4.87);
	j_init[1] = vpMath::rad(-127.69);
	j_init[2] = vpMath::rad(-91.34);
	j_init[3] = vpMath::rad(-140.45);
	j_init[4] = vpMath::rad(-44.99);
	j_init[5] = vpMath::rad(-0.2);*/
/*
  //0.5 m depth
	j_init[0] = vpMath::rad(-44.22);
	j_init[1] = vpMath::rad(-151.17);
	j_init[2] = vpMath::rad(-99.14);
	j_init[3] = vpMath::rad(-17.27);
	j_init[4] = vpMath::rad(88.36);
	j_init[5] = vpMath::rad(91.60);
*/
  //to the sky and wall
	j_init[0] = vpMath::rad(16.52);
	j_init[1] = vpMath::rad(-56.24);
	j_init[2] = vpMath::rad(-118.27);
	j_init[3] = vpMath::rad(-67.21);
	j_init[4] = vpMath::rad(84.13);
	j_init[5] = vpMath::rad(96.01);
  
  /*
  j_init[0] = vpMath::rad(13.57);
	j_init[1] = vpMath::rad(-74.55);
	j_init[2] = vpMath::rad(-130.63);
	j_init[3] = vpMath::rad(36.28);
	j_init[4] = vpMath::rad(85.43);
	j_init[5] = vpMath::rad(93.26);
  */
/*
  //0.3 m depth
	j_init[0] = vpMath::rad(-38.24);
	j_init[1] = vpMath::rad(-124.0);
	j_init[2] = vpMath::rad(-101.22);
	j_init[3] = vpMath::rad(-42.47);
	j_init[4] = vpMath::rad(88.19);
	j_init[5] = vpMath::rad(97.87);
*/

  UR10.setCameraArticularPose(j_init);

  vpTime::wait(1000);
#endif

#ifdef WITHCAMERA
    vpImage<unsigned char> Iacq;

    #ifdef WITHFLIRCAM
      //Parametres intrinseques pour FlirCam
	    int larg = 640/redFac, haut = 512/redFac;

      CamFlirSpinnaker<unsigned char> grabber(larg,haut,8,0); 
    #elif defined(WITHUVCTHETA)
      //Parametres intrinseques pour UVC Theta X FHD
	    int larg = 1920/redFac, haut = 960/redFac;

      CamUVCtheta<unsigned char> grabber(larg,haut,8,0); 
    #endif
   vpTime::wait(1000);
    //Acquisition
    for(int iac=0;iac<10;iac++)
    grabber.getFrame(Iacq);

    vpImageTools::resize(Iacq, I_des, larg, haut, vpImageTools::vpImageInterpolationType::INTERPOLATION_CUBIC);//INTERPOLATION_LINEAR);//

    std::cout << I_des.getHeight() << " " << I_des.getWidth() << std::endl;
#endif

#ifdef OPT_DISP_MIN
    vpDisplayX disp_I_des;
    disp_I_des.init(I_des, 100, 100, "I_des");
    vpDisplay::display(I_des);
    vpDisplay::flush(I_des);
#ifdef OPT_CLICK
    vpDisplay::getClick(I_des);
#endif //OPT_CLICK
#endif //OPT_DISP_MIN

#ifdef INDICATORS
  s.str("");
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "resultat/Id." << FILE_EXT;
  filename = s.str();
  vpImageIo::write(I_des, filename);
#endif //INDICATORS

    // 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
    /*
    prSensorPoseFeature sp_feature(); 
    pr3DPointFeature tdp_feature(sp_feature); //d X / d P
    prNormalizedImagePointFeature nip_feature(tdp_feature, perspCam); //d x / d X
    prDigitalImagePointFeature dip_feature(nip_feature, perspCam); //d u / d x
    prPhotometricnnGMS pnnGM_feature(dip_feature); //d G / d u
    //pnnGM_feature.getJacobian() internally computes myJacobian * parent_feature.getJacobian(), parent_feature calling itself its own parent feature getJacobian until there is no more ancester
    */
    //initialisation de l'AV
    //prPhotometricnnGMS<prCartesian2DPointVec>,
    //prPosePerspectiveEstim
    prCameraPoseEstim<prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >, 
                      prSSDCmp<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec> > > servo;

    bool dofs[6] = {true, true, true, true, true, true};
    //bool dofs[6] = {true, true, true, false, false, true}; // no coupling?

    servo.setdof(dofs[0], dofs[1], dofs[2], dofs[3], dofs[4], dofs[5]);
    
    _camera->setPixelRatio(_camera->getau()/redFac, _camera->getav()/redFac);
    _camera->setPrincipalPoint(_camera->getu0()/redFac, _camera->getv0()/redFac);
    servo.setSensor(_camera);

    //prepare the desired image 
    prRegularlySampledCPImage<unsigned char> IP_des(I_des.getHeight(), I_des.getWidth()); //the regularly sample planar image to be set from the acquired/loaded perspective image
    IP_des.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    std::cout << "build" << std::endl;
    IP_des.buildFrom(I_des, _camera); 
    std::cout << "built" << std::endl;

//    IP_des.toAbsZN(); //prepare pixels intensities ? 
    prRegularlySampledCPImage<float> GP(I_des.getHeight(), I_des.getWidth()); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_des;
    prPhotometricnnGMS<prCartesian2DPointVec> GP_sample_des(lambda_g);

    double t0;
    t0 = vpTime::measureTimeMs();
    fSet_des.buildFrom(IP_des, GP, GP_sample_des, false, true); // Goulot !
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_des built in: " << -t0 << " ms" << std::endl;

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<float> PGM_des_f(I_des.getHeight(), I_des.getWidth());
    vpImage<unsigned char> PGM_des_u;
    vpPoseVector pp;
    fSet_des.sampler.toImage(PGM_des_f, pp, _sensor);
    vpImageConvert::convert(PGM_des_f, PGM_des_u);
#endif

#ifdef OPT_DISP_MAX
    vpDisplayX disp_PGM_des;
    disp_PGM_des.init(PGM_des_u, 100+I_des.getWidth()+5, 100, "PGM_des");
    vpDisplay::display(PGM_des_u);
    vpDisplay::flush(PGM_des_u);
#endif

#ifdef INDICATORS
    //to save iterations
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/PGMd." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(PGM_des_u, filename);
#endif //INDICATORS

    servo.buildFrom(fSet_des);

    servo.initControl(1.0f, 0.5f);
    //servo.initControl(0.2f, 0.3f);

    prPhotometricnnGMS<prCartesian2DPointVec> GP_sample(lambda_g);
    std::cout << "nb features : " << fSet_des.set.size() << std::endl;

#ifdef WITHROBOT
  std::cout << "Deplacement vers pose initiale " << std::endl;
	vpColVector p_init;
  p_init.resize(6);

//Get the initial lambda_g value
  float metFac = 1.f;
  if(argc < 5)
  {
#ifdef VERBOSE
      std::cout << "no metric factor given" << std::endl;
#endif
  }
  else
    metFac = atof(argv[4]);

  float shiftX = 0.f;
  if(argc < 6)
  {
#ifdef VERBOSE
      std::cout << "no shift X given" << std::endl;
#endif
  }
  else
    shiftX = atof(argv[5]);

  float shiftZ = 0.f;
  if(argc < 7)
  {
#ifdef VERBOSE
      std::cout << "no shift Z given" << std::endl;
#endif
  }
  else
    shiftZ = atof(argv[6]);

  float rotY = 0.f;
  if(argc < 8)
  {
#ifdef VERBOSE
      std::cout << "no rotate Y given" << std::endl;
#endif
  }
  else
    rotY = atof(argv[7]);

  p_init[0] = shiftX/metFac;
  p_init[2] = shiftZ/metFac;
  p_init[4] = rotY*M_PI/180.0;

  std::cout << p_init.t() << std::endl;

	UR10.setCameraRelativePose(p_init);

  vpTime::wait(5000);
#endif

#ifdef WITHCAMERA
  //Acquisition
  grabber.getFrame(Iacq);
  vpImageTools::resize(Iacq, I_cur, larg, haut, vpImageTools::vpImageInterpolationType::INTERPOLATION_CUBIC);//INTERPOLATION_LINEAR);//
#endif

    //3. Successive computation of the current features set for every acquired image that are used to control the camera toward the desired image
    bool updateSampler = true;
    bool poseJacobianCompute = true;
    //activate the M-Estimator
    bool robust = false;

    //Build current features set
#ifdef OPT_DISP_MIN
    vpDisplayX disp_I_cur;
    disp_I_cur.init(I_cur, 100, 100+I_des.getHeight()+30, "I_cur");

    vpDisplay::display(I_cur);
    vpDisplay::flush(I_cur);
#endif
        
    // Current features set setting from the current image
    prRegularlySampledCPImage<unsigned char> IP_cur(I_des.getHeight(), I_des.getWidth());
    IP_cur.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    IP_cur.buildFrom(I_cur, _camera); 
        
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_cur;
    t0 = vpTime::measureTimeMs();
    fSet_cur.buildFrom(IP_cur, GP, GP_sample, poseJacobianCompute, updateSampler); // Goulot !
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_cur built in: " << -t0 << " ms" << std::endl;

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<float> PGM_cur_f(I_cur.getHeight(), I_cur.getWidth());
    vpImage<unsigned char> PGM_cur_u;
    fSet_cur.sampler.toImage(PGM_cur_f, pp, _sensor);
    vpImageConvert::convert(PGM_cur_f, PGM_cur_u);
#endif

#ifdef OPT_DISP_MAX
    vpDisplayX disp_PGM_cur;
    disp_PGM_cur.init(PGM_cur_u, 100+I_des.getWidth()+5, 100+PGM_des_u.getHeight()+30, "PGM_cur");
    vpDisplay::display(PGM_cur_u);
    vpDisplay::flush(PGM_cur_u);
#endif

#ifdef INDICATORS
    //to save iterations
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/PGM." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(PGM_cur_u, filename);
#endif

#if defined(INDICATORS) || defined(OPT_DISP_MIN)
    //Compute differences of images for illustration purpose
    vpImage<unsigned char> Idiff(haut, larg);
    vpImageTools::imageDifference(I_cur,I_des,Idiff) ;
#endif

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    //Compute differences of PGM for illustration purpose
    vpImage<unsigned char> PGMdiff(haut, larg) ;
    vpImageTools::imageDifference(PGM_cur_u,PGM_des_u,PGMdiff) ;
#endif

#ifdef OPT_DISP_MIN
    // Affiche de l'image de difference
    vpDisplayX disp_I_diff;
    disp_I_diff.init(Idiff, 100, 100+I_des.getHeight()+30+I_cur.getHeight()+30, "I_cur-I_des (for information)") ;

    vpDisplay::display(Idiff);
    vpDisplay::flush(Idiff);
#endif
#ifdef OPT_DISP_MAX
    // Affiche la difference de PGM
    vpDisplayX disp_PGM_diff;
    disp_PGM_diff.init(PGMdiff, 100+I_des.getWidth()+5, 100+PGM_des_u.getHeight()+30+PGM_cur_u.getHeight()+30, "PGM_cur-PGM_des (minimized)") ;

    vpDisplay::display(PGMdiff);
    vpDisplay::flush(PGMdiff);
#endif //OPT_DISP_MAX


  //Control loop
  // ----------------------------------------------------------
  unsigned int nbDOF = 6, numDOF, indDOF;
  int iter   = 1;
  vpColVector v6(6), v;
  double residual;
#ifdef INDICATORS
/*  vpPoseVector p;
  std::vector<vpPoseVector> v_p;*/
  vpColVector p;
  std::vector<vpColVector> v_p;
  std::vector<double> v_residuals;
  std::vector<vpImage<unsigned char> > v_I_cur, v_PGM_cur;
  std::vector<vpImage<unsigned char> > v_Idiff, v_PGMdiff;
  std::vector<double> v_tms;
#endif //INDICATORS
  double tms, duree;
	do
	{
    tms = vpTime::measureTimeMs();
    std::cout << "--------------------------------------------" << iter++ << std::endl ;

#ifdef INDICATORS
#ifdef WITHROBOT
    UR10.getCameraPoseRaw(p);
#endif //WITHROBOT
#endif //INDICATORS

#ifdef WITHCAMERA
		grabber.getFrame(Iacq);
    vpImageTools::resize(Iacq, I_cur, larg, haut, vpImageTools::vpImageInterpolationType::INTERPOLATION_CUBIC);//INTERPOLATION_LINEAR);//
#endif

    //update current features set
    //std::cout << "build current acquisition model" << std::endl;
    IP_cur.buildFrom(I_cur, _camera); 
    //std::cout << "updateMeasurement" << std::endl;
    fSet_cur.updateMeasurement(IP_cur, GP, GP_sample, poseJacobianCompute, updateSampler);  
    //std::cout << "control" << std::endl;
    //Compute control vector
    residual = 0.5*servo.control(fSet_cur, v, robust);

    std::cout << "error : " << residual << std::endl;

    //update the DOFs
    indDOF = 0;
    for (numDOF = 0 ; numDOF < nbDOF ; numDOF++)
        if (dofs[numDOF])
        {
            v6[numDOF] = -v[indDOF];
            indDOF++;
        }
        else
            v6[numDOF] = 0;

    std::cout << "v6 : " << v6.t() << std::endl;

#ifdef WITHROBOT
    UR10.setCameraVelocity(v6);
#endif

#if defined(OPT_DISP_MIN) || defined(INDICATORS)
    vpImageTools::imageDifference(I_cur,I_des,Idiff) ;
#endif

#if defined(OPT_DISP_MAX) || defined(INDICATORS)
    fSet_cur.sampler.toImage(PGM_cur_f, pp, _sensor);
    vpImageConvert::convert(PGM_cur_f, PGM_cur_u);
    vpImageTools::imageDifference(PGM_cur_u,PGM_des_u,PGMdiff) ;
#endif

#ifdef OPT_DISP_MIN
		vpDisplay::display(I_cur);
		vpDisplay::flush(I_cur);

    vpDisplay::display(Idiff);
    vpDisplay::flush(Idiff);

#endif
#ifdef OPT_DISP_MAX
		vpDisplay::display(PGM_cur_u);
		vpDisplay::flush(PGM_cur_u);

    vpDisplay::display(PGMdiff);
    vpDisplay::flush(PGMdiff);
#endif
    duree = vpTime::measureTimeMs() - tms;
    std::cout << "duration : " << duree <<std::endl;

    //vpDisplay::getClick(I_cur);

#ifdef INDICATORS
    v_p.push_back(p);
    v_residuals.push_back(residual);
    v_I_cur.push_back(I_cur);
    v_PGM_cur.push_back(PGM_cur_u);
    v_Idiff.push_back(Idiff);
    v_PGMdiff.push_back(PGMdiff);
    v_tms.push_back(duree);
#endif //INDICATORS      
  
	}
	while(!vpDisplay::getClick(I_cur, false));

#ifdef INDICATORS
    //save pose list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/poses.txt";
    filename = s.str();
    std::ofstream ficPoses(filename.c_str());
    //save residual list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/residuals.txt";
    filename = s.str();
    std::ofstream ficResiduals(filename.c_str());
    //save the processing times
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/times.txt";
    filename = s.str();
    std::ofstream ficTimes(filename.c_str());

    for(unsigned int i = 0 ; i < v_p.size() ; i++)
    {
      ficPoses << v_p[i].t() << std::endl;
      ficResiduals << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << v_residuals[i] << std::endl;
      ficTimes << v_tms[i] << std::endl;


      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/I/I." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_I_cur[i], filename);

      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/PGM/PGM." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_PGM_cur[i], filename);

      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/IErr/IErr." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_Idiff[i], filename);

      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/PGMErr/PGMErr." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_PGMdiff[i], filename);
    }

    ficPoses.close();
    ficResiduals.close();
    ficTimes.close();
#endif //INDICATORS
    
	return 0;
}

/*!
 \file PGMPerspectiveVisualServoing.cpp
 \brief Photometric Gaussian Mixture (PGM) -based Visual Servoing. SSD compares current and desired PGMs for perpsective camera control (6 DOFs), exploiting PeR core, core_extended, io, features, estimation and sensor_pose_estimation modules
 * example command line :
./build/PGMPerspVS_levels ./PGM_Perspective_VisualServoing_media/calibration/calib_640_512.xml 6 5.0 1 1000 500 10 -200 5
or
./PGMPerspVS_levels ../PGM_Perspective_VisualServoing_media/calibration/calib_640_512.xml 6 5.0 1 1000 500 10 -200 5
 \param xmlFic the perspective camera calibration xml file
 \param redFac the image scale reduction factor
 \param lambda_g the initial Gaussian expansion parameter
 \param levels the amount PGM-VS sequence relying on levels of lambda_g (1: 1 PGM-VS with lambda_g; 2: 1 PGM-VS with lambda_g + 1 PGM-VS with the minimal lambda_g = 0.34; 3: 1 PGM-VS with lambda_g + 1 PGM-VS with lambda_g/2 + 1 PGM-VS with the minimal lambda_g = 0.34; etc)
 \param metFac the factor to transform shiftX to meters
 \param sceneDepth the positive depth of the scene at desired pose (in coherent units regarding metFac)
 \param shiftX the signed lateral shift (in coherent units regarding metFac)
 \param shiftZ the signed depth shift (in coherent units regarding metFac)
 \param rotY the signed vertical rotation (in degrees)
 *
 \author Guillaume CARON
 \version 0.1
 \date September- 2020
 */

#define WITHROBOT
#define WITHCAMERA

#ifdef WITHROBOT
  #include "src/C_UR.h"
#endif

#ifdef WITHCAMERA
//  #include "src/CamFlir.hpp"
  #include "src/CamFlirSpinnaker.hpp"
#endif

#include <iostream>
#include <iomanip>

#include <per/prPerspective.h>

#include <per/prPerspectiveXML.h>
#include <per/prRegularlySampledCPImage.h>

#include <per/prPhotometricnnGMS.h>
#include <per/prFeaturesSet.h>

#include <per/prSSDCmp.h>

#include <per/prPosePerspectiveEstim.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR
//#define MIN_LAMBDA_G 0.34
#define MIN_LAMBDA_G 1.0
//#define MIN_LAMBDA_G 0.5

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
    prSensorModel *perspCam_sensor = new prPerspective();
    prPerspective *perspCam = (prPerspective *)perspCam_sensor;

    // Load the camera parameters from the XML file
    {
        prPerspectiveXML fromFile(argv[1]);
        
        fromFile >> (*perspCam);
    }

#ifdef VERBOSE
    std::cout << "Loading the XML file to an empty rig..." << std::endl;

    // If a sensor is loaded, print its parameters
    std::cout << "the perspective camera intrinsic parameters are alpha_u = " << perspCam->getau() << " ; alpha_v = " << perspCam->getav() << " ; u_0 = " << perspCam->getu0() << " ; v_0 = " << perspCam->getv0() << std::endl;

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

  //Get the number of levels of PGM-VS
  unsigned int levels = 1;
  if(argc < 5)
  {
#ifdef VERBOSE
      std::cout << "no levels provided" << std::endl;
#endif
  }
  else
  {
    levels = atoi(argv[4]);

    // check min value of levels
    if(levels == 0)
      levels = 1;

    // check max value of levels
    unsigned int maxLevels = 2;
    float lambda_g_test = lambda_g;
    while(lambda_g_test > 2.0*MIN_LAMBDA_G)
    {
      maxLevels++;
      lambda_g_test *= 0.5;
    }

    if(levels > maxLevels)
      levels = maxLevels;
  }
    
    //Loading the desired image with respect to which servo the camera
    vpImage<unsigned char> I_des, I_cur;
#ifndef WITHCAMERA
    if(argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    //Get filename thanks to boost
    char *desiredImageFilename = (char *)argv[5];

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
	C_UR UR10("192.168.1.3", 30003, 0.120);//0.032);

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
	j_init[0] = vpMath::rad(-43.63);
	j_init[1] = vpMath::rad(-109.51);
	j_init[2] = vpMath::rad(-94.18);
	j_init[3] = vpMath::rad(-63.91);
	j_init[4] = vpMath::rad(88.46);
	j_init[5] = vpMath::rad(92.64);
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

/*
//red table
  //0.25 m depth
	j_init[0] = vpMath::rad(-118.10);
	j_init[1] = vpMath::rad(-100.44);
	j_init[2] = vpMath::rad(-137.12);
	j_init[3] = vpMath::rad(-30.09);
	j_init[4] = vpMath::rad(88.61);
	j_init[5] = vpMath::rad(94.94);
*/
/*
  //objects to grasp
  //0.25 m depth
	j_init[0] = vpMath::rad(-170.17);
	j_init[1] = vpMath::rad(-159.73);
	j_init[2] = vpMath::rad(-112.02);
	j_init[3] = vpMath::rad(94.55);
	j_init[4] = vpMath::rad(85.74);
	j_init[5] = vpMath::rad(88.81);
*/
/*
  //ground
  //0.5 m depth 1
	j_init[0] = vpMath::rad(-135.91);
	j_init[1] = vpMath::rad(-163.16);
	j_init[2] = vpMath::rad(-95.35);
	j_init[3] = vpMath::rad(-6.66);
	j_init[4] = vpMath::rad(90.63);
	j_init[5] = vpMath::rad(91.46);
*/
/*
  //desk, Marylin
	j_init[0] = vpMath::rad(-51.13);
	j_init[1] = vpMath::rad(-129.05);
	j_init[2] = vpMath::rad(-96.63);
	j_init[3] = vpMath::rad(-41.79);
	j_init[4] = vpMath::rad(88.69);
	j_init[5] = vpMath::rad(84.95);
*/

  //ground
  //0.5 m depth 2
	j_init[0] = vpMath::rad(-130.02);
	j_init[1] = vpMath::rad(-159.33);
	j_init[2] = vpMath::rad(-99.84);
	j_init[3] = vpMath::rad(-5.89);
	j_init[4] = vpMath::rad(90.14);
	j_init[5] = vpMath::rad(97.37);

/*
  //ground
  //0.4 m depth 3
	j_init[0] = vpMath::rad(-129.95);
	j_init[1] = vpMath::rad(-165.05);
	j_init[2] = vpMath::rad(-95.51);
	j_init[3] = vpMath::rad(-4.5);
	j_init[4] = vpMath::rad(90.13);
	j_init[5] = vpMath::rad(97.37);
*/
  UR10.setCameraArticularPose(j_init);

  vpTime::wait(5000);
#endif

    //Parametres intrinseques pour FlirCam
	  int larg = 640/redFac, haut = 512/redFac;
	  
#ifdef WITHCAMERA
    vpImage<unsigned char> Iacq;

    CamFlirSpinnaker<unsigned char> grabber(640,512,8,0); 
   
    //Acquisition
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

  //save the desired pose to file
  s.str("");
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "resultat/desiredPose.txt";
  filename = s.str();
  std::ofstream ficDesiredPose(filename.c_str());

  vpColVector p;
#ifdef WITHROBOT   
  UR10.getCameraPoseRaw(p);
#endif
  ficDesiredPose << p.t() << std::endl;

  ficDesiredPose.close();
#endif //INDICATORS

    // 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
    
    //initialisation de l'AV
    prPosePerspectiveEstim<prPhotometricnnGMS<prCartesian2DPointVec>,
                           prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >, 
                           prSSDCmp<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec> > > servo;

    bool dofs[6] = {true, true, true, true, true, true};
    //bool dofs[6] = {true, true, true, false, false, true}; // no coupling?

    servo.setdof(dofs[0], dofs[1], dofs[2], dofs[3], dofs[4], dofs[5]);
    
    perspCam->setPixelRatio(perspCam->getau()/redFac, perspCam->getav()/redFac);
    perspCam->setPrincipalPoint(perspCam->getu0()/redFac, perspCam->getv0()/redFac);
    servo.setSensor(perspCam);

    //prepare the desired image 
    prRegularlySampledCPImage<unsigned char> IP_des(I_des.getHeight(), I_des.getWidth()); //the regularly sample planar image to be set from the acquired/loaded perspective image
    IP_des.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    std::cout << "build" << std::endl;
    IP_des.buildFrom(I_des, perspCam); 
    std::cout << "built" << std::endl;

//    IP_des.toAbsZN(); //prepare pixels intensities ? 
    //prepare the array of desired photometric Gaussian Mixtures (1 per level)
    prRegularlySampledCPImage<float> GP(I_des.getHeight(), I_des.getWidth()); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > *fSet_des = new prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >[levels];

    prPhotometricnnGMS<prCartesian2DPointVec> GP_sample_des(lambda_g);
    //PGM of the highest lambda_g
    double t0;
    t0 = vpTime::measureTimeMs();
    fSet_des[0].buildFrom(IP_des, GP, GP_sample_des, false, true); 
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_des 0 built in: " << -t0 << " ms" << std::endl;

    //PGM of the intermediary lambda_g (lambda_g/=2)
    for(unsigned int iLevel = 1 ; iLevel < (levels-1) ; iLevel++)
    {
      GP_sample_des.setLambda(0.5*GP_sample_des.getLambda());
      t0 = vpTime::measureTimeMs();
      fSet_des[iLevel].buildFrom(IP_des, GP, GP_sample_des, false, true); 
      t0 -= vpTime::measureTimeMs();
      std::cout << "fSet_des " << iLevel << " built in: " << -t0 << " ms" << std::endl;
    }

    //PGM of the smallest possible lambda_g (similar to P only but with analytic gradients)
    if(levels > 1)
    {
      GP_sample_des.setLambda(MIN_LAMBDA_G); //ssi nbSigma == 3
      t0 = vpTime::measureTimeMs();
      fSet_des[levels-1].buildFrom(IP_des, GP, GP_sample_des, false, true); 
      t0 -= vpTime::measureTimeMs();
      std::cout << "fSet_des " << levels-1 << " built in: " << -t0 << " ms" << std::endl;      
    }

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<float> PGM_des_f(I_des.getHeight(), I_des.getWidth());
    vpImage<unsigned char> PGM_des_u_cur_level;
    vpImage<unsigned char> *PGM_des_u = new vpImage<unsigned char>[levels];
    vpPoseVector pp;
    for(unsigned int iLevel = 0 ; iLevel < levels ; iLevel++)
    {
      fSet_des[iLevel].sampler.toImage(PGM_des_f, pp, perspCam_sensor);
      vpImageConvert::convert(PGM_des_f, PGM_des_u[iLevel]);
    }
#endif

#ifdef OPT_DISP_MAX
    vpDisplayX disp_PGM_des;
    PGM_des_u_cur_level = PGM_des_u[0];
    disp_PGM_des.init(PGM_des_u_cur_level, 100+I_des.getWidth()+5, 100, "PGM_des");
    vpDisplay::display(PGM_des_u_cur_level);
    vpDisplay::flush(PGM_des_u_cur_level);
#endif

#ifdef INDICATORS
    //to save iterations
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/PGMd." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(PGM_des_u_cur_level, filename);
#endif //INDICATORS


  //Get the metric factor
  float metFac = 1.f;
  if(argc < 6)
  {
#ifdef VERBOSE
      std::cout << "no metric factor given" << std::endl;
#endif
  }
  else
    metFac = atof(argv[5]);

  //Get the desired scene depth
  float sceneDepth = 0.5f;
  if(argc < 7)
  {
#ifdef VERBOSE
      std::cout << "no scene depth provided" << std::endl;
#endif
  }
  else
    sceneDepth = atof(argv[6])/metFac;



    servo.buildFrom(fSet_des[0]);

    //initControl a appeler une seule fois si le meme sampler est utilise pour tous les niveaux de lambda_g
    float gain = 0.5f; bool shallow = false;   //if 6 DoF F_Max 0.5f
    //float gain = 1.f; bool shallow = false;   //if 6 DoF F_Max 0.5f
    //float gain = 1.f; bool shallow = false;  //if 4 DoF F_max
    //float gain = 1.0f; bool shallow = true; //if 4 DoF F_min
    //servo.initControl(0.1f, sceneDepth); //if 6 DoF
    std::cout << "sceneDepth : " << sceneDepth << std::endl;
    servo.initControl(gain, sceneDepth); 

    //define the (array of) GP_sample (array mainly for saving to file reasons)
    prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample = new prPhotometricnnGMS<prCartesian2DPointVec>[levels];
    GP_sample[0] = prPhotometricnnGMS<prCartesian2DPointVec>(lambda_g);
    //GP_sample[0].setLambda(lambda_g); //Ca devrait etre possible: possible bug dans les surcharges d'operateurs
    std::cout << "nb features : " << fSet_des[0].set.size() << std::endl;



  float shiftX = 0.f;
  if(argc < 8)
  {
#ifdef VERBOSE
      std::cout << "no shift X given" << std::endl;
#endif
  }
  else
    shiftX = atof(argv[7]);

  float shiftY = 0.f;
  if(argc < 9)
  {
#ifdef VERBOSE
      std::cout << "no shift Y given" << std::endl;
#endif
  }
  else
    shiftY = atof(argv[8]);

  float shiftZ = 0.f;
  if(argc < 10)
  {
#ifdef VERBOSE
      std::cout << "no shift Z given" << std::endl;
#endif
  }
  else
    shiftZ = atof(argv[9]);

  float rotX = 0.f;
  if(argc < 11)
  {
#ifdef VERBOSE
      std::cout << "no rotate X given" << std::endl;
#endif
  }
  else
    rotX = atof(argv[10]);

  float rotY = 0.f;
  if(argc < 12)
  {
#ifdef VERBOSE
      std::cout << "no rotate Y given" << std::endl;
#endif
  }
  else
    rotY = atof(argv[11]);

  float rotZ = 0.f;
  if(argc < 13)
  {
#ifdef VERBOSE
      std::cout << "no rotate Z given" << std::endl;
#endif
  }
  else
    rotZ = atof(argv[12]);

	vpColVector p_init;
  p_init.resize(6);

  p_init[0] = shiftX;// /metFac;
  p_init[1] = shiftY;// /metFac;
  p_init[2] = shiftZ;// /metFac;
  p_init[3] = rotX;// *M_PI/180.0; 
  p_init[4] = rotY;// *M_PI/180.0; 
  p_init[5] = rotZ;// *M_PI/180.0; 

  std::cout << "Deplacement vers pose initiale " << p_init.t() << std::endl;

#ifdef WITHROBOT
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
    prRegularlySampledCPImage<unsigned char> IP_cur(I_cur.getHeight(), I_cur.getWidth());
    IP_cur.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    IP_cur.buildFrom(I_cur, perspCam); 

    //prepare the array of current photometric Gaussian Mixtures (1 per level)
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > *fSet_cur = new prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >[levels];
    t0 = vpTime::measureTimeMs();
    std::cout << "lg : " << GP_sample[0].getLambda() << std::endl;
    fSet_cur[0].buildFrom(IP_cur, GP, GP_sample[0], poseJacobianCompute, updateSampler); 
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_cur 0 built in: " << -t0 << " ms" << std::endl;

    //current PGM of the intermediary lambda_g (lambda_g/=2)
    for(unsigned int iLevel = 1 ; iLevel < (levels-1) ; iLevel++)
    {
      GP_sample[iLevel] = prPhotometricnnGMS<prCartesian2DPointVec>(0.5*GP_sample[iLevel-1].getLambda());
      t0 = vpTime::measureTimeMs();
      fSet_cur[iLevel].buildFrom(IP_cur, GP, GP_sample[iLevel], poseJacobianCompute, updateSampler);
      t0 -= vpTime::measureTimeMs();
      std::cout << "fSet_cur " << iLevel << " built in: " << -t0 << " ms" << std::endl;
    }

    //current PGM of the smallest possible lambda_g (similar to P only but with analytic gradients)
    if(levels > 1)
    {
      GP_sample[levels-1] = prPhotometricnnGMS<prCartesian2DPointVec>(MIN_LAMBDA_G); //ssi nbSigma == 3
      t0 = vpTime::measureTimeMs();
      fSet_cur[levels-1].buildFrom(IP_cur, GP, GP_sample[levels-1], poseJacobianCompute, updateSampler); 
      t0 -= vpTime::measureTimeMs();
      std::cout << "fSet_cur " << levels-1 << " built in: " << -t0 << " ms" << std::endl;
    }


#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<float> PGM_cur_f(I_cur.getHeight(), I_cur.getWidth());
    vpImage<unsigned char> PGM_cur_u;
    fSet_cur[0].sampler.toImage(PGM_cur_f, pp, perspCam_sensor);
    vpImageConvert::convert(PGM_cur_f, PGM_cur_u);
#endif

#ifdef OPT_DISP_MAX
    vpDisplayX disp_PGM_cur;
    disp_PGM_cur.init(PGM_cur_u, 100+I_des.getWidth()+5, 100+PGM_des_u_cur_level.getHeight()+30, "PGM_cur");
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
    vpImageTools::imageDifference(PGM_cur_u,PGM_des_u_cur_level,PGMdiff) ;
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
    disp_PGM_diff.init(PGMdiff, 100+I_des.getWidth()+5, 100+PGM_des_u_cur_level.getHeight()+30+PGM_cur_u.getHeight()+30, "PGM_cur-PGM_des (minimized)") ;

    vpDisplay::display(PGMdiff);
    vpDisplay::flush(PGMdiff);
#endif //OPT_DISP_MAX


  //Control loop
  // ----------------------------------------------------------
  unsigned int iLevel = 0;
  unsigned int nbDOF = 6, numDOF, indDOF;
  int iter   = 1;
  vpColVector v6(6), v(6);
  v = 1;
  double residual_back = 1e30, residual = 0.8*residual_back, residual_diff;
#ifdef INDICATORS
/*  vpPoseVector p;
  std::vector<vpPoseVector> v_p;*/
//  vpColVector p;
  std::vector<vpColVector> v_p;
  std::vector<double> v_residuals;
  std::vector<double> v_lambdas;
  std::vector<vpImage<unsigned char> > v_I_cur, v_PGM_cur;
  std::vector<vpImage<unsigned char> > v_Idiff, v_PGMdiff;
  std::vector<double> v_tms;
  std::vector<bool> v_servo_actif;

  double cond;
  std::vector<double> v_svd;
#endif //INDICATORS
  double tms, duree;
  vpMouseButton::vpMouseButtonType btn;
  bool btn_pressed = false;
  bool servo_actif = false;
  double fact_gain = 1.0;
	do
	{
    tms = vpTime::measureTimeMs();
    if(servo_actif)
      std::cout << "--------------------------------------------" << iter++ << std::endl ;

    //residual_diff = 0.5*residual_diff+0.5*fabs(residual-residual_back);
    residual_diff = fabs(residual-residual_back);
    std::cout << iLevel << " | residual diff / residual_back : " << residual_diff/residual_back << std::endl;
    //seuil pour 4DoF 1e-3
    //seuil pour 6DoF 1e-4
    if(((iter < 600) || ((residual_diff > (gain*(5e-4)*residual_back)) )) && (!shallow || (sqrt(v.sumSquare()) > 1e-5)))
    //if((iter < 60) || (sqrt(v.sumSquare()) > 1e-2))
    {
      residual_back = residual;
    }
    else
    {
      if(iLevel < (levels-1))
      {
        iLevel++;
        if(iLevel == levels)
          fact_gain = 0.25;
          //break;

        servo.buildFrom(fSet_des[iLevel]);
        PGM_des_u_cur_level = PGM_des_u[iLevel];
#ifdef OPT_DISP_MAX
		vpDisplay::display(PGM_des_u_cur_level);
		vpDisplay::flush(PGM_des_u_cur_level);
#endif

        residual_back = 1e30; //as residuals with different lambda_g are not comparable
      }
    }

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
    IP_cur.buildFrom(I_cur, perspCam); 
    fSet_cur[iLevel].updateMeasurement(IP_cur, GP, GP_sample[iLevel], poseJacobianCompute, updateSampler); 

    //Compute control vector
    residual = 0.5*servo.control(fSet_cur[iLevel], v, robust);

    std::cout << "error : " << residual << std::endl;

    v *= fact_gain;

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
    if(servo_actif)
      UR10.setCameraVelocity(v6);
#endif

#if defined(OPT_DISP_MIN) || defined(INDICATORS)
    vpImageTools::imageDifference(I_cur,I_des,Idiff) ;
#endif

#if defined(OPT_DISP_MAX) || defined(INDICATORS)
    fSet_cur[iLevel].sampler.toImage(PGM_cur_f, pp, perspCam_sensor);
    vpImageConvert::convert(PGM_cur_f, PGM_cur_u);
    vpImageTools::imageDifference(PGM_cur_u,PGM_des_u_cur_level,PGMdiff) ;
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

#ifdef INDICATORS
    v_p.push_back(p);
    v_residuals.push_back(residual);
    v_lambdas.push_back(GP_sample[iLevel].getLambda());
    v_I_cur.push_back(I_cur);
    v_PGM_cur.push_back(PGM_cur_u);
    v_Idiff.push_back(Idiff);
    v_PGMdiff.push_back(PGMdiff);
    v_tms.push_back(duree);

    v_servo_actif.push_back(servo_actif);

    cond = servo.getCond(robust);
    v_svd.push_back(cond);
#endif //INDICATORS      
  
    btn_pressed=vpDisplay::getClick(I_cur, btn, false);

    if(btn_pressed && (btn == vpMouseButton::button3))
      servo_actif = !servo_actif;

	}
	while(!btn_pressed || (btn != vpMouseButton::button1));

  v6.resize(6);
#ifdef WITHROBOT   
  UR10.setCameraVelocity(v6);
#endif

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
    //save lambda_g list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/lambda_g.txt";
    filename = s.str();
    std::ofstream ficLambda(filename.c_str());
    //save the processing times
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/times.txt";
    filename = s.str();
    std::ofstream ficTimes(filename.c_str());
    //save svd list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/cond.txt";
    filename = s.str();
    std::ofstream ficSVD(filename.c_str());
    //save servo actif list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/servoActif.txt";
    filename = s.str();
    std::ofstream ficServo(filename.c_str());

    for(unsigned int i = 0 ; i < v_p.size() ; i++)
    {
      ficPoses << v_p[i].t() << std::endl;
      ficResiduals << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << v_residuals[i] << std::endl;
      ficLambda << v_lambdas[i] << std::endl;
      ficTimes << v_tms[i] << std::endl;
      ficSVD << v_svd[i] << std::endl;
      ficServo << v_servo_actif[i] << std::endl;

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
    ficLambda.close();
    ficTimes.close();
    ficSVD.close();
    ficServo.close();
#endif //INDICATORS
    
	return 0;
}

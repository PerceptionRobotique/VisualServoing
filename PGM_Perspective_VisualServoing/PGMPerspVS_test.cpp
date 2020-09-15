/*!
 \file PGMPerspectiveVisualServoing.cpp
 \brief Photometric Gaussian Mixture (PGM) -based Visual Servoing. SSD compares current and desired PGMs for perpsective camera control (6 DOFs), exploiting PeR core, core_extended, io, features, estimation and sensor_pose_estimation modules
 * example command line :
./PGMPerspVS_test ../PGM_Perspective_VisualServoing_media/calibration/calib.xml 3 0.3 ../PGM_Perspective_VisualServoing_media/images_full/desired.png
 \param xmlFic the dual fusheye camera calibration xml file
 \param redFac the image scale reduction factor
 \param lambda_g the initial Gaussian expansion parameter
 \param imDes the desired image
 \param Mask the image file of the mask (white pixels are to be considered whereas black pixels are not)
 *
 \author Guillaume CARON
 \version 0.1
 \date September- 2020
 */

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

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

#define VERBOSE

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
    
    //Loading the desired image with respect to which servo the camera
    vpImage<unsigned char> I_des;
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

    vpDisplayX disp;
    disp.init(I_des, 25, 25, "I_des");
    vpDisplay::display(I_des);
    vpDisplay::flush(I_des);
    
    //lecture de l'image "masque"
    //Chargement du masque
    vpImage<unsigned char> Mask;
    if(argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no mask image given" << std::endl;
#endif
        Mask.resize(I_des.getHeight(), I_des.getWidth(), 255);
    }
    else
    {
        try
        {
            vpImageIo::read(Mask, argv[5]);
        }
        catch(vpException e)
        {
            std::cout << "unable to load mask file" << std::endl;
            Mask.resize(I_des.getHeight(), I_des.getWidth(), 255);
        }
    }
       
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
    prRegularlySampledCPImage<unsigned char> IP_des(I_des.getHeight()/redFac, I_des.getWidth()/redFac); //the regularly sample planar image to be set from the acquired/loaded perspective image
std::cout << "IP_des initialized" << std::endl;
    IP_des.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    
    IP_des.buildFrom(I_des, perspCam, &Mask); 
std::cout << "IP_des built" << std::endl;

vpImage<unsigned char> I_r(I_des.getHeight()/redFac, I_des.getWidth()/redFac);
vpPoseVector p;
IP_des.toImage(I_r, p, perspCam_sensor);
//to save iterations
std::ostringstream s;
std::string filename;
s.str("");
s.setf(std::ios::right, std::ios::adjustfield);
s << "./mixtures/" << std::setfill('0') << std::setw(6) << "IP_des.png";
filename = s.str();
vpImageIo::write(I_r, filename);
    

//    IP_des.toAbsZN(); //prepare pixels intensities ? 
    prRegularlySampledCPImage<float> GP(I_des.getHeight()/redFac, I_des.getWidth()/redFac); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);
std::cout << "GP initialized" << std::endl;
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_des;
    prPhotometricnnGMS<prCartesian2DPointVec> GP_sample_des(lambda_g);

    std::cout << "fSet_des buildFrom" << std::endl;
    double t0;
    t0 = vpTime::measureTimeMs();
    fSet_des.buildFrom(IP_des, GP, GP_sample_des, false, true); // Goulot !
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_des built in: " << -t0 << " ms" << std::endl;

vpImage<float> I_rf(I_des.getHeight()/redFac, I_des.getWidth()/redFac);
fSet_des.sampler.toImage(I_rf, p, perspCam_sensor);
vpImageConvert::convert(I_rf, I_r);
//to save iterations
s.str("");
s.setf(std::ios::right, std::ios::adjustfield);
s << "./mixtures/" << std::setfill('0') << std::setw(6) << "GP_des.png";
filename = s.str();
vpImageIo::write(I_r, filename);

    servo.buildFrom(fSet_des);
std::cout << "servo built" << std::endl;
    servo.initControl();
std::cout << "servo initialized" << std::endl;    
    prPhotometricnnGMS<prCartesian2DPointVec> GP_sample(lambda_g);
    std::cout << "nb features : " << fSet_des.set.size() << std::endl;
    
    //3. Successive computation of the current festures set for every acquired image that are used to control the camera toward the desired image
    bool updateSampler = true;
    bool poseJacobianCompute = true;
    //activate the M-Estimator
    bool robust = false;
    vpImage<unsigned char> I_cur;

    //Build current features set
    vpImageIo::read(I_cur, "/home/gcaron/Developpement/MIS-PR/VisualServoing/PGM_Perspective_VisualServoing/PGM_Perspective_VisualServoing_media/images_full/current_test.png");

    vpDisplayX disp2;
    disp2.init(I_cur, 500, 50, "I_cur");

    vpDisplay::display(I_cur);
    vpDisplay::flush(I_cur);
        
    // Current feature set setting from the current image
    prRegularlySampledCPImage<unsigned char> IP_cur(I_des.getHeight()/redFac, I_des.getWidth()/redFac);
    IP_cur.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    IP_cur.buildFrom(I_cur, perspCam, &Mask); 
        
    prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_cur;
    t0 = vpTime::measureTimeMs();
    fSet_cur.buildFrom(IP_cur, GP, GP_sample, poseJacobianCompute, updateSampler); // Goulot !
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_cur built in: " << -t0 << " ms" << std::endl;

fSet_cur.sampler.toImage(I_rf, p, perspCam_sensor);
vpImageConvert::convert(I_rf, I_r);
//to save iterations
s.str("");
s.setf(std::ios::right, std::ios::adjustfield);
s << "./mixtures/" << std::setfill('0') << std::setw(6) << "GP_init.png";
filename = s.str();
vpImageIo::write(I_r, filename);

    //Compute control vector
    vpColVector v;
    t0 = vpTime::measureTimeMs();
    servo.control(fSet_cur, v, robust);
    t0 -= vpTime::measureTimeMs();
    std::cout << "servo.control: " << -t0 << " ms" << std::endl;

    std::cout << "control vector: " << v.t() << std::endl;


    //update current features set
    vpImageIo::read(I_cur, "/home/gcaron/Developpement/MIS-PR/VisualServoing/PGM_Perspective_VisualServoing/PGM_Perspective_VisualServoing_media/images_full/current_test.png");

    vpDisplay::display(I_cur);
    vpDisplay::flush(I_cur);

    t0 = vpTime::measureTimeMs();
    IP_cur.buildFrom(I_cur, perspCam, &Mask); 
    
    fSet_cur.updateMeasurement(IP_cur, GP, GP_sample, poseJacobianCompute, updateSampler); // Goulot !
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_cur.updateMeasurement: " << -t0 << " ms" << std::endl;

fSet_cur.sampler.toImage(I_rf, p, perspCam_sensor);
vpImageConvert::convert(I_rf, I_r);
//to save iterations
s.str("");
s.setf(std::ios::right, std::ios::adjustfield);
s << "./mixtures/" << std::setfill('0') << std::setw(6) << "GP_cur.png";
filename = s.str();
vpImageIo::write(I_r, filename);

    //Compute control vector
    t0 = vpTime::measureTimeMs();
    servo.control(fSet_cur, v, robust);
    t0 -= vpTime::measureTimeMs();
    std::cout << "servo.control: " << -t0 << " ms" << std::endl;

    std::cout << "control vector: " << v.t() << std::endl;

    
	return 0;
}

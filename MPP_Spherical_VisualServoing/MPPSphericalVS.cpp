/*!
 \file MPPSphericalVS.cpp
 \brief Mixture of Photometric Potentials applied to spherical camera (using here a panoramic equirectangular projection). The control law is built on the minimization of the cost between a reference image taken at desired pose and a current image used to update the control law.
 \param subdivLevel the number of subdivisions used to built the spherical image
 \param lambda_g_1 first expansion parameter used to converge quickly to the desired pose
 \param lambda_g_2 second expansion parameter used to refine the positioning and achieve high precision
 \param camNum camera number located in the /dev/video* folder (0 is typically the built-in laptop webcam and 2 is the desired camera to use)
 \param eMcFile path to the transformation between the robot's end-effector and the camera attached to it
 *
 \author Antoine ANDRÉ
 \version 0.1
 \date September- 2023
 */

#define WITHROBOT
#define WITHCAMERA

#ifdef WITHROBOT
#include <visp/vpRobotFranka.h>
#endif

#ifdef WITHCAMERA
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <iostream>
#include <iomanip>

#include <per/prEquirectangular.h>
#include <per/prRegularlySampledCSImage.h>
#include <per/prPoseSphericalEstim.h>
#include <per/prSSDCmp.h>
#include <per/prFeaturesSet.h>
#include <per/prPhotometricGMS.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

#define VERBOSE

#define INDICATORS
#define FILE_EXT "png"
#define OPT_DISP_MIN
#define OPT_DISP_MAX
// #define OPT_CLICK

/*!
 * \fn main()
 * \brief Main function of the MPP SSD spherical VS
 *
 * 1. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
 * 2. Grabbing frames from the camera to initialize both the robot, the spherical images and the visual servoing class
 * 3. Successive computation of the current festures set for every acquired image that are used to control the camera toward the desired image
 * 4. Save the MPP-SSD for each image, processing times and computed velocities
 *
 * \return
 *          0 if the program ended without any issue
 *         -1 if no subdivision, no rough and no fine lambda are given
 *         -4 if no camera number is given
 *         -5 if no end-effector to camera transformation file (.yaml) is given
 */
int main(int argc, char **argv)
{
#ifdef INDICATORS
    std::ostringstream s;
    std::string filename;

    boost::filesystem::create_directory("resultat");
    boost::filesystem::create_directory("resultat/I");
    boost::filesystem::create_directory("resultat/IErr");
    boost::filesystem::create_directory("resultat/MPP");
    boost::filesystem::create_directory("resultat/MPPErr");
#endif // INDICATORS

    if (argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no subdivision levels given" << std::endl;
#endif
        return -1;
    }
    unsigned int subdivLevel = atoi(argv[1]);

    if (argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no initial wide lambda_g" << std::endl;
#endif
        return -1;
    }
    float lambda_g_1 = atof(argv[2]);

    if (argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no initial fine lambda_g" << std::endl;
#endif
        return -1;
    }
    float lambda_g_2 = atof(argv[3]);

    // Loading the desired image with respect to which servo the camera
    vpImage<unsigned char> I_des, I_cur;
    vpImage<unsigned char> Iacq;
    cv::Mat Iacq_cv;
#ifdef WITHCAMERA
    if (argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no cam number given (identify which camera to use in /dev/video*)" << std::endl;
#endif
        return -4;
    }
    cv::VideoCapture cap;
    cap.open(atoi(argv[4]), cv::CAP_ANY);
    if (!cap.isOpened())
    {
        std::cout << "ERROR! Unable to open camera\n";
    }
    std::cout << "init frame grabber with few images" << std::endl;
    for (int i = 0; i < 20; i++)
    {
        cap.read(Iacq_cv);
        if (Iacq_cv.empty())
        {
            std::cout << "ERROR! blank frame grabbed\n";
            break;
        }
    }
#else
    if (argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    // Get filename thanks to boost
    char *desiredImageFilename = (char *)argv[5];

    try
    {
        vpImageIo::read(I_des, desiredImageFilename);
    }
    catch (vpException e)
    {
#ifdef VERBOSE
        std::cout << "unable to load the desired image file" << std::endl;
#endif
        return -5;
    }
#endif

#ifdef WITHROBOT

    std::string opt_robot_ip = "192.168.8.2";
    if (argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no camera pose file provided" << std::endl;
#endif
        return -5;
    }
    vpPoseVector ePc;
    vpHomogeneousMatrix eMc;
    char *cameraPoseFile = (char *)argv[5];
    std::cout << cameraPoseFile << std::endl;
    ePc.loadYAML(cameraPoseFile, ePc);
    eMc.buildFrom(ePc);

    vpRobotFranka robot;
    std::cout << "Try to connect to robot ip: " << opt_robot_ip << std::endl;
    robot.connect(opt_robot_ip);
    std::cout << "Panda robot connected" << std::endl;

    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    vpColVector currentJointPose({0, 0, 0, 0, 0, 0, 0});
    vpColVector currentCamPose({0, 0, 0, 0, 0, 0});
    vpColVector currentVelocities({0, 0, 0, 0, 0, 0, 0});

    robot.getPosition(vpRobot::JOINT_STATE, currentJointPose);

    vpColVector robotJointPoseRef = {0, -0.785398163397, 0, -2.35619449019 + 0.3, 0, 1.57079632679 - 0.3, 0};
    robot.setPosition(vpRobot::JOINT_STATE, robotJointPoseRef);

    vpTime::wait(5000);
#endif

#ifdef WITHCAMERA
    cv::Mat I_des_cv;
    cap.read(I_des_cv);
    if (I_des_cv.empty())
    {
        std::cout << "ERROR! blank frame grabbed\n";
        return -1;
    }

    int roi_x = 14;
    int roi_y = 120;
    int roi_width = 3840 - roi_x;
    int roi_height = 2160 - 2 * roi_y;
    vpImage<unsigned char> I_des_cropped(roi_height, roi_width, 255);

    double scaleFactor = sqrt((double)roi_height * (double)roi_width / (double)(0.5 * (20.0 * pow(4, subdivLevel)) + 2.0));
    int resizedWidth = round((double)roi_width / scaleFactor);
    int resizedHeight = round((double)roi_height / scaleFactor);

    prEquirectangular ecam(resizedWidth * 0.5 / M_PI, resizedHeight * 0.5 / (M_PI * 0.5), resizedWidth * 0.5, resizedHeight * 0.5);

    vpImageConvert::convert(I_des_cv, I_des);
    vpImageTools::crop(I_des, roi_y, roi_x, roi_height, roi_width, I_des_cropped);

    vpImage<unsigned char> I_des_small(resizedHeight, resizedWidth);
    vpImageTools::resize(I_des_cropped, I_des_small, vpImageTools::INTERPOLATION_AREA);
    vpImage<unsigned char> Mask_small(resizedHeight, resizedWidth, 255);

#endif

#ifdef OPT_DISP_MIN
    vpImage<unsigned char> I_des_for_disp(I_des_cropped.getHeight() / 4, I_des_cropped.getWidth() / 4, 255);
    vpImageTools::resize(I_des_cropped, I_des_for_disp, vpImageTools::INTERPOLATION_AREA);
    vpDisplayX disp_I_des;
    disp_I_des.init(I_des_for_disp, 100, 100, "I_des");
    vpDisplay::display(I_des_for_disp);
    vpDisplay::flush(I_des_for_disp);
#ifdef OPT_CLICK
    vpDisplay::getClick(I_des);
#endif // OPT_CLICK
#endif // OPT_DISP_MIN

#ifdef INDICATORS
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/Id." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(I_des, filename);
#endif // INDICATORS

    // VS objects initialization, considering the pose control of a spherical camera from the feature set of mixture of photometric potentials use to build the control law based on the SSD cost minimization between the desired and the current image

    // initialisation de l'AV
    bool truncGauss = 1;
    bool poseJacobianCompute = true;
    double lambda = 0.8;
    prPoseSphericalEstim<prFeaturesSet<prCartesian3DPointVec, prPhotometricGMS<prCartesian3DPointVec>, prRegularlySampledCSImage>, prSSDCmp<prCartesian3DPointVec, prPhotometricGMS<prCartesian3DPointVec>>> visual_servo, visual_servo_g2;
    prRegularlySampledCSImage<unsigned char> IS_des(subdivLevel);
    prRegularlySampledCSImage<unsigned char> IS_cur(subdivLevel);
    prRegularlySampledCSImage<float> GS(subdivLevel);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample_des(lambda_g_1, truncGauss == 1);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample_cur(lambda_g_1, truncGauss == 1);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample(lambda_g_1, truncGauss == 1);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample_des_g2(lambda_g_2, truncGauss == 1);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample_cur_g2(lambda_g_2, truncGauss == 1);
    prPhotometricGMS<prCartesian3DPointVec> GS_sample_g2(lambda_g_2, truncGauss == 1);
    prFeaturesSet<prCartesian3DPointVec, prPhotometricGMS<prCartesian3DPointVec>, prRegularlySampledCSImage> fSet_des, fSet_des_g2, fSet_cur, fSet_cur_g2;

    bool dofs[6] = {true, true, true, true, true, true};

    visual_servo.setdof(dofs[0], dofs[1], dofs[2], dofs[3], dofs[4], dofs[5]);
    visual_servo_g2.setdof(dofs[0], dofs[1], dofs[2], dofs[3], dofs[4], dofs[5]);

    // prepare the desired image
    IS_des.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    IS_des.buildFromEquiRect(I_des_small, ecam, &Mask_small);
    IS_des.toAbsZN();

    double t0;
    t0 = vpTime::measureTimeMs();
    fSet_des.buildFrom(IS_des, GS, GS_sample_des, poseJacobianCompute);
    fSet_des_g2.buildFrom(IS_des, GS, GS_sample_des_g2, poseJacobianCompute);
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_des built in: " << -t0 << " ms" << std::endl;

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<unsigned char> MPP_des_u(I_des_small.getHeight(), I_des_small.getWidth());
    vpPoseVector pp(0, 0, 0, 0, 0, 0);
    IS_des.toEquiRect(MPP_des_u, pp, ecam, &Mask_small);
#endif

#ifdef INDICATORS
    // to save iterations
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/MPPd." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(MPP_des_u, filename);
#endif // INDICATORS

    visual_servo.buildFrom(fSet_des);
    visual_servo.initControl(lambda, true); // gain, cst_Z

    visual_servo_g2.buildFrom(fSet_des_g2);
    visual_servo_g2.initControl(lambda, true); // gain, cst_Z

#ifdef WITHROBOT
    std::cout << "Moving to initial position" << std::endl;
    vpColVector p_init({0.004807091816, -1.470364629, 0.01191167801, -2.667201404, 0.01274757143, 1.196476878, 0.01050977993});

    robot.setPosition(vpRobot::JOINT_STATE, p_init);

    vpTime::wait(1000);
#endif

#ifdef WITHCAMERA
    // Acquisition
    vpImage<unsigned char> I_cur_cropped;
    vpImage<unsigned char> I_cur_small(I_des_small.getHeight(), I_des_small.getWidth());
    cv::Mat I_cur_cv;
    cap.read(I_cur_cv);
    if (I_cur_cv.empty())
    {
        std::cout << "ERROR! blank frame grabbed\n";
        return -1;
    }
    vpImageConvert::convert(I_cur_cv, I_cur);
    vpImageTools::crop(I_cur, roi_y, roi_x, roi_height, roi_width, I_cur_cropped);
    vpImageTools::resize(I_cur_cropped, I_cur_small, vpImageTools::INTERPOLATION_AREA);
#endif

    // Successive computation of the current features set for every acquired image that are used to control the camera toward the desired image
    bool updateSampler = true;
    // activate the M-Estimator
    bool robust = false;

    // Build current features set
#ifdef OPT_DISP_MIN
    vpDisplayX disp_I_cur;
    vpImage<unsigned char> I_cur_for_disp(I_des_for_disp.getHeight(), I_des_for_disp.getWidth());
    vpImageTools::resize(I_cur_cropped, I_cur_for_disp, vpImageTools::INTERPOLATION_AREA);
    disp_I_cur.init(I_cur_for_disp, 25, 25, "I_cur");

    vpDisplay::display(I_cur);
    vpDisplay::flush(I_cur);
#endif

    // Current features set setting from the current image
    IS_cur.setInterpType(prInterpType::IMAGEPLANE_BILINEAR);
    IS_cur.buildFromEquiRect(I_cur_small, ecam, &Mask_small);
    IS_cur.toAbsZN();

    t0 = vpTime::measureTimeMs();
    fSet_cur.buildFrom(IS_cur, GS, GS_sample_cur, poseJacobianCompute);
    fSet_cur_g2.buildFrom(IS_cur, GS, GS_sample_cur_g2, poseJacobianCompute);
    t0 -= vpTime::measureTimeMs();
    std::cout << "fSet_cur built in: " << -t0 << " ms" << std::endl;

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    vpImage<unsigned char> MPP_cur_u(I_cur_small.getHeight(), I_cur_small.getWidth());
    IS_des.toEquiRect(MPP_cur_u, pp, ecam, &Mask_small);
#endif

#ifdef OPT_DISP_MAX
    vpDisplayX disp_MPP_cur;
    disp_MPP_cur.init(MPP_cur_u, 25, 25, "MPP_cur");
    vpDisplay::display(MPP_cur_u);
    vpDisplay::flush(MPP_cur_u);
#endif

#ifdef INDICATORS
    // to save iterations
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/MPP." << FILE_EXT;
    filename = s.str();
    vpImageIo::write(MPP_cur_u, filename);
#endif

#if defined(INDICATORS) || defined(OPT_DISP_MIN)
    // Compute differences of images for illustration purpose
    vpImage<unsigned char> Idiff(I_cur_for_disp.getHeight(), I_cur_for_disp.getWidth());
    vpImageTools::imageDifference(I_cur_for_disp, I_des_for_disp, Idiff);
#endif

#if defined(INDICATORS) || defined(OPT_DISP_MAX)
    // Compute differences of MPP for illustration purpose
    vpImage<unsigned char> MPPdiff(MPP_cur_u.getHeight(), MPP_cur_u.getWidth());
    vpImageTools::imageDifference(MPP_cur_u, MPP_des_u, MPPdiff);
#endif

#ifdef OPT_DISP_MIN
    // Affiche de l'image de difference
    vpDisplayX disp_I_diff;
    disp_I_diff.init(Idiff, 25, 25, "I_cur-I_des (for information)");

    vpDisplay::display(Idiff);
    vpDisplay::flush(Idiff);
#endif
#ifdef OPT_DISP_MAX
    // Affiche la difference de MPP
    vpDisplayX disp_MPP_diff;
    disp_MPP_diff.init(MPPdiff, 100 + I_des.getWidth() + 5, 100 + MPP_des_u.getHeight() + 30 + MPP_cur_u.getHeight() + 30, "MPP_cur-MPP_des (minimized)");

    vpDisplay::display(MPPdiff);
    vpDisplay::flush(MPPdiff);
#endif // OPT_DISP_MAX

    // Control loop
    //  ----------------------------------------------------------
    unsigned int nbDOF = 6, numDOF, indDOF;
    int servoNum = 0;
    int iter = 1;
    int iterStable = 0;
    vpColVector v6(6), v;
    double residual, residualPrev;
    double seuil = 1e-3;
    residual = 1e20;
    residualPrev = 2 * residual;

#ifdef INDICATORS
    /*  vpPoseVector p;
      std::vector<vpPoseVector> v_p;*/
    vpColVector p;
    std::vector<vpColVector> v_p;
    std::vector<double> v_residuals;
    std::vector<vpImage<unsigned char>> v_I_cur, v_MPP_cur;
    std::vector<vpImage<unsigned char>> v_Idiff, v_MPPdiff;
    std::vector<double> v_tms;
#endif // INDICATORS

#ifdef WITHROBOT
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
#endif
    double tms, duree;
    do
    {
        tms = vpTime::measureTimeMs();
        std::cout << "--------------------------------------------" << iter++ << std::endl;

        if (fabs(residual - residualPrev) < seuil)
        {
            iterStable++;
            std::cout << iterStable << " stab\t";
        }
        else
        {
            iterStable = 0;
        }
        if (iterStable >= 5 && servoNum == 0)
        {
            servoNum = 1;
            iterStable = 0;
            seuil = 5e-2;
            residual = 1e20;
            residualPrev = 2 * residual;
        }
        if (iterStable >= 5 && servoNum == 1)
        {
            break;
        }
        residualPrev = residual;

#ifdef INDICATORS
#ifdef WITHROBOT
        robot.getPosition(vpRobot::JOINT_STATE, p);
#endif // WITHROBOT
#endif // INDICATORS

#ifdef WITHCAMERA
        cap.read(I_cur_cv);
        if (I_cur_cv.empty())
        {
            std::cout << "ERROR! blank frame grabbed\n";
            return -1;
        }
        vpImageConvert::convert(I_cur_cv, I_cur);
        vpImageTools::crop(I_cur, roi_y, roi_x, roi_height, roi_width, I_cur_cropped);
        vpImageTools::resize(I_cur_cropped, I_cur_small, vpImageTools::INTERPOLATION_AREA);
#endif

        IS_cur.buildFromEquiRect(I_cur_small, ecam, &Mask_small);
        IS_cur.toAbsZN();

        switch (servoNum)
        {
        case 0:
            fSet_cur.buildFrom(IS_cur, GS, GS_sample_cur, poseJacobianCompute);
            residual = 0.5 * visual_servo.control(fSet_cur, v, robust);
            break;
        case 1:
            fSet_cur_g2.buildFrom(IS_cur, GS, GS_sample_cur_g2, poseJacobianCompute);
            residual = 0.5 * visual_servo_g2.control(fSet_cur_g2, v, robust);
            break;

        default:
            std::cout << "please provide a servo number" << std::endl;
            break;
        }

        std::cout << "error : " << residual << std::endl;

        // update the DOFs
        indDOF = 0;
        for (numDOF = 0; numDOF < nbDOF; numDOF++)
            if (dofs[numDOF])
            {
                v6[numDOF] = -v[indDOF];
                indDOF++;
            }
            else
                v6[numDOF] = 0;

        std::cout << "v6 : " << v6.t() << std::endl;

#ifdef WITHROBOT
        robot.setVelocity(vpRobot::CAMERA_FRAME, v6);
#endif

#if defined(OPT_DISP_MIN) || defined(INDICATORS)
        vpImageTools::resize(I_cur_cropped, I_cur_for_disp, vpImageTools::INTERPOLATION_AREA);
        vpImageTools::imageDifference(I_cur_for_disp, I_des_for_disp, Idiff);
#endif

#if defined(OPT_DISP_MAX) || defined(INDICATORS)
        IS_cur.toEquiRect(MPP_cur_u, pp, ecam, &Mask_small);
        vpImageTools::imageDifference(MPP_cur_u, MPP_des_u, MPPdiff);
#endif

#ifdef OPT_DISP_MIN
        vpDisplay::display(I_cur_for_disp);
        vpDisplay::flush(I_cur_for_disp);

        vpDisplay::display(Idiff);
        vpDisplay::flush(Idiff);

#endif
#ifdef OPT_DISP_MAX
        vpDisplay::display(MPP_cur_u);
        vpDisplay::flush(MPP_cur_u);

        vpDisplay::display(MPPdiff);
        vpDisplay::flush(MPPdiff);
#endif
        duree = vpTime::measureTimeMs() - tms;
        std::cout << "duration : " << duree << std::endl;

#ifdef INDICATORS
        v_p.push_back(p);
        v_residuals.push_back(residual);
        v_I_cur.push_back(I_cur);
        v_MPP_cur.push_back(MPP_cur_u);
        v_Idiff.push_back(Idiff);
        v_MPPdiff.push_back(MPPdiff);
        v_tms.push_back(duree);
#endif // INDICATORS

    } while (!vpDisplay::getClick(I_cur, false));

#ifdef WITHROBOT
    robot.setRobotState(vpRobot::STATE_STOP);
#endif

#ifdef WITHCAMERA
    cap.release();
#endif

    std::cout << "Visual Servoing task finished, saving accumulated data" << std::endl;

#ifdef INDICATORS
    // save pose list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/poses.txt";
    filename = s.str();
    std::ofstream ficPoses(filename.c_str());
    // save residual list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/residuals.txt";
    filename = s.str();
    std::ofstream ficResiduals(filename.c_str());
    // save the processing times
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/times.txt";
    filename = s.str();
    std::ofstream ficTimes(filename.c_str());

    for (unsigned int i = 0; i < v_p.size(); i++)
    {
        ficPoses << v_p[i].t() << std::endl;
        ficResiduals << std::fixed << std::setw(11) << std::setprecision(6) << v_residuals[i] << std::endl;
        ficTimes << v_tms[i] << std::endl;

        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << "resultat/I/I." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
        filename = s.str();
        vpImageIo::write(v_I_cur[i], filename);

        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << "resultat/MPP/MPP." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
        filename = s.str();
        vpImageIo::write(v_MPP_cur[i], filename);

        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << "resultat/IErr/IErr." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
        filename = s.str();
        vpImageIo::write(v_Idiff[i], filename);

        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << "resultat/MPPErr/MPPErr." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
        filename = s.str();
        vpImageIo::write(v_MPPdiff[i], filename);
    }

    ficPoses.close();
    ficResiduals.close();
    ficTimes.close();
#endif // INDICATORS

    return 0;
}

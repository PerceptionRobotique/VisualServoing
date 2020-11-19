#ifndef __C_UR_H__
#define  __C_UR_H__

#include <visp/vpClient.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>
#include <visp/vpImage.h>
#include <visp/vpDot.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>

#include <string>

#include <rtde_receive_interface.h>

using namespace ur_rtde;

class C_UR
{
public:
	C_UR(std::string hostName, unsigned int portNum, double _delta_t = 0.004, double _L = 0.087);
    ~C_UR();

	void setParametresIntrinseques(float au, float av, float u0, float v0, float k1 = 0.0f, float ik1 = 0.0f);
	void setParametresExtrinseques(vpPoseVector &pv);

	int setCameraVelocity(vpColVector &v);
  int setCameraArticularPose(vpColVector &j);
  int setCameraRelativePose(vpColVector &p);
  int getCameraPoseRaw(vpColVector &p);
  int getCameraPose(vpPoseVector &p);
    
	int selectionPrimitivesDesirees(vpImage<unsigned char> &I, bool opt_display = true);
	int selectionPrimitivesInitiales(vpImage<unsigned char> &I, bool opt_display = true);
	int suiviDesPrimitivesEtCommande(vpImage<unsigned char> &I, bool opt_display = true);

  /*  int calculDePose(vpImage<vpRGBa> &Ic, bool opt_display = true);
    
    void afficheProfils();*/
    
private:
	vpClient clientUR;
  RTDEReceiveInterface rtde_control;
	vpColVector v;
  double delta_t;

	vpDot d[4];
	vpImagePoint iP[4];
	vpImagePoint cogd[4];
    vpCameraParameters cam;
    
	vpHomogeneousMatrix cMo;
	vpPoint P[4];
    double L;

	vpServo tache;

	vpFeaturePoint fp[4], fpd[4];

	int iter;
	double residu, residu_1;

   /* vpPlot plotter;    */
};

#endif // __C_UR_H__

#include "C_UR.h"

#include <iostream>

#include <visp/vpDisplayOpenCV.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpExponentialMap.h>

C_UR::C_UR(std::string hostName, unsigned int portNum, double _delta_t, double _L) : v(6), delta_t(_delta_t), rtde_control(hostName)
{
	std::cout << clientUR.connectToHostname(hostName, portNum) << std::endl;

//    setParametresIntrinseques(400, 400, 320, 256);
    //setParametresIntrinseques(748.7802858,751.0111906,332.5158055,253.3211204,-0.1950275526,0.1950275526);
    setParametresIntrinseques(2700,2700,640,512);

    L=_L;
    
    P[0].setWorldCoordinates(0, 0, 0);
    P[1].setWorldCoordinates(L, 0, 0);
    P[2].setWorldCoordinates(L, L, 0);
    P[3].setWorldCoordinates(0, L, 0);

	tache.setLambda(0.7);
//	tache.setLambda(2.0);
    //a.	Utilisez la méthode setServo
    tache.setServo(vpServo::EYEINHAND_CAMERA);
    tache.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
 }

C_UR::~C_UR()
{
	vpColVector v(6);
	v = 0;
    setCameraVelocity(v);
}

int C_UR::setCameraVelocity(vpColVector &cv)
{
	if(cv.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

    //v *= delta_t; // apply sampling time to UR

		//osChaine << "movej(p[" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "], a=1, v=0.5)\n";
    //osChaine << "movej(pose_trans(get_actual_tcp_pose(), p[" << v[0] << ", " << v[1] << ", " << v[2] << ," " << v[3] << ", " << v[4] << ", " << v[5] << "]), t=" << delta_t << ")\n";

    // OK !!!!!!
    std::vector<double> actual_tcp_pose = rtde_control.getActualTCPPose();

    //std::cout << actual_tcp_pose[0] << " " << actual_tcp_pose[1] << " " << actual_tcp_pose[2] << " " << actual_tcp_pose[3] << " " << actual_tcp_pose[4] << " " << actual_tcp_pose[5] << std::endl;

    vpRotationMatrix bRc(vpThetaUVector(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]));
    vpTranslationVector btc;
    vpHomogeneousMatrix bMc(btc, bRc);

//    std::cout << cMb << std::endl;

    vpVelocityTwistMatrix bVc(bMc);

    vpColVector bv = bVc * cv;

//    osChaine << "speedl([" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "], a=2.0, t=0.05)\n";//, t=" << delta_t << ")\n";
    osChaine << std::fixed << std::setprecision( 4 ) << "speedl([" << bv[0] << ", " << bv[1] << ", " << bv[2] << ", " << bv[3] << ", " << bv[4] << ", " << bv[5] << "], a=0.05, t=0.33)\n";//, t=" << delta_t << ")\n";

// OK, mais "stops"
//    osChaine << "movej(pose_trans(get_actual_tcp_pose(), p[" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "]), t=" << delta_t << ")\n"; //, r=0.0011

//    osChaine << "movej(p[" << p01[0] << ", " << p01[1] << ", " << p01[2] << ", " << p01[3] << ", " << p01[4] << ", " << p01[5] << "], t=" << delta_t << ")\n"; //, r=0.0011

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::setCameraArticularPose(vpColVector &j)
{
  if(j.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

		osChaine << "movej([" << j[0] << ", " << j[1] << ", " << j[2] << ", " << j[3] << ", " << j[4] << ", " << j[5] << "], a=1, v=0.5)\n";

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::setCameraRelativePose(vpColVector &p)
{
	if(p.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

    osChaine << "movel(pose_trans(get_actual_tcp_pose(), p[" << p[0] << ", " << p[1] << ", " << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "]), a=0.1, v=0.5)\n";

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::getCameraPoseRaw(vpColVector &p)
{
  p = rtde_control.getActualTCPPose();

	return 0;
}

int C_UR::getCameraPose(vpPoseVector &p)
{
  std::vector<double> actual_tcp_pose = rtde_control.getActualTCPPose();

  vpRotationMatrix bRc(vpThetaUVector(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]));   
  vpTranslationVector btc;
  vpHomogeneousMatrix bMc(btc, bRc);

  p.buildFrom(bMc);

	return 0;
}



void C_UR::setParametresIntrinseques(float au, float av, float u0, float v0, float k1, float ik1)
{
    if( (k1 == 0.0f) && (ik1 == 0.0f) )
        cam.initPersProjWithoutDistortion(au, av, u0, v0);
    else
        cam.initPersProjWithDistortion(au, av, u0, v0, k1, ik1);
}

void C_UR::setParametresExtrinseques(vpPoseVector &pv)
{
    cMo.buildFrom(pv);
}


int C_UR::selectionPrimitivesDesirees(vpImage<unsigned char> &I, bool opt_display)
{
	int i;
	//cible un peu éloignee de la camera robot
	iP[0].set_i(1024*0.75);
	iP[0].set_j(1024*0.25+(1280-1024)*0.5);
	iP[1].set_i(1024*0.75);
	iP[1].set_j(1024*0.75+(1280-1024)*0.5);
	iP[2].set_i(1024*0.25);
	iP[2].set_j(1024*0.75+(1280-1024)*0.5);
	iP[3].set_i(1024*0.25);
	iP[3].set_j(1024*0.25+(1280-1024)*0.5);

	//cible remplit le champ de vue (optique 8mm)
	/*iP[0].set_i(350);
	iP[0].set_j(200);
	iP[1].set_i(350);
	iP[1].set_j(400);
	iP[2].set_i(150);
	iP[2].set_j(400);
	iP[3].set_i(150);
	iP[3].set_j(200);*/


	//initialisation des primitives desirees
	for (i=0 ; i < 4 ; i++)
	{
		vpFeatureBuilder::create(fpd[i], cam, iP[i]);
		//fpd[i].set_Z(1.5); //???????????????????????????
	}
	return 0;
}

int C_UR::selectionPrimitivesInitiales(vpImage<unsigned char> &I, bool opt_display)
{
	//Affichages et selection des quatre points
    int i ;
    for (i=0 ; i < 4 ; i++)
    {
        if (opt_display) {
            d[i].setGraphics(true) ;
        }
        else {
            d[i].setGraphics(false) ;
        }
    }
    
    vpImagePoint cog[4];
    try{
        std::cout << "Cliquer sur les quatre points dans le sens direct"
        << std::endl  ;
        for (i=0 ; i < 4 ; i++)
        {
            d[i].initTracking(I) ;
            
            d[i].track(I, cog[i]) ;
            vpDisplay::flush(I) ;
        }
    }
    catch(...)
    {
        vpERROR_TRACE("Error in tracking initialization ") ;
        return(-1) ;
    }
    if (opt_display)
    {
        for (i=0 ; i < 4 ; i++)
		{
            vpDisplay::displayCross(I, cog[i], 10, vpColor::red) ;
		}
        vpDisplay::flush(I) ;
    }

    // Calcul de pose
    vpPose pose ;
    pose.clearPoint() ;
    for (i=0 ; i < 4 ; i++)
    {
        double x=0, y=0;
        vpPixelMeterConversion::convertPoint(cam, cog[i], x,y)  ;
        P[i].set_x(x) ;
        P[i].set_y(y) ;
        
        pose.addPoint(P[i]) ;
    }
    pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo) ;

	std::cout << "pose calculee : " << vpPoseVector(cMo).t() << std::endl;

    //affichage du repere dans l'image
    if( opt_display )
    {
        pose.display(I,cMo,cam, 0.15) ;
        vpDisplay::flush(I) ;
		vpDisplay::getClick(I) ;
    }

	//initialisation des primitives courantes
	for (i=0 ; i < 4 ; i++)
	{
		vpFeatureBuilder::create(fp[i], cam, d[i]);
		P[i].changeFrame(cMo);
		fp[i].set_Z(P[i].get_Z());
		fpd[i].set_Z(P[i].get_Z());
		tache.addFeature(fp[i], fpd[i]);
	}
	iter = 0;
	residu = 1e10;
	residu_1 = residu - 1;
}

int C_UR::suiviDesPrimitivesEtCommande(vpImage<unsigned char> &I, bool opt_display)
{    
	int i;
    vpImagePoint cog[4];
    try{
        for (i=0 ; i < 4 ; i++)
        {
            d[i].track(I, cog[i]) ;
            vpDisplay::flush(I) ;
        }
    }
    catch(...)
    {
        vpERROR_TRACE("Error in tracking initialization ") ;
        return(-1) ;
    }
    if (opt_display)
    {
        for (i=0 ; i < 4 ; i++)
		{
            vpDisplay::displayCross(I, cog[i], 10, vpColor::red) ;
			vpDisplay::displayLine(I, cog[i], iP[i], vpColor::green, 2);
			vpDisplay::displayCross(I, iP[i], 10, vpColor::green) ;
		}
        vpDisplay::flush(I) ;
    }

	//mise à jour des primitives courantes
	for (i=0 ; i < 4 ; i++)
	{
		vpFeatureBuilder::create(fp[i], cam, d[i]);
	}

	v = tache.computeControlLaw();
    
	setCameraVelocity(v);

	residu_1 = residu;
    residu = sqrt(tache.computeError().sumSquare())*0.25; // / 4;
        
    std::cout << "iter : " << iter << " | residu : " << residu << std::endl;

	if( (++iter > 150) || (fabs(residu-residu_1) < 1e-4) )
		return 1;

	return 0;
}

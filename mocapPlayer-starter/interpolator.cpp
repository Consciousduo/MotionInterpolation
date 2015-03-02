#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}


//tested!!!
void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}
//tested!!!
void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
  // angles[0] =>¦× angles[1]=>¦È angles[2]=>¦Õ
  /*
  R = Rz(¦Õ)Ry(¦È)Rx(¦×)
  */
	double anglesRadius[3];
	for(int i=0; i<3; i++)
    anglesRadius[i] = (angles[i]/180)*M_PI;

  R[0] = cos(anglesRadius[1])*cos(anglesRadius[2]);
  R[1] = sin(anglesRadius[0])*sin(anglesRadius[1])*cos(anglesRadius[2])-cos(anglesRadius[0])*sin(anglesRadius[2]);
  R[2] = cos(anglesRadius[0])*sin(anglesRadius[1])*cos(anglesRadius[2])-sin(anglesRadius[0])*sin(anglesRadius[2]);

  R[3] = cos(anglesRadius[1])*sin(anglesRadius[2]);
  R[4] = sin(anglesRadius[0])*sin(anglesRadius[1])*sin(anglesRadius[2])-cos(anglesRadius[0])*cos(anglesRadius[2]);
  R[5] = cos(anglesRadius[0])*sin(anglesRadius[1])*sin(anglesRadius[2])-sin(anglesRadius[0])*cos(anglesRadius[2]);

  R[6] = -sin(anglesRadius[1]);
  R[7] = sin(anglesRadius[0])*cos(anglesRadius[1]);
  R[8] = cos(anglesRadius[0])*cos(anglesRadius[1]);
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  printf("!!!!LinearInterpolationEuler!!!!\n");
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this

 int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  printf("testing functions\n");
  //Quaternion<double> testQ.Set();
  double testAngle[3];
  double resultAngle[3];
  testAngle[0] = 0;
  testAngle[1] = 90;
  testAngle[2] = 0;
  double R[9];
  Euler2Rotation(testAngle, R);
  Rotation2Euler(R, resultAngle);

  printf("test angle: %f %f %f\n", testAngle[0], testAngle[1], testAngle[2]);
  printf("result angle: %f %f %f\n", resultAngle[0], resultAngle[1], resultAngle[2]);

  
  Quaternion<double> testQ;
  Euler2Quaternion(testAngle, testQ);
  testQ.Print();
  Quaternion2Euler(testQ, resultAngle);
  printf("result angle: %f %f %f\n", resultAngle[0], resultAngle[1], resultAngle[2]);



  // students should implement this
  printf("!!!!LinearInterpolationQuaternion!!!!\n");
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
		
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
       
		double eularAngleStart[3], eularAngleEnd[3];
		eularAngleStart[0] = startPosture->bone_rotation[bone].p[0];
		eularAngleStart[1] = startPosture->bone_rotation[bone].p[1];
		eularAngleStart[2] = startPosture->bone_rotation[bone].p[2];

		eularAngleEnd[0] = endPosture->bone_rotation[bone].p[0];
		eularAngleEnd[1] = endPosture->bone_rotation[bone].p[1];
		eularAngleEnd[2] = endPosture->bone_rotation[bone].p[2];

		Quaternion<double> qstart, qend;
		Euler2Quaternion(eularAngleStart, qstart);
		Euler2Quaternion(eularAngleStart, qend);
		Quaternion<double> interpolatedQuaternion = Slerp(t, qstart, qend);
		double interpolatedEular[3];
		Quaternion2Euler(interpolatedQuaternion, interpolatedEular);
		
	    interpolatedPosture.bone_rotation[bone].setValue(interpolatedEular[0], interpolatedEular[1], interpolatedEular[2]);
	  //interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
	  }
	    pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		

      }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double R[9];
	Euler2Rotation(angles, R);
	q = q.Matrix2Quaternion(R);
	//q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	//q.Normalize();
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
	
	double cos¦È =  qStart.Gets()*qEnd_.Gets()+
				   qStart.Getx()*qEnd_.Getx()+
				   qStart.Gety()*qEnd_.Gety()+
				   qStart.Getz()*qEnd_.Getz();
	//printf("cos¦È = %f   ", cos¦È);

	double ¦È = acos(cos¦È);
	Quaternion<double> result;
	result = qStart*(sin((1-t)*¦È)/sin(¦È))+qEnd_*(sin(t*¦È)/sin(¦È));
	result.Normalize();
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  result = Slerp(2.0f, p, q);
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}


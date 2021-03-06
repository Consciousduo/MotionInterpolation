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

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
  // angles[0] =>�� angles[1]=>�� angles[2]=>��
  /*
  R = Rz(��)Ry(��)Rx(��)
  */
	double anglesRadius[3];
	for(int i=0; i<3; i++)
    anglesRadius[i] = (angles[i]/180)*M_PI;

  R[0] = cos(anglesRadius[1])*cos(anglesRadius[2]);
  R[1] = sin(anglesRadius[0])*sin(anglesRadius[1])*cos(anglesRadius[2])-cos(anglesRadius[0])*sin(anglesRadius[2]);
  R[2] = cos(anglesRadius[0])*sin(anglesRadius[1])*cos(anglesRadius[2])+sin(anglesRadius[0])*sin(anglesRadius[2]);

  R[3] = cos(anglesRadius[1])*sin(anglesRadius[2]);
  R[4] = sin(anglesRadius[0])*sin(anglesRadius[1])*sin(anglesRadius[2])+cos(anglesRadius[0])*cos(anglesRadius[2]);
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
	printf("Interpolation type: Bezier Interpolation Eular\n");
	 int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
  int startKeyframe = 0;
  int previousKeyframe = -1, nextKeyframe = -1;

  while (startKeyframe + N + 1 < inputLength)
  {//while start
    int endKeyframe = startKeyframe + N + 1;
	nextKeyframe = endKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
	Posture * previousPosture;
	Posture * nextPosture;
	
	//get previous posture
	if(previousKeyframe == -1){
		previousPosture = NULL;
	}else{
		previousPosture = pInputMotion->GetPosture(previousKeyframe);
	}

	//get next posture
	if(nextKeyframe >= inputLength){
		nextPosture = NULL;
	}else{
		nextPosture = pInputMotion->GetPosture(nextKeyframe);
	}

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
		
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // bezier interpolate root position/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
		
		vector rootPosition_start, rootPosition_end, rootPosition_previous, rootPosition_next;
		if(previousKeyframe != -1){
			rootPosition_previous = previousPosture->root_pos;
		}
		if(nextKeyframe<inputLength){
			rootPosition_next = nextPosture->root_pos;
		}
			rootPosition_start = startPosture->root_pos;
			rootPosition_end = endPosture->root_pos;
		
		vector A_startRoot, A_endRoot, AstartRoot, AendRoot, BstartRoot, BendRoot;
	  
		if(previousKeyframe != -1){
		A_startRoot = (rootPosition_previous*(-1.0)+rootPosition_start*(2.0))*(0.5)+(rootPosition_end)*0.5;
		AstartRoot = rootPosition_start*(1-1.0/3)+A_startRoot*(1.0/3);
		BstartRoot = rootPosition_start*(1+1.0/3)+A_startRoot*(-1.0/3);
		}else{
		AstartRoot = rootPosition_start*(1-1.0/3)+(rootPosition_next*(-1.0)+rootPosition_end*(2.0))*(1.0/3);
		}

		if(nextKeyframe<inputLength){
		A_endRoot = (rootPosition_start*(-1.0)+rootPosition_end*(2.0))*(0.5)+(rootPosition_next)*0.5;
		AendRoot = rootPosition_end*(1-1.0/3)+A_endRoot*(1.0/3);
		BendRoot = rootPosition_end*(1+1.0/3)+A_endRoot*(-1.0/3);	
		}else{
		BendRoot = rootPosition_end*(1-1.0/3)+(rootPosition_previous*(-1.0)+rootPosition_start*(2.0))*(1.0/3);
		}
	  
		interpolatedPosture.root_pos = DeCasteljauEuler(t, rootPosition_start, AstartRoot, BendRoot, rootPosition_end);
	  
	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
       
		vector eularAngleStart;
		vector eularAngleEnd;
		vector eularAnglePrevious;
		vector eularAngleNext;
		if(previousKeyframe != -1){
			eularAnglePrevious = previousPosture->bone_rotation[bone];
		}
		if(nextKeyframe<inputLength){
			eularAngleNext = nextPosture->bone_rotation[bone];
		}

		eularAngleStart = startPosture->bone_rotation[bone];
		eularAngleEnd = endPosture->bone_rotation[bone];
		
		
		vector A_start, A_end, Astart, Aend, Bstart, Bend;
		//compute control point P2 P3
		if(previousKeyframe != -1){
		A_start = (eularAnglePrevious*(-1.0)+eularAngleStart*(2.0))*(0.5)+(eularAngleEnd)*0.5;
		Astart = eularAngleStart*(1-1.0/3)+A_start*(1.0/3);
		Bstart = eularAngleStart*(1+1.0/3)+A_start*(-1.0/3);
		}else{
		Astart = eularAngleStart*(1-1.0/3)+(eularAngleNext*(-1.0)+eularAngleEnd*(2.0))*(1.0/3);
		}

		if(nextKeyframe<inputLength){
		A_end = (eularAngleStart*(-1.0)+eularAngleEnd*(2.0))*(0.5)+(eularAngleNext)*0.5;
		Aend = eularAngleEnd*(1-1.0/3)+A_end*(1.0/3);
		Bend = eularAngleEnd*(1+1.0/3)+A_end*(-1.0/3);	
		}else{
		Bend = eularAngleEnd*(1-1.0/3)+(eularAnglePrevious*(-1.0)+eularAngleStart*(2.0))*(1.0/3);
		}

	    interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, eularAngleStart, Astart, Bend, eularAngleEnd);
	  }
	    pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);	
      }

	previousKeyframe = startKeyframe;
    startKeyframe = endKeyframe;
  }//while end

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

 
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	//for testing
	/*
	double testEular[3];
	double resultEular[3];
	testEular[0] = 0; testEular[1] = 80; testEular[2] = 0; 
	printf("testEular: %f %f %f\n", testEular[0], testEular[1], testEular[2]);

	double R[9];
	Euler2Rotation(testEular, R);
	Rotation2Euler(R, resultEular);
	printf("resultEular: %f %f %f\n", resultEular[0], resultEular[1], resultEular[2]);

	Quaternion<double> resultQ, qqq;
	Euler2Quaternion(testEular, resultQ);
	printf("Euler2Quaternion function: ");
	resultQ.Print();
	
	Euler2Rotation(testEular, R);
	printf("R : \n %f %f %f\n %f %f %f\n %f %f %f\n",R[0],R[1],R[2],R[3],R[4],R[5],R[6],R[7],R[8]);
	qqq = resultQ.Matrix2Quaternion(R);
	qqq.Print();


	double a11=10000;
	double b11[3];
	resultQ.GetRotation(&a11, b11);
	printf("angle : %f\n", (a11/M_PI)*180);
	*/

  // students should implement this
  printf("Interpolation type: Linear Interpolation Quaternion\n");
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
       
		double eularAngleStart[3];
		double eularAngleEnd[3];
		eularAngleStart[0] = startPosture->bone_rotation[bone].p[0];
		eularAngleStart[1] = startPosture->bone_rotation[bone].p[1];
		eularAngleStart[2] = startPosture->bone_rotation[bone].p[2];

		eularAngleEnd[0] = endPosture->bone_rotation[bone].p[0];
		eularAngleEnd[1] = endPosture->bone_rotation[bone].p[1];
		eularAngleEnd[2] = endPosture->bone_rotation[bone].p[2];

		Quaternion<double> qstart, qend;
		Euler2Quaternion(eularAngleStart, qstart);
		Euler2Quaternion(eularAngleEnd, qend);

		Quaternion<double> interpolatedQuaternion = Slerp(t, qstart, qend);
		double interpolatedEular[3];
		Quaternion2Euler(interpolatedQuaternion, interpolatedEular);
		
	    interpolatedPosture.bone_rotation[bone].setValue(interpolatedEular[0], interpolatedEular[1], interpolatedEular[2]);
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
  printf("Interpolation type: Bezier Interpolation Quaternion\n");

  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
  int startKeyframe = 0;
  int previousKeyframe = -1, nextKeyframe = -1;

  while (startKeyframe + N + 1 < inputLength)
  {//while start
    int endKeyframe = startKeyframe + N + 1;
	nextKeyframe = endKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
	Posture * previousPosture;
	Posture * nextPosture;
	
	//get previous posture
	if(previousKeyframe == -1){
		previousPosture = NULL;
	}else{
		previousPosture = pInputMotion->GetPosture(previousKeyframe);
	}

	//get next posture
	if(nextKeyframe >= inputLength){
		nextPosture = NULL;
	}else{
		nextPosture = pInputMotion->GetPosture(nextKeyframe);
	}

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
		
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // bezier interpolate root position
      	vector rootPosition_start, rootPosition_end, rootPosition_previous, rootPosition_next;
		if(previousKeyframe != -1){
			rootPosition_previous = previousPosture->root_pos;
		}
		if(nextKeyframe<inputLength){
			rootPosition_next = nextPosture->root_pos;
		}
			rootPosition_start = startPosture->root_pos;
			rootPosition_end = endPosture->root_pos;
		
		vector A_startRoot, A_endRoot, AstartRoot, AendRoot, BstartRoot, BendRoot;
	  
		if(previousKeyframe != -1){
		A_startRoot = (rootPosition_previous*(-1.0)+rootPosition_start*(2.0))*(0.5)+(rootPosition_end)*0.5;
		AstartRoot = rootPosition_start*(1-1.0/3)+A_startRoot*(1.0/3);
		BstartRoot = rootPosition_start*(1+1.0/3)+A_startRoot*(-1.0/3);
		}else{
		AstartRoot = rootPosition_start*(1-1.0/3)+(rootPosition_next*(-1.0)+rootPosition_end*(2.0))*(1.0/3);
		}

		if(nextKeyframe<inputLength){
		A_endRoot = (rootPosition_start*(-1.0)+rootPosition_end*(2.0))*(0.5)+(rootPosition_next)*0.5;
		AendRoot = rootPosition_end*(1-1.0/3)+A_endRoot*(1.0/3);
		BendRoot = rootPosition_end*(1+1.0/3)+A_endRoot*(-1.0/3);	
		}else{
		BendRoot = rootPosition_end*(1-1.0/3)+(rootPosition_previous*(-1.0)+rootPosition_start*(2.0))*(1.0/3);
		}
	  
		interpolatedPosture.root_pos = DeCasteljauEuler(t, rootPosition_start, AstartRoot, BendRoot, rootPosition_end);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
       
		double eularAngleStart[3];
		double eularAngleEnd[3];
		double eularAnglePrevious[3];
		double eularAngleNext[3];
		if(previousKeyframe != -1){
			eularAnglePrevious[0] = previousPosture->bone_rotation[bone].p[0];
			eularAnglePrevious[1] = previousPosture->bone_rotation[bone].p[1];
			eularAnglePrevious[2] = previousPosture->bone_rotation[bone].p[2];
		}
		if(nextKeyframe<inputLength){
			eularAngleNext[0] = nextPosture->bone_rotation[bone].p[0];
			eularAngleNext[1] = nextPosture->bone_rotation[bone].p[1];
			eularAngleNext[2] = nextPosture->bone_rotation[bone].p[2];
		}


		eularAngleStart[0] = startPosture->bone_rotation[bone].p[0];
		eularAngleStart[1] = startPosture->bone_rotation[bone].p[1];
		eularAngleStart[2] = startPosture->bone_rotation[bone].p[2];

		eularAngleEnd[0] = endPosture->bone_rotation[bone].p[0];
		eularAngleEnd[1] = endPosture->bone_rotation[bone].p[1];
		eularAngleEnd[2] = endPosture->bone_rotation[bone].p[2];

		Quaternion<double> qstart, qend, qprevious, qnext;
		Euler2Quaternion(eularAngleStart, qstart);
		Euler2Quaternion(eularAngleEnd, qend);
		if(previousKeyframe != -1){
		Euler2Quaternion(eularAnglePrevious, qprevious);
		}
		if(nextKeyframe<inputLength){
		Euler2Quaternion(eularAngleNext, qnext);
		}

		//now compute Astart Aend Bstart Bend using (q (q q) q)
		// qprevious qstart qend to compute Astart Bstart
		// qstart qend qnext to compute Aend Bend
		Quaternion<double> Astart, Bstart, A_start, Aend, Bend, A_end;
		
		if(previousKeyframe != -1){
		A_start = Slerp(0.5, Double(qprevious, qstart), qend);
		Astart = Slerp( 1.0/3, qstart, A_start);
		Bstart = Slerp(-1.0/3, qstart, A_start);
		}else{
		Astart = Slerp( 1.0/3, qstart, Double(qnext,qend));
		}

		if(nextKeyframe<inputLength){
		A_end = Slerp(0.5, Double(qstart, qend), qnext);
		Aend = Slerp( 1.0/3, qend, A_end);
		Bend = Slerp(-1.0/3, qend, A_end);
		}else{
		Bend = Slerp( 1.0/3, qend, Double(qprevious,qstart));
		}
		
		//now we can do interpolation
		Quaternion<double> interpolatedQuaternion = DeCasteljauQuaternion(t, qstart, Astart, Bend, qend);

		double interpolatedEular[3];
		Quaternion2Euler(interpolatedQuaternion, interpolatedEular);
	    interpolatedPosture.bone_rotation[bone].setValue(interpolatedEular[0], interpolatedEular[1], interpolatedEular[2]);
	  }
	    pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);	
      }

	previousKeyframe = startKeyframe;
    startKeyframe = endKeyframe;
  }//while end

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double R[9];
	Euler2Rotation(angles, R);
	q = q.Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	q.Normalize();
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
	
	double cos��1 = qStart.Gets()*qEnd_.Gets()+
				   qStart.Getx()*qEnd_.Getx()+
				   qStart.Gety()*qEnd_.Gety()+
				   qStart.Getz()*qEnd_.Getz();
	double ��1 = acos(cos��1);

	Quaternion<double> temp = -1*qEnd_;
	double cos��2 =  qStart.Gets()*temp.Gets()+
					qStart.Getx()*temp.Getx()+
					qStart.Gety()*temp.Gety()+
					qStart.Getz()*temp.Getz();
	double ��2 = acos(cos��2);

	double ��;
	if(fabs(��1)<=fabs(��2)){
		�� = ��1;
	}else{
		�� = ��2;
		qEnd_ = temp;
	}
	double sin�� = sin(��);
	if(sin�� != 0){
	Quaternion<double> result;
	double co1, co2;
	co1 = sin((1-t)*��)/sin(��);
	co2 = sin(t*��)/sin(��);
	result = co1*qStart+co2*qEnd_;
	result.Normalize();
	return result;
	}
	return qEnd_;	
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
  vector Q0, Q1, Q2, R0, R1;
  vector result;
  Q0 = p0*(1-t)+p1*t;
  Q1 = p1*(1-t)+p2*t;
  Q2 = p2*(1-t)+p3*t;

  R0 = Q0*(1-t)+Q1*t;
  R1 = Q1*(1-t)+Q2*t;

  result = R0*(1-t)+R1*t;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> Q0, Q1, Q2, R0, R1;
  Quaternion<double> result;
  Q0 = Slerp(t, p0, p1);
  Q1 = Slerp(t, p1, p2);
  Q2 = Slerp(t, p2, p3);

  R0 = Slerp(t, Q0, Q1);
  R1 = Slerp(t, Q1, Q2);

  result = Slerp(t, R0, R1);
  return result;
}


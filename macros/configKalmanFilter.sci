// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function kalmanFilter=configKalmanFilter(MotionModel,InitialLocation,InitialError,MotionNoise, MeasurementNoise)
//Create Kalman filter for object tracking
//
//Calling Sequence
//kalmanFilter=configKalmanFilter(MotionModel,InitialLocation,InitialError,MotionNoise, MeasurementNoise)
//
//Parameters
//kalmanFilter:A structure with fields:StateTransitionModel,MeasurementModel,ControlModel,State,StateCovariance, ProcessNoise,MeasurementNoise
//MotionModel:Motion model, specified as a string 'ConstantVelocity' or 'ConstantAcceleration'. The motion model you select applies to all dimensions. For example, for the 2-D Cartesian coordinate system. This mode applies to both X and Y directions.
//InitialLocation:Initial location of object, specified as a numeric vector. This argument also determines the number of dimensions for the coordinate system. For example, if you specify the initial location as a two-element vector, [x0, y0], then a 2-D coordinate system is assumed.
//InitialError:Initial estimate uncertainty variance, specified as a two- or three-element vector. The initial estimate error specifies the variance of the initial estimates of location, velocity, and acceleration of the tracked object. The function assumes a zero initial velocity and acceleration for the object, at the location you set with the InitialLocation property.
//MotionNoise:Deviation of selected and actual model, specified as a two- or three-element vector. The motion noise specifies the tolerance of the Kalman filter for the deviation from the chosen model. This tolerance compensates for the difference between the object's actual motion and that of the model you choose. Increasing this value may cause the Kalman filter to change its state to fit the detections. Such an increase may prevent the Kalman filter from removing enough noise from the detections. The values of this property stay constant and therefore may affect the long-term performance of the Kalman filter.
//MeasurementNoise:Variance inaccuracy of detected location, specified as a scalar. It is directly related to the technique used to detect the physical objects. Increasing the MeasurementNoise value enables the Kalman filter to remove more noise from the detections. 
//
//Description
//This function provides a simple approach for configuring the KalmanFilter structure for tracking a physical object in a Cartesian coordinate system. The tracked object may move with either constant velocity or constant acceleration.
//
//Examples
//k=configKalmanFilter("ConstantVelocity",[2 3],[0.1 1],[0.1 0.1],0.5);
//
//Authors
//Gursimar Singh

	if type(MotionModel) ~=10 then
		error(msprintf("Motion Model must be a string"));
	end	
	szM=size(MotionNoise);
	szI=size(InitialError);
	if MotionModel=="ConstantVelocity" then
		As=[1 1;0 1];
		Hs=[1 0];
		if szM ~=2  
			error(msprintf("MotionNoise must be 2 coulumn vector"));
		end
		if szI ~=2  
			error(msprintf("InitialError must be 2 coulumn vector"));
		end	
	elseif MotionModel=="ConstantAccerlation" then
		As=[1 1 0;0 1 1;0 0 1];
		Hs=[1 0 0];
		if szM ~=3 
			error(msprintf("MotionNoise must be 3 coulumn vector"));
		end
		if szI ~=3 
			error(msprintf("InitialError must be 3 coulumn vector"));
		end	
	else
		error(msprintf("Enter a valid Motion Model"));
	end	
	A=As;
	H=Hs;	
	dim=size(InitialLocation);
	if dim(1) ~= 1 then
		error(msprintf("InitialLocation must be vector"));
	end	
	dim=dim(2);
	for i=1:dim-1
		A=sysdiag(A,As);
		H=sysdiag(H,Hs);
	end
	for i=1:dim
		X(2*i-1)=InitialLocation(i);
		X(2*i)=0;
	end
	

	P = diag(repmat(InitialError, [1, dim]));
	Q = diag(repmat(MotionNoise, [1, dim]));
	R = diag(repmat(MeasurementNoise, [1, dim]));

	kalmanFilter=struct("StateTransitionModel",A,"MeasurementModel",H,"ControlModel",[],"State",X,"StateCovariance",P ,"ProcessNoise",Q,"MeasurementNoise",R);

endfunction

// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:  Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [params]=Calibrate(objectpoints,imagepoints1,varargin)
// This function Performs calibration for the fisheye model.
//
// Calling Sequence
//
// calib=Calibrate(objectpoints,imagepoints);
// calib=Calibrate(objectpoints,imagepoints,cameraMatrix,distCoeffs);
// [cameraMatrix ,distCoeffs,rotationMatrix,TranslationVector] =Calibrate(objectpoints,imagepoints); 
//
// Parameters
// calib: a Struct containing all the camera Params
// objectpoints: a  2d set of actual object points.
// imagepoints: a list of imagepoints of a stereo image.
// cameraMatrix: camera matrix of a camera.
// distCoeffs: distortion coefficients of a camera.it should be 4*1 or 1*4.
// TranslationVector: Output vector of translation vectors estimated for each pattern view. 
// rotationMatrix: Output vector of rotation vectors (see Rodrigues ) estimated for each pattern view. That is, each k-th rotation vector together with the corresponding k-th translation vector brings the calibration pattern from the model coordinate space (in which object points are specified) to the world coordinate space, that is, a real position of the calibration pattern in the k-th pattern view (k=0.. M -1).
//
// Description
//
// This function Performs calibration for the fisheye camera model.
//
// Examples
// i1 = imread('left1.jpg',0);
// new1 = detectCheckerboardCorner(i1,[7,10]);
// new2 = list(new1);
// f =Calibrate(worldPoints,new2);
// f
// f.cameraMatrix1
// f1 = undistortImage(i1, f.cameraMatrix1);
// imshow(f1);
// f2 = undistortImage(i1, f.cameraMatrix1,"distCoeffs",f.distortionCoefficients1);
// imshow(f2);
//   
  
   [lhs rhs]=argn(0);
  
     	if lhs>1
         error(msprintf(" Too many input arguments\n"));
    	elseif rhs>4
        error(msprintf(" Too many input arguments,maximum number of arguments is 4\n"));
    	
    	end 

	if rhs==2
        [a b e f] =raw_Calibrate(objectpoints,imagepoints1);
        elseif rhs==3
        [a b e f]=raw_Calibrate(objectpoints,imagepoints1,varargin(1));
        elseif rhs==4
        [a b e f]=raw_Calibrate(objectpoints,imagepoints1,varargin(1),varargin(2));
	
        end
       
       

        params=struct('cameraMatrix1',a,'distortionCoefficients1',b,'rotationMatrix',e,'TranslationVector',f);
endfunction

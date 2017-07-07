// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh & Nihar Rao
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [params] = calibrateCamera(objectpoints,imagepoints,imageSize,varargin)
//This function Performs camera calibration.Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern.It returns a struct of all the parameters like translation,rotation vector,camera matrix etc.
//
//Calling Sequence
//params=calibrateCamera(objectpoints,imagepoints,imageSize)
//params=calibrateCamera(objectpoints,imagepoints,imageSize,cameraMatrix)
//params=calibrateCamera(objectpoints,imagepoints,imageSize,cameraMatrix,distortionCoeffs)
//
//Parameters
//params:Structure of all the parameters like translation,rotation vector,camera matrix and distortionCoefficients.
//objectpoints:These are the worldpoints of teh checkboard.It can be obtained from genCheckerboardCorner.
//imagepoints:These are the detected checker board corners.Can be obtained from detectCheckerboardPoints.
//imageSize:Specified as width and height.
//cameraMatrix:Input/Output 3x3 floating-point camera matrix.  
// 
//distortionCoeffs:Output vector of distortion coefficients of 4, 5, 8, 12 or 14 elements.It can also be specified as input.
//
//Description
//This function Performs camera calibration.Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern.The coordinates of 3D object points and their corresponding 2D projections in each view must be specified.The depth co-ordinate of the object is assumed to be zero.That may be achieved by using an object with a known geometry and easily detectable feature points. Such an object is called a calibration pattern, and Scilab has built-in support for a chessboard as a calibration rig.
//
//Examples
//boardCols=7;
//boardRows=10;
//checkerSize=10;
//worldPoint=genCheckerboardPoints([boardCols boardRows],checkerSize);
//imagePoints=detectCheckerboardCorner(im,[boardRows,boardCols]);
//imagePoints=list(imagePoints)
//im=imread("images/checkboard.jpg",0);
//sz=size(im);
//f=calibrateCamera(worldPoints,imagePoints,[sz(2),sz(1)]);
//image=undistort(im,f.cameraMatrix,f.distortionCoefficients);
//imshow(image) 
//
//Authors
//Gursimar Singh
//Nihar Rao
//
//See also
//genCheckerboardPoints
//detectCheckerboardCorner
//undistort

     [lhs rhs]=argn(0);
  
     	if lhs>1
            error(msprintf(" Too many [a b c d ]put arguments\n"));
    	elseif rhs>5
            error(msprintf(" Too many input arguments,maximum number of arguments is 7\n"));
    	elseif rhs<3
            error(msprintf("the function needs atleast 3 arguments\n"));
    	end 

	    if rhs==3
            [a b c d ]=raw_calibrateCamera(objectpoints,imagepoints,imageSize);
        elseif rhs==4
            [a b c d ]=raw_calibrateCamera(objectpoints,imagepoints,imageSize,varargin(1));
        elseif rhs==5
            [a b c d ]=raw_calibrateCamera(objectpoints,imagepoints,imageSize,varargin(1),varargin(2));
        end
       
       

        params=struct('cameraMatrix',a,'distortionCoefficients',b,'rotationMatrix',c,'TranslationVector',d);
endfunction

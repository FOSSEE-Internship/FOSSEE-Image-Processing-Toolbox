// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function params = stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize, varargin)
// This function returns a set of transformation matrices which helps to callibrate camera 
// and also rectify the images taken by the camera.
// 
// Calling Sequence
// params = stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize)
//
// Parameters
// objectpoints: Vector of vectors of the calibration pattern points.
// imagepoints1: Vector of vectors of the projections of the calibration pattern points, observed
// by the first camera.
// imagePoints2: Vector of vectors of the projections of the calibration pattern points, observed
// by the second camera. 
// imageSize: Size of the image used only to initialize intrinsic camera matrix.
// Returns: A struct with the following values<itemizedlist><listitem><para> cameraMatrix1 </para></listitem><listitem><para> distortionCoefficients1 </para></listitem><listitem><para> cameraMatrix2 </para></listitem><listitem><para> distortionCoefficients2 </para></listitem><listitem><para> rotationMatrix </para></listitem><listitem><para> TranslationVector </para></listitem><listitem><para> DepthMap </para></listitem><listitem><para> ProjectionMatrix1 </para></listitem><listitem><para> ProjectionMatrix2 </para></listitem></itemizedlist>
//
//
// Description
// The function estimates transformation between two cameras making a stereo pair and also
// computes the rotation matrices for each camera that (virtually) make both camera image 
// planes the same plane. Consequently, this makes all the epipolar lines parallel and thus
// simplifies the dense stereo correspondence problem
//
// Examples
// stacksize("max");
// img_1 = imread("images/left1.jpg", 0);
// img_2 = imread("images/right1.jpg", 0);
// w1 = genCheckerboardPoints([10, 7], 8);
// ip1 = detectCheckerboardCorner(img_1, [7, 10]);
// ip2 = detectCheckerboardCorner(img_2, [7, 10]);
// ip1l = list(ip1);
// ip2l = list(ip2);
// op = stereoCalibrateAndRect(w1, ip1l, ip2l, size(img_1));
// [map map1] = disparity(img_1, img_2);
// img = reconstructScene(op.DepthMap, map1, 1);
//
// See also
// imread
// genCheckerboardPoints
// detectCheckerboardCorner
// disparity
// reconstructScene
//
// Authors
// Siddhant Narang

    [lhs rhs] = argn(0);
     	
    if lhs > 1
        error(msprintf("Too many output arguments\n"));
	elseif rhs > 8
        error(msprintf("Too many input arguments, maximum number of arguments is 7\n"));
	elseif rhs < 4
        error(msprintf("The function needs atleast 3 arguments\n"));
	end 

	if rhs == 4
        [a b c d e f g h i] = raw_stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize);
    elseif rhs == 5
        [a b c d e f g h i] = raw_stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize, varargin(1));
    elseif rhs == 6
        [a b c d e f g h i] = raw_stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize, varargin(1), varargin(2));
	elseif rhs == 7
        [a b c d e f g h i] = raw_stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize, varargin(1), varargin(2), varargin(3));
    elseif rhs == 8
        [a b c d e f g h i] = raw_stereoCalibrateAndRect(objectpoints, imagepoints1, imagepoints2, imageSize, varargin(1), varargin(2), varargin(3), varargin(4));
        end

    params = struct('cameraMatrix1', a, 'distortionCoefficients1', b, 'cameraMatrix2', c,'distortionCoefficients2', d, 'rotationMatrix', e, 'TranslationVector', f, "DepthMap", g, "ProjectionMatrix1", h, "ProjectionMatrix2", i);

endfunction
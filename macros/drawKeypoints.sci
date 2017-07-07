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

function [image] = drawKeypoints(img,keypoints,varargin)
//This function is used to draw the detected keypoints in an image.
//
//Calling Sequence 
//[image]=drawKeypoints(img,keypoints)
//[image]=drawKeypoints(img,keypoints,"color",[r,g,b]);
//[image]=drawKeypoints(img,keypoints,"color",[r,g,b],"flags",flag);
//
//Parameters
//image:Output image on which keypoints are drawn.
//img:Input image on which keypoints will be drawn.
//keypoints: Keypoints detected in an image via detectSURFfeatures,detectFastfeatures etc.
//"Name" - Value arguments:
//"color":specified as a row vector of rgb color values between [0-255].
//"flags":flag for with different drawing options.Flag can be 0,1,2 or 4.
//
// 0: DEFAULT:Output image matrix will be created (Mat::create), i.e. existing memory of output image may be reused. Two source image,matches and single keypoints will be drawn. For each keypoint only the center point will be drawn (without the circle around keypoint with keypoint size and orientation).
// 1: DRAW_OVER_OUTIMG :Output image matrix will not be created (Mat::create). Matches will be drawn on existing content of output image.
// 2: NOT_DRAW_SINGLE_POINTS:Single keypoints will not be drawn.
// 4: DRAW_RICH_KEYPOINTS : For each keypoint the circle around keypoint with keypoint size and orientation will be drawn. 
//
//Examples
//image=imread("images/lena.jpg");
//keypoints=detectAgastFeatures(image);
//new_image=drawKeypoints(image,keypoints);
//
//See also
//detectAgastFeatures
//detectFASTFeatures
//
//Authors
//Gursimar Singh


img_list=mattolist(img);
 [lhs rhs] = argn(0);
     if rhs>6 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<2 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end

	if rhs==2
	y=raw_drawKeypoints(img_list,keypoints);
	elseif rhs==4 then
	y=raw_drawKeypoints(img_list,keypoints,varargin(1),varargin(2));
	elseif rhs==6 then
	y=raw_drawKeypoints(img_list,keypoints,varargin(1),varargin(2),varargin(3),varargin(4));
	else
	error(msprintf("Invalid argument format"));
	end

channel = size(y);
	
	for i = 1:channel
		image(:,:,i) = y(i);
			end
endfunction
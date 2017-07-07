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

function [x] = drawMatch(img1,img2,keyPoints1,keyPoints2,indexPairs,distance,varargin)
//The function draws the matched fetures between to images. 
//
//Calling Sequence 
//[image]=drawMatch(img1,img2,keypoints1,keypoints2,indexPairs,distance)
//[image]=drawMatch(img1,img2,keypoints1,keypoints2,indexPairs,distance,"color",[r,g,b]);
//[image]=drawMatch(img1,img2,keypoints1,keypoints2,indexPairs,distance,"color",[r,g,b],"flags",flag);
//
//Parameters
//image:Output image on which keypoints are drawn.
//img1:Input image1 for drawing matched features.
//img2:Input image2 for drawing matched features.
//keypoints1: Keypoints detected in image1 via detectSURFfeatures,detectFastfeatures etc.
//keypoints2: Keypoints detected in image2 via detectSURFfeatures,detectFastfeatures etc.
//indexPairs: indexPairs obtained from matchFeatures function.Indices of matched keypoints/features.
//distance: distance between descriptors.
//"Name" - Value arguments:
//"color":specified as a row vector of rgb color values between [0-255].
//"flags":flag for with different drawing options.Flag can be 0,1,2 or 4.<itemizedlist><listitem>0 - DEFAULT - Output image matrix will be created, i.e. existing memory of output image may be reused. Two source image,matches and single keypoints will be drawn. For each keypoint only the center point will be drawn (without the circle around keypoint with keypoint size and orientation).</listitem><listitem>1 - DRAW_OVER_OUTIMG - Output image matrix will not be created. Matches will be drawn on existing content of output image.</listitem><listitem>2 - NOT_DRAW_SINGLE_POINTS - Single keypoints will not be drawn.</listitem><listitem>4 - DRAW_RICH_KEYPOINTS - For each keypoint the circle around keypoint with keypoint size and orientation will be drawn.</listitem></itemizedlist> 
//
//Description
//The function draws the matched fetures between to images.Match is a line connecting two keypoints.
//
//Examples
// stacksize("max");
// img_1 = imread("images/table1.jpg", 0);
// img_2 = imread("images/table2.jpg", 0);
// lis1 = detectFASTFeatures(img_1, "MinConstrast", 0.2);
// lis2 = detectFASTFeatures(img_2, "MinConstrast", 0.2);
// features_1 = extractFeatures(img_1, lis1.Location, "cornerPoints", "Metric", lis1.Metric);
// features_2 = extractFeatures(img_2, lis2.Location, "cornerPoints", "Metric", lis2.Metric)
// dimage = drawKeypoints(img_1, lis1.Location);
// [matches, distance] = matchFeatures(features_1.Features, features_2.Features);
// matchedImage = drawMatch(img_1, img_2, lis1.Location, lis2.Location, matches, distance);
//
//Authors
//Gursimar Singh
//
//See also
//matchFeatures
//extractFeatures
//detectFastFeatures

img_list1=mattolist(img1);
img_list2=mattolist(img2);
 [lhs rhs] = argn(0);
     if rhs>10 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<6 then
	 	error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 	error(msprintf("Too many output arguments"));
     end

	if rhs==6
		y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance);
	elseif rhs==8 then
		y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance,varargin(1),varargin(2));
	elseif rhs==10 then
		y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance,varargin(1),varargin(2),varargin(3),varargin(4));
	else
		error(msprintf("Invalid argument format"));
	end

	channel = size(y);
	
	for i = 1:channel
		x(:,:,i) = y(i);
	end
endfunction

// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma,Suraj Prakash
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [x,x1] = epipolarlines(img1,img2,qid,tid,fund,k1,k2)
// The function draws epipolar lines by feature matching using stereo images.
//
// Calling Sequence
// [iz,iz1] = epipolarlines(img1,img2,qid,tid,fund,k1,k2)
// 
// Parameters
// F : A 3 * 3 fundamental matrix computed from stereo images. It should be double or single
// image1 : The first input stereo image.
// image2 : The second input stereo image.
// qid : The queryid of the matched keypoints of the input images.
// tid : The tid of the matched keypoints points of the input images.
// k1 : The detected keypoints of the first image.
// k2 : The detected keypoints of the second image.
// iz : The output image containing the drawn epipolar lines.
// iz1 : The output image containing the drawn epipolar lines.
//
// Description
// The function determines the epipolar lines by matching the features of two stereo images using the detected keypoints. 
//
// Examples
// i = imread('left11.jpg',0);
// ii = detectAkazeFeatures(i);
// i1 = imread('right11.jpg',0);
// ii1 = detectAkazeFeatures(i1);
// [matches, distance] = matchFeatures(ii.Features, ii1.Features);
// f1 = estimateFundamentalMat(ii.KeyPoints(1:644,:),ii1.KeyPoints);
// [iz,iz1] = epipolarlines(i,i1,matches(1),matches(2),f1,ii.KeyPoints,ii1.KeyPoints);
//



img_list1=mattolist(img1);

img_list2=mattolist(img2);
 
 
 [lhs rhs] = argn(0);
     
     if rhs>7 then
         error(msprintf("Too many input arguments"));
     end
     
     if rhs<7 then
	 error(msprintf("Not enough input arguments"));
	 end
     
     if lhs >2
	 error(msprintf("Too many output arguments"));
     end


 [y,y1]=raw_epipolarlines(img_list1,img_list2,qid,tid,fund,k1,k2);


channel = size(y);
	
	for i = 1:channel
		x(:,:,i) = y(i);
	end
channel1 = size(y1);
	
	for i = 1:channel1			
		x1(:,:,i) = y1(i);
	end




endfunction

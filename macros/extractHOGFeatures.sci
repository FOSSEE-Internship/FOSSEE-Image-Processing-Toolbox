// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [featureVector , keypoints] = extractHOGFeatures(I, varargin)
// This function is used to extract features from an image.
//
// Calling Sequence
// im = imread(image);
// feat = extractHOGFeatures(im)
// feat = extractHOGFeatures(im,name,value)
// feat = extractHOGFeatures(im,"cellsize",cellsize)
// feat = extractHOGFeatures(im,"cellsize",cellsize,"BlockSize",Blocksize,)
// feat = extractHOGFeatures(im,"cellsize",cellsize,"BlockSize",Blocksize,"BlockOverlap",BlockOverlap)
// feat = extractHOGFeatures(im,"cellsize",cellsize,"BlockSize",Blocksize,"BlockOverlap",BlockOverlap,"NumBins",Numbins)
//
// Parameters
// feat : Feature matrix returned by obtaining features from a particular image.
// cellsize : The size of the cell specified by the user for extractng features.
// Blocksize : The dimensions of the block containing multiplte cells for feature extraction. An image comprises of multiple blocks
// BlockOverlap :  This matrix specifies the dimension of the block which moves or traverses across an image for feature extraction.
// Numbins :  Number of bins.The histogram contains 9 bins corresponding to angles 0, 20, 40 … 160.
//
// Description
// feat = extractHOGFeatures(I) returns extracted HOG features from a truecolor or grayscale input image, I. The features are returned in a 1-by-N vector,where N is the HOG feature length. 
//
// Examples
// i = imread('download.jpg')
// tr = extractHOGFeatures(i);
// tr1 = extractHOGFeatures(i,"CellSize",[8,8]);
// tr2 = extractHOGFeatures(i,"CellSize",[8,8],"BlockSize",[16,16]);
// tr3 = extractHOGFeatures(i,"CellSize",[8,8],"BlockSize",[16,16],"BlockOverlap",[8,8]);
// tr4 = extractHOGFeatures(i,"CellSize",[8,8],"BlockSize",[16,16],"BlockOverlap",[8,8],"NumBins",9);


	img = mattolist(I);
	[lhs rhs] = argn(0);
	if rhs<1 then
		error(msprintf("Not enough input arguments"));
	end
	if rhs>9 then
		error(msprintf("Too many input arguments"));
	end	
	l = rhs-1;
	if(l==0) then
		[featureVector, keypoints] = raw_extractHOGFeatures(img);
	elseif (l==2) then
		[featureVector, keypoints] = raw_extractHOGFeatures(img,varargin(1),varargin(2));
	elseif (l==4) then
		[featureVector, keypoints] = raw_extractHOGFeatures(img,varargin(1),varargin(2),varargin(3),varargin(4));
	elseif (l==6) then
		[featureVector, keypoints] = raw_extractHOGFeatures(img,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
	else
		[featureVector, keypoints] = raw_extractHOGFeatures(img,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
	end
		
endfunction


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

function [out]=extractLBPFeatures(img,varargin)
// This function is used to extract features from an image.
//
// Calling Sequence
// im = imread(image);
// feat = extractLBPFeatures(im);
// feat = extractLBPFeatures(im,name,value);
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors)
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors,"Radius",Radius)
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors,"Radius",Radius,"Upright",Upright)
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors,"Radius",Radius,"Upright",Upright,"Interpolation",Interpolation)
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors,"Radius",Radius,"Upright",Upright,"Interpolation",Interpolation,"CellSize",CellSize)
// feat = extractLBPFeatures(im,"NumNeighbors",NumNeighbors,"Radius",Radius,"Upright",Upright,"Interpolation",Interpolation,"CellSize",CellSize,"Normalization",Normalization)
//
// Parameters
// feat : Feature matrix returned by obtaining features from a particular image.
// cellsize : The size of the cell specified by the user for extractng features.
// NumNeighbors : It denotes the number of neighbors presnt across the vicinity of the central pixel.
// Radius : The radius of the circle on which the points correspond to image coordinates.
// Upright : Rotation invariance flag, specified as the comma-separated pair consisting of 'Upright' and a logical scalar. When you set this property to true, the LBP features do not encode rotation information. Set 'Upright' to false when rotationally invariant features are required.
// Interpolation :Interpolation method used to compute pixel neighbors, specified as the comma-separated pair consisting of 'Interpolation' and the character vector 'Linear' or 'Nearest'. Use 'Nearest' for faster computation, but with less accuracy.  
// Normalization :Type of normalization applied to each LBP cell histogram, specified as the comma-separated pair consisting of 'Normalization' and the character vector 'L2' or 'None'. To apply a custom normalization method as a post-processing step, set this value to 'None'.
//
// Description
// This function is used to extract features from an image by comparing pixel values between the central and neighbouring pixels,further , it stores the features in the form of a matrix and returns this matrix to the user.
//
// Examples
// i1= imread('kevin.jpg',0);
// i2= imread('air.jpg',0);
// i3= imread('bike.jpg',0);
// rr= extractLBPFeatures(i1);
// rr1 = extractLBPFeatures(i2);
// rr2 =extractLBPFeatures(i3);

         input1 = mattolist(img);
         [lhs rhs] = argn(0);
         if rhs>13 then
         error(msprintf("Too many input arguments"));
         end
         if rhs<1 then
	 error(msprintf("Not enough input arguments"));
         end
         
         if rhs==13 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9),varargin(10),varargin(11),varargin(12));
         out(:,:,1)=a;
         elseif rhs==12 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9),varargin(10),varargin(11));
         out(:,:,1)=a;
         elseif rhs==11 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9),varargin(10));
         out(:,:,1)=a;
         elseif rhs==10 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9));
         out(:,:,1)=a;
         elseif rhs==9 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
         out(:,:,1)=a;
         elseif rhs==8 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7));
         out(:,:,1)=a;
         elseif rhs==7 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
         out(:,:,1)=a;
         elseif rhs==6 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5));
         out(:,:,1)=a;
         elseif rhs==5 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3),varargin(4));
         out(:,:,1)=a;
         elseif rhs==4 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2),varargin(3));
         out(:,:,1)=a;
         elseif rhs==3 then
         a=raw_extractLBPFeatures(input1,varargin(1),varargin(2));
         out(:,:,1)=a;
         elseif rhs==2 then
         a=raw_extractLBPFeatures(input1,varargin(1));
         out(:,:,1)=a;
         else
         a=raw_extractLBPFeatures(input1);
         out(:,:,1)=a;
         end
      
endfunction

// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Rohit Suri, Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function[dstImg] = roiPoly(srcImg, col_list, row_list)
//This function creates mask by selecting the ROI as a poygon.
//This function creates mask by selecting the ROI as a poygon.
//
// Calling Sequence
//  I = imimread(image-location)
//  c = [c1 c2 ..............cn]
//  r = [r1 r2 ..............rn]  
//  BW = roipoly(I, c, r)
// 
// Parameters
// I : Input image
// c : a 1xn vector specifying the column indices of the vertices of the n sided polygon
// r : a 1xn vector specifying the row indices of the vertices of the n sided polygon
// BW: Output mask image for input I
// 
// Description
// Returns the ROI specified by the polygon described by vectors c and r, which specify the column 
// and row indices of each vertex, respectively. c and r must be the same size.
//
// Examples
// I = imread('image_01.jpeg');
// c = [50 50 100 100]   //defining the column indices of the vertices of the ROI
// r = [100 150 150 100 ]   //defining the corresponding row indices of the vertices of the ROI
// BW = roipoly(I, c, r) //creating a mask
// imshow(BW)   //viewing the mask
// c=colorChange(I,mask,2.4 ,2.0, 2.2); //using this mask in colorChange function
// imshow(c);
//
// Examples
// I = imread('../images/color3.jpg');
// c = [ 150 200 250 300 250 200 ] ;  //defining the column indices of the vertices of the ROI
// r = [ 135 100 100 135 170 170 ] ;  //defining the corresponding row indices of the vertices of the ROI
// gray = rgb2gray(I) ;                           //for a grayscale image
// BW = roiPoly(I, c, r); //creating a mask
// imshow(BW)   //viewing the mask
//
// Examples
// I = imread('/images/color3.jpg');
// c1 = [200 250 300 250  150 200 ]
// r1 = [ 170 170 135 100 135 100]  //undesired output because changing the order of points 
// BW = roiPoly(I,c1,r1);
// imshow(BW)
//
// Authors
// Vinay Bhat
// Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>3
        error(msprintf(" Too many input arguments"));
    elseif rhs<3
        error(msprintf(" input arguments missing"));
    end
    
   	srcMat = mattolist(srcImg)

	out = raw_roiPoly(srcMat,row_list,col_list)
	
	
	channels = size(out)
	
	for i = 1:channels
		dstImg(:,:,i) = out(i)
	end
	
endfunction

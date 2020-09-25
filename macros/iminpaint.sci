// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [pImg] = iminpaint(srcImg1,mask,radius,methodNum)
//Inpaints the unwanted area in an image using the neighboring pixel points
//
//Calling Sequence
//pImg = iminpaint(srcImg1,maskedRectangle,radius,methodNum)
//
//Parameters
//srcImg : It is the input image
//maskedRectangle: It is a vector of double used which contains the co-ordinates of the masking rectangle
//radius : Radius of a circular neighborhood of each point inpainted that is considered by the algorithm. 
//methodNum : Inpainting method that could be one of the following,INPAINT_NS Navier-Stokes based method or INPAINT_TELEA Method by Alexandru Telea
//
//Description
//The function reconstructs the selected image area from the pixel near the area boundary. The function may be used to remove dust and scratches from a scanned photo, or to remove undesirable objects from still images.
//
//Examples
//a = imread("p1.jpeg");
//b = [210,40,100,200];
//typeOfMethod = 2;
//radius = 50;
//c = iminpaint(a,b,radius,typeOfMethod);
//
//Examples
//a = imread("orig.jpg");
//b = [100,157,120,150];
//typeOfMethod = 1;
//radius = 5;
//c = iminpaint(a,b,radius,typeOfMethod);
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Ashish Mantosh, NIT Rourkela
	srcMat1 = mattolist(srcImg1)
	srcMat2 = mattolist(mask)
	out=raw_iminpaint(srcMat1,srcMat2,radius,methodNum)

	channels = size(out)

	for i = 1 : channels
		pImg(:,:,i) = (out(i))
	end
	pImg = double(pImg)
endfunction

// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [tImg] = applyTransformer(srcImg1, srcImg2, typeOfMethod, hessianThreshold, rpTPS, sfAffine)	
// This function is used to apply affine or TPS transformation to image.
//
// Calling Sequence
// [ tImg] =  applyTransformer(srcImg1, srcImg2, typeOfMethod, hessianThreshold, rpTPS, sfAffine)
// 
// Parameters
// srcImg1 : It is the first input image. 
// srcImg2 : It is the  second input image, which is also the target image. 
// typeOfMethod : It is used as a flag to pick a certain type of transformation. Use value '1' for 'Affine Transformation' and '2' for 'Thin Plate Spline Shape Transformation'. It is of double type.
// hessianThreshold : It is the threshold value for Hessian keypoint detector in SURF(Speeded-Up Robust Features). It is of double type.
// rpTPS : It is used to set the regularization parameter for relaxing the exact interpolation requirements of the TPS algorithm. It is of double type.
// sfAffine : It is used to set the full-affine condition for Affine Transformation. If true, the function finds as optimal transformation with no additional restrictions(6 degrees of freedom). Otherwise, the class of transformations to choose from is limited to combination of translation, rotation & uniform scaling(5 degrees of freedom).
// tImg : The transformed image of the target(srcImg2). It is of hypermat type.
//
// Description
// This function is used to perform shape transformation, the user gets to choose and apply the type of transformation she/he wishes to perform.
//
// Examples
// affine transformation
// a = imread("/images/bryan.jpeg");
// b = imread("/images/p1.jpg");
// typeOfMethod=1
// hessianThreshold=5000;
// rpTPS=25000;
// sfAffine=%f;
// img=applyTransformer(a,b,typeOfMethod,hessianThreshold, rpTPS, 
//
// Examples
// a= imread("/images/lena.jpeg");
// b= imread("/images/bryan.jpeg");
// typeOfMethod=1
// hessianThreshold=5000;
// rpTPS=2000;
// sfAffine=%t;
// img=applyTransformer(a,b,typeOfMethod,hessianThreshold, rpTPS,sfAffine);
//
// Examples
// TPS shape transformation
// a = imread("/images/photo.jpg");
// b= imread("/images/photo1.jpg");
// typeOfMethod=2
// hessianThreshold=5000;
// rpTPS=800;
// sfAffine=%t;
// img=applyTransformer(a,b,typeOfMethod,hessianThreshold, rpTPS,sfAffine);
// 
// Examples
// a = imread("/images/b1.jpeg");
// b= imread("/images/b2.jpeg");
// typeOfMethod=1
// hessianThreshold=5000;
// rpTPS=800;
// sfAffine=%f;
// img=applyTransformer(a,b,typeOfMethod,hessianThreshold, rpTPS,sfAffine);
//
// Examples
// a = imread("/images/photo.jpg");
// b= imread("/images/photo1.jpg");
// typeOfMethod=2
// hessianThreshold=5000;
// rpTPS=800;
// sfAffine=%t;
// img=applyTransformer(a,b,typeOfMethod,hessianThreshold, rpTPS,sfAffine);
//
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela

	srcMat1 = mattolist(srcImg1);
	srcMat2 = mattolist(srcImg2);

	[out] = raw_applyTransformer(srcMat1, srcMat2, typeOfMethod, hessianThreshold, rpTPS, sfAffine)

	channels = size(out)

	for i = 1:channels
		tImg(:, :, i) = (out(i))
	end

endfunction

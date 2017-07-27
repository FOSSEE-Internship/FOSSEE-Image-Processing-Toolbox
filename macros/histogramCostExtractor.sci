// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik & Shubham Lohakare
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [costMat] = histogramCostExtractor(srcImg1, srcImg2, typeOfMethod, hessianThreshold, varargin)
// This function computes the cost matrix.
//
// Calling Sequence
// [ costMatrix ] = histogramCostExtractor(srcImg1, srcImg2, typeOfMethod=3, hessianThreshold); // Norm based cost 
// [ costMatrix ] = histogramcostExtractor(srcImg1, srcImg2, typeOfMethod=1, hessianThreshold, nDummies, defaultCost); // Chi as well as EMDL1 based cost extraction
// [ costMatrix ] = histogramCostExtractor(srcImg1, srcImg2, typeOfMethod=2, hessianThreshold, nDummies, defaultCost); // EMDL1 based cost extraction
//
// Parameters
// srcImg1 : It is the first input image. 
// srcImg2 : It is the second input image.
// typeOfMethod : It is used as a flag to pick a certain type of transformation. Use value '1' for 'Chi based cost ectraction', '2' for 'EMDL1 based cost extraction' and '3' for 'Norm based cost extraction'. It is of double type.
// hessianThreshold : It is the threshold value for Hessian keypoint detector in SURF(Speeded-Up Robust Features). It is of double type.
// rpTPS : It is used to set the regularization parameter for relaxing the exact interpolation requirements of the TPS algorithm. It is of double type.
// costMatrix : It is the cost matrix. 
//
// Description
// This function is used to calculate the histogram based cost matrix of two images, the user gets to choose and apply the type of transformation she/he wishes to perform.
//
// Examples
// // Chi based cost extraction
// a= imread("/images/n.jpeg");
// b= imread("/images/n1.jpeg");
// typeOfMethod=1;
// hessianThreshold=5000;
// nDummies=25;
// defaultCost=0.2;
// c=histogramCostExtractor(a,b,typeOfMethod,hessianThreshold,nDummies,defaultCost);
//
// Examples
// // EMDL1
// a = imread("/images/n.jpeg");
// b = imread("/images/n1.jpeg");
// typeOfMethod=2;
// hessianThreshold=5000;
// nDummies=25;
// defaultCost=0.2;
// c=histogramCostExtractor(a,b,typeOfMethod,hessianThreshold,nDummies,defaultCost);
//
// Examples
// Norm based cost extraction
// a = imread("/images/n.jpeg");
// b= imread("/images/n1.jpeg");
// typeOfMethod=3;
// hessianThreshold=5000;
// c=histogramCostExtractor(a,b,typeOfMethod,hessianThreshold);
//
// Authors
// Ashish Mantosh Barik, NIT Rouekela
// Shubham Lohakare, NITK Surathkal

	srcMat1 = mattolist(srcImg1)
	srcMat2 = mattolist(srcImg2)	

	[lhs, rhs] = argn(0)

	select rhs
		case 4 then
			costMat = raw_histogramCostExtractor(srcMat1, srcMat2, typeOfMethod, hessianThreshold)
		case 6 then
			costMat = raw_histogramCostExtractor(srcMat1, srcMat2, typeOfMethod, hessianThreshold, varargin(1), varargin(2))
	end

endfunction

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
function dist = distanceExtractor(srcImg1, srcImg2, typeOfMethod, varargin)
// This function computes the shape-distance between two images.
//
// Calling Sequence
// [ dist ] = distanceExtractor(srcImg1, srcImg2, typeOfMethod); // Hausdorrf distance
// [ dist ] = distanceExtractor(srcImg1, srcImg2, typeOfMethod, nAngularBins, innerRadius, nRadialBins, outerRadius, iterations); // Shape Context
//
// Parameters
// srcImg1 : It is the first input image. 
// srcImg2 : It is the  second input image. 
// typeOfMethod : It is used as a flag to pick a certain type of Shape Distance calculation technique. Use '1' for 'Shape Context' and '2' for 'Hausdorrf'.
// nAngularBins : Establish the number of angular bins for the Shape Context Descriptor used in the shape matching pipeline. 
// nRadialBins : Establish the number of radial bins for the Shape Context Descriptor used in the shape matching pipeline. 
// innerRadius : Set the inner radius of the shape context descriptor.
// outerRadius : Set the outer radius of the shape context descriptor.
// dist : It is the calculated distance. It is of double type.
//
// Description
// This function is used to compute the shape distance between two shapes defined by its contours.
//
// Examples
// // Hausdorff distance extractor
// a = imread("/images/bnwhite.jpg");
// b = imread("/images/bryan.jpeg");
// typeOfMethod=2;//2 is for hausdorff
// c=distanceExtractor(a,b,typeOfMethod);orff
//
// Examples
// // Shape Context Distance extractor
// a = imread("/images/photo.jpg");
// b = imread("/images/photo1.jpg");
// typeOfMethod=1; //1 for ShapeContext
// nAngularBins=12;
// nRadialBins=4;
// innerRadius=0.2;
// outerRadius=2;
// iterations=3;
// ndummies = 25;
// defaultCost = 0.2;
// rpTps =0 ;
// dist=distanceExtractor(a,b,typeOfMethod,nAngularBins,nRadialBins,innerRadius,outerRadius,iterations,ndummies,dC,rpTps);
//
// Examples
// Incorrect usage
// a=4; (not hypermat)
// b=88; (not hypermat)
// typeOfMethod=1; //1 for ShapeContext
// nAngularBins=12;
// nRadialBins=4;
// innerRadius=2;
// outerRadius=0.2;
// iterations=300;
// ndummies = 25;
// defaultCost = 0.2;
// rpTps =0 ;
// dist=distanceExtractor(a,b,typeOfMethod,nAngularBins,nRadialBins,innerRadius,outerRadius,iterations,ndummies,dC,rpTps);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela



	srcMat1 = mattolist(srcImg1);
	srcMat2 = mattolist(srcImg2);

	[lhs, rhs] = argn(0)

	select rhs
		case 3 then	// Hausdorff
			dist = raw_distanceExtractor(srcMat1, srcMat2, typeOfMethod)
	
		case 8 then	// Shape Context
			dist = raw_distanceExtractor(srcMat1, srcMat2, typeOfMethod, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
	end
	

endfunction

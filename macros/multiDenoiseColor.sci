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
function [denoise] = multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength, filterStrengthColor, templateWindowSize, searchWindowSize, n,varargin)
// Removes colored noise from an image taking reference images of the same captured in small intervals
//
// Calling Sequence
//  out = multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength, filterStrengthColor,templateWindowSize, searchWindowSize, n, srcImg1, srcImg2, srcImg3)
// out = multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength,filterStrengthColor, templateWindowSize, searchWindowSize, n, srcImg1, srcImg2, srcImg3,srcImg4)
// out = multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength,filterStrengthColor, templateWindowSize, searchWindowSize, n,srcImg1, srcImg2, srcImg3,srcImg4,srcImg5)
//
// Parameters
// imgToDenoiseIndex : Target image to denoise index in srcImgs sequence 
// temporalWindowSize : Number of surrounding images to use for target image denoising. Should be odd. 
// filterStrength : Parameter regulating filter strength. Bigger h value perfectly removes noise but also removes image details, smaller h value preserves details but also preserves some noise.
// filterStrengthColor : same as filterStrength but for color components of noise.
// templateWindowSize : Size in pixels of the template patch that is used to compute weights. Should be odd.
// searchWindowSize : Size in pixels of the window that is used to compute weighted average for given pixel. Should be odd. 
// n : number of images passed.
// srcImgs : The input images which are passed. They are variable arguments.
//
// Description
// Modification of denoiseColor function for images sequence where consequtive images have been captured in small period of time. It removes colored noise.The output is a denoised image
//
// Examples
// a= imread("img1.jpg");
// b= imread("img2.jpg");
// c= imread("img3.jpg");
// imgToDenoiseIndex=1;
// temporalWindowSize =1;
// templateWindowSize =7;
// searchWindowSize =21;
// filterStrength = 10;
// filterStrengthColor = 10;
// n = 3;
// k=multiDenoiseColor(imgToDenoiseIndex,temporalWindowSize,filterStrength,filterStrengthColor,templateWindowSize,searchWindowSize,n,a,b,c);
//
// Examples
// a= imread("img1.jpg");
// b= imread("img2.jpg");
// c= imread("img3.jpg");
// d= imread("img4.jpg");
// imgToDenoiseIndex=1;
// temporalWindowSize =1;
// templateWindowSize =7;
// searchWindowSize =21;
// filterStrength = 100;
//filterStrengthColor=30;
// n = 4;
// choice = 1;
// k=multiDenoiseColor(imgToDenoiseIndex,temporalWindowSize,filterStrength,filterStrengthColor,templateWindowSize,searchWindowSize,n,a,b,c,d);
//
// Examples
// a= imread("pic.jpeg");
// b= imread("pic1.jpeg");
// c= imread("pic2.jpeg");
// d= imread("pic3.jpeg");
// e= imread("pic4.jpeg");
// imgToDenoiseIndex=3;
// temporalWindowSize =3;
// templateWindowSize =7;
// searchWindowSize =21;
// filterStrength = 75;
//filterStrengthColor = 50;
// n = 5;
// choice = 2;
// k=multiDenoiseColor(imgToDenoiseIndex,temporalWindowSize,filterStrength,filterStrengthColor,templateWindowSize,searchWindowSize,n,a,b,c,d,e);
//
// Authors
// Shubham Lohakare, NITK Surathkal
// Ashish Mantosh, NIT Rourkela
		[lhs, rhs] = argn(0)

	select rhs
		case 10 then
			for i = 1:n
				varargin(i) = mattolist(varargin(i))
			end
			out = raw_multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength, templateWindowSize, searchWindowSize, n,varargin(1), varargin(2), varargin(3))
		case 11 then
			for i = 1:n
				varargin(i) = mattolist(varargin(i))
			end
			out = raw_multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength, templateWindowSize, searchWindowSize, n, varargin(1), varargin(2), varargin(3), varargin(4))
		case 12 then 
			for i = 1:n
				varargin(i) = mattolist(varargin(i))
			end
			out = raw_multiDenoiseColor(imgToDenoiseIndex, temporalWindowSize, filterStrength, templateWindowSize, searchWindowSize, n,varargin(1), varargin(2), varargin(3), varargin(4), varargin(5))
	end
	
	channel = size(out)

	for i= 1: channel
		denoise(:,:,i) = (out(i))
	end
	
endfunction

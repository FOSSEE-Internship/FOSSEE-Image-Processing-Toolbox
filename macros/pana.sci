// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: M Avinash Reddy & Manoj Sree Harsha & Ebey Abraham
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=pana(img1,varargin)
	//Creates the output panorama image using two or more images 
	//
	//Calling Sequence
	//stacksize('max')
	//img1=imread('path of the image file')
	//img2=imread('path of the image file')
	//out = pana(img1,img2)
	//img3=imread('path of the image file')
	//out = pana(img1,img2,img3)
	//img4=imread('path of the image file')
	//out = pana(img1,img2,img3,img4)
	//img5=imread('path of the image file')
	//out = pana(img1,img2,img3,img4,img5)
	//img6=imread('path of the image file')
	//out = pana(img1,img2,img3,img4,img5,img6)
	//
	//Parameters
	//img1 : an image 
	//img2 : an image
	//img3 : an image
	//img4 : an image
	//img5 : an image
	//img6 : an image
	//
	//Description
	//The images pass through a stiching pipeline before the final panorama is formed.
	//Features are extracted from each image and matching is done on two consecutive images to ensure the continuity in images.
	//After this camera parameters are estimated which is required to do particular type of warping.
	//After warping is done exposure is compensated in all images so as to get a uniform exposure throughout the panaroma.
	//Seam estimation is done next to get the exact portions of images to be blended.
	//Finally, the images are blended to form the panorama.
	//
	//Examples
	//stacksize('max');
	//img1=imread('images/campus_017.jpg');
	//img2=imread('images/campus_016.jpg');
	//img3=imread('images/campus_015.jpg');
	//img4=imread('images/campus_014.jpg');
	//img5=imread('images/campus_013.jpg');
	//img6=imread('images/campus_012.jpg');
	//out=pana(img1,img2,img3,img4,img5,img6);
	//
	//Examples
	//stacksize('max');
	//a=imread('images/s1.jpg');
	//b=imread('images/s2.jpg');
	//c=imread('images/s3.jpg');
	//y=pana(a,b,c);
	//Authors
	//    M Avinash Reddy , Manoj Sree Harsha , Ebey Abraham
    	[lhs rhs]=argn(0)
    	if lhs>1
        	error(msprintf(" Too many output arguments"))
	end
    	if rhs>6
        	error(msprintf(" Too many input arguments,maximum number of arguments is 6"))
	end
    	if rhs<2
        	error(msprintf("the function needs atleast 2 arguments"))
    	end
	image=mattolist(img1)
	for i=1:rhs-1
		varargin(i)=mattolist(varargin(i))
	end
    	if rhs==2
    		res=raw_panorama(image,varargin(1))
    	elseif rhs==3
       		res=raw_panorama(image,varargin(1),varargin(2))
    	elseif rhs==4
       		res=raw_panorama(image,varargin(1),varargin(2),varargin(3))
    	elseif rhs==5
       		res=raw_panorama(image,varargin(1),varargin(2),varargin(3),varargin(4))
    	elseif rhs==6
       		res=raw_panorama(image,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5))
    	end

	channel=size(res)
	for i = 1: channel
		out(:,:,i) = (res(i))
	end
	out=double(out)
endfunction

// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Manoj Sree Harsha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function x = exposure(algo_type,img_index,varargin)
	//Compensate exposure in the specified image.
	//
	//Calling Sequence
	//stacksize('max')
	//image1=imread('path of the image file')
	//image2=imread('path of the image file')
	//y = exposure(algo_type,img_index,image1,image2)
	//image3=imread('path of the image file')
	//y = exposure(algo_type,img_index,image1,image2,image3)
	//image4=imread('path of the image file')
	//y = exposure(algo_type,img_index,image1,image2,image3,image4)
	//image5=imread('path of the image file')
	//y = exposure(algo_type,img_index,image1,image2,image3,image4,image5)
	//image6=imread('path of the image file')
	//y = exposure(algo_type,img_index,image1,image2,image3,image4,image6)
	//
	//Parameters
	//algo_type : an integer between 1 and 3 (both inclusive) specifying the algorithm to be used for exposure compensation.
	//img_index : index of the image on which exposure compensator will be applied
	//image1 : an image
	//image2 : an image
	//image3 : an image
	//image4 : an image
	//image5 : an image
	//image6 : an image
	//
	//Description
	//y = exposure(algo_type,img_index,image1,image2) returns an image whose exposure is compensated.
	//Features are extracted from each image and matching is done on two consecutive images to ensure the continuity in images.
	//After this camera parameters are estimated which is required to do particular type of warping.
	//After warping is done exposure is compensated in an image whose index is mentioned.
	//
	//Examples
	//a=imread('images/lenahi.jpg');
	//b=imread('images/lenalow.jpg');
	//algo_type=1;
	//img_index=1;
	//y=exposure(algo_type,img_index,a,b);
	//imshow(y)
	//Authors
	//    Manoj Sree Harsha

[lhs rhs]=argn(0)
if rhs<4
		error(msprintf("Function need atleast 4 arguments"))
elseif rhs>8
		error(msprintf("Too many input arguments,max. no of arguments is 8"))
end
for i=1:(rhs-2)
	varargin(i)=mattolist(varargin(i))
end

if rhs==4
		y=raw_exposure(algo_type,img_index,varargin(1),varargin(2));
elseif rhs==5
		y=raw_exposure(algo_type,img_index,varargin(1),varargin(2),varargin(3));
elseif rhs==6
		y=raw_exposure(algo_type,img_index,varargin(1),varargin(2),varargin(3),varargin(4));
elseif rhs==7
		y=raw_exposure(algo_type,img_index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5));
elseif rhs==8
		y=raw_exposure(algo_type,img_index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
end
channels = size(y)
	for i = 1:channels
		x(:, :, i) = (y(i))
	end
	x=double(x);
endfunction

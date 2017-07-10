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

function [res]=calfocal(image,varargin)
  //Estimates focal lengths for each given camera.
  //
  //Calling Sequence
  //res = calfocal(image1,image2)
  //res = calfocal(image1,image2,image3)
  //res = calfocal(image1,image2,image3,image4)
  //res = calfocal(image1,image2,image3,image4,image5)
  //res = calfocal(image1,image2,image3,image4,image5,image6)
  //
  //Parameters
  //image1 : an image
  //image2 : an image
  //image3 : an image
  //image4 : an image
  //image5 : an image
  //image6 : an image
  //
  //Description
  //res = calfocal(image1,image2) return a 1x2 matrix.
  //Features are extracted from each image and matching is done on two consecutive images to ensure the continuity in images.
  //After this camera parameters are estimated along with their focal lengths are estimated.
  //
  //Examples
  //a=imread('images/campus_000.jpg');
  //b=imread('images/campus_001.jpg');
  //res=calfocal(a,b);
  //Authors
  //    Manoj Sree Harsha

[lhs rhs]=argn(0)

    	if lhs>1
        	error(msprintf(" Too many output arguments"))
    	elseif rhs>6
        	error(msprintf(" Too many input arguments,maximum number of arguments is 6"))
    	elseif rhs<2
        	error(msprintf("the function needs atleast 2 arguments"))
    	end
	image=mattolist(image)
	for i=1:rhs-1
		varargin(i)=mattolist(varargin(i))
	end
    	if rhs==2
    		res=raw_focals(image,varargin(1))
    	elseif rhs==3
       		res=raw_focals(image,varargin(1),varargin(2))
    	elseif rhs==4
       		res=raw_focals(image,varargin(1),varargin(2),varargin(3))
    	elseif rhs==5
       		res=raw_focals(image,varargin(1),varargin(2),varargin(3),varargin(4))
    	elseif rhs==6
       		res=raw_focals(image,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5))
    	end

	res=double(res)
endfunction

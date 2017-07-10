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

function res = rotestimate(cam_index,varargin)
  //Estimates camera intrinsic parameters matrix(K).
  //
  //Calling Sequence
  //img1=imread('path of the image file')
  //img2=imread('path of the image file')
  //res = rotestimate(cam_index,img1,img2)
  //img3=imread('path of the image file')
  //res = rotestimate(cam_index,img1,img2,img3)
  //img4=imread('path of the image file')
  //res = rotestimate(cam_index,img1,img2,img3,img4)
  //img5=imread('path of the image file')
  //res = rotestimate(cam_index,img1,img2,img3,img4,img5)
  //img6=imread('path of the image file')
  //res = rotestimate(cam_index,img1,img2,img3,img4,img5,img6)
  //
  //Parameters
  //cam_index : cam_index of the camera for which K is returned.
  //img1 : an image
  //img2 : an image
  //img3 : an image
  //img4 : an image
  //img5 : an image
  //img6 : an image
  //
  //Description
  //The images pass through a stiching pipeline.
  //Features are extracted from each image and matching is done on two consecutive images to ensure the continuity in images.
  //After this camera parameters are estimated.
  //
  //Examples
  //img1=imread('images/campus_000.jpg');
  //img2=imread('images/campus_001.jpg');
  //cam_index=1
  //res=rotestimate(1,img1,img2);
  //Authors
  //    Manoj Sree Harsha

[lhs rhs]=argn(0)
    	if lhs>1
        	error(msprintf(" Too many output arguments"))
    	elseif rhs>7
        	error(msprintf(" Too many input arguments,maximum number of arguments is 7"))
    	elseif rhs<3
        	error(msprintf("the function needs atleast 3 arguments"))
    	end
		for i=1:(rhs-1)
			varargin(i)=mattolist(varargin(i))
		end
		if rhs==3
       		res=raw_rotestimate(cam_index,varargin(1),varargin(2));
    	elseif rhs==4
       		res=raw_rotestimate(cam_index,varargin(1),varargin(2),varargin(3));
    	elseif rhs==5
       		res=raw_rotestimate(cam_index,varargin(1),varargin(2),varargin(3),varargin(4));
    	elseif rhs==6
       		res=raw_rotestimate(cam_index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5));
    	elseif rhs==7
       		res=raw_rotestimate(cam_index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
    	end
endfunction

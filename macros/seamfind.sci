// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: M Avinash Reddy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out] = seamfind(index,varargin)
	//Estimates the seams in input images (which are going to be stitched to form a panaroma)  
	//
	//Calling Sequence
	//stacksize('max')
	//img1=imread('path of the image file')
	//img2=imread('path of the image file')
	//out = seamfind(i,img1,img2)
	//img3=imread('path of the image file')
	//out = seamfind(i,img1,img2,img3)
	//img4=imread('path of the image file')
	//out = seamfind(i,img1,img2,img3,img4)
	//img5=imread('path of the image file')
	//out = seamfind(i,img1,img2,img3,img4,img5)
	//img6=imread('path of the image file')
	//out = seamfind(i,img1,img2,img3,img4,img5,img6)
	//
	//Parameters
	//index : index of the estimated image required by the user. It should be between 1 and number of input images. 
	//img1 : an image 
	//img2 : an image
	//img3 : an image
	//img4 : an image
	//img5 : an image
	//img6 : an image
	//
	//Description
	//This function estimates the seams in images which are going to be blended to form a panaroma image. The output of the function is the binary and of the updated mask and the 'index'th image.
	//
	//Examples
	//stacksize('max');
	//img1=imread('images/campus_017.jpg');
	//img2=imread('images/campus_016.jpg');
	//out=seamfind(1,img1,img2);
	//Authors
	//    M Avinash Reddy   
	
    	[lhs rhs]=argn(0)
    	if lhs>1
        	error(msprintf(" Too many output arguments"))
	end
    	if rhs>7
        	error(msprintf(" Too many input arguments,maximum number of arguments is 7"))
	end
    	if rhs<3
        	error(msprintf("the function needs atleast 3 arguments"))
    	end
	for i=1:rhs-1
		varargin(i)=mattolist(varargin(i))
	end
    	if rhs==3
    		res=raw_seamfind(index,varargin(1),varargin(2))
    	elseif rhs==4
       		res=raw_seamfind(index,varargin(1),varargin(2),varargin(3))
    	elseif rhs==5
       		res=raw_seamfind(index,varargin(1),varargin(2),varargin(3),varargin(4))
    	elseif rhs==6
       		res=raw_seamfind(index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5))
    	elseif rhs==7
       		res=raw_seamfind(index,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6))
    	end

	channel=size(res)
	for i = 1: channel
		out(:,:,i) = (res(i))
	end
	out=double(out)
endfunction

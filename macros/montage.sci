// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai , M Avinash Reddy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=montage(rows,cols,varargin)
	//Creates a montage of same sized input images 
	//
	//Calling Sequence
	//out = montage(1,1,img1)
	//out = montage(1,2,img1,img2)
	//out = montage(3,1,img1,img2,img3)
	//out = montage(2,2,img1,img2.img3,img4)
	//out = montage(1,5,img1,img2,img3,img4,img5)
	//out = montage(3,2,img1,img2,img3,img4,img5,img6)
	//
	//Parameters
	//rows : number of rows in montage 
	//cols : number of columns in montage
	//varargin : variable number of input images
	//
	//Description
	//The function creates a montage with the list of images. Montage is created row-wise with images taken in order from the list. Note that the following condition should hold true:- no_of_rows*no_of_cols >= no_of_images > (no_of_rows-1)*(no_of_cols)
	//
	//Examples
	//img1=imread('images/lena.jpg');
	//img2=imread('images/monkey.jpeg');
	//out=montage(1,2,img1,img2);
	//Authors
	//    Sukul Bagai , M Avinash Reddy 

	[lhs rhs]=argn(0)

	for i=1:rhs-2
		varargin(i)=mattolist(varargin(i))
	end	

	if rhs==3
		a=raw_montage(rows,cols,varargin(1));
	elseif rhs==4
		a=raw_montage(rows,cols,varargin(1),varargin(2));
	elseif rhs==5
		a=raw_montage(rows,cols,varargin(1),varargin(2),varargin(3));
	elseif rhs==6
		a=raw_montage(rows,cols,varargin(1),varargin(2),varargin(3),varargin(4)); 
	elseif rhs==7
		a=raw_montage(rows,cols,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5)); 
	elseif rhs==8
		a=raw_montage(rows,cols,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));  
	end
        dimension=size(a)
        for i = 1: dimension
             out(:,:,i)=(a(i));
        end
     	out=double(out)
endfunction;

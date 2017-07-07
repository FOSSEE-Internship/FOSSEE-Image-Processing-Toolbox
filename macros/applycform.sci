// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh,Tess  Zacharias
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [outImg] = applycform(srcImg,transform)
//Apply device-independent color space transformation.    
//
//Calling Sequence 
//[outImg]=applycform(srcImage,transform)
//
//Parameters
//outImg:Output transformed image same number of channels as Input,Depth 8U.
//srcImage:Input image
//transform:Transformation string given as inputColorspace2ouptutColorspace.Valid strings are-<itemizedlist><listitem><para>'xyz2lab'		:Convert from the XYZ to the L*a*b color space.</para></listitem><listitem><para>'lab2xyz'		:Convert from the L*a*b to the XYZ color space.</para></listitem><listitem><para>'srgb2xyz'	:Convert from the standard-RGB to the XYZ color space.</para></listitem><listitem><para>'xyz2srgb'	:Convert from the XYZ to the standard-RGB color space.</para></listitem><listitem><para>'srgb2lab'  	:Convert from the standard-RGB to the L*a*b color space.</para></listitem><listitem><para>'lab2srgb' 	:Convert from the L*a*b to the standard-RGB color space.</para></listitem><listitem><para>'xyz2uvl' 	:Convert from the XYZ to the uvL color space.</para></listitem><listitem><para>'uvl2xyz'		:Convert from the uvL to the XYZ color space.</para></listitem></itemizedlist>
//
//Description
//outImg = applycform(srcImage,transform) converts the color values in srcImage to the color space specified in the color transformation string transfrom.
//
//Examples
//im=imread("images/lena.jpeg",1);
//img=applycform(im,"srgb2xyz");
//imshow(img);
//
//Examples
//im=imread("images/lena.jpeg",1);
//img=applycform(im,"srgb2xyz");
//img=applycform(img,"xyz2uvl");
//img=applycform(img,"uvl2xyz");
//img=applycform(img,"xyz2srgb");
//imshow(img);
//
//Authors
//Gursimar Singh
//Tess  Zacharias

[lhs,rhs]=argn(0);

	if rhs>2 then
        error(msprintf("Too many input arguments"));
     end
     if rhs<2 then
	   error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	   error(msprintf("Too many output arguments"));
     end

	srcMat = mattolist(srcImg)
	if size(srcImg)==1 then
		error(msprintf("NUmber of channels of input image must be 3 or 4"));
	end

	out = raw_applycform(srcMat,transform)
	sz=size(out)
	for i=1:sz
		outImg(:,:,i)=out(i);
	end	
endfunction

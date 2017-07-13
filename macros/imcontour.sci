// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Tess Zacharias 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [contourMat] = imcontour(matA)
//Draws a contour plot for the input image
//
//Calling Sequence
//contourMat = imcontour(srcImg)
//
//Parameters
//srcImg : THe input image
//
//Description
//The function draws contour outlines in the image.
//
//Examples
//a = imread("lena.jpeg");
//k = imcontour(a);
//imshow(k)
//
//Examples
//a = imread("photo1.jpg");
//k = imcontour(a);
//imshow(k)
//
//Examples
//a = imread("bryan.jpg");
//k = imcontour(a);
//imshow(k)
//
//Authors
//Tess zacharias
//Shubham Lohakare	
	srcMat = mattolist(matA)

	out = raw_imcontour(srcMat)

	ch = size(out)

	for i=1:ch
		contourMat(:,:,i) = out(i)
	end
		
	
endfunction

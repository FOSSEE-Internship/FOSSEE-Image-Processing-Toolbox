// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=getrotationmatrix2d(centerX,centerY, doubleAngle, doubleScale)
//Calculate matrix of 2D rotation
//
//Calling Sequence
//[out]=getrotationmatrix2d(centerX,centerY, doubleAngle, doubleScale)
//
//Parameters
//out:2D Rotataion Matrix
//centerX: x-co-ordinate of center in the image
//centerY: y-co-ordinate of center in the image
//doubleAngle:Rotation angle in degrees.Positive value means counter-clockwise.
//doubleScale:Isotropic scale factor.
//
//Description
//Calculates matrix of 2D rotation
//
//Examples
//rot=getRotationMatrix2D(30,40,30,2);
//
//Authors
//Shubheksha Jalan

		[lhs,rhs]=argn(0);
		if lhs>1
			error(msprintf("Wrong number of output arguments"));
		elseif rhs>4
			error(msprintf(" Too many input arguments"));
		elseif rhs<4
			error(msprintf(" Too few input arguments"));
		end
         out= raw_getRotationMatrix2D(centerX,centerY, doubleAngle, doubleScale);
endfunction;

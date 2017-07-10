// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai , Abhilasha Sancheti
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [textSize,baseLine]= getTextSize(inputtext,fontface,fontscale,thickness)
	//Calculates the width and height of a text string
	//
	//Calling Sequence
	//[textSize,baseLine]= getTextSize(inputtext,fontface,fontscale,thickness)
	//
	//Parameters
	//inputtext : Input text string
	//fontface : Font type 
	//fontscale : Font scale factor that is multiplied by the font-specific base size 
	//thickness : Thickness of the lines used to draw a text
	//textSize : Output parameter- The size of a box that contains the specified text
	//baseLine : Output parameter- y-coordinate of the baseline relative to the bottom-most text point
	//
	//Description
	//The function getTextSize calculates and returns the size of a box that contains the specified text
	//Examples
	//[textSize,baseLine]= getTextSize('helloall','FONT_HERSHEY_PLAIN',1,1);
	//Authors
	//    Sukul Bagai , Abhilasha Sancheti  
    [textSize,baseLine]=raw_getTextSize(inputtext,fontface,fontscale,thickness)
endfunction

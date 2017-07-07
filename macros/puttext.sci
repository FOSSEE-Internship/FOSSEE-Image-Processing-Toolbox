// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function new_image = puttext(image, x1, y1, fontscale, r_value, g_value, b_value, text, fontface, varargin)
// The function putText renders the specified text string in the image. 
//
// Calling Sequence
// out = putText(img, x,y, fontScale, r,g,b,text,fontface);
// out = putText(img, x,y, fontScale, r,g,b,text,fontface,thickness);
// out = putText(img, x,y, fontScale, r,g,b,text,fontface,thickness,lineType);
// out = putText(img, x,y, fontScale, r,g,b,text,fontface,thickness,lineType, bottomLeftOrigin);
//
// Parameters
// img: Image.
// text: Text string to be drawn.
// x: x coordinate of the Bottom-left corner of the text string in the image.
// y: y coordinate of the Bottom-left corner of the text string in the image.
// font: CvFont structure initialized using InitFont().
// fontFace: Font type. One of FONT_HERSHEY_SIMPLEX, FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX, FONT_HERSHEY_COMPLEX, FONT_HERSHEY_TRIPLEX, FONT_HERSHEY_COMPLEX_SMALL, FONT_HERSHEY_SCRIPT_SIMPLEX, or FONT_HERSHEY_SCRIPT_COMPLEX, where each of the font ID’s can be combined with FONT_ITALIC to get the slanted letters.
// fontScale: Font scale factor that is multiplied by the font-specific base size.
// r,g,b: The rgb values of the text color as desired by the user.
// thickness: Thickness of the lines used to draw a text.
// lineType: Line type. See the line for details.
// bottomLeftOrigin: When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner.
 
//
// Description
// The function putText renders the specified text string in the image. Symbols that cannot be rendered using the specified font are replaced by question marks.
//
// Examples
// i = imread('lena.jpeg',0);
// ii = puttext(i,3,40,0.8,0,255,0,"hello","FONT_HERSHEY_SIMPLEX");

    
      [ lhs, rhs ] = argn(0)
	
	image_list = mattolist(image)
	
	select rhs
		case 9 then
			out = raw_puttext(image_list, x1, y1, fontscale, r_value, g_value, b_value, text, fontface)
		
		case 10 then
			out = raw_puttext(image_list, x1, y1, fontscale, r_value, g_value, b_value, text, fontface, varargin(1))
			
		case 11 then
			out = raw_puttext(image_list, x1, y1, fontscale, r_value, g_value, b_value, text, fontface, varargin(1), varargin(2))
		
		case 12 then
			out = raw_puttext(image_list, x1, y1, fontscale, r_value, g_value, b_value, text, fontface, varargin(1), varargin(2), varargin(3))
	end
	
	sz = size(out)
	
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end
	
endfunction

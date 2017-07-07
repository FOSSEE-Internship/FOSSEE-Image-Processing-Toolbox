// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function xyz = whitepoint(input_string)
// Returns a three element vector defining the illumination xyz values for different lighting conditions. 
//
// Calling Sequence
// xyz = whitepoint(string);
//
// Parameters
// string: The following are the possible values for this argument and their descriptions<itemizedlist><listitem><para> d65- CIE standard illuminant D65, [0.9504, 1.0000, 1.0888]. Reperesents noon daylight with correlated color temperature of 6504 K.</para></listitem><listitem><para>d50- CIE standard illuminant D50, [0.9642, 1.0000, 0.8251]. Represents warm daylight at sunrise or sunset with correlated color temperature of 5003 K.</para></listitem><listitem><para>a- CIE standard illuminant A, [1.0985, 1.0000, 0.3558]. Reperesents typical, domestic, tungsten-filament lighting with correlated color temperature of 2856 K. </para></listitem><listitem><para>c- CIE standard illuminant C, [0.9807, 1.0000, 1.1822]. Simulates average or north sky daylight with correlated color temperature of 6774 K. </para></listitem><listitem><para>icc- Profile Connection Space (PCS) illuminant used in ICC profiles. Approximation of [0.962, 1.000, 0.8249] using fixed-point, signed, 32-bit numbers with 16 fractional bits.  Actual value: [31595,32768, 27030]/32768. </para></listitem><listitem><para>c- CIE standard illuminant C, [0.9807, 1.0000, 1.1822]. Represents average or north sky daylight with correlated color temperature of 6774 K.</para></listitem><listitem><para>d55- CIE standard illuminant D55, [0.9568, 1.0000, 0.9214]. Represents mid-morning or mid-afternoon daylight with correlated color temperature of 5500 K. </para></listitem></itemizedlist>
// 
//
// Description
// This function is used to get a vector of xyz color space representation for different lighting 
// conditions which can be used to get images with different lighting properties.
//
// Examples
// xyz = whitepoint("a");
// img = imread("images/lena.jpeg");
// image = applycform(img, "srgb2xyz");
// temp1 = image(:, :, 1)
// temp3 = image(:, :, 3)
// image(:, :, 1) = temp3 * xyz(1);
// image(:, :, 2) = image(:, :, 2) * xyz(2);
// image(:, :, 3) = temp1 * xyz(3);
// imshow(image);
//
// See also 
// imshow
// imread
// applycform
// 
// Authors
// Tess Zacharias

	xyz = raw_whitepoint(input_string)
endfunction

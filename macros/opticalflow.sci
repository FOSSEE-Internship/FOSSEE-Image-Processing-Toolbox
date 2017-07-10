// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Manoj Sree Harsha & M Avinash Reddy
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [] = opticalflow(a)
  //Calculates optical flow.
  //
  //Calling Sequence
  //opticalflow(vid)
  //
  //Parameters
  //vid : a video
  //
  //Description
  //opticalflow(vid) returns an opticalflow video and it will be written to 'out.avi' file (In the current working directory).
  //
  //Examples
  //opticalflow('ball.mp4');
  //opticalflow('vtest.avi');
  //Authors
  //    Manoj Sree Harsha , M Avinash Reddy

  raw_opticalflow(a);

endfunction

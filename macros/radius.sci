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

function [y]=radius(q,f,r,varargin)
  //Finds all the points within the circle of radius r with centre as query point q, in the dataset f
  //
  //Calling Sequence
  //y = radius(q,f,r)
  //y = radius(q,f,r,algo_type)
  //
  //Parameters
  //q : a row vector containing the query point
  //f : f is the set of data points and the number of columns in f should be same as that of q to match point dimensionality
  //r : a positive integer specifying the radius.
  //algo_type : a integer between 1 and 4 (both inclusive) specifying the algorithm to be used for the radius search.
  //
  //Description
  //y = radius(q,f,r) return a 2x(Number of points) matrix .
  //The first row of y contains the indices of all the points in the dataset(f) within the circle of radius r.
  //The second row contains the corresponding distances(squared euclidean distance).
  //
  //Examples
  //q=[1 2 3];
  //f=[1 2 3;4 5 6;7 8 9];
  //r=30;
  //y=radius(q,f,r);
  //Authors
  //    Manoj Sree Harsha

[lhs rhs]=argn(0)

    if lhs>1
        error(msprintf(" Too many output arguments"))
    elseif rhs>4
        error(msprintf(" Too many input arguments,maximum number of arguments is 4"))
    elseif rhs<3
        error(msprintf("the function needs atleast 3 arguments"))
    end
 f1=mattolist(f);

 if rhs==3
   y=raw_radius(q,f1,r)
 elseif rhs==4
   y=raw_radius(q,f1,r,varargin(1))
 end

endfunction

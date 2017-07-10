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

function [out] = pclradius(pcloud,q,r)
	  //Finds all the points within the circle of radius r with centre as query point q, in the pointcloud pcloud
	  //
	  //Calling Sequence
	  //y = pclradius(pcloud,q,r)
	  //
	  //Parameters
	  //pcloud : pointCloud object.
	  //q : a row vector containing the query point
	  //r : a positive integer specifying the radius.
	  //out : a matrix. 
	  //
	  //Description
	  //out = pclradius(pcloud,q,r) return a 2x(Number of points) matrix .
	  //The first row of out contains the indices of all the points in the pointCloud within the circle of radius r.
	  //The second row contains the corresponding squared distances(squared euclidean distance).
	  //
	  //Examples
	  //a=pcread('data/cube.ply');
	  //q=[1 1 -1];
	  //r=10;
	  //out=pclradius(a,q,r);
	  //
	  //a=pcread('data/bun0.pcd');
	  //q=[0.5 0.5 0.5];
	  //r=2;
	  //out=pclradius(a,q,r);
	  //Authors
	  //    Manoj Sree Harsha

	data=pcloud.Location;
	out = raw_radsearch(data,q,r);

endfunction

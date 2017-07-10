// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: M Avinash Reddy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out] = pclknn(pcloud,q,k)
	//Finds the k nearest neighbours of the query point q, in the point cloud.
	//
	//Calling Sequence
	//pcloud=pcread('path of point cloud file')
	//out = pclknn(pcloud,q,k)
	//
	//
	//Parameters
	//pcloud : a point cloud structure read using pcread function
	//q : a three dimensional query point  
	//k : a positive integer specifying the number of nearest neighbours in pcloud for the query point p.
	//
	//Description
	//out = knn(q,f,k) returns a 2xk matrix .
	//The first row of out contains the indices of the k nearest points in the dataset(f) from the query point.
	//The second row contains the corresponding distances(squared euclidean distance).
	//
	//Examples
	//q=[0 0 0];
	//data=pcread('data/cube.ply')
	//k=3;
	//out=pclknn(data,q,k);
	//Authors
	//    M Avinash Reddy   

	if size(q)~=3
		error(msprintf("Size of the query point must be three"))
	end
	data=pcloud.Location
	out = knn(q,data,k)

endfunction

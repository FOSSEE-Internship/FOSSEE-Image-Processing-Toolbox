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

function [out]=knn(q,f,k,varargin)
	//Finds the k nearest neighbours of the query point q, in the dataset f 
	//
	//Calling Sequence
	//out = knn(q,f,k)
	//out = knn(q,f,k,algo_type)
	//
	//Parameters
	//q : a row vector containing the query point 
	//f : f is the set of data points and the number of columns in f should be same as that of q to match point dimensionality
	//k : a positive integer specifying the number of nearest neighbours in f for the query point p.
	//algo_type : a integer between 1 and 4 (both inclusive) specifying the algorithm to be used for the knn search.
	//
	//Description
	//out = knn(q,f,k) returns a 2xk matrix .
	//The first row of out contains the indices of the k nearest points in the dataset(f) from the query point.
	//The second row contains the corresponding distances(squared euclidean distance).
	//algo_type 1 corresponds to linear search algorithm.
	//algo_type 2 corresponds to search using a set of randomized kd-trees. 
	//algo_type 3 corresponds to search using hierarchical k-means tree.
	//algo_type 4 corresponds to combined search using both kd-trees and k-means algorithm.
	//
	//Examples
	//q=[1];
	//f=[1;2;10;-1];
	//k=3;
	//out=knn(q,f,k);
	//
	//Examples
	//q=[1,1];
	//f=[1,3;2,0;10,10;-1,1];
	//k=3;
	//algo_type=2;
	//out=knn(q,f,k,algo_type);
	//Authors
	//    M Avinash Reddy   
	
	[lhs rhs]=argn(0)

	if lhs>1
        	error(msprintf(" Too many output arguments"))
    	elseif rhs>4
        	error(msprintf(" Too many input arguments,maximum number of arguments is 4"))
    	elseif rhs<3
        	error(msprintf("the function needs atleast 3 arguments"))
    	end

	f1=mattolist(f)
	
	if rhs==3
		out=raw_knn(q,f1,k)
	elseif rhs==4
		out=raw_knn(q,f1,k,varargin(1))
	end

endfunction

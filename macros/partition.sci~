// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Rohit Suri, Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [varargout]=partition(imageSet,groupSizesOrPercentages,varargin)
// This function is used to split an imageSet.
//
// Calling Sequence
// imgSet = imageSet(location) //or imgSet = imageSet(location,'recursive') 
// [set1 set2 ... setN] = partition(imgSet, groupSizes)
// [set1 set2 ... setN] = partition(imgSet, groupPercentages)
// [set1 set2 ... setN] = partition(..., method)
//
// Parameters
// set: Output imageSet structure
// imgSet: Input imageSet to be split
// groupSizes: Matrix that specifies size of output sets
// groupPercentages: Matrix that specifies percentage content of each output set
// method: (Optional) Determines whether split is 'sequential' or 'randomized' (Default: 'sequential')
//
// Description
// This function splits an imageSet into two or more imageSets on the basis of the groupSizes or groupPercentages provided.
//
// Examples
// 
// imgSet = imageSet('/images/test','recursive');
// [set1 set2 set3] = partition(imgSet,[2 , 3], 'randomized');
//
// imgSet = imageSet('/images/test','recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
//
// imgSet = imageSet('/images/airplanes');
// [set1 set2] = partition(imgSet,[5]);
//
// imgSet = imageSet('/images/test','recursive');
// [trainingSet testSet] = partition(imgSet,[4], 'randomized');
//
// Authors
//  Rohit Suri
//  Rohan Gurve

    [lhs rhs]=argn(0);
    if rhs>3
        error(msprintf(" Too many input arguments"));
    end
    cols=size(groupSizesOrPercentages,'c');
    if lhs>cols+1 then
        error(msprintf(" Too many output arguments"));
    end
    imgSetList=imageSetToList(imageSet);
    
    select rhs
        case 2 then
            sets=raw_partition(imgSetList,groupSizesOrPercentages);
        case 3 then
            sets=raw_partition(imgSetList,groupSizesOrPercentages,varargin(1));
    end
    for i=1:lhs
        for j=1:length(sets(i)(3))
            for k=1:sets(i)(3)(j)
                imgLocations(k)=sets(i)(4)(j)(1,k);
            end    
         imgSet(1,j)=struct('Description',sets(i)(2)(j),'ImageLocation',imgLocations,'Count',int32(sets(i)(3)(j)) );
	     imgLocations=[]
        end    
     varargout(i) = imgSet;
    end
endfunction



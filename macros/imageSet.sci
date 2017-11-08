// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:Rohit Suri
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [imgSet]=imageSet(imageFolder,varargin)
// This function is used to create a collection of images.
//
// Calling Sequence
// imgSet = imageSet(location)
// imgSet = imageSet(location, 'recursive')
//
// Parameters
// imgSet: Structure containing collection of images
// location: Address of the folder from which collection is to be created
//
// Description
// This function creates an imageSet structure with the following attributes- Description, ImageLocation, and Count.
//
// Examples
// imgSet = imageSet(directory);
//
// imgSet = imageSet(directory,'recursive');
//
// Authors
// Rohit Suri

    [lhs rhs]=argn(0);
    if lhs>1
         error(msprintf(" Too many output arguments"));
    elseif rhs<1
        error(msprintf(" Not enough input arguments"));
    elseif rhs>2
        error(msprintf(" Too many input arguments"));
    end
    if rhs==1 then
        imgSetList=raw_imageSet(imageFolder);
        for i=1:imgSetList(3)
            imgLocations(i)=imgSetList(4)(1)(1,i);
        end
        imgSet=struct('Description',imgSetList(2),'ImageLocation',imgLocations,'Count',double(imgSetList(3)));
    else
        imgSetList=raw_imageSet(imageFolder,varargin(1));
        for i=1:length(imgSetList(3))
            for j=1:imgSetList(3)(i)
                imgLocations(j)=imgSetList(4)(i)(1,j);
            end
            imgSet(1,i)=struct('Description',imgSetList(2)(i),'ImageLocation',imgLocations,'Count',int32(imgSetList(3)(i)));
            imgLocations=[];
       end
    end
endfunction

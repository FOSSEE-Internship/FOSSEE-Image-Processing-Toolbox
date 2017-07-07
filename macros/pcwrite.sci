// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [] = pcwrite(pointCloud, filename, varargin)
// This function is used to write 3-D point cloud to PLY or PCD file.
//
// Calling Sequence
// [] = pcwrite(pointCloud, filename)
// [] = pcwrite(pointCloud, filename, fileFormat, fileType)
// 
// Parameters
// pointCloud : Object for storing point cloud, specified as a pointCloud object.
// filename : File name, specified as a character vector, specify the file name with an extension incase of two input argument.(default encoding is ASCII)
// fileFormat : The input file type must be a PLY or PCD format file.(choose between ".ply" or ".pcd")
// fileType : Choose from the following encoding, PLY - 'ascii', 'binary' and PCD - 'ascii', 'binary', or 'compressed'.
//
// Description
// Writes the point cloud object, ptCloud, to the PLY or PCD file specified by the input.
// 
// Examples
// // Write 3-D Point Cloud to PLY File
// ptCloud = pcread('teapot.ply');
// pcshow(ptCloud);
// pcwrite(ptCloud,'teapotOut','ply','binary');
//
// Examples
// // Write 3-D Point Cloud to PCD File
// ptCloud = pcread('teapot.ply');
// pcshow(ptCloud);
// pcwrite(ptCloud,'teapotOut','pcd','binary');
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela	
	if(strcmp(pointCloud.dataType,'PointCloud')~=0) then
		error(msprintf("function expects a PointCloud"))
	end
typeof(pointCloud.Width)
	[lhs, rhs] = argn(0)

	select rhs
		
		case 2 then			
			raw_pcwrite(pointCloud.Width, pointCloud.Height, pointCloud.is_dense, pointCloud.Location, pointCloud.Count, pointCloud.Color, pointCloud.XLimits, pointCloud.YLimits, pointCloud.ZLimits, filename)
		case 4 then
			raw_pcwrite(pointCloud.Width, pointCloud.Height, pointCloud.is_dense, pointCloud.Location, pointCloud.Count, pointCloud.Color, pointCloud.XLimits, pointCloud.YLimits, pointCloud.ZLimits, filename, varargin(1), varargin(2))
			
	end

endfunction

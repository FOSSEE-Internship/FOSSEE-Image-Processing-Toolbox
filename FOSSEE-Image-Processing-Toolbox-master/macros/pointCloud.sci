function [pc] = pointCloud(pointCord, varargin)

	[lhs rhs] = argn(0)

	if lhs > 1
		error(msprintf('Too many output arguments'));
	
	elseif rhs > 13 
        error(msprintf('Too many input arguments, maximum number of arguments is thirteen.\n'));
	
	elseif rhs < 1
        error(msprintf('The function needs atleast one argument.'));
	end

	

	if rhs == 1
		[location numberOfPoints] = raw_pointCloud(pointCord);
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints);
	
	elseif rhs == 3
		[location rgb numberOfPoints] = raw_pointCloud(pointCord, varargin(1), varargin(2));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb);
	
	elseif rhs == 5
		[location numberOfPoints rgb intensity] = raw_pointCloud(pointCord, varargin(1), varargin(2), varargin(3), varargin(4));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb, 'Intensity', intensity);
	
	elseif rhs == 7
		[location numberOfPoints rgb intensity normal] = raw_pointCloud(pointCord, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb, 'Normal', normal, 'Intensity', intensity);
	
	elseif rhs == 9
		[location numberOfPoints rgb intensity normal xlimit] = raw_pointCloud(pointCord, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb, 'Normal', normal, 'Intensity', intensity, 'XLimits', xlimit);
	
	elseif rhs == 11
		[location numberOfPoints rgb intensity normal xlimit ylimit] = raw_pointCloud(pointCord, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb, 'Intensity', normal, 'Normal', intensity, 'XLimits', xlimit, 'YLimits', ylimit);
	
	elseif rhs == 13
		[location numberOfPoints rgb intensity normal xlimit ylimit zlimit] = raw_pointCloud(pointCord, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12));
		pc = struct('dataType', 'PointCloud', 'Location', location, 'Count', numberOfPoints, 'Color', rgb, 'Normal', normal, 'Intensity', intensity, 'XLimits', xlimit, 'YLimits', ylimit, 'ZLimits', zlimit);
	end
endfunction

function [varargout] = BRISKPoints(inp)
// This function returns a BRISKPoints struct for storing BRISK interest points.
//
// Calling Sequence
//	 myStruct = BRISKPoints(Location of KeyPoints)
//
// Parameters
// results: BRISKPoints struct which contains Location, Orientation, Metric, Scale and Count.
// Location: Location of Interest Point.
//
// Description
// This function returns a BRISKPoints struct with default values. 
//
// Examples
// image = imread('sample.jpg');
// results = detectBRISKFeatures(image);
// myStruct = BRISKPoints(results.KeyPoints);
//
// Authors
//  Shashank Shekhar
	[ lhs, rhs ] = argn(0)
	if rhs > 1 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end
	[a b c d e] = ocv_BRISKPoints(inp);
	varargout(1) = struct('Location', a, 'Orientation', b, 'Metric', c ,'Scale', d, 'Count', e);	
endfunction

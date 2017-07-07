function varargout=SURFPoints(varargin)
// This function returns a SURFPoints struct for storing SURF interest points.
//
// Calling Sequence
//	 myStruct = SURFPoints(Location of Interest Points)
//
// Parameters
// results: SURFPoints struct which contains Location, Orientation, Metric, SignOfLaplacian, Scale and Count.
// Location: Location of Interest Point.
//
// Description
// This function returns a SURFPoints struct with default values. 
//
// Examples
// image = imread('sample.jpg');
// results = detectSURFFeatures(image);
// myStruct = SURFPoints(results.KeyPoints);
//
// Authors
//  Shashank Shekhar
    [lhs rhs]=argn(0);
    if lhs<1 then
         error(msprintf(" Not enough output arguments. One expected"))
    elseif lhs>1 then
         error(msprintf(" Too many output arguments. One expected"))
    end
    if rhs>1 then
    	error(msprintf(" Too many input arguments. One expected"))
	end
    [a b c d e f] = ocv_SURFPoints(varargin(1));
    varargout(1) = struct('KeyPoints', a, 'Orientation', b, 'Metric', c ,'SignOfLaplacian', d,'Scale', e, 'Count', f );
endfunction
    

function [ overlapRatio ] = bboxOverlapRatio(bboxA, bboxB, ratioType)
// Compute the overlap ratio between the bounding boxes A and B.
//
// Calling Sequence
// [ overlapRatio ] = bboxOverlapRatio(bboxA, bboxB);
// [ overlapRatio ] = bboxOverlapRatio(bboxA, bboxB, ratioType);
//
// Parameters
// bboxA: Bounding box A of the form [x, y, width, height];
// bboxB: Boundng box B of the form [x, y, width, height];
// ratioType (Optional Argument): Method to be used to compute the ratio. Set this to either 'Union' or 'Min'. Default computation method is set to 'Union'.
// overlapRatio: Overlap ratio between the bounding boxes A and B.
//
// Description
// Compute the overlap ratio between the bounding boxes A and B.
//
// Examples
// bboxA = [1 2 3 4];
// bboxB = bboxA + 100;
// overlapRatioMin = bboxOverlapRatio(bboxA, bboxB, 'Min');
// overlapRatioUnion = bboxOverlapRatio(bboxA, bboxB);
//
// Authors
// Tanmay Chaudhari

         [lhs rhs] = argn(0)
         if rhs>3 then
         error(msprintf("Too many input arguments"))
         elseif rhs==3 then
         a=opencv_bboxOverlapRatio(input1,input2,varargin(1))
         out=a
         elseif rhs==2 then
         a=opencv_bboxOverlapRatio(input1,input2)
         out=a  
         end
	
endfunction

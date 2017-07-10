// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tanmay Chaudhari
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

   function [ out ] = bboxOverlapRatio(bboxA, bboxB, varargin)
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
      // bboxA = [0 0 50 50];
      // bboxB = [0 0 200 200];
      // overlapRatioMin = bboxOverlapRatio(bboxA, bboxB, 'Min');
      // overlapRatioUnion = bboxOverlapRatio(bboxA, bboxB);
      //
      // Authors
      //      Tanmay Chaudhari

         [lhs rhs] = argn(0)
         if rhs>3 then
         error(msprintf("Too many input arguments"))
         elseif rhs==3 then
         a=raw_bboxOverlapRatio(bboxA,bboxB,varargin(1))
         out=a
         elseif rhs==2 then
         a=raw_bboxOverlapRatio(bboxA,bboxB)
         out=a  
         end
	
endfunction

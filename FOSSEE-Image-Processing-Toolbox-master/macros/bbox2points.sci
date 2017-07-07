function [ bbox ] = bbox2points(rectangle)
// Returns list of corner points of a rectangle.
//
// Calling Sequence
// bbox = bbox2points(rectangle);
//
// Parameters
// rectangle: A Nx4 matrix where each row is a rectangle of the form [x, y, width, height];
// points: Returns 4x2xN size matrix which contains all the 4 co-ordinates of each of the N bounding boxes.  
//
// Description
// List of corner points of a rectangle.
//
// Examples
// bbox = [1 2 3 4; 5 6 7 8];
// results = bbox2points(bbox);
//
// Authors
// Tanmay Chaudhari

        a=opencv_bbox2points(rectangle);
        out(:,:,1)=a(1);
	
endfunction

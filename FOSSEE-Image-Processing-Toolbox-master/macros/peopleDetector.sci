function bboxes = peopleDetector(image, varargin)
// Detects people in an image
//
// Calling Sequence
// [bboxes] = peopleDetector(image)
// [bboxes] = peopleDetector(image, ["Parameter1", value1, ["Parameter2", value2 ... ]])
//
// Parameters
// image : input image 
// hitThreshold : threshold for distance between features and SVM classifying plane. Default value is 0.
// winStride : Window stride. Multiple of block stride. Default value (8, 8)
// padding : Default value (16, 16).
// scale : Coefficient of the detection window increase. Default value 1.05
// finalThreshold : Coefficient to regulate the similarity threshold. Some people may cover more boxex. To regulate that it is used. Default value 2.
// useMeanshiftGrouping : Default value false.
// bboxes : M * 4 matrix denoting M bounding boxes for detected people
//
// Description
// The peopleDetector function detects standing people in the image using the Histogram of Oriented Gradient (HOG) features and a trained Support Vector Machine
// (SVM) classifier. It detects standing peole in the image. It returns M * 4 matrix having M detected people in the image. Each row of matrix contains
// [x, y, width, height] field. x, y represent the upper left corner point of the bounding box. width and height represent the width and height of the bounding box
// respectively. 
//
// Examples
// I = imread('standing.jpg')
// [bboxes] = peopleDetector(I)
// I = imread('standing.jpg')
// [bboxes] = peopleDetector(I, "scale", 1.02)
//
// Authors
// Suraj Prakash



	[ lhs rhs ] = argn(0)
	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end
	
	if rhs > 13 then
		error(msprintf("Too many input arguments"))
	end
	
	if modulo(rhs, 2) <> 1 then
		error(msprintf("Wrong number of input arguments"))
	end
	
	image_list = mattolist(image)
	
	if rhs == 1 then
		
		bboxes = opencv_peopleDetector(image_list)
		
	elseif rhs == 3 then
		
		bboxes = opencv_peopleDetecotr(image_list, varargin(1), varargin(2))
	
	elseif rhs == 5 then
		
		bboxes = opencv_peopleDetector(image_list, varargin(1), varargin(2), varargin(3), varargin(4))
	
	elseif rhs == 7 then
		
		bboxes = opencv_peopleDetector(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
	
	elseif rhs == 9 then
		
		bboxes = opencv_peopleDetector(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varagin(7), varagin(8))
		
	elseif rhs == 11 then
		
		bboxes = opencv_peopleDetector(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10))
	
	elseif rhs == 13 then
		bboxes = opencv_peopleDetector(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12))
	end
	
endfunction

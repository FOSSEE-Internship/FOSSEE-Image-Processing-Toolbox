function [varargout] = evaluateImageRetrieval(image, IndexImage, ExpectedID, varargin)
// This function is used to evaluate the Image Search Results
//
// Calling Sequence
// 	averagePrecision = evaluateImageRetrieval(queryImage, ImageIndex, expectedIDs, Name, Value... );
//	[averagePrecision imageID Scores] = evaluateImageRetrieval(queryImage, ImageIndex, expectedIDs);
//
// Parameters
//	queryImage: The query image, for which the similar image has to be found. Can be a grayscale or a RGB image
//	ImageIndex: imageIndex object that contains the data set of all the images to be compared
//	expectedIDs: A row or column vector containing the IDs of expected Similarity
//	NumResults [Optional Input Argument]: Maximum number of results to be returned. Value: any integer (20 default)
//	ROI [Optional Input Argument]: Query Image search region. Format [ x y width height ]. Default: [1 1 size(Image,2) size(Image,1)]
//	averagePrecision: Average Precision Metric. Value in the range [0 1]
//	imageID: M-by-1 vector consisting of Ranked Index of retrieved Images
//	Scores: M-by-1 vector containing the similarity metric in the range 0 to 1
//
// Description
// It returns the average precision metric for measuring the accuracy of image search results for the queryImage.
//
// Examples
// 		imgSet = imageSet(directory,'recursive');
//    [trainingSet testSet] = partition(imgSet,[0.8]);
//    bag = bagOfFeatures(trainingSet);
//    imageindex = indexImages(trainingSet, bag);
//		queryImage = imread('sample.jpg');
//		imageIDs = retrieveImages(queryImage, imageindex);
//		exp_id = [3 4 1 2];			/*For a 4 element image set*/
//		precision = evaluateImageRetreival(queryImage, imageindex, exp_id);
//
// Authors
//  Umang Agrawal
//	Rohit Suri

	/// varargout(1) = average_precision
	/// varargout(2) = index
	/// varargout(3) = score
	[ lhs rhs ] = argn(0)
	if rhs > 7 then
		error(msprintf("Too many input arguments"))
	end

	if lhs > 3 then
		error(msprintf("Too many output arguments"))
	end
	image_list = mattolist(image)

	if lhs == 1 then
		select rhs
			case 3 then

				average_precision= opencv_evalutateImageRetrieval(image_list, IndexImage, ExpectedID)

			case 5 then

				average_precision = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2))

			case 7 then

				average_precision = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2), varargin(3), varargin(4))
		end

		varargout(1) = average_precision

	elseif lhs == 2 then
		select rhs
			case 3 then

				[average_precision, index] = opencv_evalutateImageRetrieval(image_list, IndexImage, ExpectedID)

			case 5 then

				[average_precision, index] = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2))

			case 7 then

				[average_precision, index] = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2), varargin(3), varargin(4))
		end

		varargout(1) = average_precision
		varargout(2) = index

	elseif lhs == 3 then
		select rhs
			case 3 then

				[average_precision, index, score] = opencv_evalutateImageRetrieval(image_list, IndexImage, ExpectedID)

			case 5 then

				[average_precision, index, score] = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2))

			case 7 then

				[average_precision, index, score] = opencv_evaluateImageRetrieval(image_list, IndexImage, ExpectedID, varargin(1), varargin(2), varargin(3), varargin(4))
		end

		varargout(1) = average_precision
		varargout(2) = index
		varargout(3) = score

	end

endfunction

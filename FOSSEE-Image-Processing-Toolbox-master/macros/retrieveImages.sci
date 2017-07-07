function [varargout] = retrieveImages(QueryImage, IndexImage, varargin)
// This function searches for similar image in a given image set
//
// Calling Sequence
// [imageID Score ImageWord] = retrieveImage(QueryImage, IndexImage, Name, Value... )
//
// Parameters
// 	QueryImage: The query image, for which the similar image has to be found. Can be a grayscale or a RGB image
//	IndexImage: imageIndex object that contains the data set of all the images to be compared
//	NumResults [Optional Input Argument]: Maximum number of results to be returned. Value: any integer (20 default)
//	ROI [Optional Input Argument]: Query Image search region. Format [ x y width height ]. Default: [1 1 size(Image,2) size(Image,1)]
//	imageID: M-by-1 vector consisting of Ranked Index of retrieved Images
//	Score: M-by-1 vector containing the similarity metric in the range 0 to 1
//	ImageWord: Visual Word mapped object for the Query Image
//
// Description
// Returns the indices corresponding to Images within imageIndex that are visually similar to the query Image
//
// Examples
// 		imgSet = imageSet(directory,'recursive');
//    [trainingSet testSet] = partition(imgSet,[0.8]);
//    bag = bagOfFeatures(trainingSet);
//    imageindex = indexImages(trainingSet, bag);
//		queryImage = imread('sample.jpg');
//		imageIDs = retrieveImages(queryImage, imageindex);
//
//	With Optional Arguments:
// 		imageIDs = retrieveImages(queryImage, imageindex, "NumResults", 10);
//
// Authors
//  Umang Agrawal
//	Rohit Suri

	/// varargout(1) = index
	/// varargout(2) = score
	/// varargout(3) = Imageword

	[ lhs rhs ] = argn(0)

	if rhs > 6 then
		error(msprintf("Too many input arguments"))
	end

	if lhs > 3 then
		error(msprintf("Too many output arguments"))
	end

	QueryImage_list = mattolist(QueryImage)

	if lhs == 1 then
		select rhs
			case 2 then

				index = opencv_retrieveImages(QueryImage_list, IndexImage)

			case 4 then

				index = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2))

			case 6 then
				index = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2), varagin(3), varargin(4))
		end

		varargout(1) = index

	elseif lhs == 2 then
		select rhs
			case 2 then

				[index, score] = opencv_retrieveImages(QueryImage_list, IndexImage)

			case 4 then

				[index, score] = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2))

			case 6 then
				[index, score] = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2), varagin(3), varargin(4))
		end

		varargout(1) = temp
		varargout(2) = score

	elseif lhs == 3 then
		select rhs
			case 2 then

				[index, score, Imageword] = opencv_retrieveImages(QueryImage_list, IndexImage)

			case 4 then

				[index, score, Imageword] = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2))

			case 6 then
				[index, score, Imageword] = opencv_retrieveImages(QueryImage_list, IndexImage, varargin(1), varargin(2), varagin(3), varargin(4))
		end

		varargout(1) = index
		varargout(2) = score
		del = struct("WordIndex", Imageword(1), "Location", Imageword(2), "Vocab_size", Imageword(3), "Count", Imageword(4))
		varargout(3) = del
	end

endfunction

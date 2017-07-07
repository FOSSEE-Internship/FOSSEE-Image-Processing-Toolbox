function classifierDescription = predictdtree(dtreeclassifier, image)
// This function is used to predict the category of an image using an image classifier.
//
// Calling Sequence
// label = predict(classifier, image)
//
// Parameters
// label: Evaluated label of the input image
// classifier: Image category classifier
// image: Input image
// k: Neighbours to consider while voting to decide the nearest neighbour
//
// Description
// This function predicts the class of an image based on the dtrees classifier.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// dtreeClassifier = traindtreeClassifier(trainingSet, bag);
// image = imread('sample.jpg');
// label = predict(dtreeClassifier, image);
//
// Author
// Shreyash Sharma
	
	image_list = mattolist(image)
	dtreeclassifier_list = classifierToList(dtreeclassifier);
	
	classifierDescription = raw_predictdtree(dtreeclassifier_list, image_list)
	
endfunction

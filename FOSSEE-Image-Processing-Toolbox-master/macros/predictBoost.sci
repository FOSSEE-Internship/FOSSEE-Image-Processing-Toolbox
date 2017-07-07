function classifierDescription = predictBoost(Boostclassifier, image)
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
// This function predicts the class of an image based on the Boost classifier.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// BoostClassifier = trainBoostClassifier(trainingSet, bag);
// image = imread('sample.jpg');
// label = predict(BoostClassifier, image);
//
// Author
// Shreyash Sharma
	
	image_list = mattolist(image)
	Boostclassifier_list = classifierToList(Boostclassifier);
	
	classifierDescription = raw_predictBoost(Boostclassifier_list, image_list)
	
endfunction

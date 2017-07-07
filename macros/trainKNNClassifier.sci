// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function classifier = trainKNNClassifier(imgSets, bag, classifierName, varargin)
// This function is used to train an image classifier using the KNN algorithm.
//
// Calling Sequence
// classifier = trainKNNClassifier(imgSets, bag, classifierName)
// classifier = trainKNNClassifier(imgSets, bag, classifierName, algorithmType)
//
// Parameters
// classifier: Image category classifier
// imgSets: Input imageSet to train the classifier on
// bag: The bagOfFeatures of the imageSet provided
// algorithmType: 
//
// Description
// This function trains a KNN classifier which can be used to predict classes of images given to it as
// input using the predictKNN() function.
//
// Examples
// imgSet = imageSet('images/trainset_2/','recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// KNNClassifier = trainKNNClassifier(trainingSet, bag, classifierName);
//
// Examples
// imgSet = imageSet('images/trainset_3/','recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// algorithmType = 1;
// KNNClassifier = trainKNNClassifier(trainingSet, bag, classifierName, algorithmType);
// save("var.dat", "KNNClassifier");
//
// See also
// imageSet
// partition
// bagOfFeatures
// mlPredict
// save
//
// Authors
// Siddhant Narang
	
	bag_list = bagStructToList(bag);
	imgSets_list = imageSetToList(imgSets);

	// Handling variable arguments.
	[lhs rhs] = argn(0)
	if lhs > 1
		error(msprintf("Too many output arguments"));
	elseif rhs < 3
		error(msprintf("Not enough input arguments"));
	elseif rhs > 4
		error(msprintf("Too many input arguments"));
	end
	if rhs == 3
		temp = raw_trainKNNClassifier(imgSets_list, bag_list, classifierName);
	elseif rhs == 4
		temp = raw_trainKNNClassifier(imgSets_list, bag_list, classifierName, varargin(1));
	end
	
	classifier = struct("ClassifierType", temp(1), "ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))

endfunction

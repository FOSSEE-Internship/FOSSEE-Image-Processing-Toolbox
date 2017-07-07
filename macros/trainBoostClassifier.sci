// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function classifier = trainBoostClassifier(imgSets, bag, classifierName, varargin)
// This function is used to train an image classifier.
//
// Calling Sequence
// imgSet = imageSet(directory,'recursive');
// 'or'
// imgSet = imageSet(image);
// bag = bagOfFeatures(imgSet);
// classifier = trainBoostClassifier(imgSets, bag);
// classifier = trainBoostClassifier(imgSets, bag,'nameclass')
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save)
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save,method)
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save,method,weak count)
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save,method,weak count,weight trim rate)
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save,method,weak count,weight trim rate,use Surrogates)
// classifier = trainBoostClassifier(imgSets, bag,'nameclass',save,method,weak count,weight trim rate,use Surrogates,maximum depth)
// [classifier,BagofFeaturesLocation,Description] = trainBoostClassifier() 
//
// Parameters
// classifier: Image category classifier location.
// BagofFeaturesLocation : location of the xml or yml file.
// Description : features obtained after training.
// imgSets: Input imageSet to train the classifier on.
// bag: The bagOfFeatures of the imageSet provided.
// image: The set of images used for creating the imageset used for training.
// nameclass: Name of the classifier one wants for their bag of features .xml or .yml file.
// save:Option for the users to save their training data.
// method: The type of boost method the user wants to use.For best results use the value 2.
// weak count:The number of weak classifiers. Default value is 100.
// weight trim rate:A threshold between 0 and 1 used to save computational time. Samples with summary weight ≤1−weighttrimrate do not participate in the next iteration of training. Set this parameter 
//                    to 0 to turn off this functionality. Default value is 0.95.
// use Surrogates: If true then surrogate splits will be built. These splits allow to work with missing data and compute variable importance correctly. Default value is false.
// maximum depth : The maximum possible depth of the tree. That is the training algorithms attempts to split a node while its depth is less than maxDepth. The root node has zero depth. The 
//                 actual depth may be smaller if the other termination criteria are met (see the outline of the training procedure here), and/or if the tree is pruned. Default value is INT_MAX.
//
// Description
// This function trains a Boost classifier which can be used to predict classes of images given to it as
// input using the predictBoost() function.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = trainBoostClassifier(trainingSet, bag);
//
	
	
	bag_list = bagStructToList(bag);
	imgSets_list = imageSetToList(imgSets);

	// Handling variable arguments.
	[lhs rhs] = argn(0)
	if lhs > 1
		error(msprintf("Too many output arguments"));
	elseif rhs < 2
		error(msprintf("Not enough input arguments"));
	elseif rhs > 9
		error(msprintf("Too many input arguments"));
	end
	if rhs == 3
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName);
	elseif rhs == 4
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1));
	elseif rhs == 5
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2));
        elseif rhs == 6
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3));
        elseif rhs == 7
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4));
        elseif rhs == 8
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5));
        elseif rhs == 9
		temp = raw_trainBoostClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6));
        end
	
	classifier = struct("ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))
	
endfunction

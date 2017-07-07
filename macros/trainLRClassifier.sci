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

function classifier = trainLRClassifier(imgSets, bag, classifierName, varargin)
// This function is used to train an image classifier using the LR algorithm.
//
// Calling Sequence
// classifier = trainLRClassifier(imgSets, bag, classifierName)
// classifier = trainLRClassifier(imgSets, bag, classifierName, learningRate)
// classifier = trainLRClassifier(imgSets, bag, classifierName, learningRate, iteration)
// classifier = trainLRClassifier(imgSets, bag, classifierName, learningRate, iteration, regularization)
// classifier = trainLRClassifier(imgSets, bag, classifierName, learningRate, iteration, regularization, trainMethod)
// classifier = trainLRClassifier(imgSets, bag, classifierName, learningRate, iteration, regularization, trainMethod, minibatch)
//
// Parameters
// classifier: Image category classifier
// imgSets: Input imageSet to train the classifier on
// bag: The bagOfFeatures of the imageSet provided
// learningRate: Defines the rate at which the classifier will learn.
// iteration: Number of iterations the training function will perform.
// regularization: Controls the kind of regularization to be applied. The types are<itemizedlist><listitem>REG_DISABLE- Regularization disabled, flag value = -1.</listitem><listitem>REG_L1- L1 norm, flag value = 0.</listitem><listitem>REG_L2- L2 norm, flag value = 1.</listitem></itemizedlist>
//
// trainMethod: Controls the kind of training method to be applied. The types are<itemizedlist><listitem>BATCH- flag value = 1.</listitem><listitem>MINI_BATCH- flag value = 0.</listitem></itemizedlist>
//
// minibatch: Specifies the number of training samples taken in each step of Mini-Batch Gradient Descent. 
//            Will only be used if trainMethod flag value is set to 0 training algorithm. 
//			  It has to take values less than the total number of training samples.
//
//
// Description
// This function trains a LR classifier which can be used to predict classes of images given to it as
// input using the predictLR() function.
//
// Examples
// imgSet = imageSet('images/trainset_2/','recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// lrclassi = trainLRClassifier(im, bag, "lrclassi", 1, 150, 0, 1, 5);
//
// Examples
// imgSet = imageSet('images/trainset_3/','recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// lrclassi = trainLRClassifier(im, bag, "lrclassi", 1, 150, 0);
// save("var.dat", "lrclassi");
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
	elseif rhs > 8
		error(msprintf("Too many input arguments"));
	end
	if rhs == 3
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName);
	elseif rhs == 4
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName, varargin(1));
	elseif rhs == 5
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2));
	elseif rhs == 6
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3));
		msprintf("6 arguments");
	elseif rhs == 7
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4));
	elseif rhs == 8
		temp = raw_trainLRClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5));
	end
	
	classifier = struct("ClassifierType", temp(1), "ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))
	
endfunction

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

function classifier = trainANNMLPClassifier(imgSets, bag, classifierName, varargin)
// This function is used to train an image classifier.
//
// Calling Sequence
// imgSet = imageSet(directory,'recursive');
// 'or'
// imgSet = imageSet(image);
// bag = bagOfFeatures(imgSet);
// classifier = trainANNClassifier(imgSets, bag);
// classifier = trainANNClassifier(imgSets, bag,'nameclass')
// classifier = trainANNClassifier(imgSets, bag,'nameclass',save)
// classifier = trainANNClassifier(imgSets, bag,'nameclass',save,alpha,nlayers)
// classifier = trainANNClassifier(imgSets, bag,'nameclass',save,alpha,nlayers,nmeth)
// classifier = trainANNClassifier(imgSets, bag,'nameclass',save,alpha,nlayers,nmeth,param1)
// classifier = trainANNClassifier(imgSets, bag,'nameclass',save,alpha,nlayers,nmeth,param1,param2)
// [classifier,BagofFeaturesLocation,Description] = trainANNClassifier() 
//
// Parameters
// classifier: Image category classifier location.
// BagofFeaturesLocation : location of the xml or yml file.
// Description : features obtained after training.
// imgSets: Input imageSet to train the classifier on.
// bag: The bagOfFeatures of the imageSet provided.
// image: The set of images used for creating the imageset used for training.
// nameclass: Name of the classifier one wants for their bag of features .xml or .yml file
// save:Option for the users to save their training data.
// aplha: The first parameter of the activation function, αlpha. Default value is 0.
// beta : The second parameter of the activation function, β. Default value is 0.
// nlayers : Describes the number of layers present.Used for handling the layersize vector and subsequently passing it in the opencvfunction.
// nmeth : Current training method used.
// param1 : Passed to setRpropDW0 for ANN_MLP::RPROP and to setBackpropWeightScale for ANN_MLP::BACKPROP.
// param2 : Passed to setRpropDWMin for ANN_MLP::RPROP and to setBackpropMomentumScale for ANN_MLP::BACKPROP.
//
// Description
// This function trains a ANN classifier which can be used to predict classes of images given to it as
// input using the predictANNMLP() function.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = trainANNMLPClassifier(trainingSet, bag);
//	
	
	bag_list = bagStructToList(bag);
	imgSets_list = imageSetToList(imgSets);

	// Handling variable arguments.
	[lhs rhs] = argn(0)
	if lhs > 1
		error(msprintf("Too many output arguments"));
	elseif rhs < 3
		error(msprintf("Not enough input arguments"));
	elseif rhs > 10
		error(msprintf("Too many input arguments"));
	end
	if      rhs == 3
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName);
	elseif rhs == 4
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1));
	elseif rhs == 4
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2));
        elseif rhs == 6
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3));
        elseif rhs == 7
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4));
        elseif rhs == 8
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5));
        elseif rhs == 9
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6));
        elseif rhs == 10
		temp = raw_trainANNMLPClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7));
	end
	
	classifier = struct("ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))
	
endfunction

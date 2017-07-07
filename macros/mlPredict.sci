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

function varargout = mlPredict(classifier, image, modelName, varargin)
// This function is used to predict the class of an image using an image classifier.
//
// Calling Sequence
// label = mlpredict(classifier, image, modelName)
// label = mlpredict(classifier, image, modelName, numberOfNeighnours) // only in case of KNN
//
// Parameters
// label: Evaluated label of the input image
// classifier: Image classifier
// image: Input image
// numberOfNeighbours: Number of neighbours to consider during prediction.
// Returns: Label
//
// Description
// This function predicts the class of an image based on the classifier provided.
//
// Examples
// load("argset3.dat", "knnclassi3"); // Use the scilab load function to load a trained classifier.
// img = imread("bike.jpg");
// resp = mlPredict(knnclassi3, img, "knn", 5);
// img = imread("car.jpg");
// resp1 = mlPredict(knnclassi3, img, "knn", 5);
//
// Examples
// load("emclassi4.dat", "emclassi4"); // Use the scilab load function to load a trained classifier.
// img = imread("images/bike.jpg");
// [prob, resp] = mlPredict(emclassi4, img, "em");
// img = imread("images/plane.jpg");
// [prob1, resp1] = mlPredict(emclassi4, img, "em");
//
// See also
// imread
// load
//
// Authors
// Siddhant Narang

	image_list = mattolist(image)
	classifier_list = classifierToList(classifier);

    select modelName
 	case "SVM" then
		varargout = raw_predictSVM(classifier_list, image_list);
	case "svm" then
		varargout = raw_predictSVM(classifier_list, image_list);
    
    case "SVMSGD" then
		varargout = raw_predictSVMSGD(classifier_list, image_list);
	case "svmsgd" then
    	varargout = raw_predictSVMSGD(classifier_list, image_list);
  	
	case "EM" then
    	varargout = raw_predictEM(classifier_list, image_list);
	case "em" then
    	varargout = raw_predictEM(classifier_list, image_list);
    
	case "LR" then
    	varargout = raw_predictLR(classifier_list, image_list);
	case "lr" then
    	varargout = raw_predictLR(classifier_list, image_list);
	
	case "KNN" then
    	varargout = raw_predictKNN(classifier_list, image_list, varargin(1));
	case "knn" then
    	varargout = raw_predictKNN(classifier_list, image_list, varargin(1));
	
	case "RT" then
    	varargout = raw_predictRT(classifier_list, image_list);
	case "rt" then
    	varargout = raw_predictRT(classifier_list, image_list);
	
	case "ANN" then
    	varargout = raw_predictANN(classifier_list, image_list);
	case "ann" then
    	varargout = raw_predictANN(classifier_list, image_list);
	
	case "DTree" then
    	varargout = raw_predictdtree(classifier_list, image_list);
	case "dtree" then
    	varargout = raw_predictdtree(classifier_list, image_list);

	case "Boost" then
    	varargout = raw_predictBoost(classifier_list, image_list);
	case "boost" then
    	varargout = raw_predictBoost(classifier_list, image_list);
    
    case "NB" then
        classifierDescription = raw_predictNB(classifier_list, image_list);
    case "nb" then
        classifierDescription = raw_predictNB(classifier_list, image_list);

    else
      mprintf("\nThe given modelName-%s is invalid\n", modelName);
    end	
endfunction

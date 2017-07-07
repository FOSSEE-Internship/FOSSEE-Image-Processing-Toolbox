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

function param = getParams(classifier, modelName)
// This function is used view the parameters of a trained classifier.
//
// Calling Sequence
// param = getParams(classifier, modelName);
//
// Parameters
// classifier: Image category classifier
// modelName: Name of the model to which the classifier belongs to.
//
// Description
// This function can be used to view the parameters of a trained classifier and
// make changes accordingly in the training process to get accurate results. 
//
// Examples
// load("argset3.dat", "knnclassi3"); // Use the scilab load function to load a trained classifier.
// params = getParamsKNN(knnclassi3, "KNN")
//
// Examples
// load("emclassi4.dat", "emclassi4"); // Use the scilab load function to load a trained classifier.
// params = getParams(emclassi4, "EM");
//
// See also
// trainEMClassifier
// trainLRClassifier
// trainKNNClassifier
// trainNBClassifier
// trainSVMClassifier
// trainSVMSGDClassifier
// trainANNClassifier
// trainRTreesClassifier
// trainDTreesClassifier
//
// Authors
// Siddhant Narang

classifier_list = classifierToList(classifier);
	
	[lhs, rhs] = argn(0);
	if lhs > 1
		error(msprintf("Too many output arguments"));
	elseif rhs < 2
		error(msprintf("Not enough input arguments"));
	elseif rhs > 2
		error(msprintf("Too many input arguments"));
	end

    select modelName
// 	case "SVM" then
//		temp = raw_getParamsSVM(classifier_list);
//		param = struct();
//	case "svm" then
//		temp = raw_getParamsSVM(classifier_list);
//		param = struct();
    
//  case "SVMSGD" then
//		temp = raw_getParamsSVMSGD(classifier_list, image);
//		param = struct();
//	case "svmsgd" then
//    	temp = raw_getParamsSVMSGD(classifier_list, image);
//		param = struct();
  	
	case "EM" then
    	temp = raw_getParamsEM(classifier_list);
		param = struct("No. of Clusters", temp(1), "Means", temp(2), "Weights", temp(3));
	case "em" then
   		temp = raw_getParamsEM(classifier_list);
		param = struct("No. of Clusters", temp(1), "Means", temp(2), "Weights", temp(3));
    
	case "LR" then
		temp = raw_getParamsLR(classifier_list);
		param = struct("Iterations", temp(1), "Learning Rate", temp(2), "MiniBatchSize", temp(3), "Regularization", temp(4), "Train Method", temp(5), "Learnt Thetas", temp(6));
	case "lr" then
    	temp = raw_getParamsLR(classifier_list);
		param = struct("Iterations", temp(1), "Learning Rate", temp(2), "MiniBatchSize", temp(3), "Regularization", temp(4), "Train Method", temp(5), "Learnt Thetas", temp(6));
	
	case "KNN" then
   		temp = raw_getParamsKNN(classifier_list);
		param = struct("Algorithm Type", temp(1), "No of Neighbours", temp(2), "Emax", temp(3));
	case "knn" then
    	temp = raw_getParamsKNN(classifier_list);
		param = struct("Algorithm Type", temp(1), "No of Neighbours", temp(2), "Emax", temp(3));
	
//	case "RT" then
//    	temp = raw_getParamsRT(classifier_list);
//	param = struct("No. of Active Variables", temp(1), "CVfolds", temp(2), "maxCatgories", temp(3),"maxDepth", temp(4),"minSample", temp(5),"reg accuracy", temp(6),"isPruned", temp(7),"UseSurrogates", temp(8),"UseSE1Rule", temp(9));
//	case "rt" then
//    	temp = raw_getParamsRT(classifier_list);
//		param = struct("No. of Active Variables", temp(1), "CVfolds", temp(2), "maxCax	xtgories", temp(3),"maxDepth", temp(4),"minSample", temp(5),"reg accuracy", temp(6),"isPruned", temp(7),"UseSurrogates", temp(8),"UseSE1Rule", temp(9));
	
//	case "ANN" then
//    	temp = raw_getParamsANN(classifier_list);
//	param = struct();
//	case "ann" then
//    	temp = raw_getParamsANN(classifier_list);
	
//	case "DTree" then
//    	temp = raw_getParamsDTree(classifier_list);
//	case "dtree" then
//    	temp = raw_getParamsDTree(classifier_list);
//		param = struct();
    
//	case "Prob" then
//      temp = raw_getParamsNB(classifier_list,image_list);
//		param = struct("Active Variables", temp(1));				
//	case "prob" then
//      temp = raw_getParamsNB(classifier_list,image_list);
//		param = struct("Active Variables", temp(1));

    else
      mprintf("\nThe given modelName-%s is invalid\n", modelName);
    end	
endfunction

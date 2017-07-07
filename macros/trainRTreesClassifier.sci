// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh, Rohit Suri & Umang Agrawal
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [classifier] = trainRTreesClassifier(imgSets,bag,classifierName,varargin)
// This function is used to train an image classifier using Random Forest Trees.
//
// Calling Sequence
// classifier = trainRtreesClassifier(imgSets, bag,classifierName);
// classifier = trainRtreesClassifier(imgSets, bag,classifierName,ActiveVarCount);
// classifier = trainRtreesClassifier(imgSets, bag,classifierName,ActiveVarCount,CalVarimp);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth,MinSampleCount,[priori]);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth,MinSampleCount,[priori],RegressionAccuracy);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth,MinSampleCount,[priori],RegressionAccuracy,TruncatedPruneTree);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth,MinSampleCount,[priori],RegressionAccuracy,TruncatedPruneTree,UseSurrogates);
// classifier = trainRtreesClassifier(imgSets,bag,classifierName,ActiveVarCount,CalVarImp,CVfolds,MaxCategories,MaxDepth,MinSampleCount,[priori],RegressionAccuracy,TruncatedPruneTree,UseSurrogates,Use1SERule);
//
// Parameters
// classifier: Image category classifier
// imgSets: Input imageSet to train the classifier on
// bag: The bagOfFeatures of the imageSet provided
// int ActiveVarCount: user defined default is 0
// bool CalVarimp: decision to calculate VarImp 
// int  CVfolds :  default is 10 user dependent   
// int MaxCategories:default 10 
// int MaxDepth:default value is INT_MAX
// int MinSampleCount:default value is 10
// float priori:default is empty – predefined probability of class or label set by user,must be row vector. 
// float RegressionAccuracy:default value 0.01 – based on error calculation   
// bool TruncatedPruneTree: default true 
// bool UseSurrogates:default is false.if true surrogates splits will be built 
// bool Use1SERule: if true makes pruning harsher
//
// Description
// This function trains an image category classifier which can be used to predict categories of images given to it as input using the mlpredict() function.
//
// Examples
// imgSet = imageSet(images/train_3,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = trainRTreesClassifier(trainingSet, bag);
// simage=imread("images/plane.jpg");
// label=mlPredict(categoryClassifier,simage,"RT");
//
// See also
// imageSet
// bagOfFeatures
//
// Authors
// Gursimar Singh
// Rohit Suri 
// Umang Agrawal

     [lhs,rhs]=argn(0);
	

	bag_list = bagStructToList(bag);
	
	imgSets_list = imageSetToList(imgSets); 


	if lhs>1
        error(msprintf(" Too many output arguments"));
    elseif rhs<3
        error(msprintf("Not enough input arguments"));
    elseif rhs>15
		error(msprintf(" Too many input arguments"));
	end

    	if rhs==3
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName);
	elseif rhs==4
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1));
	elseif rhs==5
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2));
	elseif rhs==6
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3));
	elseif rhs==7
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4));
	elseif rhs==8
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5));
	elseif rhs==9
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
	elseif rhs==10
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7));
	elseif rhs==11
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
	elseif rhs==12
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9));
	elseif rhs==13
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9),varargin(10));
	elseif rhs==14
    	temp = raw_trainRTreesClassifier(imgSets_list, bag_list,classifierName,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8),varargin(9),varargin(10),varargin(11));
	end                                                                                                                                                                  

	classifier = struct("Classifier Type:",temp(1),"ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))
	
endfunction

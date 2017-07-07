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

function classifier = trainNBClassifier(imgSets,bag,classifiername)
// This function is used to train an image classifier using Naive Bayes Algorithm.
//
// Calling Sequence
// classifier = trainNBClassifier(imgSets,bag,classifiername)
//
// Parameters
// classifier: Image category classifier
// imgSets: Input imageSet to train the classifier on
// bag: The bagOfFeatures of the imageSet provided
// classifiername: Name of the ouput yml classifier file.
//
// Description
// This function trains an image category classifier which can be used to predict categories of images given to it as input using the predict() function.
//
// Examples
// imgSet = imageSet(images/train_2,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = trainNBClassifier(trainingSet, bag);
// simage=imread("images/bike.jpg");
// [label,prob]=mlpredict(categoryClassifier,simage,"NB");
//
// See also
// bagOfFeatures
// imageSet
// mlpredict
// Authors
// Gursimar Singh
// Rohit Suri
// Umang Agarwal

     [lhs,rhs]=argn(0);
	

	bag_list = bagStructToList(bag);
	
	imgSets_list = imageSetToList(imgSets);
	

	if lhs>1
         	error(msprintf(" Too many output arguments"));
    	elseif rhs<1
        	error(msprintf(" Not enough input arguments"));
    	elseif rhs>3
			error(msprintf(" Too many input arguments"));

	elseif rhs==3
	temp = raw_trainNBClassifier(imgSets_list, bag_list,classifiername);
    	else
    temp = raw_trainNBClassifier(imgSets_list, bag_list,classifiername);  
	end                                                                    
	classifier = struct("Classifier Type :",temp(1),"ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4));
	
endfunction

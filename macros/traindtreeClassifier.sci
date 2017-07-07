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

function classifier = traindtreeClassifier(imgSets, bag, classifierName, varargin)
// This function is used to train an image classifier.
//
// Calling Sequence
// imgSet = imageSet(directory,'recursive');
// 'or'
// imgSet = imageSet(image);
// bag = bagOfFeatures(imgSet);
// classifier = traindtreeClassifier(imgSets, bag);
// classifier = traindtreeClassifier(imgSets, bag,'nameclass')
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,,maximum depth)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count,reg_acc)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count,reg_acc,use Surrogates)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count,reg_acc,use Surrogates,max_cat,cv_f)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count,reg_acc,use Surrogates,max_cat,cv_f,use1_se)
// classifier = traindtreeClassifier(imgSets, bag,'nameclass',save,maximum depth,minimum sample count,reg_acc,use Surrogates,max_cat,cv_f,use1_se,is_pruned)
// [classifier,BagofFeaturesLocation,Description] = traindtreeClassifier() 
//
// Parameters
// classifier: Image category classifier location
// BagofFeaturesLocation : location of the xml or yml file.
// Description : features obtained after training.
// imgSets: Input imageSet to train the classifier on
// bag: The bagOfFeatures of the imageSet provided
// image: The set of images used for creating the imageset used for training
// nameclass: Name of the classifier one wants for their bag of features .xml or .yml file
// save:Option for the users to save their training data.
// maximum depth : The maximum possible depth of the tree. That is the training algorithms attempts to split a node while its depth is less than maxDepth. The root node has zero depth. The 
//                 actual depth may be smaller if the other termination criteria are met (see the outline of the training procedure here), and/or if the tree is pruned. Default value is INT_MAX.
// minimum sample count : If the number of samples in a node is less than this parameter then the node will not be split. The Default value is 10.
// reg_acc : Termination criteria for regression trees. If all absolute differences between an estimated value in a node and values of train samples in this node are less than this parameter then
//            the node will not be split further. Default value is 0.01f
// use Surrogates: If true then surrogate splits will be built. These splits allow to work with missing data and compute variable importance correctly. Default value is false.
// max_cat : Cluster possible values of a categorical variable into K<=maxCategories clusters to find a suboptimal split. If a discrete variable, on which the training procedure tries to make a split,
//           takes more than maxCategories values, the precise best subset estimation may take a very long time because the algorithm is exponential. Instead, many decision trees engines (including
//           our implementation) try to find sub-optimal split in this case by clustering all the samples into maxCategories clusters that is some categories are merged together. The clustering is
//           applied only in n > 2-class classification problems for categorical variables with N > max_categories possible values. In case of regression and 2-class classification the optimal split
//           can be found efficiently without employing clustering, thus the parameter is not used in these cases. Default value is 10.
// cv_f : If CVFolds > 1 then algorithms prunes the built decision tree using K-fold cross-validation procedure where K is equal to CVFolds. Default value is 10.
// use1_se : If true then a pruning will be harsher. This will make a tree more compact and more resistant to the training data noise but a bit less accurate. Default value is true
// is_pruned : If true then pruned branches are physically removed from the tree. Otherwise they are retained and it is possible to get results from the original unpruned (or pruned less aggressively)
//             tree. Default value is true.
//
//
// Description
// This function trains a dtree classifier which can be used to predict classes of images given to it as
// input using the predictdtree() function.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = traindtreeClassifier(trainingSet, bag);
//
	
	
	bag_list = bagStructToList(bag);
	imgSets_list = imageSetToList(imgSets);

	
	[lhs rhs] = argn(0)
	if lhs > 1
		error(msprintf("Too many output arguments"));
	elseif rhs < 2
		error(msprintf("Not enough input arguments"));
	elseif rhs > 13
		error(msprintf("Too many input arguments"));
	end
	if     rhs == 3
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName,);
	elseif rhs == 4
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1));
	elseif rhs == 5
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2));
        elseif rhs == 6
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3));
        elseif rhs == 7
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4));
        elseif rhs == 8
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5));
        elseif rhs == 9
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6));
        elseif rhs == 10
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7));
	elseif rhs == 11
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8));
        elseif rhs == 12
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9));
        
        elseif rhs == 13
		temp = raw_traindtreeClassifier(imgSets_list, bag_list, classifierName, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10));

        end
	
	classifier = struct("ClassifierLocation", temp(2), "BagofFeaturesLocation", temp(3), "Description", temp(4))
	
endfunction

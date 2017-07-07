function classifierDescription = predict(Categoryclassifier, image,modelName,varargin)
// This function is used to predict the category of an image using an image classifier of SVM Model.
//
// Calling Sequence
// label = predict(classifier, image,modelName)
//
// Parameters
// label: Evaluated label of the input image
// classifier: Image category classifier
// image: Input image
//
// Description
// This function predicts the category of an image based on the category classifier provided.
//
// Examples
// imgSet = imageSet(directory,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
// bag = bagOfFeatures(trainingSet);
// categoryClassifier = trainImageCategoryClassifier(trainingSet, bag);
// image = imread('sample.jpg');
// label = predict(categoryClassifier, image,"ANN");
//
// Authors
//Shreyash Sharma
	
	image_list = mattolist(image)
	Categoryclassifier_list = classifierToList(Categoryclassifier);
        select modelName
//        case "SVM" then
//	classifierDescription = raw_predictSVM(Categoryclassifier_list, image_list);
//        case "SVMSGD" then
//        classifierDescription = raw_predictSVMSGD(Categoryclassifier_list, image_list);
//	case "svm" then
//	classifierDescription = raw_predictSVM(Categoryclassifier_list, image_list);
//        case "svmsgd" then
//        classifierDescription = raw_predictSVMSGD(Categoryclassifier_list, image_list);
//        case "EM" then
//        classifierDescription = raw_predictEM(Categoryclassifier_list, image_list);
//	case "em" then
//        classifierDescription = raw_predictEM(Categoryclassifier_list, image_list);
//        case "LR" then
//        classifierDescription = raw_predictLR(Categoryclassifier_list, image_list);
//	case "lr" then
//        classifierDescription = raw_predictLR(Categoryclassifier_list, image_list);
//	case "KNN" then
//        classifierDescription = raw_predictKNN(Categoryclassifier_list, image_list);
//	case "knn" then
//        classifierDescription = raw_predictKNN(Categoryclassifier_list, image_list);
//	case "RT" then
//        classifierDescription = raw_predictRT(Categoryclassifier_list, image_list);
//	case "rt" then
//        classifierDescription = raw_predictRT(Categoryclassifier_list, image_list);

	case "ANN" then
        classifierDescription = raw_predictANN(Categoryclassifier_list, image_list);
	case "ann" then
        classifierDescription = raw_predictANN(Categoryclassifier_list, image_list);
	case "DTree" then
        classifierDescription = raw_predictdtree(Categoryclassifier_list, image_list);
	case "dtree" then
        classifierDescription = raw_predictdtree(Categoryclassifier_list, image_list);
        case "Boost" then
        classifierDescription = raw_predictBoost(Categoryclassifier_list, image_list);
	case "boost" then
        classifierDescription = raw_predictBoost(Categoryclassifier_list, image_list);
                   
        else
          mprintf("\nthe given modelName-%s is invalid! the model name can be upper case or lower case!\n ",modelName);
        end

	



	
	
endfunction

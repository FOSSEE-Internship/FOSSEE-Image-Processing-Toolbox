function param = getParams(classifier,modelName)

classifier_list = classifierToList(classifier);

/// Add your structure arguments as in case of RT and NB
	
    select modelName
// 	case "SVM" then
//	temp = raw_getParamsSVM(classifier_list);
//	param = struct();
//	case "svm" then
//	temp = raw_getParamsSVM(classifier_list);
//	param = struct();
    
//  case "SVMSGD" then
//	temp = raw_getParamsSVMSGD(classifier_list, image);
//	param = struct();
//	case "svmsgd" then
//    	temp = raw_getParamsSVMSGD(classifier_list, image);
//	param = struct();
  	
//	case "EM" then
//    	temp = raw_getParamsEM(classifier_list);
//	param = struct();
//	case "em" then
//   	temp = raw_getParamsEM(classifier_list);
//	param = struct();
    
//	case "LR" then
//    	temp = raw_getParamsLR(classifier_list);
//	param = struct();
//	case "lr" then
//    	temp = raw_getParamsLR(classifier_list);
//	param = struct();
	
//	case "KNN" then
 //   	temp = raw_getParamsKNN(classifier_list);
//	param = struct();
//	case "knn" then
//    	temp = raw_getParamsKNN(classifier_list);
//	param = struct();
	
//	case "RT" then
//     	temp = raw_getParamsRT(classifier_list);
//	param = struct("No. of Active Variables", temp(1), "CVfolds", temp(2), "maxCatgories", temp(3),"maxDepth", temp(4),"minSample", temp(5),"reg accuracy", temp(6),"isPruned", temp//          (7),"UseSurrogates", temp(8),"UseSE1Rule", temp(9));
//	case "rt" then
//    	temp = raw_getParamsRT(classifier_list);
//	param = struct("No. of Active Variables", temp(1), "CVfolds", temp(2), "maxCatgories", temp(3),"maxDepth", temp(4),"minSample", temp(5),"reg accuracy", temp(6),"isPruned", temp //(7),"UseSurrogates", temp(8),"UseSE1Rule", temp(9));
	
	case "ANN" then
    	temp = raw_getParamsANN(classifier_list);
	param = struct("bpms", temp(1), "bws", temp(2),"d0",temp(3),"d1",temp(4),"d2",temp(5),"d3",temp(6),"d4",temp(7),"nmeth",temp(8));
	case "ann" then
    	temp = raw_getParamsANN(classifier_list);
	param = struct("bpms", temp(1), "bws", temp(2),"d0",temp(3),"d1",temp(4),"d2",temp(5),"d3",temp(6),"d4",temp(7),"nmeth",temp(8));
	
	case "DTree" then
    	temp = raw_getParamsdtree(classifier_list);
    	param = struct("cv_f", temp(1), "max_cat", temp(2),"md",temp(3),"msc",temp(4),"reg_acc",temp(5),"prune",temp(6),"use1_se",temp(7),"use_surr",temp(8));
    
	case "dtree" then
    	temp = raw_getParamsdtree(classifier_list);
        param = struct("cv_f", temp(1), "max_cat", temp(2),"md",temp(3),"msc",temp(4),"reg_acc",temp(5),"prune",temp(6),"use1_se",temp(7),"use_surr",temp(8));
    
    	 case "Boost" then
        temp = raw_getParamsBoost(classifier_list,image_list);
	param = struct("bt", temp(1), "wc", temp(2),"wtr",temp(3),"md",temp(4),"use_surr",temp(5));				///// add structures here
    	case "boost" then
        temp = raw_getParamsBoost(classifier_list,image_list);
	param = struct("bt", temp(1), "wc", temp(2),"wtr",temp(3),"md",temp(4),"use_surr",temp(5));

    else
      mprintf("\nThe given modelName-%s is invalid\n", modelName);
    end	
endfunction

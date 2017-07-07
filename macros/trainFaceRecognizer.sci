// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function faceClassifier=trainFaceRecognizer(imgSet,algoName)
//The function saves an xml file for the trained model for an imageset of faces.
//
// Calling Sequence
//faceClassifier=trainFaceRecognizer(imgSet,algoName)
//
//Parameters
//faceClassifier: A classifier structure with following fields:
//ClassifierType - Algorithm with which the recognizer was trained.
//ClassifierLocation - Location of the xml file generated after training.
//DescriptionCount - Number of images used in training the recognizer.  
//imgSet: It is a structure,output of the function imageSet.
//algoName: Name of the algo to be used for training.Valid strings are:"LBPH","EIGEN","FISHER". 
//
//Description
//The function saves an xml file of the trained model for an imageset of faces.The trained model than can be used with facePredict function to predict the label of the test image. 
//
//Examples 
// //Face Recognition with LBPH algorithm.ImageFolder contains subfolders with images of different subjects that are to be recognized in different images.The subfolder name is considered to be the label for those set of images.     
//imgSet=imageSet("images/trainset_face","recursive");
//tr=trainFaceRecognizer(imgSet,"LBPH");
//image=imread("images/s1.pgm");
//[p(1),c(1)]=facePredict(tr,image);
//image=imread("images/s2.pgm");
//[p(2),c(2)]=facePredict(tr,image);
//image=imread("images/s3.pgm");
//[p(3),c(3)]=facePredict(tr,image);
//
//See also
// imageSet
// facePredict
//
//Authors
//Gursimar Singh



[lhs,rhs]=argn(0);
	
	
	imgSets_list = imageSetToList(imgSet); 
	//disp(imgSets_list)
	

		if lhs>1
         error(msprintf(" Too many output arguments"));
    	elseif rhs<1
        error(msprintf(" Not enough input arguments"));
    	elseif rhs>2
		error(msprintf(" Too many input arguments"));
		end
	temp = raw_trainFaceRecognizer(imgSets_list,algoName);
                                                                  
	faceClassifier = struct("ClassifierType",temp(1),"ClassifierLocation", temp(2), "DescriptionCount", temp(3));


endfunction

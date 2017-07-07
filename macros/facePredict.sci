// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
// Training Dataset credits : AT&T Laboratories Cambridge

function [predictedLabel,confidence]=facePredict(classifier,image)
//Predict face label in an image. 
//
//Calling Sequence
//[predictedLabel]=facePredict(classifier,image)
//[predictedLabel,confidence]=facePredict(classifier,image) 
//
//Parameters
//predictedLabel:The predicted label of the input image.It is the name of the folder in which the input existed when classifier was trained.
//confidence:More value of the confidence more is the deviation of the input image with the original image.Confidence =0 means exact match.
//classifier:A face classifier structure obtained from trainFaceRecognizer with following fields<itemizedlist><listitem>ClassifierType - Algorithm with which the recognizer was trained.</listitem><listitem>ClassifierLocation - Location of the xml file generated after training.</listitem><listitem>DescriptionCount - Number of images used in training the recognizer.</listitem></itemizedlist> 
//image:Input image
//
//Description
//The function predicts the label of the input image from the image set by which the cascade classifier was trained.
//
//Examples
//imgSet=imageSet("images/trainset_face","recursive");
//tr=trainFaceRecognizer(imgSet,"LBPH");
//image=imread("images/s1.pgm");
//[p(1),c(1)]=facePredict(tr,image);
//image=imread("images/s2.pgm");
//[p(2),c(2)]=facePredict(tr,image);
//image=imread("images/s3.pgm");
//[p(3),c(3)]=facePredict(tr,image);
//
//Authors
//Gursimar Singh
//
//See also
//imageSet
//trainFaceRecognizer

	image_list = mattolist(image)
	if ~isstruct(classifier)
		error(msprintf("Structure of classifier required\n"));
	end

	classifier_list = list(classifier.ClassifierType,classifier.ClassifierLocation,classifier.Description);
	
	[predictedLabel,confidence] = raw_facePredict(classifier_list, image_list);
	
endfunction

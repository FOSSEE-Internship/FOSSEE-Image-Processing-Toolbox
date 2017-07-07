function [invertedImageIndexStruct]=invertedIndexImages(imgSet,bagOfFeatures)
// This function creates image search index
//
// Calling Sequence
//  imageIndex = invertedIndexImages(imgSet, bagOfFeatures );
//
// Parameters
//  imgSet: Collection of images with their location and labels
//  bagOfFeatures: Bag of Visual Words
//  SaveFeatureLocations [Optional Input Argument]: A boolean value inicating whether feature locations has to be saved or not. Values: [True(default)|False]
//  imageIndex: Search Index Object containing the mapping of visual vocabulary to each image in the image set
//
// Description
//  Creates a search Index Object that has been mapped to visual words that can be used for retrieve Images
//
// Examples
//    imgSet = imageSet(directory,'recursive');
//    [trainingSet testSet] = partition(imgSet,[0.8]);
//    bag = bagOfFeatures(trainingSet);
//    imageindex = invertedIndexImages(trainingSet, bag);
//
// With Optional Arguments:
//    imageindex = invertedIndexImages(trainingSet, bag, "SaveFeatureLocations", 1);
//
// Authors
//  Umang Agrawal
//  Rohit Suri
//  Sridhar Reddy

    invertedImageIndexList=indexImages(imgSet,bagOfFeatures);
    invertedImageIndexStruct=struct('Type','invertedImageIndex','ImageLocation',invertedImageIndexList(5), 'ImageWords',invertedImageIndexList(2), 'WordFrequency',invertedImageIndexList(3),'BagOfFeatures',bagOfFeatures,)
endfunction

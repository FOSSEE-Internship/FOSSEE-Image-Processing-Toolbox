function trainCascadeObjectDetector(outputFolder,positiveInstances,negativeImages,varargin)
// It creates trained cascade XML file  
//
// Calling Sequence
// trainCascadeObjectDetector(outputFolder,positiveInstances,"negativeImages")
//
// Parameters
// outputFolder: Folder name to store trained cascade (cascade.xml) and intermediate files  
// positiveInstances: Array of structure with image file names (including path) and an M-by-4 matrix of bounding box.
// negativeImages: path to a negative images folder
// numPos: number of positive samples. Default-30
// numNeg: number of negative samples. Default- 20
// nsplits: number of splits. Default- 2
// numStages: number of cascade stages to be trained. Default- 30
// featureType: Type of features to be used, possible types are HAAR, LBP, HOG. Default-HAAR
// minHitRate: Minimal desired hit rate for each stage of the classifier and value in the range 0 and 1 inclusive. Default- 0.995
// maxFalseAlarmRate: Maximal desired false alarm rate for each stage of the classifier and value in the range 0 and 1 inclusive. Default- 0.5
// w: Width of training samples (in pixels). Default-25
// h: Height of training samples (in pixels). Default-25
//
// Description
// It produces dataset of positive  samples in a file with .vec extension and negative samples are enumerated in a special text file in
// which  each line contains an image filename of negative sample image. Negative images must not contain detected objects.
//
// By using these two files trainCascade will create cascade.xml file inside the outputFolder which is used to detect objects in an image.
//
// Examples
// positiveInstances(1)=struct("path",'image1.jpg',"bbox",[1 5 2 3]);
// positiveInstances(2)=struct("path",'image2.jpg',"bbox",[2 7 4 2]);
// trainCascadeObjectDetector("data",positiveInstances,"negativeImagesFolder");
// trainCascadeObjectDetector("data",positiveInstances,"negativeImagesFolder","numStages",35,"featureType","HOG");
// trainCascadeObjectDetector("data",positiveInstances,"negativeImagesFolder","minHitRate",0.9,"numPos",8,"numNeg",5);

    [lhs rhs]=argn(0);
    if rhs<3 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>21 then
         error(msprintf(" Too many input arguments to the function"))
    elseif modulo(rhs-3,2)
         error(msprintf(" wrong number of input arguments,name-value pairs not macthed"))
    end
    
    //validating variables
    if ~isdir(outputFolder) then
        error(msprintf(" wrong input argument #1, existing directory expected"))
    elseif ~isdir(negativeImages)
        error(msprintf(" wrong input argument #3,existing directory expected"))
    end
    
    //default values
    numPos=30;
    numNeg=20;
    numStages=30;
    nsplits=2;
    featureType="HAAR"
    minHitRate=0.995
    maxFalseAlarmRate=0.5
    w=25
    h=25
    
    for i=1:2:rhs-3
       if strcmpi(varargin(i),"numPos")==0 then
            i=i+1;
            numPos=varargin(i);
            if numPos<0 then
                error(msprintf(" numPos value must be positive"))
            end
        elseif strcmpi(varargin(i),'numNeg')==0 then
            i=i+1;
            numNeg=varargin(i);
            if numNeg<0 then
                error(msprintf(" numNeg value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'numStages')==0 then
            i=i+1;
            numStages=varargin(i);
            if numStages<0 then
                error(msprintf(" numStages value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'nslits')==0 then
            i=i+1;
            nsplits=varargin(i);
            if nsplits<0 then
                error(msprintf(" nsplits value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'featureType')==0 then
            i=i+1;
            featureType=varargin(i);
            if strcmpi(featureType,'haar') & strcmpi(featureType,'lbp') & strcmpi(featureType,'hog')
                error(msprintf(" wrong input argument #%d,featureType not matched",i));
            end
            
        elseif strcmpi(varargin(i),'minHitRate')==0 then
            i=i+1;
            minHitRate=varargin(i);
            if minHitRate<0 | minHitRate>1 then
                error(msprintf(" minHitRate value must lie in between 0 and 1"))
            end
            
        elseif strcmpi(varargin(i),'maxFalseAlarmRate')==0 then
            i=i+1;
            maxFalseAlarmRate=varargin(i);
            if maxFalseAlarmRate<0  | minFalseRate>1 then
                error(msprintf(" maxFalseAlarmRate value must lie in between 0 and 1"))
            end
            
        elseif strcmpi(varargin(i),'w')==0 then
            i=i+1;
            w=varargin(i);
            if h<0 then
                error(msprintf(" w value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'h')==0 then
            i=i+1;
            h=varargin(i);
            if h<0 then
                error(msprintf(" h value must be positive"))
            end
        else
            error(msprintf(_(" Wrong value for input argument #%d",i)));
        end
    end
    
    [nRows noOfPositiveInstances]=size(positiveInstances);
    fields=fieldnames(positiveInstances);
    fd = mopen('positive.txt','wt');
    for i=1:noOfPositiveInstances
        mfprintf(fd,'%s 1',getfield(fields(1),positiveInstances(i)));
        boxVals=getfield(fields(2),positiveInstances(i));
        for j=1:4
            mfprintf(fd,' %d',boxVals(j));
        end
        mfprintf(fd,'\n');
    end
    mclose(fd);
    
    disp("Creating positive samples:");
    cmd=sprintf("opencv_createsamples -info positive.txt -num%d -vec positive.vec -w %d -h %d",numPos,w,h);
    unix_w(cmd);
    if isdir(negativeImages)
        if getos()=="Linux"
            temp=strcat(["ls ",negativeImages])
        elseif getos()=="Windows"
            temp=strcat(["dir ",negativeImages])
        end
        s=unix_g(temp);
        [noOfFilesInFolder noOfCols]=size(s);
        fd = mopen('negative.txt','wt');
        for i=1:noOfFilesInFolder
            [path,fname,extension]=fileparts(s(i))
            if ~strcmp(extension,".jpg") | ~strcmp(extension,".jpeg") | ~strcmp(extension,".png") | ~strcmp(extension,".bmp") | ~strcmp(extension,".pgm") | ~strcmp(extension,".JPG") | ~strcmp(extension,".JPEG") | ~strcmp(extension,".PNG") | ~strcmp(extension,".BMP") | ~strcmp(extension,".PGM")
                mfprintf(fd,'%s/%s\n',negativeImages,s(i));
            end
        end
     end
     disp("Training Cascade:");
     cmd=sprintf("opencv_traincascade -data %s -vec positive.vec -bg negative.txt -numPos %d -numNeg %d -numStages %d -nsplits %d -featureType %s -minHitRate %d -maxFalseAlarmRate %d -w %d -h %d",outputFolder,numPos,numNeg,numStages,nsplits,featureType,minHitRate,maxFalseAlarmRate,w,h);
     unix_w(cmd);
endfunction;

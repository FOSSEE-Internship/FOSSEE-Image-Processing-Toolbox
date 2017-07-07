function varargout=cameraParameters(varargin)
// It returns the cameraParameters structure
//
// Calling Sequence
// cameraParams=cameraParameters(Name,value);
//
// Parameters
// IntrinsicMatrix: 3-by-3 matrix, it specifies the principal point, skew and focal length. Default- 3-by-3 identity matrix
// RadialDistortion: 2 or 3 element vector. Default- [0 0 0]
// TangentialDistortion: 2 element vector. Default- [0 0]
// RotationVectors: P-by-3 matrix. Each row of the matrix represents 3-D rotation of the camera. Default- []
// TranslationVectors: P-by-3 matrix. Each row of the matrix represents 3-D translation of the camera. Number of rotation vectors should be equal to number of transation vectors. Default- []
// WorldPoints: M-by-2 matrix. M indicates the  number of keypoints. Default- []
// WorldUnits: Unit of measure of world points. Default- "mm"
// EstimateSkew: logical scalar. Default- %F
// NumRadialDistortionCoefficients: 2 or 3. Default- 2
// EstimateTangentialDistortion: logical scalar. Default- %F
// ReprojectionErrors: M-by-2-p array. Number of channels must be equal to number of rotation vectors and number of rows should be equal to number of world points. Default-[]
//
// Description
// This function creates cameraParameters struct based on the arguments provided. The struct contains intrinsic, extrinsic, estimation and lens properties.
//
// Examples
// cameraParms=cameraParams("IntrinsicMatrix", [2 0 0; 0 3 0;4 5 1]);
// Output:
//   IntrinsicMatrix: [3x3 constant]
//   PrincipalPoint: [4,5]
//   FocalLength: [2,3]
//   Skew: 0
//   RadialDistortion: [0,0,0]
//   TangentialDistortion: [0,0]
//   RotationMatrices: [0x0 constant]
//   RotationVectors: [0x0 constant]
//   TranslationVectors: [0x0 constant]
//   meanReprojectionError: 0
//   ReprojectionErrors: [0x0 constant]
//   ReprojectedPoints: [0x0 constant]
//   NumPatterns: 0
//   WorldPoints: [0x0 constant]
//   WorldUnits: "mm"
//   EstimateSkew: 0
//   NumRadialDistortionCoefficients: 2
//   EstimateTangentialDistortion: 0

    [lhs rhs]=argn(0);
    if lhs<1 then
         error(msprintf(" Not enough output arguments"))
    elseif lhs>1 then
         error(msprintf(" Too many output arguments to the function"))
    end
    
    //default values
    IntrinsicMatrix=[1 0 0;0 1 0;0 0 1];
    RadialDistortion=[0 0 0];
    TangentialDistortion=[0 0];
    RotationVectors=[];
    TranslationVectors=[];
    WorldPoints=[];
    WorldUnits='mm';
    EstimateSkew=0;
    NumRadialDistortionCoefficients=2;
    EstimateTangentialDistortion=0;
    ReprojectionErrors=[];
    
    imRows=0,imCols=0;
    rdRows=0,rdCols=0;
    tdRows=0,tdCols=0;
    rvRows=0,rvCols=0;
    tvRows=0,tvCols=0;
    wpRows=0,wpCols=0;
    
    if modulo(rhs,2) then
         error(msprintf(" Wrong number of input arguments"));
    end
    
    for i=1:2:rhs
       if strcmpi(varargin(i),"IntrinsicMatrix")==0 then
            i=i+1;
            IntrinsicMatrix=varargin(i);
            [imRows imCols]=size(IntrinsicMatrix);
            if ~imRows==3 | ~imCols==3 then
                error(msprintf(" wrong value for the input argument %d, IntrinsicMatrix must be 3*3",i))
            end
            
        elseif strcmpi(varargin(i),'RadialDistortion')==0 then
            i=i+1;
            RadialDistortion=varargin(i);
            [rdRows rdCols]=size(RadialDistortion);
            if ~rdRows==1 | (~rdCols==2 & ~rdCols==3) then
                error(msprintf(" wrong value for the input argument %d, RadialDistortion must be a vector with 2 or 3 elements",i))
            end
            
        elseif strcmpi(varargin(i),'TangentialDistortion')==0 then
            i=i+1;
            TangentialDistortion=varargin(i);
            [tdRows tdCols]=size(TangentialDistortion);
            if ~tdRows==1 | ~tdCols==2 then
                error(msprintf(" wrong value for the input argument %d, TangentialDistortion must be 2-element vector",i))
            end
            
        elseif strcmpi(varargin(i),'RotationVectors')==0 then
            i=i+1;
            RotationVectors=varargin(i);
            [rvRows rvCols]=size(RotationVectors);
            if  ~rvCols==3  then
                error(msprintf(" wrong value for the input argument %d, RotationVectors must be M*3",i))
            end
            
        elseif strcmpi(varargin(i),'TranslationVectors')==0 then
            i=i+1;
            TranslationVectors=varargin(i);
            [tvRows tvCols]=size(TranslationVectors);
            if  ~tvCols==3  then
                error(msprintf(" wrong value for the input argument %d, TranslationVectors must be M*3",i))
            end
            
        elseif strcmpi(varargin(i),'WorldPoints')==0 then
            i=i+1;
            WorldPoints=varargin(i);
            [wpRows wpCols]=size(WorldPoints);
            if  ~wpCols==2 then
                error(msprintf(" wrong value for the input argument %d, WorldPoints must be M*2",i))
            end
            
        elseif strcmpi(varargin(i),'WorldUnits')==0 then
            i=i+1;
            WorldUnits=varargin(i);
            if ~typeof(WorldUnits)=='string'
                error(msprintf(" wrong value for the input argument %d, WorldUnits must be string",i))
            end
            
        elseif strcmpi(varargin(i),'EstimateSkew')==0 then
            i=i+1;
            EstimateSkew=varargin(i);
            if EstimateSkew==%T
                EstimateSkew=1;
            elseif EstimateSkew==%F
                EstimateSkew=0;
            else
                error(msprintf(" wrong value for the input argument %d,EstimateSkew must be logical scalar",i))
            end

        elseif strcmpi(varargin(i),'NumRadialDistortionCoefficients')==0 then
            i=i+1;
            NumRadialDistortionCoefficients=varargin(i);
            if ~(NumRadialDistortionCoefficients==2 | NumRadialDistortionCoefficients==3) then
                error(msprintf(" wrong value for the input argument %d, NumRadialDistortionCoefficients must be 2 or 3",i))
            end
            
        elseif strcmpi(varargin(i),'EstimateTangentialDistortion')==0 then
            i=i+1;
            EstimateTangentialDistortion=varargin(i);
            EstimateSkew=varargin(i);
            if EstimateSkew==%T
                EstimateSkew=1;
            elseif EstimateSkew==%F
                EstimateSkew=0;
            else
                error(msprintf(" wrong value for the input argument %d, EstimateTangentialDistortion must be logical scalar",i))
            end
            
        elseif strcmpi(varargin(i),'ReprojectionErrors')==0 then
            i=i+1;
            ReprojectionErrors=varargin(i);
            [reRows reCols reChannels]=size(ReprojectionErrors);
            if ~reCols==2 then
                error(msprintf(" wrong value for the input argument %d, ReprojectionErrors must be M-by-2-by-P array",i))
            end
            
        else
            error(msprintf(" Wrong value for the input argument %d, %s is not a recognised parameter",i,varargin(i)));
        end
    end
    
    if ~rvRows==tvRows then
        error(msprintf(" wrong value for the input arguments, RotationalVectors and TranslationVectors must be of same size"))
    end
    if ~isempty(ReprojectionErrors) then
        if ~reRows==wpRows then
            error(msprintf("wrong value for the input arguments, World points and ReprojectctionErrors must contain same number of rows"))
        end
        if ~reChannels==rvRows then
            error(msprintf("wrong value for the input arguments, Number of patterns in ReprojectionErrors and RotationVectors must be of same size"))
        end
    end
    
    RotationMatrices=[]
    FocalLength=[IntrinsicMatrix(1,1) IntrinsicMatrix(2,2)];
    PrincipalPoint=[IntrinsicMatrix(3,1) IntrinsicMatrix(3,2)];
    Skew=IntrinsicMatrix(2,1);
    for i=1:rvRows
        RotationVector=RotationVectors(i,:);
        angle=norm(RotationVector);
        if angle < %eps
            RotationMatrices(:,:,i) = eye(3);
        else
            n= RotationVector ./ angle;
            A = [n(1)^2,    n(1)*n(2), n(1)*n(3);...
                 n(2)*n(1), n(2)^2,    n(2)*n(3);...
                 n(3)*n(1), n(3)*n(2), n(3)^2    ];
             
            B = [  0,  -n(3),  n(2);...
                 n(3),   0,   -n(1);...
                -n(2),  n(1),   0   ];
            RotationMatrices(:,:,i) = eye(3) * cos(angle) + (1 - cos(angle)) * A + sin(angle) * B;  
        end
    end
   
    distCoeffs(1,1)=RadialDistortion(1,1);
    distCoeffs(1,2)=RadialDistortion(1,2);
    distCoeffs(1,3)=TangentialDistortion(1,1);
    distCoeffs(1,4)=TangentialDistortion(1,2);
    if rdCols==3
        distCoeffs(1,5)=RadiallDistortion(1,3);
    else
        distCoeffs(1,5)=0;
    end
    
    ReprojectedPoints=[];
    meanReprojectionError=0;
    NumPatterns=rvRows;
    if(WorldPoints <> [])
        for i=1:NumPatterns
            ReprojectedPoints(:,:,i)=opencv_projectPoints(WorldPoints,RotationVectors(i,:),TranslationVectors(i,:),IntrinsicMatrix,distCoeffs);
        end
    end
    if ~isempty(ReprojectionErrors) then
        ImagePoints=ReprojectedPoints-ReprojectionErrors;
        totalErr=0;
        for i=1:rvRows
            err=0;
            for j=1:wpRows
                err=err+((ReprojectedPoints(j,1,i)-ImagePoints(j,1,i))+(ReprojectedPoints(j,2,i)-ImagePoints(j,2,i))).^2;
            end
            totalErr=totalErr+err;
        end
        totalPoints=rvRows*wpRows;
        meanReprojectionError=sqrt(totalErr/totalPoints);
    end
    
    varargout(1)=struct('IntrinsicMatrix',IntrinsicMatrix,'PrincipalPoint',PrincipalPoint,'FocalLength',FocalLength,'Skew',Skew,'RadialDistortion',RadialDistortion,'TangentialDistortion',TangentialDistortion,'RotationMatrices',RotationMatrices,'RotationVectors',RotationVectors,'TranslationVectors',TranslationVectors,'meanReprojectionError',meanReprojectionError,'ReprojectionErrors',ReprojectionErrors,'ReprojectedPoints',ReprojectedPoints,'NumPatterns',NumPatterns,'WorldPoints',WorldPoints,'WorldUnits',WorldUnits,'EstimateSkew',EstimateSkew,'NumRadialDistortionCoefficients',NumRadialDistortionCoefficients,'EstimateTangentialDistortion',EstimateTangentialDistortion);
endfunction;

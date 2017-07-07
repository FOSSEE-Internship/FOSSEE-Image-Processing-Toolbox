function displayErrors(varargin)
     [lhs rhs]=argn(0);
     if rhs<2 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>2 then
         error(msprintf(" Too many input arguments. Two arguments expected."))
    end
    typeOferrors =0; 
    errors = varargin(1);
    if ~isstruct(errors) then
        error(msprintf("Invalid Input Argument. A CameraCalibrationError Struct or StereoCalibrationError Struct expected. "))
    end    
    errorNames = fieldnames(errors);
    [size1 zz] = size(errorNames);  

    if size1 == 2 then
        typeOferrors = 1;
    elseif  size1 == 5 then
        typeOferrors = 2;
    else
         error(msprintf("Invalid Input Argument. A CameraCalibrationError Struct or StereoCalibrationError Struct expected. "))    
    end
    
    if typeOferrors == 1 then
        cameraParam = varargin(2);
        checkCCE(errors);
        if ~isstruct(cameraParam) then
            error(msprintf(" A CameraParameters Struct expected"))            
        end        
        jj = fieldnames(cameraParam)
        [size3 zz] = size(jj);               
        if size3~=18 then
            error(msprintf("An CameraParameters struct expected."))   
        end                        
        checkCameraParam(cameraParam);    
        focalErr = string(cameraParam.FocalLength) + " +/- ";
        fE = string(errors.IntrinsicsErrors.FocalLengthError) ;
        prinErr = string(cameraParam.PrincipalPoint) + " +/- ";
        pE = string(errors.IntrinsicsErrors.PrincipalPointError);        
        radErr = string(cameraParam.RadialDistortion) + " +/- ";    
        rdE = string(errors.IntrinsicsErrors.RadDistortionError);
        for i=1:2
            focalErr(i) = focalErr(i) + fE(i);
            prinErr(i) = prinErr(i) + pE(i);
        end
        [size1 xx] = size(radErr); 
        size1 =size1*xx;
        for i=1:size1
            radErr(i) = radErr(i) + rdE(i);
        end
        
        rotVecErr = string(cameraParam.RotationVectors) + " +/- ";
        rvE = string(errors.ExtrinsicsErrors.RotVectorsError);
        [size21 size22] = size(rotVecErr);
        printf("%d  %d \n", size21,size22);
        for i=1:size21
            for j=1:size22
                rotVecErr(i,j) = rotVecErr(i,j) + rvE(i,j);
            end
        end
        
        tranVecErr = string(cameraParam.TranslationVectors) + " +/- ";
        tvE = string(errors.ExtrinsicsErrors.TransVectorsError);
        [size31 size32] = size(tranVecErr);
        printf("%d  %d \n", size31,size32);
        for i=1:size31
            for j=1:size32
                tranVecErr(i,j) = tranVecErr(i,j) + tvE(i,j);
            end
        end
                   
                    
        printf("\n\t\t\tStandard Errors of Estimated Camera Parameters\n");
        printf("\t\t\t----------------------------------------------\n\n");  
        printf(" Intrinsics\n");
        printf(" ----------\n");
        printf(" Focal length (pixels):          [\t");
        printf("%s\t\t%s\t]\n", focalErr(1), focalErr(2));
        printf(" Principal point (pixels):       [\t");
        printf("%s\t\t%s\t]\n", prinErr(1), prinErr(2));
        printf(" Radial distortion:              [\t");
        for i=1:size1
           printf("%s\t\t", radErr(i));
        end
        printf("\t]\n\n");           
        printf(" Extrinsics\n");
        printf(" ----------\n");
        printf(" Rotation vectors:\n");
        for i=1:size21
        printf("                                 [\t");
            for j=1:size22
                printf("%s\t\t", rotVecErr(i,j));
            end
            printf("\t]\n");
        end
        if size21*size22==0 then
            printf("\n");
        end                
        printf(" Translation vectors (mm):\n");
        for i=1:size21
            printf("                                 [\t");
            for j=1:size22
                printf("%s\t\t", tranVecErr(i,j));
            end
            printf("\t]\n");
            
        end
        printf("\n");
               
    else
        stereoParam = varargin(2);
        checkSCE(errors);       
        if ~isstruct(stereoParam) then
            error(msprintf(" A Stereo Parameter Struct expected"))            
        end        
        jj = fieldnames(stereoParam)
        [size4 zz] = size(jj);  
        if size4~=10 then
            error(msprintf("A Stereo Parameters Struct Expected"))
        end
        checkStereoParam(stereoParam);
        focalErr1 = string(stereoParam.CameraParameters1.FocalLength) + " +/- ";
        fE1 = string(errors.Camera1IntrinsicsErrors.FocalLengthError) ;
        prinErr1 = string(stereoParam.CameraParameters1.PrincipalPoint) + " +/- ";
        pE1 = string(errors.Camera1IntrinsicsErrors.PrincipalPointError);        
        radErr1 = string(stereoParam.CameraParameters1.RadialDistortion) + " +/- ";    
        rdE1 = string(errors.Camera1IntrinsicsErrors.RadDistortionError);
        for i=1:2
            focalErr1(i) = focalErr1(i) + fE1(i);
            prinErr1(i) = prinErr1(i) + pE1(i);
        end
        [size11 xx] = size(radErr1); 
        size11 =size11*xx;
        for i=1:size11
            radErr1(i) = radErr1(i) + rdE1(i);
        end
        
        focalErr2 = string(stereoParam.CameraParameters2.FocalLength) + " +/- ";
        fE2 = string(errors.Camera2IntrinsicsErrors.FocalLengthError) ;
        prinErr2 = string(stereoParam.CameraParameters2.PrincipalPoint) + " +/- ";
        pE2 = string(errors.Camera2IntrinsicsErrors.PrincipalPointError);        
        radErr2 = string(stereoParam.CameraParameters2.RadialDistortion) + " +/- ";    
        rdE2 = string(errors.Camera2IntrinsicsErrors.RadDistortionError);
         for i=1:2
            focalErr2(i) = focalErr2(i) + fE2(i);
            prinErr2(i) = prinErr2(i) + pE2(i);
        end
        [size12 xx] = size(radErr2); 
        size12 =size12*xx;
        for i=1:size12
            radErr2(i) = radErr2(i) + rdE2(i);
        end        

        rotVecErr = string(stereoParam.CameraParameters1.RotationVectors) + " +/- ";
        rvE = string(errors.Camera1ExtrinsicsErrors.RotVectorsError);
        [size21 size22] = size(rotVecErr);
        printf("%d  %d \n", size21,size22);
        for i=1:size21
            for j=1:size22
                rotVecErr(i,j) = rotVecErr(i,j) + rvE(i,j);
            end
        end
        
        tranVecErr = string(stereoParam.CameraParameters1.TranslationVectors) + " +/- ";
        tvE = string(errors.Camera1ExtrinsicsErrors.TransVectorsError);
        [size31 size32] = size(tranVecErr);
        printf("%d  %d \n", size31,size32);
        for i=1:size31
            for j=1:size32
                tranVecErr(i,j) = tranVecErr(i,j) + tvE(i,j);
            end
        end

        rotOfCam = stereoParam.RotationOfCamera2;
        rotOfCamErr = string([rotOfCam(1,1) rotOfCam(2,1) rotOfCam(3,1)]) + " +/- ";
        rotE = string(errors.RotationOfCamera2Error);
        for i=1:3
            rotOfCamErr(i) = rotOfCamErr(i) + rotE(i); 
        end
        
        transOfCamErr = string(stereoParam.TranslationOfCamera2) + " +/- ";
        toCE = string(errors.TransOfCamera2Error)   ;
        for i=1:3
            transOfCamErr(i) = transOfCamErr(i) + toCE(i);
        end
            


        printf("\n\t\t\tStandard Errors of Estimated Stereo Camera Parameters\n");
        printf("\t\t\t-----------------------------------------------------\n\n");  
        printf(" Camera 1 Intrinsics\n");
        printf(" -------------------\n");
        printf(" Focal length (pixels):         [\t");
        printf("%s\t\t%s\t]\n", focalErr1(1), focalErr1(2));
        printf(" Principal point (pixels):      [\t");
        printf("%s\t\t%s\t]\n", prinErr1(1), prinErr1(2));
        printf(" Radial distortion:             [\t");      
        for i=1:size11
           printf("%s\t\t", radErr1(i));
        end
        printf("\t]\n\n");           
        printf(" Camera 1 Extrinsics\n");
        printf(" -------------------\n");
        printf(" Rotation vectors:\n");
        for i=1:size21
        printf("                                [\t");
            for j=1:size22
                printf("%s\t\t", rotVecErr(i,j));
            end
            printf("\t]\n");
        end
        if size21*size22==0 then
            printf("\t]\n");
        end                
        printf(" Translation vectors (mm):\n");
        for i=1:size21
            printf("                                [\t");
            for j=1:size22
                printf("%s\t\t", tranVecErr(i,j));
            end
            printf("\t]\n");            
        end
        printf("\n"); 
        printf(" Camera 2 Intrinsics\n");
        printf(" -------------------\n");
        printf(" Focal length (pixels):         [\t");
        printf("%s\t\t%s\t]\n", focalErr2(1), focalErr2(2));
        printf(" Principal point (pixels):      [\t");
        printf("%s\t\t%s\t]\n", prinErr2(1), prinErr2(2));
        printf(" Radial distortion:             [\t");
        for i=1:size12
           printf("%s\t\t", radErr2(i));
        end 
        printf("\t]\n\n");                          
        printf(" Position And Orientation of Camera 2 Relative to Camera 1\n");
        printf(" ---------------------------------------------------------\n");
        printf(" Rotation of camera 2:           [\t");
        for i=1:3
           printf("%s\t\t", rotOfCamErr(i));
        end        
            printf("\t]\n");
        printf(" Translation of camera 2 (mm):   [\t");
        for i=1:3
           printf("%s\t\t", transOfCamErr(i));
        end 
        printf("\t]\n");          
    end   

endfunction

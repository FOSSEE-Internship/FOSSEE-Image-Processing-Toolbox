// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function mask =roiFreeHand(Image)         
// This is a gui based function which is used to create a mask by selecting the region of interest
//
// Calling Sequence
// src = imread(image-location-for-src)
// mask = roiFreeHand(src)
//
// Parameters
// mask: Output 8 bit mask image with same size as input image
// image: Input image to be masked
//
// Description
// This function allows the user to create a mask by selecting the region of interest in the image.
// Start selecting the region of interest by pressing the left mouse button and moving in the clockwise direction. 
// Don't leave the left mouse button until you are done selecting the region. Once the region has been selected, 
// wait until the process completes.
// More than one region of interest can be selected by repeating the same procedure.
// Once you are done selecting the ROI, press the small 's' key.This would stop the process.
// This function returns a 8 bit mask image  with (the ROI being white and the rest of the region being black).
//
// Note
// more than one ROI can be selected from the image. Press key 's' only after all ROI have been selected.
// Move the mouse only in the clockwise direction.
// This algorithm assumes that the selected points in the ROI are very close to each other. Thus for best result, move 
// the mouse slowly while selecting the ROI
//
// Examples
// 
// src = imread("/images/color2.jpeg");
// mask = roiFreeHand(src); //creating a mask
// imshow(mask) //view the mask
//
// outputImage1 = colorChange(src, mask) //using this mask in the colorChange function
// imshow(outputImage1);//view the output
//
// Authors
//  Rohan Gurve
    
    imshow(Image);
     
    FigureHandle = gcf();
    FigureHandle.figure_name = "Create mask - Select ROI ";
    Diagram=gca();
    //Diagram.axes_visible = ['off' 'off' 'off'];
    //Diagram.isoview = 'on';
      ImageList=mattolist(Image); 
    [rows cols]=size(ImageList(1));
    
    temp_mask = zeros(ImageList(1));
    while(1) do
      
      pointsList = list(); 
       
       while(1) //start selection of points only when the user presses the left  mouse button
        [b,x_new,y_new]=xclick();
         if(b==0 & ( (x_new <= cols) & (x_new>=0) & (y_new<= rows) & (y_new>=0))) then//left mouse button has been pressed 
            break;                                                          //consider the point only if it lies inside the image         
         elseif(b==115) //key s has been pressed
            break;
         end; 
       end;   
            
        if(b==0) then
            
            
            x_new = ceil(x_new); y_new = ceil(y_new);    
            pointsList($+1)=[x_new y_new];
            
            rep=[x_new,y_new,0];
            plot([x_new x_new], [y_new y_new],'rx');        
                while rep(3)~=-5 do // left mouse button has been pressed
                  rep=xgetmouse([%t %t]);
                  x_temp= ceil(rep(1));
                  y_temp= ceil(rep(2));
                  if( (x_temp <= cols) & (x_temp>=0) & (y_temp<= rows) & (y_temp>=0)) then// consider the point only if it lies inside the image 
                      x_old=x_new;y_old=y_new;  
                      x_new=x_temp;y_new=y_temp;
                      x = [x_old x_new];
                      y = [y_old y_new];
                      
                      pointsList($+1)=[x_new y_new];
                      
                        plot(x, y);
                   end;                 
                end;
             
       //-**ROI has been selected - the computation of the ROI will take palce now **-/        
       
       //first marking the boundary
       FigureHandle.figure_name = "Wait (processing.............)";
    
       
       for i=1: (length(pointsList) -1)
           //intensity;
           if( (pointsList(i+1)(1,1) - pointsList(i)(1,1)) < 0) //moved from right to left 
             intensity = 0; //negative area
           
                   if( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) < 0) //moved from up to down 
                    //-- moved from up to down  & from right to left  --//
                      increase_ht = 0 ;
                     for j=pointsList(i+1)(1,1):pointsList(i)(1,1)
                        temp_mask(rows - pointsList(i+1)(1,2) - increase_ht  +1 ,j) = 1; //marking the boundary as 1
                          
                         increase_ht=increase_ht+1;
                                           
                     end;   
                         
                    elseif( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) > 0)  //moved from down to up 
                    //-- moved from down to up  & from right to left --// 

                     increase_ht = 0 ;
                     for j=pointsList(i+1)(1,1):pointsList(i)(1,1)
                        temp_mask(rows - pointsList(i+1)(1,2) + increase_ht  +1 ,j) = 1; //marking the boundary as 1
                          
                        increase_ht=increase_ht + 1;
                     end;   
                    
                    else//both points have same height
                       for j=pointsList(i+1)(1,1):pointsList(i)(1,1)
                        temp_mask(rows - pointsList(i+1)(1,2)  +1 ,j) = 1; //marking the boundary as 1
                        end;
                    
                    end
                    
                
           elseif( (pointsList(i+1)(1,1) - pointsList(i)(1,1)) > 0)
             intensity =255;//positive area if movement from left to right
           
                 if( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) < 0) //moved from up to down 
                 //-- moved from up to down  and from left to right --//

                 increase_ht = 0 ;
                 for j=pointsList(i)(1,1):pointsList(i+1)(1,1)
                    temp_mask(rows - pointsList(i)(1,2) + increase_ht  +1 ,j) = 1; //marking the boundary as 1
                      
                    increase_ht=increase_ht + 1;
                 end;   
                     
                 
                 
                elseif( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) > 0) //moved from down to up
                 //-- moved from down to up and from left to right --//
                 
                   increase_ht = 0 ;
                     for j=pointsList(i)(1,1):pointsList(i+1)(1,1)
                        temp_mask(rows - pointsList(i)(1,2) - increase_ht  +1 ,j) = 1; //marking the boundary as 1
                        
                        increase_ht=increase_ht+1;
                     end;   
                 else //both points have same height
                        for j=pointsList(i)(1,1):pointsList(i+1)(1,1)
                        temp_mask(rows - pointsList(i)(1,2) +1 ,j) = 1; //marking the boundary as 1
                        
                        //increase_ht=increase_ht+1;
                        end;   
                     
                 
               end
            
            else//both coordinate have the same x coordinate
                if( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) < 0) //moved from up to down 
                    for j=0:(pointsList(i)(1,2) - pointsList(i+1)(1,2))
                        temp_mask(rows - pointsList(i)(1,2) + j +1 ,pointsList(i)(1,1)) = 1; //marking the boundary as 1
                    end      
                else
                for j=0:(pointsList(i+1)(1,2) - pointsList(i)(1,2)) //moved from down to up
                        temp_mask(rows - pointsList(i)(1,2) - j +1 ,pointsList(i)(1,1)) = 1; //marking the boundary as 1
                    end 
                
                end 
                
          end  //if
       
      end //for loop 

       
       //next marking the area
        for i=1: (length(pointsList) -1)
           //intensity;
           if( (pointsList(i+1)(1,1) - pointsList(i)(1,1)) < 0) //moved from right to left 
             intensity = 0; //negative area
           
                 
                 if( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) < 0) //moved from up to down 
                   //-- moved from up to down  & from right to left  --//
                     
                     for j=0:(pointsList(i)(1,1)-pointsList(i+1)(1,1))
                         increase_ht = 1 ;
                        while( ((rows - pointsList(i+1)(1,2) -j + increase_ht  +1 ) <= rows) &  (temp_mask(rows - pointsList(i+1)(1,2) -j + increase_ht  +1 ,pointsList(i+1)(1,1)+j) ~ = 1) ) //loop until it reaches the any boundary or image border
                         temp_mask(rows - pointsList(i+1)(1,2) -j + increase_ht  +1 ,pointsList(i+1)(1,1)+j) = 0; //marking the underneath area as 0
                          increase_ht=increase_ht + 1;  
                         
                        end;
                   end;   
                         
                     
                 elseif( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) > 0)  //moved from down to up 
                    //-- moved from down to up  & from right to left --// 
                 
                         
                     for j=0:(pointsList(i)(1,1)-pointsList(i+1)(1,1))
                        increase_ht = 1 ;
                        while( ((rows - pointsList(i+1)(1,2) + j + increase_ht  +1) <= rows) &   (temp_mask(rows - pointsList(i+1)(1,2)+j + increase_ht  +1 ,pointsList(i+1)(1,1)+j) ~= 1) ) //loop until it reaches any boundary or image border
                          temp_mask(rows - pointsList(i+1)(1,2) + j + increase_ht  +1 ,pointsList(i+1)(1,1)+j) = 0; //marking the underneath area as 0
                          increase_ht = increase_ht + 1 ;  
                        end;
                     end;    
                 
                 else //both point on same height
                        for j=0:(pointsList(i)(1,1)-pointsList(i+1)(1,1))
                        increase_ht = 1 ;
                            while( ((rows - pointsList(i+1)(1,2) + increase_ht  +1) <= rows) &   (temp_mask(rows - pointsList(i+1)(1,2) + increase_ht  +1 ,pointsList(i+1)(1,1)+j) ~= 1) ) //loop until it reaches any boundary or image border
                              temp_mask(rows - pointsList(i+1)(1,2) + increase_ht  +1 ,pointsList(i+1)(1,1)+j) = 0; //marking the underneath area as 0
                              increase_ht = increase_ht + 1 ;  
                            end;
                        end;
                 
                 
                 end //"moved from up to down" 'if' end
           
           elseif( (pointsList(i+1)(1,1) - pointsList(i)(1,1)) > 0)// moved from left to right
             intensity =255;//positive area if movement from left to right
           
                 if( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) < 0) //moved from up to down 
                 //--moved from up to down  and from left to right --//

                     for j=0:(pointsList(i+1)(1,1)-pointsList(i)(1,1))
                       increase_ht = 1 ;
                     
                        while(((rows - pointsList(i)(1,2) + increase_ht + j  +1) <= rows)  & (temp_mask(rows - pointsList(i)(1,2) + increase_ht +1 +j,j+pointsList(i)(1,1)) ~ = 1)  ) //loop until it reaches the any boundary or image border
                          temp_mask(rows - pointsList(i)(1,2) + increase_ht +j +1 ,j+pointsList(i)(1,1)) = 255; //marking the underneath area as 255
                          increase_ht = increase_ht + 1 ;  
                        end;
                     end;    
   
               elseif( (pointsList(i+1)(1,2) - pointsList(i)(1,2)) > 0) //moved from down to up
                 //-- moved from down to up and from left to right --//
                 
                         
                     for j=0:(pointsList(i+1)(1,1)-pointsList(i)(1,1))
                       increase_ht = 1 ;
                     
                        while( ((rows - pointsList(i)(1,2) + increase_ht -j  +1) <= rows ) & (temp_mask(rows - pointsList(i)(1,2) -j  + increase_ht  +1 ,pointsList(i)(1,1)+j) ~ = 1) ) //loop until it reaches the any boundary or image border
                          temp_mask(rows - pointsList(i)(1,2) - j + increase_ht  +1 ,pointsList(i)(1,1)+j) = 255; //marking the underneath area as 255
                          increase_ht = increase_ht + 1 ;   
                        end;
                     
                     end;   
                else
                    for j=0:(pointsList(i+1)(1,1)-pointsList(i)(1,1))
                       increase_ht = 1 ;
                     
                        while( ((rows - pointsList(i)(1,2) + increase_ht +1) <= rows ) & (temp_mask(rows - pointsList(i)(1,2) + increase_ht  +1 ,pointsList(i)(1,1)+j) ~ = 1) ) //loop until it reaches the any boundary or image border
                          temp_mask(rows - pointsList(i)(1,2)  + increase_ht  +1 ,pointsList(i)(1,1)+j) = 255; //marking the underneath area as 255
                          increase_ht = increase_ht + 1 ;   
                        end;
                     end;
                
                     
                     
                     
              end;//moved from up to down's if  end
            end; //if statement   
       end;//for loop's end
       
        FigureHandle.figure_name = "Create mask - Select ROI";
      
             //-** the computation of the selected ROI has been completed **-/ 
       //mask = pointsList;
       
      else //b==115
          break;//ROI has been selected - stopping the selection process
       
  end  //if(b==0)
   mask = temp_mask;
   
   end  //while(1) loop's end   
       
    
endfunction

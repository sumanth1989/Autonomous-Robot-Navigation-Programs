clc;
clear;        %Clear figures and variables
import mbed.*
% Bluetooth connection configuration to MBED
%mymbed = serial('COM11', 'BaudRate', 9600);
%Set-up
%1.  use command imaqhwinfo to determine the InstalledAdaptors
%1a. Install the OS General Video Interface if "winvideo" is not installed
%2.  use info=imaqhwinfo('winvideo') to determine the device ID of 
%    connected cameras
%3.  use info=imaqhwinfo('winvideo',ID) where ID is the device ID, to
%    determine the supported formats for that camera


%Reset the video adapters. (Useful for rerunning code)
imaqreset

%Declare a video variable (what i call it..  )
%inputs are (InstalledAdaptor,ID,supportedformat)
vid=videoinput('macvideo',1,'YCbCr422_1280x720');

%Initializes manual triggering
triggerconfig(vid, 'manual');

%Sets number of frames per trigger 
set(vid,'FramesPerTrigger',1 );

%Sets the number of triggering to be done
%in this case its infinte or "inf" since its a while loop 
set(vid,'TriggerRepeat', inf);

%Start the video
start(vid);

%open figure and make it as large as possible
f = figure;

fopen(mymbed);

while(1)% The while
    obj = 0
    tic;
    %Triggers the video to capture the image
    trigger(vid);
    
    %Gets the image and place in the image variable �im�
    im=getdata(vid,1);
   
    %Flushes the video variable vid to improve memory usage
    %and also speed up the triggering process
    flushdata(vid);
    
    %Converts the color space from ycbr to rgb (rgb display)
    %im=ycbcr2rgb(im);
    
    %Show the rgb image
    imshow(im,'InitialMagnification','fit')
    
    title('Color Image Feed')
    im = imresize(im,[640 480]);  % Resize Image to required dimensions as per assignment requirement
   
    % Get RGB color intensities from image
    im_red = im(:,:,1);
    im_green = im(:,:,2); 
    im_blue = im(:,:,3);
     
    % Calculate threshold and convert image to binary format
    level = 200/256; 
    bw_red = im2bw(im_red,level);
    bw_blue = im2bw(im_blue,level);
    bw_green = im2bw(im_green,level);
    
    % Color segregation
    color_red=(bw_red) & (~bw_green) & (~bw_blue);
    %bwmorph(color_red,'clean');
    color_yellow=(bw_red) & (bw_green) & (~bw_blue);
    %bwmorph(color_yellow,'clean');
    color_white=(bw_red) & (bw_green) & (bw_blue);
     
    % Shape recognition for stop sign
    oct_cc = bwconncomp(color_red);
    oct = regionprops(oct_cc,'Area','Perimeter','Extent');
    num_oct = size(oct,1);
    oct_area = [oct.Area];
    oct_perimeter = [oct.Perimeter];
    oct_extent = [oct.Extent];
    
    % Shape recognition for Yield sign
    tri_cc = bwconncomp(color_yellow);
    tri = regionprops(tri_cc,'Area','Perimeter','Extent');
    num_tri = size(tri,1);
    tri_perimeter = [tri.Perimeter];
    tri_area = [tri.Area];
    tri_extent = [tri.Extent];
    
    
    % Shape recognition for Change of speed sign
    sq_cc = bwconncomp(color_white);
    sq = regionprops(sq_cc,'Area','Perimeter','Extent');
    num_sq = size(sq,1);
    sq_perimeter = [sq.Perimeter];
    sq_area = [sq.Area];
    sq_extent = [sq.Extent];
    
   % Thresholds for detecting Stop sign
    for count = 1:num_oct
        if oct_area(count)>9000
             oct_circ = oct_perimeter(count).^2./(4*pi*oct_area(count));
            if 0.3 <=oct_extent(count) && oct_extent(count)>= 0.4 && 0.8<= oct_circ && oct_circ<= 1.4
                obj = 1
                fprintf(mymbed, '1');
                break
            end
        end
    end
    
    % Threshold for detecting Yield sign
    for counter = 1:num_tri
         if tri_area(counter) > 6000   
          tri_circ = tri_perimeter(counter).^2./(4*pi*tri_area(counter));
            if   0.4<= tri_extent(counter) && tri_extent(counter)<= 0.6 && 1.4<= tri_circ && tri_circ<=2
             obj = 2
             fprintf(mymbed,'2');
                 break
            end
         end
    end
    
    % Threshold for detecting Speed sign
    for counters = 1:num_sq
         if sq_area(counters) > 6000   
            if 0.8<= sq_extent(counters) && sq_extent(counters)<= 1.2
            obj = 3
            fprintf(mymbed,'3');
                 break
            end
         end
    end
    
    % Wait state as per assignment requirements of 1 frame every 2 seconds
    pause(2); 
    
    % Press a to stop code execution
    if(get(f,'currentkey') == 'a')
       break;
    end
end

fclose(mymbed);
%use this to remove the video object from the memory
delete(mymbed);
clear('mymbed');
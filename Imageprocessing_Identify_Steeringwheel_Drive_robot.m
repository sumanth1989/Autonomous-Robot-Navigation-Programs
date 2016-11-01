close all;
clear;        %Clear figures and variables

import mbed.*
mymbed = serial('/dev/tty.usbmodem1422', 'BaudRate', 9600);
turn_angle = 0

imaqreset


vid=videoinput('macvideo',2,'ARGB32_1920x1080');

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
    
    %Triggers the video to capture the image
    trigger(vid);
    
    %Gets the image and place in the image variable �im�
    im=getdata(vid,1);
    
    %Flushes the video variable vid to improve memory usage
    %and also speed up the triggering process
    flushdata(vid);
    
    %% Show the rgb image
    im = fliplr(im);
    imshow(im,'InitialMagnification','fit');
    title('Color Image Feed')
    im = imresize(im,[640 480]);
    
   
   %% Get  color intensities from image
    im_red = im(:,:,1);
    im_green = im(:,:,2); 
    im_blue = im(:,:,3);
     
    %% Calculate threshold and convert image to binary format
    level = 220/256; 
    bw_red = im2bw(im_red,level);
    bw_blue = im2bw(im_blue,level);
    bw_green = im2bw(im_green,level);
    
    %% Color elimination
    
    color_red=(bw_red) & (~bw_green) & (~bw_blue);
    rect_cc = bwconncomp(color_red);
    rect = regionprops(rect_cc,'Extent','Orientation','Area','Perimeter');
    rect_area = [rect.Area];
    rect_extent = [rect.Extent];
    rect_perimeter = [rect.Perimeter];
    rect_orientation = [rect.Orientation];
    num_rect = size(rect,1);
    
    %% Calculating the turn angle and speed
    for count = 1: num_rect
        
        if   rect_area(count) > 4000 && rect_area(count) <= 12000
                
                robot_speed = 'S'
                fprintf(mymbed,robot_speed);
                turn_angle = rect_orientation(count)
                fprintf(mymbed,turn_angle);
                break
                
        elseif   rect_area(count) > 12000 && rect_area(count) <= 20000
                
                robot_speed = 'L'
                fprintf(mymbed,robot_speed);
                turn_angle = rect_orientation(count)
                fprintf(mymbed,turn_angle);
                break
                
        elseif   rect_area(count) > 20000 
                
                robot_speed = 'H'
                fprintf(mymbed,robot_speed);
                turn_angle = rect_orientation(count)
                fprintf(mymbed,turn_angle);
                break
                
        end
        
    end
    
end

fclose(mymbed);
%use this to remove the video object from the memory
delete(mymbed);

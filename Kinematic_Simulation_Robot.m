clear
clc
close all

%import the robot model for the simulation
load robot

%Model Coefficients
p1 = 0;
p2 = 30.79;

robot_width = 0.125;
wheel_radius = 0.021;

%time step
dt = 0.01;

%playback speed multiplier, something to make the simulation run a bit
%faster
play_speed = 1;

%PWM commands for left and right wheel and the amount of time for each input
LeftInput = [0.2; 0.3; 0.2; 0.3; 0.375; 0.3; 0.4; 0.1; 0.3; 0.2; 0.3; 0.4; ...
             0.5; 0.25; 0.15; 0.3; -0.2; 0.3; 0.1; 0.3; -0.1; 0.3; 0.1; 0.3];

RightInput = [0.4; 0.3; 0.4; 0.3; 0.13; 0.3; 0.17; 0.5; 0.3; -0.2; 0.3; 0.1; ...
              0.0; 0.25; 0.4; 0.3; 0.4; 0.3; 0.4; 0.3; 0.4; 0.3; 0.4; 0.3];
          
Times = [1.8559;0.4;2.3;1.1;5.7;0.75;1.2;1.5;0.8;1.5;1.9;0.5;1.5;0.6; ...
         1.1;1.9;1.05;1.6;1.7;0.8;1.4;0.5;1.1;1.1];

%Convert commands to useful stuff (angular velocities rad/sec:D ); originally had a 2nd
%order conversion, but now it's a linear relationship for pwm to speed
%conversion; changed the values of p1 and p2 to reflect the linear equation
leftSpeeds = p1*LeftInput(:,1).^2 + p2*LeftInput(:,1);
rightSpeeds = p1*RightInput(:,1).^2 + p2*RightInput(:,1);

%Puts the time into a linear relationship
timeIdx = zeros(length(Times),1);
for j = 2:length(Times)+1
    timeIdx(j) = timeIdx(j-1) + Times(j-1);
end

%Integrals! 
k = 1;
t = (0:dt:timeIdx(end))';
x = zeros(length(t),3);

%starting orientation of the robot and it's position
x(1,:) = [2.7925,0,0.8];

for j = 2:length(t)
    %check for new command
    if timeIdx(k+1) < t(j)
        k = k + 1;
    end
    
    %calculate derivatives
    df = @(x)([wheel_radius*(rightSpeeds(k)-leftSpeeds(k))/(robot_width), ...
               wheel_radius*(leftSpeeds(k)+rightSpeeds(k))*cos(x(1))/2, ...
               wheel_radius*(leftSpeeds(k)+rightSpeeds(k))*sin(x(1))/2]);
    
    %Use RK4 (runge-kutta) method
    K1 = dt*df(x(j-1,:));
    K2 = dt*df(x(j-1,:)+0.5*K1);
    K3 = dt*df(x(j-1,:)+0.5*K2);
    K4 = dt*df(x(j-1,:)+K3);
    x(j,:) = x(j-1,:)+1/6*(K1+2*K2+2*K3+K4);
end

%Plotting in 3D 
ax = axes('Xlim',[-2,2],'YLim',[-2,2],'Zlim',[0,0.2]);
handletrans = hgtransform;
view(3);
axis equal
grid on;
xlabel 'X Axis: meters';
ylabel 'Y Axis: meters';
zlabel 'Z Axis: meters';

handlepatch = patch(robotFV, 'FaceColor', [.2 .4 1.0], 'EdgeColor', ...
                    'none', 'FaceLighting',    'gouraud', ...
                    'AmbientStrength', 0.15, 'Parent', handletrans);

axis(ax,[min(x(:,2))-0.1 max(x(:,2))+0.1 min(x(:,3))-0.1 max(x(:,3))+0.1 0 0.2])

%Add a camera light, and tone down the specular highlighting
camlight('right');
material('metal');
hold on
handleplot = plot(ax,x(1:2,2),x(1:2,3),'red');
hold off

%plot out the spectacular robot model and its path
for j = 1:play_speed*5:length(t)
    set(handletrans,'Matrix',makehgtform('translate',[x(j,2),x(j,3),0],'zrotate',x(j,1)));
    set(handleplot,'Xdata',x(1:j,2),'YData',x(1:j,3));
    pause(0.05);
end




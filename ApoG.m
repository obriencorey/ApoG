%data from thrustcurves.com
%must be in seconds/newtons

%load such that no headers are loaded.
%Tdata = csvread('AeroTech_G64W.csv',5); %Units of seconds and newtons
%Tdata = csvread('AeroTech_H219T.csv',5);
%Tdata = csvread('AeroTech_I1299N.csv',5);
%Tdata = csvread('Cesaroni_338I180-14A.csv',5);
Tdata = csvread('AeroTech_L2200G.csv',5);
fclose('all');

t = Tdata(:,1);
F = Tdata(:,2);

%All inputs
mass = 25; %kg;
g = 9.81;
refTemp = 288.15; %k

%general geometric data
noseconelength = 0.419; 
bodylength = 0.787 + 0.483; 
bodydiameter = 4 / 39.37;

%fin data
rootchord = 0.37;
tipchord = 0.0856;
span = 0.0965;
finnumber = 3;
avgchord = (0.37 + 0.0856)/2;
%calculate the area using average chord length and span
finarea = .5 * avgchord * span;
%reference area for CD calc
A = pi * ((bodydiameter) / 2)^2;
%parachute is 36" in diameter
Aparachute = pi * ((36 / 39.37) / 2)^2;

%Set to 1 if simulating deployment at apogee
deployAtApogee = 0;
%Set to 0 if using delay time
deployAltitude = 0; %meters
%Set to 1000 if using deployAltitude
delayTime = 14;

%assume CDmin on way up
%CDmax on way down
CDmin = 0.55;
CDmax = 0.75;

%From fruity chutes.com for 36" parachute
CDparachute = 1.87;
flighttime = 220; %seconds, estimate for the flight time
 

%Resolution Modifier; makes thrust curve less discrete
Res = ceil(100 / length(t));
%User interpolation to increase resolution
t_extended = interp(Tdata(:,1),Res);
F_extended = interp(Tdata(:,2),Res);

%calculate number of time steps
timesteps = length(t_extended);

%determine burntime and estimate flight time
burntime = t_extended(end);

%number of samples for calculations
n = flighttime*50;
time = transpose(linspace(0,flighttime,n));
timeperstep = flighttime / n;

%calculate density as function of altitude
altitude = linspace(0,10000,n);

%equation from wikipedia using standard atmosphere lapse rate.
density = 1.225*((refTemp)./ (refTemp + -0.0065.*(altitude))).^(1 + (g*0.0289644)/(8.3144598*-0.0065)); 
%temperature as function of altitude
temperature = refTemp+ -0.0065.*(altitude);
%dynamic viscosity as function of altitude
mu = (1.458e-6.*temperature.^(3/2))./(temperature + 110.4);


acceleration = zeros(n,1);
accelerationgs = zeros(n,1);
velocity = zeros(n,1);
position = zeros(n,1);

acceleration(1) = 0;
velocity(1) = 0;
position(1) = 0;

%approximating fins as flat plate drag
%using laminar and turbulent shear stress
%first generate matrix of positions along fin
fincoords = linspace(0,avgchord,100);
fincoordsdx = avgchord/100;

for index = 2:n-1
    
    %calculate force from motor
    %if motor is still burning
    if(time(index) <= burntime)
        %map from the time matrix to the motor time matrix
        %find closest value to avoid small errors
        [minValue,closestIndex] = min(abs(t_extended-time(index)));
        
        %Thrust will be found using the closest index
        fThrust = F_extended(closestIndex);
        
        if(fThrust < 0)
            fThrust = 0;
        end
       
    else
        fThrust = 0;
    end
    
    %find value of rho to use for drag equation
    %we want density at current altitude
    [minValue,closestIndex] = min(abs(altitude-position(index)));
    rho = density(closestIndex);
    
    %calculate drag based on flat plate theory
    %first generate reynolds numbers based on position
    re = (rho*velocity(index).* fincoords)./mu(closestIndex);
    
    %loop through and calculate skin friction coefficient
    skindrag = 0;
    for index2 = 1:length(fincoords)
        if(re(index2) > 5e5)
            Cf = 0.664 / sqrt(re(index2));
            %last time this part of the statement is entered,
            %it will record the transition region
            tranlocation = fincoords(index2);
        else
            Cf = 0.027 / re(index2)^(1/7);
        end
        
        skindrag = skindrag + rho*velocity(index)^2 / 2 * fincoordsdx * span;
        
    end
    skindrag = skindrag * 0.01;
    
    
    
    if(velocity(index) >= 0)
        %if going up then use this equation for acceleration
        dragForce = -.5*(rho/mass)*velocity(index)^2*A*CDmin - skindrag;
    else
        %if below deploy altitude - 5 (5 is for time for chute to open
        %then use drag of parachute to calculate drag
        if(position(index) <= (deployAltitude-5) || deployAtApogee || time(index) >= delayTime)
            %if going down then use this equation for acceleration
            %drag of parachute
            %and drag of rocket 
            dragForce = .5*(rho/mass)*velocity(index)^2*Aparachute*CDparachute + .5*(rho/mass)*velocity(index)^2*A*CDmax;
        else
            %if going down then use this equation for acceleration
            dragForce = .5*(rho/mass)*velocity(index)^2*A*CDmax;
        end
    end
        acceleration(index+1) = fThrust/mass + dragForce - g;
        velocity(index+1) = velocity(index) + acceleration(index+1)*timeperstep;
        position(index+1) = position(index) + velocity(index+1)*timeperstep;
    
    %fprintf("Acceleration: %f \tFthrust: %f \n",acceleration(index),fThrust);
    
    if(fThrust == 0 && position(index) <= 0)
        acceleration(index + 1) = 0;
        velocity(index + 1) = 0;
        position(index + 1) = 0;
    end
    
end

for i = 1:n
    if(acceleration(i)/9.81 < 10)
        accelerationgs(i) = acceleration(i)/9.81;
    else
        accelerationgs(i) = 1;
    end
end

[minValue,closestIndex] = min(abs(time-burntime));
timeatburnout = time(closestIndex);

fig = figure;

left_color = [0 0 0];
right_color = [0 .5 .5];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);

grid on
hold on
positionfeet = position.*3.28084;
yyaxis left
ylabel("Altitude (feet)")
plot(time,positionfeet)
plot(timeatburnout, positionfeet(closestIndex),'r.','MarkerSize',15)
yyaxis right
ylim([-2 15])
ylabel("Acceleration (Gs)")
plot(time,accelerationgs)
title("Altitude vs. Time")
xlabel("seconds")
legend("Altitude",sprintf('Burnout = %.2f s',timeatburnout),"Acceleration");


 

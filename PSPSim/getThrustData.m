function thrust = getThrustData(time)
ThrustData = csvread('thrustcurves/Cesaroni_338I180-14A.csv',5);
%Increase discrete data resolution by factor of Res
Res = 20;

%Create more detailed matrices of data
time_extended = interp(ThrustData(:,1),Res);
thrust_extended = interp(ThrustData(:,2),Res);
burntime = time_extended(end);

if(time < burntime)
    %Search for discrete increment of time that is closest to inputted time
    [minValue,closestIndex] = min(abs(time_extended-time));
    %return corresponding value of thrust
    thrust = thrust_extended(closestIndex);
else
    %if motor has burned out assume thrust is 0
    thrust = 0;
end

end


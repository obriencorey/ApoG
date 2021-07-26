function readThrustData()
ThrustData = csvread('thrustcurves/Cesaroni_338I180-14A.csv',5);
disp(ThrustData)
end
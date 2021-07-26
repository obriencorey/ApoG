function [vehicledata] = getVehicleData();
%%% Purpose of this function is to input general
%%% vehicle parameters and to output a basic diagram
%%% of the vehicle. This function can then be called by various
%%% simulations to produce accurate flight estimates

    %%% overall vehicle mass
    mass = 0.5; %kg
    
    %%% overall vehicle length
    length = 1.3; %m
    
    %%% overall vehicle diameter
    diam = 0.1; %m
    
    %%%length of nosecone
    nclength = 0.3; %m
    
    %%%create diagram of rocket
    x = linspace(0,length,1000);
    
    R_0 = ((diam/2)^2 + nclength^2) / (2*diam);
    
    x_nose = linspace(0,nclength,100);
   
    nosecone = sqrt(R_0^2 - (L - x_nose).^2) + diam/2 - R_0
    
    
    
    vehicledata = [vehiclemass, vehiclelength];

end
    %%%resolution
    resolution = 1000;
    %%% overall vehicle mass
    mass = 0.5; %kg
    
    %%% overall vehicle length
    len = 2.5; %m
    
    %%% overall vehicle diameter
    diam = 0.2; %m
    
    %%%length of nosecone
    nclength = 0.4; %m
    
    %%%fin root chord 
    rchord = 0.2; %m
    
    %%%fin tip chord
    tchord = 0.05; %m
    
    %%%fin span
    fspan = 0.3; %m
    
    %%%chord off
    
    %%%fin fwd sweep angle (measured from body tube to leading edge)
    fwdangle = 40; %degrees
    
    %%%fin aft sweep angle (measure from trailing edge to body tube)
    aftangle = 20; %degrees
    
    %%%fin offset (distance from trailing edge to edge of body tube)
    finoffset = 0.4; %m
    
    %%%create diagram of rocket
    
    %%%create body section
    x = linspace(nclength,len,resolution);
    body = zeros(1,length(x));
    body = body + diam/2;
    
    %%%create nosecone
    R_0 = ((diam/2)^2 + nclength^2) / (diam);
    x_nose = linspace(0,nclength,1000);
    %%%equation for ogive nosecone
    nosecone = sqrt(R_0^2 - (nclength - x_nose).^2) + diam/2 - R_0;
    
    %%%create fins
    %create matrix from front of fins to back of fins
    finfront = len-finoffset-rchord;
    finback = len-finoffset+rchord;
    
    x_fins = linspace(finfront,finback,resolution);
    fins = zeros(1,length(x_fins));
    
    for i = 1:resolution
        if(i < 2)
            fins(i) = diam/2;
        else
            if (x_fins(i) > finfront && x_fins(i) < finfront + fspan * cos(fwdangle * pi / 180))
                %leading angle portion of fin
                fins(i) = diam/2 + tan(fwdangle * pi / 180) * (x_fins(i) - finfront);
            else if (x_fins(i) > finfront + fspan * cos(fwdangle * pi / 180) && x_fins(i) < finfront + fspan * cos(aftangle * pi / 180))
                %straight portion 
                fins(i) = fspan;
                else
                    %trailing angle portion of fin
                    fins(i) = fspan - tan(aftangle * pi / 180) * (x_fins(i) - finfront);
                    disp("yo")
                end
            end
        end
    end
    
    
    
    hold on
    axis equal
    grid on
    plot(x_nose,nosecone,'k')
    plot(x,body,'k')
    plot(x_nose,-nosecone,'k')
    plot(x,-body,'k')
    plot(x_fins,fins,'k')
    plot(x_fins,-fins,'k')
    line([x(end) x(end)],[diam/2 -diam/2],'Color','black')
    xlim([-.3 len+.3]);
    
%%%print in inches
inches = 1;

%%%number of fins, right now only works for 3 or 4 fins
numfins = 4;

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
rchord = 0.4; %m

%%%fin tip chord
tchord = 0.1; %m

%%%fin span
fspan = 0.2; %m

%%%chord off

%%%fin fwd sweep angle (measured from body tube to leading edge)
fwdangle = 40; %degrees

%%%fin aft sweep angle (measure from trailing edge to body tube)
aftangle = 70; %degrees

%%%fin offset (distance from trailing edge to edge of body tube)
finoffset = 0.1; %m

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
finback = len-finoffset;

x_fins = linspace(finfront,finback,resolution);
fins = zeros(1,length(x_fins));

for i = 1:resolution
    if(i < 2)
        fins(i) = diam/2;
    else
        if (x_fins(i) > finfront && x_fins(i) < finfront + fspan/tan(fwdangle * pi / 180))
            %leading angle portion of fin
            fins(i) = diam/2 + tan(fwdangle * pi / 180) * (x_fins(i) - finfront);
        else if (x_fins(i) > finfront + fspan/tan(fwdangle * pi / 180) && x_fins(i) < finfront + fspan/tan(fwdangle * pi / 180) + tchord)
            %straight portion 
            fins(i) = diam/2 + fspan;
            %fins(i) = diam/2 + tan(fwdangle * pi / 180) * (finfront + fspan * cos(fwdangle * pi / 180) - finfront);
            else
                %trailing angle portion of fin
                fins(i) = -(fspan / ((finfront + rchord) - (finfront + fspan/tan(fwdangle * pi / 180) + tchord)))*(x_fins(i)-(finfront + fspan/tan(fwdangle * pi / 180) + tchord)) + diam/2 + fspan;
                %fins(i) = fspan - tan(aftangle * pi / 180) * (x_fins(i) - finfront);
            end
        end
    end
end

    
    
hold on
axis equal
grid on
linewidth = 2;

if(numfins == 4)
    finscale = 1;
else
    finscale =  sin(60*pi/180);
end
if(inches)
    plot(39.3701*x_nose,39.3701*nosecone,'r','Linewidth',linewidth)
    plot(39.3701*x,39.3701*body,'k','Linewidth',linewidth)
    plot(39.3701*x_nose,39.3701*-nosecone,'r','Linewidth',linewidth)
    line([39.3701*x_nose(end) 39.3701*x_nose(end)],[39.3701*diam/2 39.3701*-diam/2],'Color','red','Linewidth',linewidth)
    plot(39.3701*x,39.3701*-body,'k','Linewidth',linewidth)

    plot(39.3701*x_fins,39.3701*fins*finscale,'b','Linewidth',linewidth)
    plot(39.3701*x_fins,39.3701*-fins*finscale,'b','Linewidth',linewidth)
    line([39.3701*x_fins(1) 39.3701*x_fins(end)],[0 0],'Color','blue','Linewidth',linewidth)



    line([39.3701*x(end) 39.3701*x(end)],[39.3701*diam/2 -39.3701*diam/2],'Color','black','Linewidth',linewidth)
    xlim([-.3*39.3701 (len+.3)*39.3701]);
    xlabel("in")
    ylabel("in")
else
    plot(x_nose,nosecone,'r','Linewidth',linewidth);
    plot(x,body,'k','Linewidth',linewidth)
    plot(x_nose,-nosecone,'r','Linewidth',linewidth)
    line([x_nose(end) x_nose(end)],[diam/2 -diam/2],'Color','red','Linewidth',linewidth)
    plot(x,-body,'k','Linewidth',linewidth)

    plot(x_fins,fins*finscale,'b','Linewidth',linewidth)
    plot(x_fins,-fins*finscale,'b','Linewidth',linewidth)
    line([x_fins(1) x_fins(end)],[0 0],'Color','blue','Linewidth',linewidth)


    line([x(end) x(end)],[diam/2 -diam/2],'Color','black','Linewidth',linewidth)
    xlim([-.3 (len+.3)]);
    xlabel("m")
    ylabel("m")
end
legend("nosecone","body");
title("diagram of launch vehicle");

time_period = [0 10];
initial = [21, 12];



[t,u] = ode45(@uderiv, time_period, initial);
plot(t,u(:,1))
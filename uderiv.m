function uderivative = uderiv(t,u)
rho = 1.225;
m = 5.44;
A = .12;
CD = 0.2;

uderivative = [u(2);-.5 * rho/m*u(2)^2];

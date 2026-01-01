function X = ode4input(odefun,t,X0,u)
 
h = diff(t);

n = length(X0);
N = length(t);
X = zeros(n,N);
F = zeros(n,4);

X(:,1) = X0;
for i = 2:N
  hi = h(i-1);
  xi = X(:,i-1);
  F(:,1) = feval(odefun,xi,u(i,:));
  F(:,2) = feval(odefun,xi+0.5*hi*F(:,1),u(i,:));
  F(:,3) = feval(odefun,xi+0.5*hi*F(:,2),u(i,:));  
  F(:,4) = feval(odefun,xi+hi*F(:,3),u(i,:));
  X(:,i) = xi + (hi/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));
end


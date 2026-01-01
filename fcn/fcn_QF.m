function [y,g,H] = fcn_QF(x,A,b,c)

y=0.5*x'*A*x+b'*x+c;
g=A*x+b;
H=A;

end
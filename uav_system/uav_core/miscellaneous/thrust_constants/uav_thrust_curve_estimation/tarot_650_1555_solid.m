% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
3.60;
4.2;
4.78;
];

% thrusts needed to hover
thrust = [
0.515;
0.59;
0.66;
];

n_motors = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g)/n_motors);
end

% print A
A

% compute the linear coeficients
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)

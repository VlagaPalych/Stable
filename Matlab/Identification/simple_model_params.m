% ������, ������� ��� ��������
% ��������� ����� � ������

g = 9.8;        % ��������� ���������� ������� (�/�^2)
rho = 1000;     % ��������� ���� (��/�^3)


% ����� � ����� ����������� ������
R = 0.08;       % ������ (�) 
L = 0.43;       % ����� (�) 
d = 0.019;      % ������ ��������� ����� � ��������� ��������� (�)
boat_m = 1;     % ����� (��)


% ���� � ����� ���������������
w = 0.04;       % ������ (�)
h = 0.08;       % ������ (�)
l = 0.05;       % ����� (�)
cargo_m = 0.3;  % ����� (��)


% ������ �������
cargo_I = cargo_m*(w^2 + h^2)/12 + cargo_m*(d + h/2)^2;
boat_I = boat_m*R^2 + boat_m*d^2 - 8*boat_m*d*R/3/pi; 
I = cargo_I + boat_I;


% ������� ���

% ������� ��������� ����� ������� ����� (������� ���������� ����� ��� ����� �����)
S_b = R^2*asin(sqrt(R^2-d^2)/R) - d*sqrt(R^2-d^2); % (�^2)

% ������� ��� ��� ������ ���� ����������
M_mg_k = (cargo_m*h/2 + boat_m*4*R/3/pi + (cargo_m+boat_m)*d)*g; % (�*�)
M_a_k = rho*g*L*S_b*d; % (�*�)


% ����������
b = 0.1; % ����� ����, ����������� ��������


% ������
% a_2y'' + a_0y' = bu

a2 = I;
a0 = -(M_mg_k - M_a_k);

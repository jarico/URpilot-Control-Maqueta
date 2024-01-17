
%% Parámetros del sistema de propulsión no lineal
Vmin = 12.5;
Vmax = 16.5;
tau  = 0.05;
MF   = [-6.7899 14.607 -0.4253 0.0213;
        -8.4461 16.962 -0.2084 0.0076; 
        -9.5828 18.733 0.1539 -0.0175; 
        -12.165 22.368 -0.0708 -0.018; 
        -12.638 23.102 0.6345 -0.0512];
MD   = [0.0103 0.0273 0.0082 0.0004; 
        -0.0179 0.0606 0.005 0.0001; 
        0.0107 0.032 0.0095 0.0002; 
        -0.0117 0.0697 0.0007 0.0007; 
        0.0227 0.0301 0.0117 -0.0002];
MV   = [0 -1075.7 3391.3 5.1514; 
        0 -1117.8 3467.7 31.49; 
        0 -1464 4031.2 0.4344; 
        0 -1601.8 4277.8 4.6327; 
        0 -1710.5 4430.8 10.996];

%% Parámetros del UAV y linealización del modelo

I  = [0.0014, 0.025, 0.0549];
Ir = 0.0005;
M  = 0.2472;
Dr = 0.09;
Dp = 0.0675;
v  = 12.5;
u  = 0.5;

F  = calculaThrust(v, u, MF, Vmin, Vmax);
P  = interpolaPoly(v, MF, Vmin, Vmax);

x  = 0 : 0.01 : 1;

% Pruebas interpolacion polinomica
for i= 1 : 4
    plot(x, polyval(MF(i,:), x)), hold on;
end
plot(x, polyval(P, x));

k = polyval(polyder(P),u);
disp(k)

% Verificación de la ganancia
Au=0.05;
disp('Valor real interpolado');
disp(calculaThrust(v, u + Au, MF, Vmin, Vmax));
disp('Valor obtenido con la ganancia');
disp(F + k*Au);

%% Calculo de la función de transferencia
s   = tf('s'); 
Pr  = 4 * Dr * k * 180 / pi / I(1) / (tau * s + 1) / s / s; 
dPr = 4 * Dr * k * 180 / pi / I(1) / (tau * s + 1) / s; 

Pp  = 4 * Dp * k * 180 / pi / I(2) / (tau * s + 1) / s / s; 
dPp = 4 * Dp * k * 180 / pi / I(2) / (tau * s + 1) / s / s; 

[numRoll, denRoll] = tfdata(dPr,'v');
[numPitch, denPitch] = tfdata(dPr,'v');

%% Control de orientación con lazo en cascada
Ts = 0.001;
z  = tf('z',Ts);

%% Especificaciones 
close all

% Estabilidad Robusta
MF = 40; % -> Defino el margen de fase deseado 
W1 = 0.5/(cos(pi*(180-MF)/(2*180)));

% Rechazo de perturbaciones
d  = 1;
ts = 4;
wn = 4/ts/d;
Wd = 2 * 12500 / 0.763 * wn^2 * s / (s^2 + 2*d*wn*s + wn^2);
figure, step(Wd);

% Especificaciones de seguimiento
% Cota para el seguimiento
close all
d=1;
ts=0.5;
wn=4/ts/d;
Wb=1/3*0.15*wn^2*s*(0.02*s+1)/(s^2+2*d*wn*s+wn^2);
step(Wb)

d   = 0.7;
ts  = 0.5;
wn  = 4/ts/d;
Mc  = wn^2/(s^2+2*d*wn*s+wn^2);
t   = 0:0.01:1.0;
y_m = step(Mc,t);
y_b = step(Wb,t);
figure,plot(t, y_m, t, y_m + y_b, t, y_m - y_b);

M   = c2d(Mc,Ts);

%M   = tf(0.0121,[1 -1.84  0.8521],Ts);


%% Plantas para el modelo discreto
c   = 1;
kt  = [0.8, 1, 0.9] * k;
Ix  = [0.8, 1, 0.9] * I(1);
int = tf(Ts,[1 -1], Ts);

for i=1:length(kt)
    for j=1:length(Ix)
        Pq(1,1,c) = 1 / Ix(j) * 180/pi * int;
        Pt(1,1,c) = 4 * Dr * kt(i) / (tau * s + 1);
        
        P1(1,1,c) = int;
        Id(1,1,c)  = tf(1,1);
        M2(1,1,c) = M;
        c         = c+1;
    end
end

Pt=c2d(Pt,Ts);
P2=Pt*Pq;
Pe=P1*P2;

%% DISEÑO EN CASCADA
C1=0;
C2=0;

%% LAZO INTERNO
% Contornos QFT
close all;
phs = 0 : -1 : -360; 

R     = 0;
nompt = 1;
W     = [0.1, 0.3, 0.5, 0.8, 1, 2, 3, 5, 8, 10, 15, 20, 50, 100];

% Contorno de estabilidad Lazo interno
A = 0;
B = P2;
C = 1;
D = P2;

Be2 = genbnds(10, W, W1, A, B, C, D, P2(1,1,1), phs);
plotbnds(Be2,[],phs),title('Bounds estabilizad lazo interno');

% Contorno de estabilidad Lazo externo
A = 0;
B = P2 * P1 * C1;
C = 1;
D = P2 * P1 * C1 + P2;

Be1 = genbnds(10, W, W1, A, B, C, D, P2(1,1,1), phs);
plotbnds(Be1,[],phs), title('Bounds estabilidad lazo externo');

% Contorno perturbaciones
A = Pq * P1;
B = 0;
C = 1 ;
D = P2 * P1 * C1 + P2;

Bd=genbnds(10, W, Wd, A, B, C, D, P2(1,1,1),phs);
plotbnds(Bd, [], phs), title('Bounds rechazo');


% Contorno tracking
% Para la planta P1 A=Pt,B=Pd D=1+L-k, E=Pk 
A = -Pe;
B = M2;
D = Id;
E = P2 + P1 * C1 * P2;
[bt] = genbnd4feedfordward_cond(W, Wb, A, B, D, E, P2(1,1,1), phs);
Bt   = adaptation2(bt,1);
plotbnds(Bt,[], phs), title('Bounds tracking');

B2 = grpbnds(Be1, Be2, Bd, Bt);
B2 = sectbnds(B2);
plotbnds(B2,[],phs), title('Bounds  P2')

%%
ws = 2 * pi / Ts;
wl = logspace(-2, log10(ws/2), 100);  % define a frequency array for loop shaping
po = P2(1,1,1);
lpshape(wl, B2, po, [], phs);
c2s

%% LAZO EXTERNO
% Contornos QFT
phs   = 0:-1:-360;
R     =0;
nompt =1;
W     = [0.1 0.3  0.5 0.8 1 2 3 5 8 10 20 50 100];

% Contorno de estabilidad Lazo externo
A = 0;
B = P2 * P1 * C2;
C = Id * P2 * C2 ;
D = P2 * P1 * C2;

Be1 = genbnds(10, W, W1, A, B, C, D, P1(1,1,1), phs);
plotbnds(Be1, [], phs),title('Bounds estabilidad lazo externo');

% Contorno perturbaciones
A = Pq * P1;
B = 0;
C = Id + P2 * C2 ;
D = P2 * P1 * C2;

Bd=genbnds(10,W,Wd,A,B,C,D,P1(1,1,1),phs);
plotbnds(Bd,[],phs),title('Bounds rechazo');

% Contorno tracking
%Para la planta P1 A=Pt,B=Pd D=1+L-k, E=Pk 
A = -Pe;
B = M2;
D = Id + P2 * C2;
E = P2 * C2 * P1;
[bt] = genbnd4feedfordward_cond(W, Wb, A, B, D, E, P1(1,1,1), phs);
Bt   = adaptation2(bt, 1);
plotbnds(Bt, [], phs), title('Bounds tracking');

B1=grpbnds(Be1,Bd,Bt);
B1=sectbnds(B1);
plotbnds(B1,[],phs), title('Bounds  P2')

%%
po=P1(1,1,1);
lpshape(wl,B1,po,[],phs);

%% Funciones empleadas para la linealización

function f = calculaThrust(v, u, m, vMin, vMax)

if v < vMin
    t = vMin;
elseif v > vMax
    t = vMax;
else
    t = v;
end

nDim   = size(m, 1);
paso   = (vMax - vMin) / (nDim - 1);
nPasos = fix((t - vMin) / paso);
v1     = vMin + nPasos * paso;

% Filas de la matriz
x1 = nPasos + 1;
x2 = x1 + 1;


% Calculo de los puntos
y1 = m(x1,1) * u^3 + m(x1,2) * u^2 + m(x1,3) * u + m(x1,4);
y2 = m(x2,1) * u^3 + m(x2,2) * u^2 + m(x2,3) * u + m(x2,4);

% Interpolacion
f = y1 + (y2 - y1) / paso * (t - v1);

end



function P = interpolaPoly (v, m, vMin, vMax)

if v < vMin
    t = vMin;
elseif v > vMax
    t = vMax;
else
    t = v;
end

nDim   = size(m, 1);
paso   = (vMax - vMin) / (nDim - 1);
nPasos = fix((t - vMin) / paso);
v1     = vMin + nPasos * paso;

% Filas de la matriz
x1 = nPasos + 1;
x2 = x1 + 1;

for i = 1 : 4
    y1 = m(x1,i);
    y2 = m(x2,i);
    P(i) = y1 + (y2 - y1) / paso * (t - v1);
end

end
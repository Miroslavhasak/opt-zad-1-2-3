addpath("..\lab_models\");
addpath("..\misc\");

%% ===========================
%   LQ Controller (with integral action)
%   Linear Quadratic Regulator applied to pendulum
% ===========================
close all
clear
n_steps=20;

Ts=0.05;
y_ref=20;     % desired reference value

sim=1;
theta_hat=[1.4060 11.0669 7.8944 0.0735];

% --- Create state-space model ---
[Ac,Bc,Cc] = create_pendulum_model(theta_hat);
[A,B,C] = ss_discretize(Ac,Bc,Cc,Ts);

m=size(C,1);
r=size(B,2);
n=size(A,1);

% --- Augmented system for integral action ---
% rozšíril som model o integrátor chyby (e = y - y_ref)
A_tilde= [A, zeros(n, m); C, eye(m)];  % spája pôvodný model s integrátorom a hovorí ako sa menia stavy a ako sa akumuluje chyba %TODO;
B_tilde= [B; zeros(m, r)];  % rozširuje vstupnú maticu aby ladila s novým rozmerom systému %TODO;
% C_tilde nebolo treba pridať lebo funkcia dare pracuje s maticami sústavy
% A,B a vahovými maticami Q,R

% --- LQ weighting matrices ---
% určuje kt fyzikálne stavy sú pre nás dôležité
Q_= diag([1, 1, 0.1]);      % váha na stavy (uhly, rýchlosť) %TODO;
% určuje ako keby cenu energie kde malé číslo dovoľuje motoru prudšie
% reagovať
R_= 0.01;                   % váha na akčný zásah (energiu) %TODO;
% toto číslo vynuluje zvyškovú chybu v ustálenom stave
Qz= 10;                     % váha na integračnú zložku (odstraňuje trvalú odchýlku) %TODO;
% spojí všetky váhy do jednej veľkej matice pre vúpočet
Q_tilde= blkdiag(Q_, Qz); %TODO;

% --- Solve Discrete-time Algebraic Riccati Equation ---
% rieši riccatiho rovnicu aby našiel najlepšiu stratégiu riadenia
[P_LQ,~,K_LQ]= dare(A_tilde, B_tilde, Q_tilde, R_); %TODO

% vybera zosilnenie pre skutočné stavy ako uhly a rýchlosť
Kx= K_LQ(1:n);              % Spätná väzba od stavov %TODO            % state feedback part
% vyberie zosilnenie pre integračnú zložku
Kz= K_LQ(n+1:end);          % Spätná väzba od integrátora %TODO        % integral feedback part

% --- Kalman filter initialization ---
P=zeros(n);
x_hat=zeros(n,1);

R=3; % measurement noise covariance
Q=diag([0.2;0.2;1]);  % process noise covariance


% Initialize control variables
z=zeros(m,1);
u=zeros(r,1);

% Logging arrays
x_hat_=[]; y_hat_=[]; y_=[]; u_=[];

% --- Simulation / Hardware selection ---
if ~sim
    COM_port=8;
    baudrate=250000;
    data_stream_ptr=data_stream_start_mex(COM_port,baudrate);
    [~,~]=data_stream_read_mex(data_stream_ptr,1,1);
else
    x=[0;0;0];
end

% ===========================
%   Simulation Loop
% ===========================
for i=1:n_steps

    if ~sim
        [y,t]=data_stream_read_mex(data_stream_ptr,1,1);
        y=double(y);
    end

    % State prediction
    x_hat=A*x_hat+B*u;

    if sim
        y=C*x+sqrt(R)*randn(); 
    end

    % Kalman filter update
    % predpovedá ako sú naše odhady nepresné
    P= A*P*A' + Q;               % Predikcia kovariancie %TODO
    % určuje či má filter viac veriť modelu alebo senzoru
    K= P*C' / (C*P*C' + R);      % Kalmanov zisk %TODO
    % rozdiel medzi tým, čo senzor nameral a čo si filter myslel
    e= y - C*x_hat;              % Inovácia (chyba merania) %TODO
    % upravuje odhad stavu na základe skutočného merania
    x_hat= x_hat + K*e;          % Odhad stavu %TODO
    % pomocný výpočet odhadovaného výstupu pre graf
    y_hat= C*x_hat;              % Odhad výstupu %TODO
    % aktualizuje nepresnosť po tom ako dostaneme nové dáta
    P= (eye(n) - K*C)*P;         % Aktualizácia kovariancie %TODO

    % LQ control law
    % vyráta akčný zásah spojením odhadov a integračnej zložky
    u= -Kx*x_hat - Kz*z;         % Riadiaci zákon: u = -Kx*x - Kz*z %TODO
    u=max(0,u);
    u=min(100,u);

    if ~sim
        data_stream_write_mex(data_stream_ptr,1,single(u));
    else
        x=A*x+B*u+chol(Q)*randn(3,1);
    end

    if ~sim
        [y_ref,~]=data_stream_read_mex(data_stream_ptr,2,1);
    end

    % Update integral state
    % tu nastáva samotná integrácia
    % ku starej sume chýb prirátame tú aktuálnu
    z= z + (y - y_ref);          % Numerická integrácia regulačnej odchýlky %TODO

    if y_ref<5
        break;
    end

    % Logging
    x_hat_=[x_hat_,x_hat];
    y_hat_=[y_hat_,C*x_hat];
    y_=[y_,y];
    u_=[u_,u];
end

% Shutdown hardware stream
if ~sim
    data_stream_write_mex(data_stream_ptr,1,single(0));
    data_stream_end_mex(data_stream_ptr);
end

%%
% ===========================
%   Plot Results
% ===========================
figure
style='-k';

subplot(4,1,1)
stairs(0:(n_steps-1),x_hat_(1,:),style,'LineWidth',1.5)
xlabel('k'); ylabel('\phi(k)'); grid on
xlim([0,n_steps-1]);

subplot(4,1,2)
stairs(0:(n_steps-1),x_hat_(2,:),style,'LineWidth',1.5)
hold on
stairs(0:(n_steps-1),y_hat_,'-g','LineWidth',1.5)
stairs(0:(n_steps-1),y_,'-b','LineWidth',1.5)
stairs(0:(n_steps-1),ones(1,n_steps)*y_ref,'--r','LineWidth',1.5)
xlabel('k'); ylabel('\theta(k)'); grid on
xlim([0,n_steps-1]);

subplot(4,1,3)
stairs(0:(n_steps-1),x_hat_(3,:),style,'LineWidth',1.5)
xlabel('k'); ylabel('\omega(k)'); grid on
xlim([0,n_steps-1]);

subplot(4,1,4)
stairs(0:(n_steps-1),u_(:),style,'LineWidth',1.5)
xlabel('k'); ylabel('u(k)'); grid on
xlim([0,n_steps-1]);

set(gcf,'position',[0,200,650,400]);


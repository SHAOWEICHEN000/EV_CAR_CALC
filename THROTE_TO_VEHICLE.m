%% ev_sim_main.m  --------------------------------------------------------
% 目標：一步到位計算車速追蹤、馬達限扭、電池 SOC、續航力
clear; clc;
close all;
%----------- 0. 參數設定（Gen‑1 Chevy Volt 範例，可自行替換） ----------
cell    = setupCell (15, 450, 4.2, 3.8, 3.0);               % Ah, g, Vmax/Vnom/Vmin
module  = setupModule(3, 8, 0.08, cell);                    % p, s, 8% overhead
battery = setupPack  (12,0.1,100,25,0.96,module);            % 12s,10%overh.,75↔25%SOC,96%η
wheel   = setupWheel (0.31, 0.6, 0.012);                    % r=0.31 m, inertia, rollCoef
motor   = setupMotor (300, 4000, 12000, 0.92, 0.2);         % Lmax,Nm ;Rated/Max RPM; η; inertia
driveTr = setupDrivetrain(0.95,0.4,9.1,0.08,0.97,...
                          battery,motor,wheel);             % inverterη,regen%,gearRatio,gearInertia,gearη
vehicle = setupVehicle(4,0,0.28,2.22,1500,75,200,driveTr,3,3);   % 4輪,0N固定阻, Cd, A(Toyota Prius), 車重,載重,其他耗電,城市風速,road angle
PV=setupsolar(2,0.22,0.95,500);                            %area, panel efficiency,DCDC efficiency,sun power
%----------- 1. 載入行車循跡 (可批次多條) -------------------------------
%cycles = {'nycc.txt','udds.txt','us06.txt','hwy.txt'};
cycles = {'throttle.txt'};

dt     = 1;                        % 取樣 1 s
for idx = 1:numel(cycles)
    M = dlmread(cycles{idx},'\t',2,0);          % [time speed(m/s)]
    t = M(:,1);  alpha = M(:,2);
    beta=M(:,3);
    n = numel(t);

    %-------- 2. 預先配置向量 -------------------------------------------
    v      = zeros(n,1);      a      = zeros(n,1);
    Td     = zeros(n,1);      Tm     = zeros(n,1);
    motRPM = zeros(n,1);      SOC    = zeros(n,1);
    dist   = zeros(n,1);      Pbat   = zeros(n,1);

    SOC(1) = battery.socFull;             % 初始滿電
    for k = 2:n
        %----- 2‑1 計算需求加速度/扭矩 ------------------------------
       %--------- 由油門 α 計算可用馬達扭矩 -----------------
        Vin     = alpha(k) / 100 * (battery.vnom*SOC(k-1));          % 逆變器輸出電壓
        motRPM(k)= v(k-1)/wheel.radius * driveTr.gearRatio * 60/(2*pi);

        if motRPM(k) < motor.RPMrated
            TmaxAvail = motor.Lmax * Vin / (battery.vnom*SOC(k-1));  % 線性降額
        else
            TmaxAvail = motor.Lmax * motor.RPMrated / motRPM(k) * Vin / (battery.vnom*SOC(k-1));
        end
        % 允許的再生扭矩 = regenTorque% × Lmax × β%
        TregenLimit = driveTr.regenTorque * motor.Lmax;            % Nm
        TminAvail   = - beta(k)/100 * TregenLimit;                 % Nm (負值)

        %% ---- 2‑3 加總訊號得到實際馬達扭矩 ---------------------------
        Tm(k) = 0;
        if ((alpha(k) >= 0)^(beta(k)==0))
            Tm(k) =  min( TmaxAvail ,  alpha(k)/100 * motor.Lmax); % DRIVE
        elseif beta(k)  > 0
            Tm(k) =  max( TminAvail , -beta(k)/100 * motor.Lmax);  % REGEN
        end

        %Tm(k) = TmaxAvail;      % 完全踩油門輸出；若想保留制動可再判斷 α<0
        T_req_regen = -beta(k)/100 * motor.Lmax;        % 期望總煞車扭矩 (負)
        T_mech = min(0, T_req_regen - Tm(k));           % 需要機械煞車的部分 (<=0)

        F_mechBrake = T_mech * driveTr.gearRatio / wheel.radius;   % (負)N
        rho   = 1.2;%air density 
        F_aero= 0.5*rho*vehicle.Cd*vehicle.A*(v(k-1)-vehicle.wind)^2;%SETUP AERO_FORCE REF:https://x-engineer.org/aerodynamic-drag/
        F_roll= wheel.rollCoef*vehicle.maxWeight*9.81*(v(k-1)>0);    %F_Roll=F_CRR*N REF:https://en.wikipedia.org/wiki/Rolling_resistance
        F_grade = vehicle.roadForce;      % 此處可改即時坡度
        F_brake=vehicle.brake_drag;

        %----- 2‑3 推進力 → 加速度 → 速度 ------------------------
        F_avail = Tm(k)*driveTr.gearRatio / wheel.radius ...
                  - F_aero - F_roll - F_grade-F_brake;
        a(k) = F_avail / vehicle.equivMass;

        v(k) = max(0, v(k-1) + a(k)*dt);          % 不倒車
        if(v(k)>vehicle.maxSpeed* 60/1000)
            v(k)=vehicle.maxSpeed*60/1000;
        else
            v(k)=v(k);
        end
        %----- 2‑4 電池功率/電流/SOC -----------------------------
        Pmot(k)  = (motRPM(k)+motRPM(k-1))/2 * 2*pi/60 * Tm(k) / (1000);  % kW
        %----- 2‑3.5  太陽能輸入功率 ------------------------- �
        G = PV.G*ones(n,1);
        Ppv = G(k)*PV.area*PV.panel_eff*PV.DCDC_eff;   % kW

        if Pmot >= 0
            Pbat(k)= vehicle.overheadPwr+ Pmot(k)/driveTr.efficiency-Ppv;%Accerlate
        else
            Pbat(k)= vehicle.overheadPwr+ Pmot(k)*driveTr.efficiency-Ppv;%declate
        end
        Ibat(k)  =1000* Pbat(k)/(battery.vnom);                     % A
        SOC(k)= SOC(k-1) - Ibat(k)*dt / (3600*battery.capacity)*100;
        if(SOC(k)>100)
            SOC(k)=100;
        else
            SOC(k)=SOC(k);
        end
         if(SOC(k)<0)
            SOC(k)=0;
        else
            SOC(k)=SOC(k);
        end
        %----- 2‑5 距離累計 ------------------------------------
        dist(k)= dist(k-1) + v(k)*dt/1000;                     % km
    end

    %----------- 3. 結果輸出 -----------------------------------------
    usable = vehicle.drivetrain.battery.socFull - vehicle.drivetrain.battery.socEmpty;
    ran_km = dist(end) * usable/(SOC(1)-SOC(end)+eps);
    fprintf('Cycle %-6s  Range ≈ %5.1f km\n', cycles{idx}, ran_km);

    %----------- 4. 繪圖（一次 4 張） ------------------------------
subplot(2,2,1)                     % ---- 第 1 張圖
yyaxis left                        % 左側 Y 軸：油門
plot(t, alpha, 'k:', 'LineWidth',1.2)
ylabel('Throttle (%)')

yyaxis right                       % 右側 Y 軸：車速
plot(t, v*1000/60, 'b', 'LineWidth',1.2)
ylabel('Speed (Km/hr)')              % 需要 km/h 就 /3.6 後改單位

xlabel('Time (s)')
title('Throttle & Vehicle speed')
legend({'Throttle','Speed'}, 'Location','best')
grid on

    legend('throttle','Speed'), title('Throttle and speed')
    subplot(2,2,2), plot(t,Td,'--', t,Tm), xlabel('t'), ylabel('Torque (Nm)')
    legend('Demand','Limited'), title('Motor torque')
    subplot(2,2,3), plot(t,SOC), xlabel('t'), ylabel('SOC (%)'), title('Battery SOC')
    subplot(2,2,4), histogram(Pbat,40), xlabel('Battery Power (kW)')
    title('Battery power distribution')
    %====================  🔵【新增】製圖區塊  =========================
figure('Name','EV simulation results','Position',[200 120 1000 750])

% 1) Battery power profile --------------------------------------------
subplot(2,2,1)
plot(t/60, Pbat)                               % 轉成分鐘
xlabel('Time (min)'), ylabel('Battery power (kW)')
title(sprintf('%s battery power profile', cycles{idx}),'FontWeight','bold')
grid on

% 2) Battery current profile ------------------------------------------
subplot(2,2,2)
plot(t/60, Ibat)
xlabel('Time (min)'), ylabel('Battery current (A)')
title(sprintf('%s battery current profile', cycles{idx}),'FontWeight','bold')
grid on

% 3) Histogram of demanded motor power --------------------------------
subplot(2,2,3)
histogram(Pmot,40,'BinLimits',[-150 150])      % 視需要調 bin
xlabel('Demands (kW)'), ylabel('Frequency')
title(sprintf('%s motor demanded power', cycles{idx}),'FontWeight','bold')
grid on

% 4) Torque vs RPM scatter + envelope ---------------------------------
subplot(2,2,4)
scatter(motRPM, Tm, 10, 'filled'), hold on

% ----- Envelope: constant torque (正向) -----------------------------------
RPMenv = [0 motor.RPMrated];
plot(RPMenv, [motor.Lmax motor.Lmax], 'k-', 'LineWidth', 1.5)     % constant torque

% ----- Envelope: constant power (正向) ------------------------------------
RPM2 = linspace(motor.RPMrated, motor.RPMmax, 200);
TconstP = motor.maxPower * 1000 ./ (2 * pi * RPM2 / 60);          % T = P/ω
plot(RPM2, TconstP, 'k-', 'LineWidth', 1.5)

% ===== 負扭矩區域的 envelope（再生煞車區）=======================
regenFactor = vehicle.drivetrain.regenTorque;
plot(RPMenv, -motor.Lmax * regenFactor * [1 1], 'k-', 'LineWidth', 1.5)
plot(RPM2, -TconstP * regenFactor, 'k-', 'LineWidth', 1.5)

xlabel('Speed (RPM)'), ylabel('Torque (N·m)')
title(sprintf('%s motor torque versus RPM', cycles{idx}), 'FontWeight', 'bold')
ylim([-motor.Lmax*1.1 motor.Lmax*1.1]), xlim([0 motor.RPMmax*1.05])
grid on
%=====================================================================

end
figure('Name','EV simulation results','Position',[200 120 1000 750])
subplot(2,2,1)
yyaxis left
plot(t, alpha,'k:','LineWidth',1.2); ylabel('Throttle (%)')
yyaxis right
plot(t, beta ,'r-','LineWidth',1.2); ylabel('Brake (%)')
xlabel('Time (s)')
title('Throttle & Brake')
legend({'Throttle','Brake'})
grid on
 subplot(2,2,2), plot(t,dist,'--'), xlabel('t'), ylabel('distance (Km)')
    legend('Demand','Limited'), title('drive distance')

%% ----------- 各層結構函式  -------------------------
function cell = setupCell(cap,wt,vmax,vnom,vmin)
cell.capacity = cap;     cell.weight = wt;
cell.vmax = vmax;        cell.vnom = vnom;     cell.vmin = vmin;
cell.energy = vnom*cap;  cell.specificEnergy = 1000*cap*vnom/wt;
end
%----------------------------------------------------
function module = setupModule(p,s,ovhd,cell)
module.numParallel=p; module.numSeries=s; module.overhead=ovhd; module.cell=cell;
module.numCells=p*s;    module.capacity=p*cell.capacity;
module.weight = module.numCells*cell.weight/(1-ovhd)/1000;      % kg
module.energy = module.numCells*cell.energy/1000;               % kWh
module.specificEnergy = 1000*module.energy/module.weight;       % Wh/kg
end
%----------------------------------------------------
function pack = setupPack(s,ovhd,socF,socE,eta,module)
pack.numSeries=s;  pack.overhead=ovhd; pack.module=module;
pack.socFull=socF; pack.socEmpty=socE; pack.efficiency=eta;
pack.numCells = module.numCells*s;
pack.weight   = module.weight*s/(1-ovhd);
pack.energy   = module.energy*s;
pack.specificEnergy = 1000*pack.energy/pack.weight;
pack.vmax = s*module.cell.vmax;  pack.vnom = s*module.cell.vnom;
pack.vmin = s*module.cell.vmin;
pack.capacity = module.capacity;       % Ah
end
%----------------------------------------------------
function wheel = setupWheel(r,inertia,roll)
wheel.radius=r; wheel.inertia=inertia; wheel.rollCoef=roll;
end
%----------------------------------------------------
function motor = setupMotor(Lmax,RPMr,RPMmax,eta,inertia)
motor.Lmax=Lmax; motor.RPMrated=RPMr; motor.RPMmax=RPMmax;
motor.efficiency=eta; motor.inertia=inertia;
motor.maxPower = 2*pi*Lmax*RPMr/60000;           % kW
end
%----------------------------------------------------
function dt = setupDrivetrain(invEff,regen,ratio,gearJ,gearEff,battery,motor,wheel)
dt.inverterEfficiency=invEff; 
dt.regenTorque=regen;% regen torque is fraction of braking power that is used to charge
dt.gearRatio=ratio; dt.gearInertia=gearJ; dt.gearEfficiency=gearEff;
dt.battery=battery; dt.motor=motor; dt.wheel=wheel;
dt.efficiency= battery.efficiency*invEff*motor.efficiency*gearEff;
end
%----------------------------------------------------
function ds = setupsolar(area,panel_eff,dcdc_eff,P_sun)
ds.area=area;   
ds.panel_eff=panel_eff;
ds.DCDC_eff=dcdc_eff;
ds.G=P_sun;
end
%----------------------------------------------------
function veh = setupVehicle(wheels,roadF,Cd,A,weight,payload,overPwr,dt,wind,angle)
veh.drivetrain=dt; veh.wheels=wheels;
veh.roadForce=weight*9.81*sind(angle);%grade force
veh.Cd=Cd;% aerodynamic drag coefficient
veh.A=A;%maximum cross-section area of the vehicle
veh.weight=weight; % Vehicle weight
veh.overheadPwr=overPwr;
veh.maxWeight = weight + dt.battery.weight + payload;
veh.rotWeight = (dt.motor.inertia+dt.gearInertia)*dt.gearRatio^2 / dt.wheel.radius^2 ...
               + wheels*dt.wheel.inertia/dt.wheel.radius^2;
veh.equivMass = veh.maxWeight + veh.rotWeight;
veh.maxSpeed  = 2*pi*dt.wheel.radius*dt.motor.RPMmax*60/(1000*dt.gearRatio); % km/h
veh.wind=wind;%city wind velocity ref:https://www.cwa.gov.tw/V8/C/W/WindSpeed/WindSpeed_All.html?CID=63&StationID=46691
veh.brake_drag=roadF;%vehicle brake drag you can choose the force
end

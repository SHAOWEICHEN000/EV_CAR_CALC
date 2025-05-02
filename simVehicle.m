%% ev_sim_main.m  --------------------------------------------------------
% 目標：一步到位計算車速追蹤、馬達限扭、電池 SOC、續航力
clear; clc;

%----------- 0. 參數設定（Gen‑1 Chevy Volt 範例，可自行替換） ----------
cell    = setupCell (15, 450, 4.2, 3.8, 3.0);               % Ah, g, Vmax/Vnom/Vmin
module  = setupModule(3, 8, 0.08, cell);                    % p, s, 8% overhead
battery = setupPack  (12,0.1,75,25,0.96,module);            % 12s,10%overh.,75↔25%SOC,96%η
wheel   = setupWheel (0.31, 0.6, 0.012);                    % r=0.31 m, inertia, rollCoef
motor   = setupMotor (300, 4000, 12000, 0.92, 0.2);         % Lmax,Nm ;Rated/Max RPM; η; inertia
driveTr = setupDrivetrain(0.95,0.4,9.1,0.08,0.97,...
                          battery,motor,wheel);             % inverterη,regen%,gearRatio,gearInertia,gearη
vehicle = setupVehicle(4,0,0.28,2.22,1500,75,200,driveTr,3,0);   % 4輪,0N固定阻, Cd, A(Toyota Prius), 車重,載重,其他耗電,城市風速,road angle

%----------- 1. 載入行車循跡 (可批次多條) -------------------------------
%cycles = {'nycc.txt','udds.txt','us06.txt','hwy.txt'};
cycles = {'nycc.txt'};
dt     = 1;                        % 取樣 1 s
for idx = 1:numel(cycles)
    M = dlmread(cycles{idx},'\t',2,0);          % [time speed(m/s)]
    t = M(:,1);  v_des = M(:,2);
    n = numel(t);

    %-------- 2. 預先配置向量 -------------------------------------------
    v      = zeros(n,1);      a      = zeros(n,1);
    Td     = zeros(n,1);      Tm     = zeros(n,1);
    motRPM = zeros(n,1);      SOC    = zeros(n,1);
    dist   = zeros(n,1);      Pbat   = zeros(n,1);

    SOC(1) = battery.socFull;             % 初始滿電
    for k = 2:n
        %----- 2‑1 計算需求加速度/扭矩 ------------------------------
        a_des = (v_des(k-1) - v(k-1))/dt;
        F_acc = vehicle.equivMass * a_des;

        rho   = 1.2;%air density 
        F_aero= 0.5*rho*vehicle.Cd*vehicle.A*(v(k-1)-vehicle.wind)^2;%SETUP AERO_FORCE REF:https://x-engineer.org/aerodynamic-drag/
        F_roll= wheel.rollCoef*vehicle.maxWeight*9.81*(v(k-1)>0);    %F_Roll=F_CRR*N REF:https://en.wikipedia.org/wiki/Rolling_resistance
        F_grade = vehicle.roadForce;      % 此處可改即時坡度
        F_brake=vehicle.brake_drag;
        F_total= F_acc + F_aero + F_roll + F_grade+F_brake;
        Td(k)  = wheel.radius*F_total / driveTr.gearRatio;

        %----- 2‑2 馬達/再生限扭 ------------------------------
        motRPM(k)= v(k-1)/wheel.radius * driveTr.gearRatio * 60/(2*pi);
        if motRPM(k) < motor.RPMrated
            Tmax = motor.Lmax;
        else
            Tmax = motor.Lmax * motor.RPMrated / motRPM(k);
        end
        Tmin = -driveTr.regenTorque*motor.Lmax;
        Tm(k)= min(max(Td(k),Tmin),Tmax);         % 實際使用扭矩

        %----- 2‑3 推進力 → 加速度 → 速度 ------------------------
        F_avail = Tm(k)*driveTr.gearRatio / wheel.radius ...
                  - F_aero - F_roll - F_grade-F_brake;
        a(k) = F_avail / vehicle.equivMass;
        v(k) = max(0, v(k-1) + a(k)*dt);          % 不倒車

        %----- 2‑4 電池功率/電流/SOC -----------------------------
        Pmot  = 1000*(motRPM(k)+motRPM(k-1))/2 * 2*pi/60 * Tm(k) / driveTr.efficiency;  % kW
        if Pmot >= 0
            Pbat(k)= vehicle.overheadPwr+ Pmot/driveTr.efficiency;%Accerlate
        else
            Pbat(k)= vehicle.overheadPwr+ Pmot*driveTr.efficiency;%declate
        end
        Ibat  = Pbat(k)/(battery.vnom*1000);                     % A
        SOC(k)= SOC(k-1) - Ibat*dt / (3600*battery.capacity)*100;

        %----- 2‑5 距離累計 ------------------------------------
        dist(k)= dist(k-1) + v(k)*dt/1000;                     % km
    end

    %----------- 3. 結果輸出 -----------------------------------------
    usable = vehicle.drivetrain.battery.socFull - vehicle.drivetrain.battery.socEmpty;
    ran_km = dist(end) * usable/(SOC(1)-SOC(end));
    fprintf('Cycle %-6s  Range ≈ %5.1f km\n', cycles{idx}, ran_km);

    %----------- 4. 繪圖（一次 4 張） ------------------------------
    figure('Name',cycles{idx},'Position',[100 100 1000 700])
    subplot(2,2,1), plot(t,v_des,'k:', t,v), xlabel('t(s)'), ylabel('v(m/s)')
    legend('Target','Actual'), title('Speed profile')
    subplot(2,2,2), plot(t,Td,'--', t,Tm), xlabel('t'), ylabel('Torque (Nm)')
    legend('Demand','Limited'), title('Motor torque')
    subplot(2,2,3), plot(t,SOC), xlabel('t'), ylabel('SOC (%)'), title('Battery SOC')
    subplot(2,2,4), histogram(Pbat,40), xlabel('Battery Power (kW)')
    title('Battery power distribution')
end


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
function veh = setupVehicle(wheels,roadF,Cd,A,weight,payload,overPwr,dt,wind,angle)
veh.drivetrain=dt; veh.wheels=wheels;
veh.roadForce=weight*9.81*sin(angle);%grade force
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

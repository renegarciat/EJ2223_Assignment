function [Dis,le,tau_p,tau_s,t_s,h_slot,h_cs,Dro,Dso,ratio,q_spp] = EssonsEstimation( ...
    T_Nm,P,Q,m,Bg1,Bt,Bc,A1,J_rms,AspectRatio,eta_estimation,cos_phi,airgap_estimation,stackingfactor,kis,dos,kcu)
%Function can be used to start estimating the parameters of the 
arguments (Input)
    T_Nm                    % [Nm] Nominal torque of the estimated design
    P                       % [-] Number of poles for the design estimation
    Q                       % [-] Number of slots for the design estimation
    m                       % [-] Number of phases for the design estimation
    Bg1                     % [T] Magnetic flux density in the Airgap 0.85 T - 1.05 T
    Bt                      % [T] Flux density tooth 1.6 T - 2.0 T
    Bc                      % [T] Flux density back iron 1.0 T - 1.5 T
    A1                      % [A/m] Peak linear current density
    J_rms                   % [A/mm^2] rms value for current density
    AspectRatio             % [-] le/tau_p ratio
    eta_estimation          % [-] Estimation for the efficiency in the airgap
    cos_phi                 % [-] Estimation of the power factor
    airgap_estimation       % [m] Estimation of the Airgap distance
    stackingfactor          % [-] The ratio of le/li
    kis                     % [-] effective iron factor in stator
    dos                     % [m] slot opening height
    kcu                     % [-] ratio of copper in the slot


end

arguments (Output)
    Dis                     % [m] Inner stator diameter
    le                      % [m] Effective length of the motor
    tau_p                   % [m] pole pitch
    tau_s                   % [m] stator pitch
    t_s                     % [m] stator tooth distance
    h_slot                  % [m] height of the stator slot
    h_cs                    % [m] height of the back iron 
    Dro                     % [m] outer rotor diameter
    Dso                     % [m] outer stator diameter
    ratio                   % [-] Dis/Dso
    q_spp                   % [-] ratio of slots per pole per phase
end
%----------Calculation constants-------------------------------------------
D_o_old=120*1E-3;
sigma_m=A1*Bg1/sqrt(2); %NOTE: Make sure that A1 is peak, not RMS.
% w_r_rads=w_r*pi/30; 
% Pmech=T_Nm*w_r_rads; % NOTE: Unused!
q_spp=Q/(m*P);                                        %slots per pole per phase
J_rms=J_rms*1E6;
%----------Calculation inner diameter of stator----------------------------

Volume=T_Nm/(2*sigma_m*eta_estimation*cos_phi);
%Dis=(Volume*P/(AspectRatio*pi))^1/3;
A=(Volume*4*pi*sqrt(AspectRatio)/sqrt(P))^(2/3);      % 6.33 Lipo Book
Dis=1/pi*sqrt(P*A/AspectRatio);                       % 6.32 Lipo Book
tau_p=pi*Dis/P;
le=AspectRatio*tau_p;
%----------Calculation outer Diameter and Slots----------------------------
tau_s=pi*Dis/Q;
t_s=tau_s*1/kis*(Bg1/Bt)*stackingfactor;
h_cs=Dis/(P*kis)*(Bg1/Bc)*stackingfactor;
% Calculate the fraction of the stator bore that is available
% for the slot opening (i.e., not taken up by the teeth)
% a=(Bg1/(kis*Bt)*stackingfactor+2/P*Bg1/(kis*Bc)*stackingfactor)^2-(1-Bg1/(Bc*kis)*stackingfactor)^2;
a = (Bg1/(kis*Bt)*stackingfactor + 2/P*Bg1/(kis*Bc)*stackingfactor)^2 - (1 - Bg1/(Bt*kis)*stackingfactor)^2;
b=(Bg1/(kis*Bt)+2/P*Bg1/(kis*Bc))*stackingfactor;
%DisDos_Opt=b/a+2*Ks/(a*k_cu*J*Dos_start)-sqrt((b/a+2*Ks/(a*k_cu*J*Dos_start))^2-1/a);
% NOTE: This formula takes A1, if its peak, convert to RMS by multiplying with sqrt(2)
fun=@(Dos)calc_Dis_from_Dos(Dos,a,b,A1,kcu,J_rms)-Dis;
Dso_min=Dis/0.8;
Dso_max=Dis/0.2;
Dso=find_root_bracketed(fun, Dso_min, Dso_max);
if Dso>D_o_old
    warning('Outer diameter stator is bigger than old design');
end
ratio=Dis/Dso;
zeta=(3*a*ratio^2-4*b*ratio+1)/((kcu*J_rms/4)*((1/ratio)^2-a));
if zeta > 0
    warning('Constraint inactive. Increase k_cu or reduce J_rms');
end
b1=pi/Q*(Dis*(1-Bg1/(kis*Bt)*stackingfactor)+2*dos);
b2=pi/Q*(Dso-Dis*(Bg1/(kis*Bt)*stackingfactor+2/P*Bg1/(Bc*kis)*stackingfactor));
h_slot=Q/(2*pi)*(b2-b1);
Dos_test=Dis+2*(h_cs+dos+h_slot);
Dro=Dis-2*airgap_estimation;
end

function Dis_calc=calc_Dis_from_Dos(Dos,a,b,A1,kcu,J_rms)
x=b/a+2*A1/(a*kcu*J_rms*Dos);
rad=x^2-1/a;
if rad<0
    warning('sqare wave term is negative');
    Dis_calc=NaN;
    return
end
r=x-sqrt(rad);
if r<=0||r>=1
    Dis_calc=NaN;
    warning('Wrong ratio');
    return;
end
Dis_calc=Dos*r;

end

function root = find_root_bracketed(fun, xmin, xmax)
%FIND_ROOT_BRACKETED Robust wrapper around fzero.
% Ensures the interval contains finite values and a sign change.

    if ~(isfinite(xmin) && isfinite(xmax) && xmax > xmin)
        error('Invalid root search interval.');
    end

    % Sample the interval and locate a finite sign change.
    xs = linspace(xmin, xmax, 300);
    ys = arrayfun(fun, xs);

    finiteMask = isfinite(ys);
    xs = xs(finiteMask);
    ys = ys(finiteMask);

    if numel(xs) < 2
        error('Root finding failed: function is not finite on the interval.');
    end

    s = sign(ys);

    idxZero = find(s == 0, 1, 'first');
    if ~isempty(idxZero)
        root = xs(idxZero);
        return;
    end

    for i = 1:(numel(xs)-1)
        if s(i) * s(i+1) < 0
            root = fzero(fun, [xs(i), xs(i+1)]);
            return;
        end
    end

    % Fallback: try from best finite initial guess.
    [~, idx] = min(abs(ys));
    root = fzero(fun, xs(idx));
end
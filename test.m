%% OUTPLOT FINAL (PERFORMANCE DE VOL SEULEMENT)
clc; close all;
disp('=== Tracé des Performances de Vol et Attitude ===');

if ~exist('out', 'var')
    error('❌ Lancez la simulation (Run) d''abord !');
end

% --- COULEURS DARK MODE ---
bg_col   = [0.15 0.15 0.15];  
ax_col   = [0 0 0];           
txt_col  = [0.9 0.9 0.9];     
col_real = [0 1 1];           
col_ref  = [1 0 0];     % 🔴 Rouge pour les références


%% ======================================================================
%  CHARGEMENT DES RÉFÉRENCES
%% ======================================================================
if exist('traj', 'var')
    t_ref = traj.p_ref(:,1);
    ref_x = traj.p_ref(:,2); ref_y = traj.p_ref(:,3); ref_z = traj.p_ref(:,4);
    ref_vx = traj.v_ref(:,2); ref_vy = traj.v_ref(:,3); ref_vz = traj.v_ref(:,4);
    ref_psi = traj.yaw_ref(:,2);
else
    warning('⚠️ Pas de variable "traj". Références désactivées.');
    t_ref = []; ref_x=[]; ref_y=[]; ref_z=[]; ref_vx=[]; ref_vy=[]; ref_vz=[]; ref_psi=[];
end


%% ======================================================================
%  1. FIGURE POSITION (X, Y, Z)
%% ======================================================================
fig1 = figure('Name', '1. Position', 'Color', bg_col, 'InvertHardcopy', 'off');

% --- X ---
ax1 = subplot(3,1,1);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_x, '--', 'Color', col_ref, 'LineWidth', 1);
    hold on;
end
h(end+1) = plot_ts(out.posx, col_real);
format_dark(ax1, 'X [m]', txt_col);
title('Comparaison Position X', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

% --- Y ---
ax2 = subplot(3,1,2);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_y, '--', 'Color', col_ref, 'LineWidth', 1);
    hold on;
end
h(end+1) = plot_ts(out.posy, col_real);
format_dark(ax2, 'Y [m]', txt_col);
title('Comparaison Position Y', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

% --- Z ---
ax3 = subplot(3,1,3);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_z, '--', 'Color', col_ref, 'LineWidth', 1);
    hold on;
end
h(end+1) = plot_ts(out.posz, col_real);
format_dark(ax3, 'Z [m]', txt_col);
title('Comparaison Altitude Z', 'Color', txt_col);
xlabel('Temps [s]', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

linkaxes([ax1 ax2 ax3], 'x');


%% ======================================================================
%  2. FIGURE VITESSE
%% ======================================================================
fig2 = figure('Name', '2. Vitesse', 'Color', bg_col, 'InvertHardcopy', 'off');

% --- Vx ---
ax1 = subplot(3,1,1);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_vx, '--', 'Color', col_ref);
    hold on;
end
h(end+1) = plot_ts(out.velx, 'g');
format_dark(ax1, 'Vx [m/s]', txt_col);
title('Comparaison Vitesse X', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

% --- Vy ---
ax2 = subplot(3,1,2);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_vy, '--', 'Color', col_ref);
    hold on;
end
h(end+1) = plot_ts(out.vely, 'g');
format_dark(ax2, 'Vy [m/s]', txt_col);
title('Comparaison Vitesse Y', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

% --- Vz ---
ax3 = subplot(3,1,3);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_vz, '--', 'Color', col_ref);
    hold on;
end
h(end+1) = plot_ts(out.velz, 'g');
format_dark(ax3, 'Vz [m/s]', txt_col);
title('Comparaison Vitesse Z', 'Color', txt_col);
xlabel('Temps [s]', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);


%% ======================================================================
%  3. FIGURE ANGLES (Euler)
%% ======================================================================
fig3 = figure('Name', '3. Angles', 'Color', bg_col, 'InvertHardcopy', 'off');

% Roll
ax1 = subplot(3,1,1);
h = [];
h(end+1) = plot_ts(out.phi, 'm');
format_dark(ax1, 'Roll (\phi)', txt_col);
title('Attitude - Roll', 'Color', txt_col);
safe_legend(h, {'Réel'}, ax_col, txt_col);

% Pitch
ax2 = subplot(3,1,2);
h = [];
h(end+1) = plot_ts(out.theta, 'm');
format_dark(ax2, 'Pitch (\theta)', txt_col);
title('Pitch', 'Color', txt_col);
safe_legend(h, {'Réel'}, ax_col, txt_col);

% Yaw
ax3 = subplot(3,1,3);
h = [];
if ~isempty(t_ref)
    h(end+1) = plot(t_ref, ref_psi, '--', 'Color', col_ref);
    hold on;
end
h(end+1) = plot_ts(out.psi, 'm');
format_dark(ax3, 'Yaw (\psi)', txt_col);
title('Comparaison Yaw', 'Color', txt_col);
xlabel('Temps [s]', 'Color', txt_col);
safe_legend(h, {'Référence','Réel'}, ax_col, txt_col);

%% ======================================================================
%  4. FIGURE FSM – État du Vent
%% ======================================================================

%% ======================================================================
%  0. FIGURE FSM (état du vent)
%% ======================================================================
%% ======================================================================
%  7. FSM STATE PLOT
%% ======================================================================

%% ======================================================================
%  7. FIGURE FSM (État de la Finite State Machine)
%% ======================================================================

disp('=== Affichage de la FSM ===');

% On récupère la variable FSM du workspace de la simulation
fsm_var = [];

if exist('fsm','var')
    fsm_var = fsm;           % cas : bloc To Workspace nommé "fsm"
elseif exist('out','var') && (isprop(out,'fsm') || isfield(out,'fsm'))
    fsm_var = out.fsm;       % cas : signal stocké dans SimulationOutput
end

% Si rien trouvé → avertissement, pas d'arrêt du script
if isempty(fsm_var)
    warning("⚠️ Impossible d'afficher la FSM : variable absente (fsm ou out.fsm).");
else
    % Vérification du contenu
    if isempty(fsm_var.Time)
        warning("⚠️ Signal FSM trouvé mais il est vide.");
    else

        % === Plot FSM ===
        figFSM = figure('Name','7. FSM State','Color',bg_col,'InvertHardcopy','off');
        stairs(fsm_var.Time, fsm_var.Data, 'LineWidth',2,'Color',[1 0.9 0]);

        xlabel("Temps [s]", "Color",txt_col);
        ylabel("État q",     "Color",txt_col);
        title("État de la Finite State Machine (q)", "Color",txt_col);

        set(gca,'Color',[0 0 0], ...
                'XColor',txt_col, 'YColor',txt_col, ...
                'GridColor','w','GridAlpha',0.15);
        grid on;

        disp("✔ FSM affichée.");
    end
end



%% ======================================================================
%  4. FIGURES RÉFÉRENCES SEULES
%% ======================================================================
if ~isempty(t_ref)

    % Position ref seule
    fig4 = figure('Name','4. Référence Position','Color',bg_col);
    subplot(3,1,1); plot(t_ref, ref_x,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'X ref [m]',txt_col); title('X ref','Color',txt_col);
    subplot(3,1,2); plot(t_ref, ref_y,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'Y ref [m]',txt_col); title('Y ref','Color',txt_col);
    subplot(3,1,3); plot(t_ref, ref_z,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'Z ref [m]',txt_col); title('Z ref','Color',txt_col); xlabel('Temps [s]','Color',txt_col);

    % Vitesses ref seules
    fig5 = figure('Name','5. Référence Vitesses','Color',bg_col);
    subplot(3,1,1); plot(t_ref, ref_vx,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'Vx ref [m/s]',txt_col);
    subplot(3,1,2); plot(t_ref, ref_vy,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'Vy ref [m/s]',txt_col);
    subplot(3,1,3); plot(t_ref, ref_vz,'--','Color',col_ref,'LineWidth',1.5); format_dark(gca,'Vz ref [m/s]',txt_col); xlabel('Temps [s]','Color',txt_col);

    % Yaw ref seul
    fig6 = figure('Name','6. Référence Yaw','Color',bg_col);
    plot(t_ref, ref_psi,'--','Color',col_ref,'LineWidth',1.5);
    format_dark(gca,'Yaw ref [rad]',txt_col);
    title('Yaw Référence','Color',txt_col);
    xlabel('Temps [s]','Color',txt_col);
end

disp('✔ Tracé terminé (Dark Mode + Références propres, sans warnings).');


%% ======================================================================
%                       FONCTIONS LOCALES
%% ======================================================================
function h = plot_ts(ts_obj, col, width)
    if nargin < 3, width = 1.5; end
    if isa(ts_obj,'timeseries')
        h = plot(ts_obj.Time, squeeze(ts_obj.Data),'Color',col,'LineWidth',width);
    else
        h = plot(ts_obj,'Color',col,'LineWidth',width);
    end
    grid on;
end

function format_dark(ax, y_lbl, txt)
    ylabel(y_lbl,'Color',txt);
    set(ax,'Color',[0 0 0], ...
           'XColor',txt,'YColor',txt, ...
           'GridColor','w','GridAlpha',0.15);
    grid on;
end

function safe_legend(handles, labels, ax_col, txt_col)
    % Garde uniquement les courbes réellement tracées
    valid = arrayfun(@(h) ~isempty(h) && isgraphics(h) && ~isempty(get(h,'XData')), handles);
    legend(handles(valid), labels(valid), 'Color', ax_col, 'TextColor', txt_col);
end

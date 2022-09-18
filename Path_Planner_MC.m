% The Space
% Mrob= (x,y,theta,dx,dy,dt, prio(t) 1D, L (trrajectory, desired states), 
%high-level functions (#failour rate (opposite of priority),#task priorities tables over 4D, role) %
%what if a robot has role 
% Mrobs =[Mrob(1);Mrob2;....] 
% Continues time (x,y) -> R^2

%% Functions  in each robot


%% Priority Planer Function
% Which robot will be the main guy for "path planner function", Algorithm of 
% fetch sides summation and finding the highest priority robot by 
%re-ordering Mrobs 
% output: Considered Robot (i) 


%% Path Planer Function 
% In: the states (variables of space)
% Out: Trajectory L

%% Solve the kinematics in time t 
% states of each robot, new updates to priority tables and new robots? bla
% bla

%% 
% Alpha = [x y R]
function [CR U] = PoF(X,dX,CR,Alpha_Ob,PH, t) % For a single robot (What robot gonna do)
%X Position, dX velocity, P (priority), Des (destination), t (time)
% CR:
% Vectorial matrix sof high priority previously done robot
% Constant multipliers (a0....) for the trajectry created by Bezier curves
% Robots occupied area, Radius of covered area, priority, goal 
% CR = [a0 a1 a2 .... an P;% First robot 
% [a0 a1 a2 .... an Alpha_Ro(1x3) Pri(1xn) U(1x2)] % Second robot 
%...
% Priority (P = [ current priority failure goal point]). 
% Alpha_Ro=  [x y R]     % Robot area
% Alpha_Ob = [x y R]     %OBstacles
% PH = Prediction Horizen (time unit) 4 sec.... 
%-----------------------------------------------------------------
% Find the location of robots based on PH

% Solve each high priority (a0, a1, a2) 
% Find the location and velocity of robots in the length of PH (till 4 sec
% for example) 
% Make the Bezier curves 
% Create the curves and get the new (a0, a1...) for each robot 
% Update CR 
% Update U(u_v,u_psi, ID), Find a for input (a0, a1, a2)... (u_v,u_psi) velocity
% 

% Bezeier Curve finding based on obstacles points 

% When no solution found by Bezeier Curve Change the velocity
% U_k-1 we keep if we fail we decrease/increase U_k 
% Worst case go back the same root 


%----------------------------------------------------
end

%function PoF(X,dX,CR,Alpha_Ob,PH, t) % For a single robot (How the CR should be arrange)


%end 
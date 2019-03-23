% Written by: Mehryar Emambakhsh
% Email: mehryar_emam@yahoo.com
% Date: 23 March 2019
% GUI Implementation of the EKF SLAM explain in the Probabilistic Robotics
% book by Sebastian Thrun, Wolfram Burgard and Dieter Fox.

function varargout = MotionModellingMoviningRobotGUI(varargin)
% MOTIONMODELLINGMOVININGROBOTGUI MATLAB code for MotionModellingMoviningRobotGUI.fig
%      MOTIONMODELLINGMOVININGROBOTGUI, by itself, creates a new MOTIONMODELLINGMOVININGROBOTGUI or raises the existing
%      singleton*.
%
%      H = MOTIONMODELLINGMOVININGROBOTGUI returns the handle to a new MOTIONMODELLINGMOVININGROBOTGUI or the handle to
%      the existing singleton*.
%
%      MOTIONMODELLINGMOVININGROBOTGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOTIONMODELLINGMOVININGROBOTGUI.M with the given input arguments.
%
%      MOTIONMODELLINGMOVININGROBOTGUI('Property','Value',...) creates a new MOTIONMODELLINGMOVININGROBOTGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MotionModellingMoviningRobotGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MotionModellingMoviningRobotGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MotionModellingMoviningRobotGUI

% Last Modified by GUIDE v2.5 04-Apr-2017 15:52:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MotionModellingMoviningRobotGUI_OpeningFcn, ...
    'gui_OutputFcn',  @MotionModellingMoviningRobotGUI_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MotionModellingMoviningRobotGUI is made visible.
function MotionModellingMoviningRobotGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MotionModellingMoviningRobotGUI (see VARARGIN)

% Choose default command line output for MotionModellingMoviningRobotGUI
warning off
handles.output = hObject;
clear global

% Update handles structure
guidata(hObject, handles);

axes(handles.axes1)
if ~isempty(handles.axes1.Children)
    delete(handles.axes1.Children)
end
set(handles.axes1, 'Xlim', [-10, 10])
set(handles.axes1, 'Ylim', [-10, 10])

axes(handles.axes2)
if ~isempty(handles.axes2.Children)
    delete(handles.axes2.Children)
end
set(handles.axes2, 'Xlim', [-10, 10])
set(handles.axes2, 'Ylim', [-10, 10])

axes(handles.axes1)
patch([0 0 1], [-1 1 0], 'g');

% Define landmarks
global Num_of_landmarks;
Num_of_landmarks = 15;
all_landmarks = (rand(Num_of_landmarks, 2) - 0.5)* 20;
hold on
for land_cnt = 1: Num_of_landmarks
   	plot(all_landmarks(land_cnt, 1), all_landmarks(land_cnt, 2), '*g', 'MarkerSize', 30)
end
hold off

% Delete the previous forms
% warning off
% try
%     delete('previously_observed_landmarks.mat')
% end
% try
%     delete('landmarks_states.mat')
% end
% try
%    delete('covariance_matrix.mat') 
% end

% Save the initial landmarks states
global landmarks_states;
landmarks_states = zeros(3 + 3* Num_of_landmarks, 1);
% save('landmarks_states.mat', 'landmarks_states')

global covariance_matrix;
covariance_matrix = eye(3 + 3* Num_of_landmarks, 3 + 3* Num_of_landmarks); covariance_matrix(1: 3, 1: 3) = zeros(3);
a_big_number = 1000;

covariance_matrix = covariance_matrix* a_big_number;
% save('covariance_matrix.mat', 'covariance_matrix')
% Saving previously detected landmarks as a number_of_lands X 3 matrix:
% containing x, y and corresponding landmarks index.
global previously_observed_landmarks
previously_observed_landmarks = [];
% save('previously_observed_landmarks.mat', 'previously_observed_landmarks')
% UIWAIT makes MotionModellingMoviningRobotGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MotionModellingMoviningRobotGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on key press with focus on figure1 and none of its controls.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

%%%%%%%%%%%%%%%%%%%%%%%%% Read the state vector and its covariance matrix
%%%%%%%%%%%%%%%%%%%%%%%%% at t - 1: mu_t_minus_1, sigma_t_minus_1
% load('landmarks_states.mat')
% load('covariance_matrix.mat')
global landmarks_states;
mu_t_minus_1 = landmarks_states;
global covariance_matrix;
sigma_t_minus_1 = covariance_matrix;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%% Obtain the control vector at t: u_t
global delta_t
delta_t = 0.35; % Time resolution (in second)
switch eventdata.Key
    case 'leftarrow'
        % Rotate the robot counterclockwise
        angular_speed = 5 * (pi/180); % angle in degrees per second
        radial_speed = 0;
    case 'rightarrow'
        % Rotate the robot clockwise
        angular_speed = -10 * (pi/180); % angle in degrees per second
        radial_speed = 0;
    case 'uparrow'
        % Go forward
        angular_speed = 0 * (pi/180); % angle in degrees per second
        radial_speed = 1;
    otherwise
        return;
end
v_t = radial_speed; w_t = angular_speed; u_t = [v_t; w_t];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%% Obtain the measurements at time t: z_t
sense_radius = 2; % in meters
z_t = doMeasurements(handles, sense_radius);
% Add noise to the measurement:
if ~isempty(z_t)
    z_t = z_t + [rand(size(z_t, 1), 2)/20, zeros(size(z_t, 1), 1)];
end
% z_t is a M X 3 matrix, which M is the number of currently detected
% landmarks.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%%%%%%%%%% Load the previously observed landmarks: c_t
% load('previously_observed_landmarks.mat')
global previously_observed_landmarks
c_t = previously_observed_landmarks;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% EKF SLAM prediction correction
[mu_t, sigma_t] = EKF_SLAM_Known_Correspondences(mu_t_minus_1, sigma_t_minus_1, u_t, z_t, c_t);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%% View the estimated robot state in the static
%%%%%%%%%%%%%%%%%%%%%%% textbox
curr_string = num2str([mu_t(1), mu_t(2) mu_t(3)* 180/pi]);
set(handles.text4, 'String', curr_string)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save the new state vector and covariance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% and update the seen landmark indexes
landmarks_states = mu_t;
% save('landmarks_states.mat', 'landmarks_states')
covariance_matrix = sigma_t;
% save('covariance_matrix.mat', 'covariance_matrix')
axes(handles.axes2)
for curr_measured_landmark_cnt = 1: size(z_t, 1)
    % Save the current measurements' indexes in previously_observed_landmarks to be
    % used as c_t
    if isempty(previously_observed_landmarks)
        previously_observed_landmarks = [previously_observed_landmarks; z_t(curr_measured_landmark_cnt, end)];
    else
        curr_land_ind = z_t(curr_measured_landmark_cnt, end);
        idx = find(curr_land_ind == previously_observed_landmarks);
        if isempty(idx)
            % This is a new landmark:
            previously_observed_landmarks = [previously_observed_landmarks; z_t(curr_measured_landmark_cnt, end)];
        else
            % Previously seen landmark: Replace the new with the old one:
            previously_observed_landmarks(previously_observed_landmarks == curr_land_ind) = z_t(curr_measured_landmark_cnt, end);
        end
    end
    
    %%%%%%%%%%%%%%% Plotting the landmarks and highlighting the currently
    %%%%%%%%%%%%%%% observablle landmark
    plot(mu_t(1 + 3* previously_observed_landmarks), mu_t(1 + 3* previously_observed_landmarks + 1), 'r.', 'MarkerSize', 25),
    current_active_landmark_index = z_t(curr_measured_landmark_cnt, end);
    hold on,
    plot(mu_t(1 + 3* current_active_landmark_index), mu_t(1 + 3* current_active_landmark_index + 1), 'b.', 'MarkerSize', 25),
    hold off
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting the uncertainties
% hold on
% for land_cnt = 1: (length(mu_t)/3) - 1
%     % Extract the corresponding covariance
%     curr_cov = covariance_matrix(3* land_cnt + 1: 3* land_cnt + 1 + 1, 3* land_cnt + 1: 3* land_cnt + 1 + 1);
%     curr_mu = landmarks_states(3* land_cnt + 1: 3* land_cnt + 1 + 1);
%     ellipse(curr_cov(1), curr_cov(end), curr_cov(2), curr_mu(1), curr_mu(2));
% end
% hold off
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

axes(handles.axes1)
% save('previously_observed_landmarks.mat', 'previously_observed_landmarks')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Visualising the new situation of the robot
% Read the current state of the robot
if length(handles.axes1.Children) == 1
    all_points = handles.axes1.Children.Vertices;
else
    for obj_cnt = 1: length(handles.axes1.Children)
        curr_char = class(handles.axes1.Children(obj_cnt));
        dot_ind = strfind(curr_char, '.');
        curr_obj_class = curr_char(dot_ind(end) + 1: end);
        if strcmp(curr_obj_class, 'Patch') == 1
            break;
        end
    end
    all_points = handles.axes1.Children(obj_cnt).Vertices;
end

% Find the current orientation of the robot
x1 = all_points(1, 1: 2);
x2 = all_points(2, 1: 2);
x3 = all_points(3, 1: 2);
mid_point = 0.5* (x1 + x2);

curr_x = mid_point(1);
curr_y = mid_point(2);

angle_vector = x3 - mid_point;
curr_theta = atan2(angle_vector(2), angle_vector(1));
if curr_theta < 0
    curr_theta = 2*pi + curr_theta;
end

% curr_x, curr_y is the centre of rotation.
% Apply the motion to the robot
x1_rot = applyRotation(x1, [curr_x, curr_y], angular_speed * delta_t);
x1_tran = x1_rot + [cos(curr_theta + angular_speed * delta_t) * radial_speed * delta_t, sin(curr_theta + angular_speed * delta_t) * radial_speed * delta_t];

x2_rot = applyRotation(x2, [curr_x, curr_y], angular_speed * delta_t);
x2_tran = x2_rot + [cos(curr_theta + angular_speed * delta_t) * radial_speed * delta_t, sin(curr_theta + angular_speed * delta_t) * radial_speed * delta_t];

x3_rot = applyRotation(x3, [curr_x, curr_y], angular_speed * delta_t);
x3_tran = x3_rot + [cos(curr_theta + angular_speed * delta_t) * radial_speed * delta_t, sin(curr_theta + angular_speed * delta_t) * radial_speed * delta_t];

% Plot the new
handles.axes1;
set(handles.axes1, 'Xlim', [-10, 10])
set(handles.axes1, 'Ylim', [-10, 10])
if length(handles.axes1.Children) == 1
    set(handles.axes1.Children, 'Vertices', [x1_tran(1) x1_tran(2); x2_tran(1) x2_tran(2); x3_tran(1) x3_tran(2)])
else
    for obj_cnt = 1: length(handles.axes1.Children)
        curr_char = class(handles.axes1.Children(obj_cnt));
        dot_ind = strfind(curr_char, '.');
        curr_obj_class = curr_char(dot_ind(end) + 1: end);
        if strcmp(curr_obj_class, 'Patch') == 1
            break;
        end
    end
    set(handles.axes1.Children(obj_cnt), 'Vertices', [x1_tran(1) x1_tran(2); x2_tran(1) x2_tran(2); x3_tran(1) x3_tran(2)])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function pnt_rot = applyRotation(input_pnt, ref_pnt, added_angle)
pnt_rot = (input_pnt - ref_pnt)* [cos(added_angle) sin(added_angle); -sin(added_angle) cos(added_angle)] + ref_pnt;

function all_z = doMeasurements(handles, sense_radius)
% Read the Robot's middle point
if length(handles.axes1.Children) == 1
    all_points = handles.axes1.Children.Vertices;
else
    for obj_cnt = 1: length(handles.axes1.Children)
        curr_char = class(handles.axes1.Children(obj_cnt));
        dot_ind = strfind(curr_char, '.');
        curr_obj_class = curr_char(dot_ind(end) + 1: end);
        if strcmp(curr_obj_class, 'Patch') == 1
            break;
        end
    end
    all_points = handles.axes1.Children(obj_cnt).Vertices;
end

% Find the current orientation of the robot
x1 = all_points(1, 1: 2);
x2 = all_points(2, 1: 2);
x3 = all_points(3, 1: 2);
mid_point = 0.5* (x1 + x2);

% Read the landmark's locations
Patch_class_idx = obj_cnt;

all_z = [];
for land_cnt = 1: length(handles.axes1.Children)
    if land_cnt ~= Patch_class_idx
        curr_landmark_obj = handles.axes1.Children(land_cnt);
        curr_landmark = [curr_landmark_obj.XData', curr_landmark_obj.YData'];
        % If it was within the detectable range, make it red:
        land_is_detectable = norm(curr_landmark - mid_point, 2) <= sense_radius;
        if land_is_detectable == 1
            set(curr_landmark_obj, 'Color', [1 0 0])
            
            % Extract measurements for this landmark
            robot_axis_vector = x3 - mid_point;
            curr_landmark_vector = curr_landmark - mid_point;
            
            [curr_r, curr_phi] = Extract_phi_and_r(robot_axis_vector, curr_landmark_vector);
            all_z = [all_z; curr_r, curr_phi, land_cnt];
        else
            set(curr_landmark_obj, 'Color', [0 1 0])
        end
    end
end

% Show the coordinate of the detected measurements on the text box
if ~isempty(all_z)
    curr_string = num2str([all_z(:, 1), all_z(:, 2)* 180/ pi all_z(:, 3)]);
else
    curr_string = ' ';
end
set(handles.text2, 'String', curr_string)
% pause

function [curr_r, curr_phi] = Extract_phi_and_r(robot_axis_vector, curr_landmark_vector)
curr_r = norm(curr_landmark_vector, 2);
% First find the bearing angle of the two cvector to positive x-axis
% robot_axis_angle = atan2(robot_axis_vector(2), robot_axis_vector(1));
landmark_axis_angle = atan2(curr_landmark_vector(2), curr_landmark_vector(1));

%%% To make sure both angles are computed from the positive x-axis
% if robot_axis_angle < 0
%     robot_axis_angle = robot_axis_angle + 2*pi;
% end
if landmark_axis_angle < 0
    landmark_axis_angle = landmark_axis_angle + 2*pi;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% phi is the distance between the atan2 angles computed above
% curr_phi = landmark_axis_angle - robot_axis_angle;
curr_phi = landmark_axis_angle;

% --- Executes on mouse press over figure background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on mouse press over figure background.
function figure1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

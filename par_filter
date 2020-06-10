clear all
close all
clc


% Initializing the variables

x = 0.1; % Initial actual state
x_N = 1; % Noise covariance in the system 
x_M = 1; % Noise covariance in the measurement 
T = 150; % Duration of the chase (i.e. number of iterations)
N = 500; % The number of particles generated system 

%Initilize the initial, prior particle distribution around the initial value

V = 2; %define the variance of the initial esimate
x_Particles = []; % define the vector of particles

% Generating the random particles from the initial prior gaussian distribution
for i = 1:N
    x_Particles(i) = x + sqrt(V) * randn;
end

%{
% Plotting the distribution the particles around this initial value of x
figure(1)
plot(1,x_P,'.k','markersize',5)
xlabel(Time step')
ylabel('Flight position')
%}

% Generating the observations from the randomly selected particles, based upon
%the given function

z_out = [x^2 / 20 + sqrt(x_M) * randn];  % The actual output vector for measurement values.
x_out = [x];  % The actual output vector for measurement values.
x_est = [x]; % Estimate from particles taken time by time
x_est_out = [x_est]; % tThe output vector of particle filter estimates used for plotting

% The functions used by:
% x = 0.5*x + 25*x/(1 + x^2) + 8*cos(1.2*(t-1)) + process noise (process noise= sqrt(x_N)*randn)
% z = x^2/20 + measurement noise (measurement noise=  sqrt(x_R)*rand)

for t = 1:T
    
    % Updating the target and observed position
    
    x = 0.5*x + 25*x/(1 + x^2) + 8*cos(1.2*(t-1)) +  sqrt(x_N)*randn;
    z = x^2/20 + sqrt(x_M)*randn;
    
    % Particle filter
    for i = 1:N
        
        % With the given the prior set of particle running each of these particles through the state
        % update model to make a new set of transitioned particles
        
        x_Par_update(i) = 0.5*x_Particles(i) + 25*x_Particles(i)/(1 + x_Particles(i)^2) + 8*cos(1.2*(t-1)) + sqrt(x_N)*randn;
       
        % Using the newly updated particle locations, update the observations
        %for each of these particles
        
        z_update(i) = x_Par_update(i)^2/20;
        
        % Generating the weights for each of these particles
        
        P_w(i) = (1/sqrt(2*pi*x_M)) * exp(-(z - z_update(i))^2/(2*x_M));
    end
    
    % Normalize to form a probability distribution (i.e. sum to 1)
    
    P_w = P_w./sum(P_w);
    
    
    % Plotting observed values and updated particle positions
    
    %{
    figure(2)
    subplot(1,2,1)
    plot(P_w,z_update,'.k','markersize',5)
    hold on
    plot(0,z,'.r','markersize',50)
    xlabel('Weight magnitude')
    ylabel('Observed values (z update)')    
    subplot(1,2,2)
    plot(P_w,x_Par_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',50)
    xlabel('Weight magnitude')
    ylabel('Updated particle positions (x_Par update)')
  
    
    
    % Plotting the before and after scenerio
    figure(3)
    subplot(1,3,1)
    plot(0,x_Par_update,'.k','markersize',5)
    title('Raw estimates')
    xlabel('Fixed time point')
    ylabel('Estimated particles for flight position')
    subplot(1,3,2)
    plot(P_w,x_Par_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',40)
    xlabel('Weight magnitude')
    title('Weighted estimates')
    %}
    
    % Resampling: 
    % From this new distribution, random sampling is performed from it to generate our new estimate particles
    
    for i = 1 : N
        x_Particles(i) = x_Par_update(find(rand <= cumsum(P_w),1));
    end
    
   %The final estimate is some metric of these final resampling, like the mean value or variance
   
    x_est = mean(x_Particles);
    
    %{
    % Plotting after resampling
    subplot(1,3,3)
    plot(0,x_Par_update,'.k','markersize',5)
    hold on
    plot(0,x_Particles,'.r','markersize',5)
    plot(0,x_est,'.g','markersize',40)
    xlabel('Fixed time point')
    title('Weight based resampling')
    %}
    
    % Saving data in arrays for plotting
    x_out = [x_out x];
    z_out = [z_out z];
    x_est_out = [x_est_out x_est];
    
end

t = 0:T;
% Plotting the estimated values along with target values
figure(4)
plot(t, x_out, '.-k', t, x_est_out, '-.r','linewidth',3);
xlabel('Time step'); 
ylabel('Target flight position');
legend('True flight position', 'Particle filter estimate');

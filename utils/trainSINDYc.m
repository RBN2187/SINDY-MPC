DERIV_NOISE = 0;

if size(x,1)==size(u,2)
    u = u';
end

%% 1. COMPUTE DERIVATIVE ASSUMING NO NOISE IN THE SIGNAL
if any(strcmp(InputSignalType,{'sine2', 'sine3', 'chirp','prbs', 'sphs','mixed', 'noise','unforced'}) == 1) && DERIV_NOISE == 0
    % compute derivative using fourth order central difference
    dx = zeros(length(x)-5,3);
    for i=3:length(x)-3
        for k=1:size(x,2)
            dx(i-2,k) = (1/(12*dt))*(-x(i+2,k)+8*x(i+1,k)-8*x(i-1,k)+x(i-2,k));
        end
    end
    % concatenate
    xaug = [x(3:end-3,:) u(3:end-3,:)];
    dx(:,size(x,2)+1) = 0*dx(:,size(x,2));
end

n = size(dx,2);

%% SPARSE REGRESSION
clear Theta Xi

% Build the candidate function library Theta
Theta = poolData(xaug,n,polyorder,usesine);
Theta_norm = zeros(size(Theta,2),1);

% Normalize the columns of Theta to avoid biasing the regression towards functions with large magnitudes
for i = 1:size(Theta,2)
   Theta_norm(i) = norm(Theta(:,i));
   Theta(:,i) = Theta(:,i)./Theta_norm(i);
end

% Excecute the regression - determine the Xi matrix
if exist('lambda_vec') == 1
    % --- Call the sparsifyDynamicsIndependent function to perform sparse regression with STATE-DEPENDENT lambda ---
    Xi = sparsifyDynamicsIndependent(Theta,dx,lambda_vec,n-1);
else
    % --- Call the sparsifyDynamics function to perform sparse regression with a single lambda ---
    Xi = sparsifyDynamics(Theta,dx,lambda,n-1);
end

%% 3. POST-PROCESSING

if n == 3
    str_vars = {'x','y','u'};
elseif n == 4
    str_vars = {'x','y','z','u'};
elseif n == 6
    str_vars = {'x1','x2','x3','x4','x5','u'};    
end

for i = 1:size(Theta,2)
   Xi(i,:) = Xi(i,:)./Theta_norm(i);
end

% --- Call the poolDataLIST function to get the list (of strings) of active functions in the identified model ---
yout = poolDataLIST(str_vars,Xi,n,polyorder,usesine);
function [Estimation_X0] = RI_EKF_update_3d(Estimation_X0, CameraMeasurementThis, Sigma_OB, Noise_type)

NumberOfLandmarksObInThisStep = size(CameraMeasurementThis,1)/3; %dimension=3

% initialise the IndexOfFeature if possible
if size(Estimation_X0.landmarks,1) > 0
    IndexOfFeature = Estimation_X0.landmarks(:,1);  % the index of landmarks that have already been observed in pose0 pose1 ...pose(I-1)
else
    IndexOfFeature = [];          
end

IndexObservedAlreadyThis = [];
IndexObservedNew = [];   
for i = 1:NumberOfLandmarksObInThisStep
    % check whether the feature is observed before or not
    M = find( IndexOfFeature == CameraMeasurementThis(3*i,2) );
    if isempty(M)
        IndexObservedNew = [IndexObservedNew;CameraMeasurementThis(3*i,2) ];
    else
        IndexObservedAlreadyThis = [IndexObservedAlreadyThis;CameraMeasurementThis(3*i,2)];
    end             
end

Estimation_X0.IndexObservedNew=IndexObservedNew;
Estimation_X0.IndexObservedAlreadyThis=IndexObservedAlreadyThis;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IndexObservedNew=[78; 97; 18] indicates that the 
% robot firstly observes landmarks 78 97 18 in this step
% IndexObservedAlreadyThis=[19; 20; 53] indicates 
% that the robot observes again landmarks 19 20 53 in this step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

orientation = Estimation_X0.orientation;
position    = Estimation_X0.position;
cov         = Estimation_X0.cov;

NumberOfFeature = size( IndexOfFeature,1);
NumberOfOldFeatureInThisStep = size(IndexObservedAlreadyThis,1);
NumberOfNewFeatureInThisStep = size(IndexObservedNew,1);
 
   
% update state and covariance (normal kalman update)
if ~isempty(IndexObservedAlreadyThis)
%     orientation = Estimation_X0.orientation;
%     position    = Estimation_X0.position;
%     cov         = Estimation_X0.cov;

    Z = zeros( NumberOfOldFeatureInThisStep*3 , 1); 
    Y = zeros( NumberOfOldFeatureInThisStep*3 , 1); 
    H = zeros(3*NumberOfOldFeatureInThisStep, 6+3*NumberOfFeature);
    
    temp = repmat({eye(3)}, NumberOfOldFeatureInThisStep,1 );
    R = blkdiag(temp{:});
    
    % update old features
    for i = 1:NumberOfOldFeatureInThisStep
        ind = find(IndexOfFeature == IndexObservedAlreadyThis(i));
        fi  = Estimation_X0.landmarks(ind,2:4)';
        Y(3*i-2:3*i,1) = ObservationModel( orientation, position, fi );
        
        
        
        ind2 = find(CameraMeasurementThis(:,2) == IndexObservedAlreadyThis(i));
        Z(3*i-2:3*i,1 ) = CameraMeasurementThis(ind2,1);
        
        H(3*i-2:3*i, 4:6) = orientation'; % the derivative of ObservationModel to [R p]
        H(3*i-2:3*i, 6+3*ind-2:6+3*ind) = -orientation';                    % the derivative of ObservationModel to fi
        
        switch Noise_type
            case 0
                R(3*i-2:3*i,3*i-2:3*i) = eye(3)*Sigma_OB^2;
            case 1
                R(3*i-2:3*i,3*i-2:3*i) = diag(CameraMeasurementThis(ind2,1).^2)*Sigma_OB^2; %WHY?
            otherwise
                sprintf('Error in EKF_update_2d: the noise_type has not been defined. Please check config_2d')
                return;
        end        
        
        
        
        
%         R(3*i-2:3*i,3*i-2:3*i) = diag(CameraMeasurementThis(ind2,1).^2)*Sigma_OB^2;
    end    
    
    
    % question @RomaTeng, different computaton scheme
    z = Z-Y;
    S = H*cov*H'+R;
    K = cov*H'*inv(S);
    s = K*z;
    
    Estimation_X0 = special_add_right(Estimation_X0,-s);
    cov = ( eye(6+3*NumberOfFeature) -K*H )*cov;
    Estimation_X0.cov = cov;
    
    % @todo @RomaTeng, right Jacobian
    %Estimation_X0.cov=JJJr(-s)*cov*(JJJr(-s))';
end  
     

% update state vector and covariance by considering 
% new feature into state and covariance (initialize)
if ~isempty(IndexObservedNew)
    % copy previous covariance
%     orientation = Estimation_X0.orientation;
%     position    = Estimation_X0.position;
%     cov         = Estimation_X0.cov;
    
    temp    = repmat({eye(3)}, NumberOfNewFeatureInThisStep, 1 );
    tempKK  = blkdiag(temp{:});
%     Sigma   = blkdiag(Estimation_X0.cov,tempKK);
    KK      = eye(6+3*(NumberOfFeature+NumberOfNewFeatureInThisStep));
    
    
    for i = 1:NumberOfNewFeatureInThisStep
        %% add new features
        indNewf = IndexObservedNew(i);
        Estimation_X0.landmarks(NumberOfFeature+i,1) = indNewf;
        m2 = find( CameraMeasurementThis(:,2) == indNewf );
        nf = CameraMeasurementThis( m2, 1 );
        Estimation_X0.landmarks(NumberOfFeature+i,2:4) = (Estimation_X0.orientation*nf+Estimation_X0.position)'; % change is to ground truth
        %% Change the state cov
        KK( 6+3*NumberOfFeature+3*i-2:6+3*NumberOfFeature+3*i,4:6 ) = eye(3);  % just the derivative of  Estimation_X0.orientation*nf+Estimation_X0.position  to [R p] 
        KK( 6+3*NumberOfFeature+3*i-2:6+3*NumberOfFeature+3*i,6+3*NumberOfFeature+3*i-2:6+3*NumberOfFeature+3*i ) = Estimation_X0.orientation; % just the derivative of  Estimation_X0.orientation*nf+Estimation_X0.position  to nf
        switch Noise_type
            case 0
                tempKK(3*i-2:3*i,3*i-2:3*i)=eye(3)*Sigma_OB^2;
            case 1
                tempKK(3*i-2:3*i,3*i-2:3*i)=diag(nf.^2)*Sigma_OB^2;
            otherwise
                sprintf('Error in EKF_update_2d: the noise_type has not been defined. Please check config_2d')
                return;
        end      
    end
    Sigma   = blkdiag(Estimation_X0.cov,tempKK); % a larger matrix the top left corner is Estimation_X0.cov (2+NumberOfNewFeatureInThisStep)*(2+NumberOfNewFeatureInThisStep)
    Estimation_X0.cov = KK*Sigma*KK';
end

% clearvars -except Estimation_X0 CameraMeasurementThis obsv_sigma
end
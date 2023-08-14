%%%%%%%%%%%%%%%
%{
Inputs: 
    estimation_x.orientation estimation_x.position estimation_x.cov estimation_x.landmarks
    CameraMeasurementThis: [x poseI;
                            y poseI]
    Estimation_X0.landmarks: N*3, the 1-th column is the index of the landmarks that have been put into the state. Notice the
                             coordinates need to be transposited
                             Here, I put the index into the first culumn, for not changing too much when applying 1D, 2D, and 3D
%}
%%%%%%%%%%%%%%%
function [Estimation_X0] = EKF_update_2d(Estimation_X0, CameraMeasurementThis, Sigma_OB, Noise_type)


NumberOfLandmarksObInThisStep = size(CameraMeasurementThis,1)/2; %dimension=2

% initialise the IndexOfFeature if possible
if size(Estimation_X0.landmarks,1) > 0
    IndexOfFeature = Estimation_X0.landmarks(:,1); % the index of landmarks that have already been observed in pose0 pose1 ...pose(I-1)
else
    IndexOfFeature = [];
end

IndexObservedAlreadyThis = [];
IndexObservedNew = [];   
for i = 1:NumberOfLandmarksObInThisStep
    % check whether the feature is observed before or not
    M = find( IndexOfFeature == CameraMeasurementThis(2*i,2) );   %NOTICE: it is not an applicable way using the index of a landmark: in practice, we cannot know the 
                                                                  %        index of a landmark, therefore, we can only compare the coordinate (morn) of the two landmarks
                                                                  % 2*i or 2*i-1 is the same index
    if isempty(M)
        IndexObservedNew = [IndexObservedNew; CameraMeasurementThis(2*i,2) ]; % if it is not observed in the last pose, then put the index in IndexObservedNew
    else
        IndexObservedAlreadyThis = [IndexObservedAlreadyThis; CameraMeasurementThis(2*i,2)];
    end             
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IndexObservedNew=[78; 97; 18] indicates that the 
% robot firstly observes landmarks 78 97 18 in this step
% IndexObservedAlreadyThis=[19; 20; 53] indicates 
% that the robot observes again landmarks 19 20 53 in this step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


NumberOfFeature = size( IndexOfFeature,1);                        %WHY? NumberOfFeature != NumberOfLandmarksObInThisStep. All features are the local frame of the observed landmark, isn't?
NumberOfOldFeatureInThisStep = size(IndexObservedAlreadyThis,1);
NumberOfNewFeatureInThisStep = size(IndexObservedNew,1);
 
   
%% update state and covariance
%{
    For the landmarks that have already been observed in the last pose, it is like a 'one-step loop closure'. This can help to
    reduce the uncertainty of the pose, manely, to reduce the covariance
%}
if ~isempty(IndexObservedAlreadyThis)
    
    orientation = Estimation_X0.orientation;
    position    = Estimation_X0.position;
    cov         = Estimation_X0.cov;

    Y = zeros( NumberOfOldFeatureInThisStep*2 , 1);   % the observation we get at poseI, it is Yn in paper Axel-2016
    Z = zeros( NumberOfOldFeatureInThisStep*2 , 1);   % the observation we calculate according to old landmarks and poseI, it is Zn in paper Axel-2016
    
    H = zeros(2*NumberOfOldFeatureInThisStep, 3+2*NumberOfFeature);  %WHY
    
    temp = repmat({eye(2)}, NumberOfOldFeatureInThisStep,1 ); % repmat copy eye(3): {eye(3)} is a cell
    R = blkdiag(temp{:});                                     % outputs a block diagonal matrix. read doc blkdiag
    
    % update old features
    for i = 1:NumberOfOldFeatureInThisStep
        ind = find(IndexOfFeature == IndexObservedAlreadyThis(i)); % the index of the IndexOfFeature that has been observed
        fi  = Estimation_X0.landmarks(ind,2:3)';                    % the coordinate of all the old features
        Z(2*i-1:2*i,1) = ObservationModel( orientation, position, fi );  % convert landmarks from ground truth to poseI (the coordinate is according to the state)
        
        ind2 = find(CameraMeasurementThis(:,2) == IndexObservedAlreadyThis(i));  % index of the old features in CameraMeasurementThis (coordinate 2*~)
        Y(2*i-1:2*i,1 ) = CameraMeasurementThis(ind2,1);           % the observations of the old landmarks in the frame of poseI
        
        H(2*i-1:2*i, 2:3) = -orientation';
        H(2*i-1:2*i, 3+2*ind-1:3+2*ind) = orientation';
% H(2*i-1:2*i, 3+2*i-1:3+2*i) = orientation';
        switch Noise_type
            case 0
                R(2*i-1:2*i,2*i-1:2*i) = eye(2)*Sigma_OB^2;
            case 1
                R(2*i-1:2*i,2*i-1:2*i) = diag(CameraMeasurementThis(ind2,1).^2)*Sigma_OB^2; %WHY?
            otherwise
                sprintf('Error in EKF_update_2d: the noise_type has not been defined. Please check config_2d')
                return;
        end 
    end    
    
    % question @RomaTeng, different computaton scheme
    %z = Y-Z;            %z: (3*2n)*1
    z = Y-Z;            %z: (3*2n)*1
    S = H*cov*H'+R;
    K = cov*H'*inv(S);
    s = K*z;            %s: (3+2n)*1 
                        %WHY? the first one is theta
    
    Estimation_X0 = special_add_right_2d(Estimation_X0, s);
    Estimation_X0.cov = ( eye(3+2*NumberOfFeature) -K*H )*cov;
end  
     
%% update state vector and covariance by considering 
% new feature into state and covariance
if ~isempty(IndexObservedNew)

    
    % copy previous covariance
    temp    = repmat({eye(2)}, NumberOfNewFeatureInThisStep, 1 );
    tempKK  = blkdiag(temp{:});                     
    KK      = eye(3+2*(NumberOfFeature+NumberOfNewFeatureInThisStep)); % actually we can just use KK = Sigma to initialize KK
    
    % add new features
    for i = 1:NumberOfNewFeatureInThisStep
        %% Add new observation into state (new landmark)
        indNewf = IndexObservedNew(i);     % the index of new observed landmark
        Estimation_X0.landmarks(NumberOfFeature+i,1) = indNewf;       % why do not use the normal way, to transposition it.
        m2 = find( CameraMeasurementThis(:,2) == indNewf ); % in which lines (two lines)
        nf = CameraMeasurementThis( m2, 1 );                % the corresponding coordinates

        Estimation_X0.landmarks(NumberOfFeature+i,2:3) = (Estimation_X0.orientation*nf+Estimation_X0.position)'; % change is to ground truth
        %% Change the state cov

        KK( 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i, 2:3 ) = eye(2); %the derivative of 'orientation*nf+position' to [R p]
        KK( 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i, 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i ) = Estimation_X0.orientation; %the derivative of 'orientation*nf+position' to nf
%         J = [0 -1;1 0];
%         KK( 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i, 1:3 ) = [(Estimation_X0.orientation)*J*(nf), eye(2)];              %the derivative of 'orientation*nf+position' to [R p]
%         KK( 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i, 3+2*NumberOfFeature+2*i-1:3+2*NumberOfFeature+2*i ) = Estimation_X0.orientation; 
        
        switch Noise_type
            case 0
                tempKK(2*i-1:2*i,2*i-1:2*i)=eye(2)*Sigma_OB^2;
            case 1
                tempKK(2*i-1:2*i,2*i-1:2*i)=diag(nf.^2)*Sigma_OB^2;
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
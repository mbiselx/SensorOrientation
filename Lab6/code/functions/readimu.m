function [data,fIMU] = readimu(varargin)
	% [data,fIMU] = readimu returns data recorded from an IMU
	%        the applications asks for the data file and
	%        for the type of IMU.
	%
	% data = readimu(file) to specify the file path
	%
	% data = readidata_mu(file,imu_name) to specify file path and
	%        type of IMU.
	% 
	% List of the IMU available: IMAR, LN200, LN200IG, IXSEA, XSENS
    %                            NAVCHIP_INT, NAVCHIP_FLT, GEN_LONG, 
    %                            GEN_DOUBLE, TXT, MAT
	
	%% IMUs DEFINITION
	imu  = struct('name',{},'externalFcn',{},'BitsPerEpoch',{},'TimeType',{},'DataType',{},'HeaderSize',{},'ScaleGyro',{},'ScaleAcc',{});
	j = 0;
	% IMAR FSAS
	j=j+1;
	imu(j).name         = 'IMAR';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'long';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 0.10000000*pi/180/3600;  % Scale gyro to rad
	imu(j).ScaleAcc     = 0.00152588/1000;         % Scale accel to m/s
	% LN200
	j=j+1;
	imu(j).name         = 'LN200';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'long';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 1/2097152.0;       % Scale gyro to rad
	imu(j).ScaleAcc     = 1/16384.0;         % scale accel to m/s
    % LN200IG
	j=j+1;
	imu(j).name         = 'LN200IG';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'long';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 1/524288.0;        % Scale gyro to rad
	imu(j).ScaleAcc     = 1/16384.0;         % scale accel to m/s
	% IXSEA
	j=j+1;
	imu(j).name         = 'IXSEA';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'double';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = pi/180/3600;       % Scale gyro to rad
	imu(j).ScaleAcc     = 1/1000;			 % scale accel to m/s
    % NAVCHIP_FLOAT
	j=j+1;
	imu(j).name         = 'NAVCHIP_FLT';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'double';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 1/3600/360*2*pi; % Scale gyro to rad
	imu(j).ScaleAcc     = 1/1000;            % scale accel to m/s
    % NAVCHIP_INT
    j=j+1;
	imu(j).name         = 'NAVCHIP_INT';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'long';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 0.00000625;       % Scale gyro to rad
	imu(j).ScaleAcc     = 39.0625e-6;       % scale accel to m/s
    % GEN_LONG 
    j=j+1;
	imu(j).name         = 'GENERIC_LONG';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'long';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 1;       % Scale gyro to rad
	imu(j).ScaleAcc     = 1;       % scale accel to m/s
    % GEN_DOUBLE
    j=j+1;
	imu(j).name         = 'GENERIC_DOUBLE';
	imu(j).TimeType     = 'double';
	imu(j).DataType     = 'double';
	imu(j).HeaderSize   = 0;
	imu(j).ScaleGyro    = 1;       % Scale gyro to rad
	imu(j).ScaleAcc     = 1;       % scale accel to m/s
	% XSENS
	j = j+1;
	imu(j).name = 'XSENS';
	imu(j).externalFcn = @readXSENSFile;
    % TEXT FILE
    j = j+1;
    imu(j).name = 'TXT'; 
    imu(j).format = '%f,%f,%f,%f,%f,%f,%f';
    imu(j).no_of_datavalues = 7;
    % MAT FILE
    j = j+1;
    imu(j).name = 'MAT'; 
    
    % check for integer imu rate of the following kind:
            % 100Hz, 200Hz, 250Hz, 333HZ, 400Hz, 500Hz, 1000Hz OR the current one
        
    acceptable_frequencies = [100; 200; 250; 333; 400; 500; 1000]; % [Hz]   
    
	
	%% WORK AROUND
	% list of IMUs (cell and char array)
	nImu = numel(imu);
	imuList = struct2cell(imu);
	imuList = imuList(1,:,:);
	imuList = reshape(imuList,nImu,1);
	imuListStr = sprintf('%s, ',imuList{:});
	imuListstr = imuListStr(1:end-2);
	
	% associative array for size of type use
	SizeOf = java.util.HashMap;
	SizeOf.put('double',8);
	SizeOf.put('long'  ,4);
	
	% check that every type exists in SizeOf exeept for the last which is a
	% txt-file
	for i = 1:nImu-2
		if isa(imu(i).externalFcn,'function_handle'),continue,end 
		if isempty(SizeOf.get(imu(i).TimeType))
			error('In %s, the format for TimeType ''%s'' could not be found in format list (var: SizeOf)',imu(i).name,imu(i).TimeType)
		end
		if isempty(SizeOf.get(imu(i).DataType))
			error('In %s, the format for DataType ''%s'' could not be found in format list (var: SizeOf)',imu(i).name,imu(i).DataType)
		end		
	end
	
	%% IMU FILE
	if nargin == 0
		[f,p] = uigetfile('*.*','Please select data file');
		if f == 0,return,end
		imufile = [p f];
	else
		imufile = varargin{1};
	end
	if ~exist(imufile,'file')
		error('File %s does not exist.',imufile)
	end
	
	%% TYPE OF IMU
	k = 0;
	if nargin > 1
		for i = 1:nImu
			if strcmpi(imu(i).name,varargin{2})
				k = i;
				break
			end
		end
		if k == 0,error('The IMU ''%s'' could not be found in the list of IMUs (%s)',varargin{2},imuListstr),end
	else
		k = listdlg('PromptString','Choose an IMU:',...
						'SelectionMode','single',...
						'ListSize',[100 200],...
						'ListString',imuList);
		if isempty(k),return,end
	end	
		
	%% READ DATA FILE
	if ishandle(imu(k).externalFcn)
		data = imu(k).externalFcn(imufile);
    elseif  strcmp(imu(k).name, 'MAT')
        tic
        % load data file
        load(imufile);
        existance = exist('data');
        
        if existance ~= 1 % the value 1 means a variable
            error('There is no "data"-fild in the mat-file open %s',imufile)
        end
        
        fIMU = 1/mean(diff(data(:,1)));
        
        fprintf(' (data @ %.2f Hz, format %s) ...',fIMU,imu(k).name); 
        
        fprintf(' done in %.1f sec.\n',toc);
        
    elseif  strcmp(imu(k).name, 'TXT')
        % Open data file
		fid = fopen(imufile,'r');
		if fid < 0,error('Cannot open %s',imufile),end
        
        % init time counter
		tic
        
        % read the data
        data = fscanf(fid,imu(k).format,[imu(k).no_of_datavalues Inf]);
        
        data = data';
        
        data = data(:,1:7);
        
%         data(:,1) = 0:0.005:(length(data(:,1))-1)*0.005;
        
        fIMU = 1/mean(diff(data(:,1)));
        
        fprintf(' (data @ %.2f Hz, format %s) ...',fIMU,imu(k).name); 
        
        % Close opened file
		fclose(fid);

		fprintf(' done in %.1f sec.\n',toc)
        
	else
		% Open data file
		fid = fopen(imufile,'r');
		if fid < 0,error('Cannot open %s',imufile),end

		% init time counter
		tic
		
		% BitsPerEpoch
		BitsPerEpoch = SizeOf.get(imu(k).TimeType) + 6*SizeOf.get(imu(k).DataType);

		% Set cursor at end of file
		fseek(fid, 0, 'eof');

		% Count epochs and control it
		nEpochs = (ftell(fid)-imu(k).HeaderSize)/BitsPerEpoch;

        %%%%TOREMOVE
        nEpochs = ((ftell(fid) -mod(ftell(fid), BitsPerEpoch))-imu(k).HeaderSize)/BitsPerEpoch;
        
		if rem(nEpochs,1)~=0,error('The files has not an expected size. Control the type of IMU or if the file is corrupted.'),end

		% display info to command window
		fprintf('%s',imufile)
		fprintf(' contains %d epochs\n',nEpochs)
		fprintf('Reading ...')

		% Init. data matrix
		data = zeros(7,nEpochs);

		% Set cursor at begining of data (skip header if it exists)
		fseek(fid, imu(k).HeaderSize, 'bof');
		% Read time
		% output                until end                       skip the size of an epoch minus the the size of time
		%data(1,:)   = fread(fid, [1,Inf], imu(k).TimeType       , BitsPerEpoch-SizeOf.get(imu(k).TimeType));
        
        %TOREMOVE !!!
        data(1,:)   = fread(fid, [1,nEpochs], imu(k).TimeType       , BitsPerEpoch-SizeOf.get(imu(k).TimeType));

		% Set cursor at begining of gyro-acc meas in file (skip header + first time value)
		fseek(fid, imu(k).HeaderSize+SizeOf.get(imu(k).TimeType) , 'bof');

		% Read acc + gyro (6*)
		data(2:7,:) = fread(fid, [6,nEpochs], ['6*' imu(k).DataType], BitsPerEpoch-6*SizeOf.get(imu(k).DataType));

		% Data Rate
		fIMU = round(1./mean( diff(data(1,:)) ));
              
        % if the calculated frequency is not part of the accepted frequencies
        if (any(fIMU==acceptable_frequencies) == 0)
            
            acceptable_frequencies = [acceptable_frequencies; fIMU];
            freqList = num2str(acceptable_frequencies);
            
            fprintf('\n-->The current frequency of %.2f [Hz] does not correspond to the usual frequencies used in the Lab\n', fIMU)
            fprintf '       Please chooce from the following list:\n'
            z = listdlg('PromptString','Choose a Frequency [Hz]:',...
                'SelectionMode','single',...
                'ListSize',[200 100],...
                'ListString',freqList);
            if isempty(z),return,end	
            
            fIMU = acceptable_frequencies(z,1);
            
        end

        
        %fIMU = round(fIMU); 
		fprintf(' (data @ %.2f Hz, sGr %f sAc %d) ...',fIMU,imu(k).ScaleGyro, imu(k).ScaleAcc); 
        
		% Scale data for accelerometer and gyroscope
		data(2:4,:) = data(2:4,:) * imu(k).ScaleGyro;  
		data(5:7,:) = data(5:7,:) * imu(k).ScaleAcc;
        
        
        
        % Scale data for frequency of the IMU, only if it is not the generic one
        if k < 7
            data(2:4,:) = data(2:4,:) * fIMU;  
            data(5:7,:) = data(5:7,:) * fIMU;
        end
            

		% Transpose to fit in vectors
		data = data';
        
		% Close opened file
		fclose(fid);

		fprintf(' done in %.1f sec.\n',toc)
	end
end
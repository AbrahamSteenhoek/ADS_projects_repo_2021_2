function [s_side1,s_side2] = IEEEP3702xThru(s_2xthru,varargin)
% Copyright 2017 The Institute of Electrical and Electronics Engineers, Incorporated (IEEE).
%
% This work is licensed to The Institute of Electrical and Electronics
% Engineers, Incorporated (IEEE) under one or more contributor license
% agreements.
%
% See the NOTICE file distributed with this work for additional
% information regarding copyright ownership. Use of this file is
% governed by a BSD-style license, the terms of which are as follows:
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
%   * Redistributions of source code must retain the above copyright
%   notice, this list of conditions, the following disclaimer, and the
%   NOTICE file.
%
%   * Redistributions in binary form must reproduce the above
%   copyright notice, this list of conditions, the following
%   disclaimer in the documentation and/or other materials provided
%   with the distribution, and the NOTICE file.
%
%   * Neither the name of The Institute of Electrical and Electronics
%   Engineers, Incorporated (IEEE) nor the names of its contributors
%   may be used to endorse or promote products derived from this
%   software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% SPDX-License-Identifier: BSD-3-Clause
%
% Function: [s_side1,s_side2] = IEEEP3702xThru(s_2xthru)
%
% Author: Jason J. Ellison, Published 7/15/2021
% Revision: v17
% 
% IEEEP3702xThru.m creates error boxes from a test fixture 2x thru. 
% 
% Input: 
% s_2xthru--an s parameter object of the 2x thru.
% 
% Outputs: 
% s_side1--an s parameter object of the error box representing the half of the 2x thru connected to port 1
% s_side2--an s parameter object of the error box representing the half of the 2x thru connected to port 2
% 
% residual test usage:
% 
% [s_side1,s_side2] = IEEEP3702xThru(s_2xthru);
% s_deembedded_dut = deembedsparams(s_fixture_dut_fixture,s_side1,s_side2);

% ------------ name-value pair import -------------
% ## defaults ##
use_ifft = true;

% ## name-value pairs ##
names = varargin(1:2:end);
values = varargin(2:2:end);

for i = 1:length(names)
    names{i} = validatestring(names{i},["ifft"]);
    switch names{i}
        case "ifft"
            use_ifft = values{i};
    end
end

f = s_2xthru.Frequencies.';
s = s_2xthru.Parameters;

% ----------- main --------------------
if use_ifft

    % strip DC point if one exists
    if f(1) == 0
        warning('DC point detected. An interpolated DC point will be included in the errorboxes.');
        flag_DC = 1;
        fold = f;
        f = f(2:end);
        s = s(:,:,2:end);
    else
        flag_DC = 0;
    end

    % interpolate S-parameters if the frequency vector is not acceptable
    if f(2) - f(1) ~= f(1)               
        
        % set the flag
        flag_df = 1;
        warning('Non-uniform frequency vector detected. A spline interpolated S-parameter matrix will be created for this calculation. The output results will be re-interpolated to the original vector.');
        fold = f;
        % sold = s; % currently unused, but is here for debug.
        df = f(2) - f(1);
        projected_n = round(f(end)/f(1));
        if projected_n <= 10000            
            if mod(f(end),f(1)) == 0
                fnew = f(1):f(1):f(end);
            else
                fnew = f(1):f(1):(f(end) - mod(f(end),f(1)));
            end
        else
            new_df = f(end)/10000;
            fnew = new_df:new_df:f(end);
            disp("interpolating from " + string(new_df) + "Hz to " + string(f(end)) + "Hz with 10000 points.");
        end
        snew = zeros(2,2,length(fnew));
        for i = 1:2
            for j = 1:2
                snew(i,j,:) = interp1(f,squeeze(s(i,j,:)),fnew,'spline');
            end
        end        
        
        s = snew;
        f = fnew;
    else
        flag_df = 0;
    end

    n = length(f);
    s11 = squeeze(s(1,1,:));

    % get e001 and e002
    % e001
    s21 = squeeze(s(2,1,:));
    dcs21 = dc_interp(s21,f);
    t21 = fftshift(ifft(makeSymmetric([dcs21;s21]),'symmetric'));
    [~,x] = max(t21);

    dcs11 = DC(s11,f);
    t11 = fftshift(ifft(makeSymmetric([dcs11;s11]),'symmetric'));
    step11 = makeStep(t11);
    z11 = -50.*(step11 + 1)./(step11 - 1);
    z11x = z11(x);

    temp = sparameters(s,f,50);
    temp = sparameters(temp,z11x);
    sr = temp.Parameters; 
    clear temp;

    s11r = squeeze(sr(1,1,:));
    s21r = squeeze(sr(2,1,:));
    s12r = squeeze(sr(1,2,:));
    s22r = squeeze(sr(2,2,:));

    dcs11r = DC(s11r,f);
    t11r = fftshift(ifft(makeSymmetric([dcs11r;s11r]),'symmetric'));
    t11r(x:end) = 0;
    e001 = fft(ifftshift(t11r));
    e001 = e001(2:n+1);

    dcs22r = DC(s22r,f);
    t22r = fftshift(ifft(makeSymmetric([dcs22r;s22r]),'symmetric'));
    t22r(x:end) = 0;
    e002 = fft(ifftshift(t22r));
    e002 = e002(2:n+1);

    % calc e111 and e112
    e111 = (s22r - e002)./s12r;
    e112 = (s11r - e001)./s21r;

    % calc e01
    k = 1;
    test = k.*sqrt(s21r.*(1 - e111.*e112));
    e01 = zeros(n,1);
    for i = 1:n
        if i > 1
            if angle(test(i))- angle(test(i-1)) > 0
                k = -1*k;
            end
        end
        e01(i) = k*sqrt(s21r(i)*(1 - e111(i)*e112(i)));    
    end

    % calc e10
    k = 1;
    test = k.*sqrt(s12r.*(1 - e111.*e112));
    e10 = zeros(n,1);
    for i = 1:n
        if i > 1
            if angle(test(i))- angle(test(i-1)) > 0
                k = -1*k;
            end
        end
        e10(i) = k*sqrt(s12r(i)*(1 - e111(i)*e112(i)));    
    end

    % S-parameters are setup correctly
    if and(flag_DC == 0, flag_df == 0)
        fixture_model_1r = zeros(2,2,n);
        fixture_model_1r(1,1,:) = e001;
        fixture_model_1r(2,1,:) = e01;
        fixture_model_1r(1,2,:) = e01;
        fixture_model_1r(2,2,:) = e111;

        fixture_model_2r = zeros(2,2,n);
        fixture_model_2r(2,2,:) = e002;
        fixture_model_2r(1,2,:) = e10;
        fixture_model_2r(2,1,:) = e10;
        fixture_model_2r(1,1,:) = e112;
    else % S-parameters are not setup correctly
        if flag_DC == 1 % DC Point was included in the original file
            fixture_model_1r = zeros(2,2,n+1);
            fixture_model_1r(1,1,2:end) = e001; fixture_model_1r(1,1,1) = dc_interp(squeeze(fixture_model_1r(1,1,2:end)),f);
            fixture_model_1r(2,1,2:end) = e01; fixture_model_1r(2,1,1) = dc_interp(squeeze(fixture_model_1r(2,1,2:end)),f);
            fixture_model_1r(1,2,2:end) = e01; fixture_model_1r(1,2,1) = dc_interp(squeeze(fixture_model_1r(1,2,2:end)),f);
            fixture_model_1r(2,2,2:end) = e111; fixture_model_1r(2,2,1) = dc_interp(squeeze(fixture_model_1r(2,2,2:end)),f);

            fixture_model_2r = zeros(2,2,n+1);
            fixture_model_2r(2,2,2:end) = e002; fixture_model_2r(1,1,1) = dc_interp(squeeze(fixture_model_2r(1,1,2:end)),f);
            fixture_model_2r(1,2,2:end) = e10; fixture_model_2r(2,1,1) = dc_interp(squeeze(fixture_model_2r(2,1,2:end)),f);
            fixture_model_2r(2,1,2:end) = e10; fixture_model_2r(1,2,1) = dc_interp(squeeze(fixture_model_2r(1,2,2:end)),f);
            fixture_model_2r(1,1,2:end) = e112; fixture_model_2r(2,2,1) = dc_interp(squeeze(fixture_model_2r(2,2,2:end)),f);
            f = [0 f];
        else % DC Point wasn't included in the original file, but the DF was not the same as f(1)
            fixture_model_1r = zeros(2,2,n);
            fixture_model_1r(1,1,:) = e001;
            fixture_model_1r(2,1,:) = e01;
            fixture_model_1r(1,2,:) = e01;
            fixture_model_1r(2,2,:) = e111;

            fixture_model_2r = zeros(2,2,n);
            fixture_model_2r(2,2,:) = e002;
            fixture_model_2r(1,2,:) = e10;
            fixture_model_2r(2,1,:) = e10;
            fixture_model_2r(1,1,:) = e112;
        end
        if flag_df == 1 % if df was not the same as f(1)
            % save the current errorboxes
            fixture_model_1r_temp = fixture_model_1r;
            fixture_model_2r_temp = fixture_model_2r;
            % initialize the new errorboxes
            fixture_model_1r = zeros(2,2,length(fold));
            fixture_model_2r = zeros(2,2,length(fold));
            % interpolate the errorboxes to the original frequency vector
            for i = 1:2
                for j = 1:2
                    fixture_model_1r(i,j,:) = interp1(f,squeeze(fixture_model_1r_temp(i,j,:)),fold,'spline');
                    fixture_model_2r(i,j,:) = interp1(f,squeeze(fixture_model_2r_temp(i,j,:)),fold,'spline');                
                end
            end
        end
        % replace the vector used for the calculation with the original vector.
        f = fold; 
    end

    % create the S-parameter objects for the errorboxes
    s_fixture_model_r1 = sparameters(fixture_model_1r,f,z11x);
    s_fixture_model_r2 = sparameters(fixture_model_2r,f,z11x);

    % renormalize the S-parameter errorboxes to the original reference impedance (assumed to be 50)
    s_side1 = sparameters(s_fixture_model_r1,50);
    s_side2 = sparameters(s_fixture_model_r2,50);
else
    z = s2z(s,50);
    ZL = zeros(2,2,length(z));
    ZR = zeros(2,2,length(z));

    for i = 1:length(z)
        ZL(:,:,i) = [z(1,1,i) + z(2,1,i) 2*z(2,1,i);2*z(2,1,i) 2*z(2,1,i)];
        ZR(:,:,i) = [2*z(1,2,i) 2*z(1,2,i);2*z(1,2,i) z(1,2,i) + z(2,2,i)];
    end

    SL = z2s(ZL,50);
    SR = z2s(ZR,50);
    
    s_side1 = sparameters(SL,f);
    s_side2 = sparameters(SR,f);
    
end

% ------------- supporting functions ------------------

function [symmetric] = makeSymmetric(nonsymmetric)
    % [symmetric] = makeSymmetric(nonsymmetric)
    % this takes the nonsymmetric frequency domain input and makes it
    % symmetric.
    %
    % The function assumes the DC point is in the nonsymmetric data

    symmetric_abs = [abs(nonsymmetric); flip(abs(nonsymmetric(2:end)))];
    symmetric_ang = [angle(nonsymmetric); -flip(angle(nonsymmetric(2:end)))];
    symmetric = symmetric_abs.*exp(1i.*symmetric_ang);
    

function [step] = makeStep(impulse)
    ustep = ones(length(impulse),1);    
    step = conv(ustep,impulse);
    step = step(1:length(impulse));
    
function [DCpoint] = DC(s,f)       
        DCpoint = 0.002; % seed for the algorithm
        err = 1; % error seed
        allowedError = 1e-12; % allowable error
        cnt = 0;
        df = f(2) - f(1);
        n = length(f);
        t = linspace(-1/df,1/df,n*2+1);
        [~,ts] = min(abs(t - (-3e-9)));
        Hr = COM_receiver_noise_filter(f,f(end)/2);
        while (err > allowedError)
            h1 = makeStep(fftshift(ifft(makeSymmetric([DCpoint;s.*Hr.']),'symmetric')));
            h2 = makeStep(fftshift(ifft(makeSymmetric([DCpoint+0.001;s.*Hr.']),'symmetric')));
            m = (h2(ts)-h1(ts))/0.001;
            b = h1(ts) - m*DCpoint;
            DCpoint = (0 - b)/m;
            err = abs(h1(ts) - 0);
            cnt = cnt+1;
        end
        
function Hr = COM_receiver_noise_filter(f,fr)
    % reciever filter in COM defined by eq 93A-20
    fdfr = f./fr;
    Hr = 1./(1 - 3.414214.*(fdfr).^2 + fdfr.^4 + 1i.*2.613126.*(fdfr - fdfr.^3)); % eq 93A-20
    
function dc = dc_interp(sin,f)
% enforces symmetric upon the first 10 points and interpolates the DC
% point.

sp = sin(1:10);
fp = f(1:10).';

snp = [conj(flip(sp)) ; sp];
fnp = [-flip(fp) ; fp];
fnew = [-flip(fp) ; 0; fp];
snew = interp1(fnp,snp,fnew,'spline');
dc = real(snew(11));

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

clc;
clear all;
close all;
folder = 'X:\EE_514\Measured_Data\EE514_Components\0603_cap';
files = getAllFiles(folder,'*.s2p',1)

port_num = 2;
data_rate = 40; %data rate in Gbps
rise_per = 0.4; % rise time - fraction of UI
sample_per_UI = 32;

pulse_shape = 1; %1 is Gaussian; 2 is Rectangular with Butterworth filter; 3 is Rectangular with Gaussian filter;  
extrapolation_method = 2; %1 is constant extrapolation; 2 is zero padding;

tic
max_ind = 30;
for i=1:max_ind
    passivity(i)=0;
    causality(i)=0;
    reciprocity(i)=0;
    x(i)=i-1;
end
num_files = length(files);
file_name = strcat('statistics',num2str(data_rate),'.csv');
fid = fopen(file_name,'w');
fprintf(fid,'%s,','file name');
fprintf(fid,'%s,','Datarte');
fprintf(fid,'%s,','Max Freq');
fprintf(fid,'%s,','Recommended Freq');
fprintf(fid,'%s,','Pass mV');
fprintf(fid,'%s,','Rec mV');
fprintf(fid,'%s,','Caus mV');
fprintf(fid,'%s,','Pass %');
fprintf(fid,'%s,','Rec %');
fprintf(fid,'%s\n','Caus %');
  
for i=1:length(files)
    disp(strcat(int2str(i),'(',int2str(length(files)),')'));
    [freq,Sdata,npts] = fromtouchn(char(files(i)));
    max_freq=round(freq(end)/1e9);

    %Frequency domain checking
    [causality_metric_freq, reciprocity_metric_freq, passivity_metric_freq] = qualityCheckFrequencyDomain(Sdata,npts,2);

    %Time domain checking
    [causality_metric, reciprocity_metric, passivity_metric] = qualityCheck(freq,Sdata,port_num,data_rate,sample_per_UI,rise_per,pulse_shape,extrapolation_method,1);

    Frequency_Causality_Passivity_Recirocity = [ causality_metric_freq passivity_metric_freq reciprocity_metric_freq]
    Time_Causality_Passivity_Recirocity = [ causality_metric/2 passivity_metric/2 reciprocity_metric/2]

    p_ind = min(round(passivity_metric/2),max_ind-1);
    r_ind = min(round(reciprocity_metric/2),max_ind-1);
    c_ind = min(round(causality_metric/2),max_ind-1);
    passivity(p_ind + 1)=passivity(p_ind + 1)+1;
    causality(c_ind + 1)=causality(c_ind + 1)+1;
    reciprocity(r_ind + 1)=reciprocity(r_ind + 1)+1;
    
  
    estimation_array(i,1) = 'i';
    estimation_array(i,2) = data_rate;
    estimation_array(i,3) = max_freq;
    estimation_array(i,4) = 3*data_rate/2;
    estimation_array(i,5) = round(passivity_metric/2);
    estimation_array(i,6) = round(reciprocity_metric/2);
    estimation_array(i,7) =  round(causality_metric/2);

    estimation_array(i,8) = round(passivity_metric_freq,2);
    estimation_array(i,9) = round(reciprocity_metric_freq,2);
    estimation_array(i,10) =  round(causality_metric_freq);
    
    fprintf(fid,'%s,',getFileName(char(files(i))));
    for k=2:7
        fprintf(fid,'%d,',estimation_array(i,k));
    end
    fprintf(fid,'%.1f,',estimation_array(i,8));
    fprintf(fid,'%.1f,',estimation_array(i,9));
    fprintf(fid,'%d\n',estimation_array(i,10));
end
toc

fclose(fid);

figure(1)
plot(x,passivity/num_files*100, 'r','lineWidth', 4);
figure(2)
plot(x,reciprocity/num_files*100, 'r','lineWidth', 4);
figure(3)
plot(x,causality/num_files*100, 'r','lineWidth', 4);

figure(4)
plot(x,passivity, 'r','lineWidth', 4);
figure(5)
plot(x,reciprocity, 'r','lineWidth', 4);
figure(6)
plot(x,causality, 'r','lineWidth', 4);

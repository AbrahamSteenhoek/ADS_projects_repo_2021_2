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



%Read Data
[freq,Sdata,npts] = fromtouchn('Data\s5_m17_de_embed_jasonmatlab.s2p');

port_num = 2;
data_rate = 20; %data rate in Gbps
rise_per = 0.4; % rise time - fraction of UI
sample_per_UI = 32;

pulse_shape = 1; %1 is Gaussian; 2 is Rectangular with Butterworth filter; 3 is Rectangular with Gaussian filter;  
extrapolation_method = 2; %1 is constant extrapolation; 2 is zero padding;
%Frequency domain checking
[causality_metric_freq, reciprocity_metric_freq, passivity_metric_freq] = qualityCheckFrequencyDomain(Sdata,npts,port_num);

%Time domain checking
[causality_metric, reciprocity_metric, passivity_metric] = qualityCheck(freq,Sdata,port_num,data_rate,sample_per_UI,rise_per,pulse_shape,extrapolation_method,1);

Frequency_Causality_Passivity_Recirocity = [ causality_metric_freq passivity_metric_freq reciprocity_metric_freq]
Time_Causality_Passivity_Recirocity = [ causality_metric/2 passivity_metric/2 reciprocity_metric/2]

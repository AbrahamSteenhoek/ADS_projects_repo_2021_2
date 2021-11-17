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
[freq,Sdata,npts] = fromtouchn('Data\M2_and _main_TX2_RX2_screw_grounded.s4p');

%IBM_case1-selected
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\01_STRIP_8GHZ_MB1_4CORNER.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\2xThru.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\IBM_DUT_01_MB14C_reorder_result_ISD.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\IBM_DUT_01_MB14C_reorder_result_SFD.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\IBM_DUT_01_MB14C_reordered.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\IBM_case1\IBM_DUT_02_MB2LP_REORDERED.s4p');


%[freq,Sdata,npts] = fromtouchn('Data\Test2.s4p');

%Molex
%[freq,Sdata,npts] = fromtouchn('Data\Molex_Case1\THRU_DUT_w_Test_Fixture.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\Molex_Case1\2x_Calibraition_Trace1.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\Molex_Case1\Deembedded_DUT_CalibrationTrace1.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\Molex_Case1\2x_Calibration_Trace2.s4p');
%[freq,Sdata,npts] = fromtouchn('Data\Molex_Case1\Deembedded_DUT_CalibrationTrace2.s4p');


if(abs(Sdata(1,2))>abs(Sdata(1,3)) && (abs(Sdata(1,2))>abs(Sdata(1,4))))
    port_order = [1,3,2,4]; %(1,2) -> (3,4) 
elseif (abs(Sdata(1,3))>abs(Sdata(1,4)))
    port_order = [1,2,3,4]; %(1,3) -> (2,4) 
else
    port_order = [1,3,4,2]; %(1,4) -> (3,2) 
end

port_num = 4;
data_rate = 10; %data rate in Gbps
rise_per = 0.4; % rise time - fraction of UI
sample_per_UI = 32;

%get Diff and Common Modes
[SdataDiff,SdataComm] = getDifferentialCommonModes(Sdata,npts,port_order);

for i=1:npts
    diff_norm(i) = norm(squeeze(SdataDiff(:,:,i)));
end


%Frequency domain checking
[causality_metric_diff_freq, reciprocity_metric_diff_freq, passivity_metric_diff_freq] = qualityCheckFrequencyDomain(SdataDiff,npts,2);
[causality_metric_comm_freq, reciprocity_metric_comm_freq, passivity_metric_comm_freq] = qualityCheckFrequencyDomain(SdataComm,npts,2);


%Time domain checking
[causality_metric_diff, reciprocity_metric_diff, passivity_metric_diff] = qualityCheck(freq,SdataDiff,2,data_rate,sample_per_UI,rise_per,1);
[causality_metric_comm, reciprocity_metric_comm, passivity_metric_comm] = qualityCheck(freq,SdataComm,2,data_rate,sample_per_UI,rise_per,2);

%Print frequency domain results
Frequency_diff_causality_Passivity_Recirocity = [ causality_metric_diff_freq passivity_metric_diff_freq reciprocity_metric_diff_freq]
Frequency_comm_causality_Passivity_Recirocity = [ causality_metric_comm_freq passivity_metric_comm_freq reciprocity_metric_comm_freq]

%Print time domain results
Time_diff_causality_Passivity_Recirocity = [ causality_metric_diff/2 passivity_metric_diff/2 reciprocity_metric_diff/2]
Time_comm_causality_Passivity_Recirocity = [ causality_metric_comm/2 passivity_metric_comm/2 reciprocity_metric_comm/2]

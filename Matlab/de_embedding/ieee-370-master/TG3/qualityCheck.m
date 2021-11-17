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

function [causality_metric, reciprocity_metric, passivity_metric] = qualityCheck(freq,Sdata,port_num,data_rate,sample_per_UI,rise_per,pulse_shape,extrapolation_method,fig_num)

if (1.5*data_rate > freq(end)/1e9)
    warning('Maximum frequency is less then recomended frequency');
end

%extrapolate max freq
[freq,Sdata] = extrapolateMatrix(freq,Sdata,port_num,data_rate*1e9,sample_per_UI,extrapolation_method);


%extrapolate dc and interpolate with uniform step
[freq,original_interpolated] = dcExtrapolateMatrix(freq,Sdata,port_num);

% figure(15)
% plot(f1,db(squeeze(Sdata(2,1,:))), 'r','lineWidth', 4); hold on;
% plot(freq,db(squeeze(original_interpolated(2,1,:))),'--b','lineWidth', 4); hold on;



%get Causal Matrix
[causal_freq,causal_matrix, delay_matrix] = createCausalMatrix(freq,original_interpolated,port_num,data_rate,rise_per);

%get Reciprocal Matrix
reciprocal_matrix = createReciprocalMatrix(original_interpolated,port_num);

%get Passive Matrix
passive_matrix = createPassiveMatrix(original_interpolated,port_num);

%get Time Domain Matrices
[time_domain_causal, time_causal] = getTimeDomainMatrix(causal_matrix, causal_freq,port_num,data_rate,rise_per,pulse_shape);
[time_domain_reciprocal, time_reciprocal] = getTimeDomainMatrix(reciprocal_matrix, freq,port_num,data_rate,rise_per,pulse_shape);
[time_domain_passive, time_passive] = getTimeDomainMatrix(passive_matrix, freq,port_num,data_rate,rise_per,pulse_shape);
[time_domain_original, time_original] = getTimeDomainMatrix(original_interpolated, freq,port_num,data_rate,rise_per,pulse_shape);

%get Time Domain Difference causality
causality_time_domain_difference_mv = getTimeDomainDifferenceMV(time_domain_causal,time_domain_original,port_num,data_rate,time_original,true,delay_matrix)

%get Time Domain Difference reciprocity
reciprocity_time_domain_difference_mv = getTimeDomainDifferenceMV(time_domain_reciprocal,time_domain_original,port_num,data_rate,time_original,false,0);

%get Time Domain Difference passivity
passivity_time_domain_difference_mv = getTimeDomainDifferenceMV(time_domain_passive,time_domain_original,port_num,data_rate,time_original,false,0)

causality_metric = round(1000*norm(causality_time_domain_difference_mv),1);
reciprocity_metric = round(1000*norm(reciprocity_time_domain_difference_mv),1);
passivity_metric = round(1000*norm(passivity_time_domain_difference_mv),1);

 
figure(fig_num)
 plot(time_causal,squeeze(time_domain_causal(2,1,:))/2, 'r','lineWidth', 4); hold on;
 plot(time_original,squeeze(time_domain_original(2,1,:))/2,'--b','lineWidth', 4); hold on;

figure(fig_num +1)
 plot(time_causal,squeeze(time_domain_causal(1,2,:))/2, 'r','lineWidth', 4); hold on;
 plot(time_original,squeeze(time_domain_original(1,2,:))/2,'--b','lineWidth', 4); hold on;

 figure(fig_num+2)
plot(time_original,squeeze(time_domain_original(1,1,:))/2,'r','lineWidth', 4); hold on;

figure(fig_num+3)
plot(time_original,squeeze(time_domain_original(2,2,:))/2,'r','lineWidth', 4); hold on;
 
%figure(fig_num+1)
%plot(time_original,squeeze(time_domain_original(1,1,:))/2,'--b','lineWidth', 4); hold on;

% 
% 
% figure(10+fig_num)
% plot(time_original,squeeze(time_domain_original(1,1,:))/2,'--b','lineWidth', 4); hold on;
% 
% figure(100+fig_num)
% plot(squeeze(time_domain_original(1,1,:))/2,'--b','lineWidth', 4); hold on;

%figure(3)
%plot(time_reciprocal,squeeze(time_domain_reciprocal(2,1,:)), 'r','lineWidth', 4); hold on;
%plot(time_original,squeeze(time_domain_original(2,1,:)),'--b','lineWidth', 4); hold on;


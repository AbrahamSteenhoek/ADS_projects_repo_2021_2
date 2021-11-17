clc 
clear

%%Deembedding resistors
fixture_DUT_fixture_S_params_10ohms = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_res\0603_RES_10OHMS.S2P");
fixture_DUT_fixture_S_params_51ohms = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_res\0603_RES_51OHMS.S2P");
fixture_DUT_fixture_S_params_820ohms = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_res\0603_RES_820OHMS.S2P");

fixture_A_S_params = sparameters("fixtureA_R.s2p");
fixture_B_S_params = sparameters("fixtureB_R.s2p");

% fixture_DUT_fixture_S_params_10ohms.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_51ohms.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_820ohms.Frequencies = fixture_A_S_params.Frequencies;

DUT_ONLY_SPARAMS_10ohms = deembedsparams(fixture_DUT_fixture_S_params_10ohms, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_51ohms = deembedsparams(fixture_DUT_fixture_S_params_51ohms, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_820ohms = deembedsparams(fixture_DUT_fixture_S_params_820ohms, fixture_A_S_params, fixture_B_S_params);

rfwrite(DUT_ONLY_SPARAMS_10ohms,'RES_0603_10OHM.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_51ohms,'RES_0603_51OHM.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_820ohms,'RES_0603_820OHM.s2p','FrequencyUnit', 'Hz');

%%
%%Deembedding capacitors
fixture_DUT_fixture_S_params_1P5PF = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_cap\0603_CAP_1P5PF.S2P");
fixture_DUT_fixture_S_params_2P7PF = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_cap\0603_CAP_2P7PF.S2P");
fixture_DUT_fixture_S_params_5P6PF = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_cap\0603_CAP_5P6PF.S2P");

fixture_A_S_params = sparameters("fixtureA_C.s2p");
fixture_B_S_params = sparameters("fixtureB_C.s2p");

% fixture_DUT_fixture_S_params_1P5PF.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_2P7PF.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_5P6PF.Frequencies = fixture_A_S_params.Frequencies;

DUT_ONLY_SPARAMS_1P5PF = deembedsparams(fixture_DUT_fixture_S_params_1P5PF, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_2P7PF = deembedsparams(fixture_DUT_fixture_S_params_2P7PF, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_5P6PF = deembedsparams(fixture_DUT_fixture_S_params_5P6PF, fixture_A_S_params, fixture_B_S_params);

rfwrite(DUT_ONLY_SPARAMS_1P5PF,'CAP_0603_1P5PF.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_2P7PF,'CAP_0603_2P7PF.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_5P6PF,'CAP_0603_5P6PF.s2p','FrequencyUnit', 'Hz');

%%
%%Deembedding inductors
fixture_DUT_fixture_S_params_1P8NH = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_ind\0603_IND_1P8NH.S2P");
fixture_DUT_fixture_S_params_56NH = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_ind\0603_IND_56NH.S2P");
fixture_DUT_fixture_S_params_100NH = sparameters("X:\EE_514\Measured_Data\EE514_Components\0603_ind\0603_IND_100NH.S2P");

fixture_A_S_params = sparameters("fixtureA_L.s2p");
fixture_B_S_params = sparameters("fixtureB_L.s2p");

% fixture_DUT_fixture_S_params_1P8NH.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_56NH.Frequencies = fixture_A_S_params.Frequencies;
% fixture_DUT_fixture_S_params_100NH.Frequencies = fixture_A_S_params.Frequencies;

DUT_ONLY_SPARAMS_1P8NH = deembedsparams(fixture_DUT_fixture_S_params_1P8NH, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_56NH = deembedsparams(fixture_DUT_fixture_S_params_56NH, fixture_A_S_params, fixture_B_S_params);
DUT_ONLY_SPARAMS_100NH = deembedsparams(fixture_DUT_fixture_S_params_100NH, fixture_A_S_params, fixture_B_S_params);

rfwrite(DUT_ONLY_SPARAMS_1P8NH,'IND_0603_1P8NH.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_56NH,'IND_0603_56NH.s2p','FrequencyUnit', 'Hz');
rfwrite(DUT_ONLY_SPARAMS_100NH,'IND_0603_100NH.s2p','FrequencyUnit', 'Hz');

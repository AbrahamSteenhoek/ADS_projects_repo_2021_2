Instructions for S-Parameters Quality Check Tool

P370/TG3, Mikheil Tsiklauri

If a single file should be checked please use “testQualityCheck_2Port.m” file. 

Input parameters are:

1. Filename:  [freq,Sdata,npts] = fromtouchn('Data\s5_m1.s2p');
2. Port number: port_num = 2;
3. Datarate: data_rate = 10;
4. Rase/fall time percentage: rise_per = 0.4;
5. samples per UI: sample_per_UI = 32;

Estimation results will be shown in a command view:

    Frequency_Causality_Passivity_Recirocity =

      53.6899  99.9999  99.2942

    Time_Causality_Passivity_Recirocity =

       0.7000        0   0.7000


The first column represents Causality, Passivity and Reciprocity
preliminary estimation results in percentage.

The second column represents Causality, Passivity and Reciprocity
estimation results in mV.

If multiple files should be checked, please use CheckFolder2port.m
file. Input parameters are the same but instead of filename the folder
name with s-parameters files should be specified:

    folder = 'data\';

The results will be saved in csv file:

    file_name = strcat('statistics',num2str(data_rate),'.csv');

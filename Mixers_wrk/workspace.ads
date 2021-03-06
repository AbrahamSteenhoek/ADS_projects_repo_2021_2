<ADSWorkspace Revision="47" Version="100">
    <Workspace Name="">
        <Library Name="ads_sources" />
        <Library Name="ads_tlines" />
        <Library Name="ads_bondwires" />
        <Library Name="ads_behavioral" />
        <Library Name="ads_textfonts" />
        <Library Name="ads_common_cmps" />
        <Library Name="ads_designs" />
        <Library Name="ads_standard_layers" />
        <Library Name="adstechlib" />
        <Library Name="ads_schematic_layers" />
        <Library Name="empro_standard_layers" />
        <Library Name="ads_rflib" />
        <Library Name="ads_simulation" />
        <Library Name="ads_datacmps" />
        <Library Name="1xEV" />
        <Library Name="3GPPFDD" />
        <Library Name="3GPPFDD_10_99" />
        <Library Name="Antennas_and_Propagation" />
        <Library Name="CDMA" />
        <Library Name="cdma2000" />
        <Library Name="Circuit_Cosimulation" />
        <Library Name="CMMB" />
        <Library Name="Controllers" />
        <Library Name="DTMB" />
        <Library Name="DTV" />
        <Library Name="EDGE" />
        <Library Name="GSM" />
        <Library Name="HDL_Blocks" />
        <Library Name="HSDPA" />
        <Library Name="HSUPA" />
        <Library Name="Instruments" />
        <Library Name="Interactive_Controls_and_Displays" />
        <Library Name="LTE" />
        <Library Name="Numeric" />
        <Library Name="Obsolete" />
        <Library Name="Signal_Converters" />
        <Library Name="Simulation_Sequencing" />
        <Library Name="Sinks" />
        <Library Name="SystemVue_Cosimulation" />
        <Library Name="TDSCDMA" />
        <Library Name="Timed" />
        <Library Name="UMB" />
        <Library Name="UWB" />
        <Library Name="WLAN" />
        <Library Name="WLAN_11n" />
        <Library Name="WMAN" />
        <Library Name="WMAN_16e" />
        <Library Name="Mixers_lib" />
        <Dataset Name="ConvGain.ds" />
        <Dataset Name="ConvGain_wFilt.ds" />
        <Dataset Name="DblConvImag.ds" />
        <Dataset Name="DCTests.ds" />
        <Dataset Name="DoubleConvHB.ds" />
        <Dataset Name="Filter_Test.ds" />
        <Dataset Name="FirstIFFiltTest.ds" />
        <Dataset Name="HotColdNF.ds" />
        <Dataset Name="HotColdNF_wFilt.ds" />
        <Dataset Name="HotColdNF_wFiltvsIF.ds" />
        <Dataset Name="HotColdNF_wFiltvsLOpwr.ds" />
        <Dataset Name="IMDLOSwpHB.ds" />
        <Dataset Name="IMDRFSwpEnv.ds" />
        <Dataset Name="IMDRFSwpHB.ds" />
        <Dataset Name="MixerTOI.ds" />
        <Dataset Name="NoiseFloor.ds" />
        <Dataset Name="RFBandFiltTest.ds" />
        <Dataset Name="RFIFcompression.ds" />
        <Dataset Name="SweptRF.ds" />
        <Dataset Name="SweptRF_NF.ds" />
        <Dataset Name="TwoToneTest.ds" />
        <Log Name="conversion_results.log" />
        <Preferences Name="layout.prf" />
        <Preferences Name="schematic.prf" />
        <LibraryDefs Name="lib.defs" />
        <ConfigFile Name="de_sim.cfg" />
        <ConfigFile Name="hpeesofsim.cfg" />
        <Folder Name="01_ReadMe">
            <Cell Name="Mixers_lib:ReadMe" />
        </Folder>
        <Log Name="search_history.log" />
        <Folder Name="02_Gilbert_cell_mixer">
            <Cell Name="Mixers_lib:GilCellMix" />
        </Folder>
        <Folder Name="03_Filters">
            <Cell Name="Mixers_lib:Filter2" />
            <Cell Name="Mixers_lib:FirstIFFiltTest" />
            <Data_Display Name="FirstIFFiltTest.dds" />
            <Cell Name="Mixers_lib:RFBandFiltTest" />
            <Data_Display Name="RFBandFiltTest.dds" />
        </Folder>
        <Folder Name="04_Simulations_DC_ConvGain_NF">
            <Cell Name="Mixers_lib:ConvGain" />
            <Data_Display Name="ConvGain.dds" />
            <Cell Name="Mixers_lib:DCTests" />
            <Data_Display Name="DCTests.dds" />
            <Cell Name="Mixers_lib:HotColdNF" />
            <Data_Display Name="HotColdNF.dds" />
            <Cell Name="Mixers_lib:HotColdNF_wFilt" />
            <Data_Display Name="HotColdNF_wFilt.dds" />
            <Cell Name="Mixers_lib:HotColdNF_wFiltvsIF" />
            <Data_Display Name="HotColdNF_wFiltvsIF.dds" />
            <Cell Name="Mixers_lib:HotColdNF_wFiltvsLOpwr" />
            <Data_Display Name="HotColdNF_wFiltvsLOpwr.dds" />
            <Data_Display Name="ConvGain_wFilt.dds" />
            <Cell Name="Mixers_lib:ConvGain_wFilt" />
            <Cell Name="Mixers_lib:SweptRF_NF" />
            <Data_Display Name="SweptRF_NF.dds" />
            <Cell Name="Mixers_lib:SweptRF" />
            <Data_Display Name="SweptRF.dds" />
        </Folder>
        <Folder Name="05_Simulations_ImageRejection_IMD_RFIFcompression">
            <Cell Name="Mixers_lib:DblConvImag" />
            <Data_Display Name="DblConvImag.dds" />
            <Cell Name="Mixers_lib:DoubleConvHB" />
            <Data_Display Name="DoubleConvHB.dds" />
            <Data_Display Name="IMDLOSwpHB.dds" />
            <Cell Name="Mixers_lib:IMDLOSwpHB" />
            <Cell Name="Mixers_lib:IMDRFSwpEnv" />
            <Data_Display Name="IMDRFSwpEnv.dds" />
            <Cell Name="Mixers_lib:IMDRFSwpHB" />
            <Data_Display Name="IMDRFSwpHB.dds" />
            <Cell Name="Mixers_lib:RFIFcompression" />
            <Data_Display Name="RFIFcompression.dds" />
        </Folder>
        <Folder Name="06_Simulations_TOI_NoiseFloor_TwoToneTest">
            <Cell Name="Mixers_lib:MixerTOI" />
            <Data_Display Name="MixerTOI.dds" />
            <Cell Name="Mixers_lib:NoiseFloor" />
            <Data_Display Name="NoiseFloor.dds" />
            <Cell Name="Mixers_lib:TwoToneTest" />
            <Data_Display Name="TwoToneTest.dds" />
        </Folder>
    </Workspace>
</ADSWorkspace>

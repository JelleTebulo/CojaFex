MODULE TS591_SPECIAL
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Module Name: TS590_MAIN
    !Version:     1.0
    !Description:  
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    !
    PERS num nLsrMon{4}:=[378,384.2,387.7,376];
    !
    PERS string sTCP_Trans{2}:=["0","0"];
    PERS string sLateralSpeed{2}:=["1","0.017"];
    PERS string sTemperingSpeed{2}:=["25","0.417"];
    PERS string sIdleSpeed{2}:=["1000","16.667"];
    PERS string sLsrMon{4}:=["378.0","384.2","387.7","376.0"];
    !
    FUNC num nUnsignedToSigned_INT_TS000_50(num UnsignedValue)
        !***********************************************************
        ! Function: nUnsignedToSigned_INT_TS000_50
        ! Machine: -
        ! Description:
        !
        ! Author: RLZ
        ! Date: 23-04-2020
        !
        !***********************************************************
        VAR num SignedValue;
        IF (UnsignedValue)>32767 THEN
            SignedValue:=(UnsignedValue-65536);
        ELSE
            SignedValue:=UnsignedValue;
        ENDIF
        !
        RETURN Round(SignedValue\Dec:=0);
    ENDFUNC

    FUNC num nSignedToUnSigned_INT_TS000_51(num SignedValue)
        !***********************************************************
        ! Function: nSignedToUnSigned_INT_TS000_51
        ! Machine: -
        ! Description:
        !
        ! Author: RLZ
        ! Version: 1.0
        !
        !***********************************************************
        VAR num UnSignedValue;
        IF (SignedValue)<0 THEN
            UnSignedValue:=(SignedValue+65536);
        ELSE
            UnSignedValue:=SignedValue;
        ENDIF
        RETURN Round(UnSignedValue\Dec:=0);
    ENDFUNC

    PROC rReceiveSpeedData()
        VAR num TempNum{3};
        !
        TempNum{1}:=nUnsignedToSigned_INT_TS000_50(GInput(giTCPSpeed_Lateral));
        TempNum{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giTCPSpeed_Tempering));
        TempNum{3}:=nUnsignedToSigned_INT_TS000_50(GInput(giTCPSpeed_Idle));
        !
        IF Tempnum{1}<1 Tempnum{1}:=1;
        IF Tempnum{1}>200 Tempnum{1}:=200;
        !
        IF Tempnum{2}<1 Tempnum{2}:=1;
        IF Tempnum{2}>200 Tempnum{2}:=200;
        !
        IF Tempnum{3}<100 Tempnum{3}:=100;
        IF Tempnum{3}>1000 Tempnum{3}:=1000;
        !
        nLateralSpeed{1}:=Tempnum{1};
        nLateralSpeed{2}:=Tempnum{1}/60;
        nTemperingSpeed{1}:=Tempnum{2};
        nTemperingSpeed{2}:=Tempnum{2}/60;
        nIdleSpeed{1}:=Tempnum{3};
        nIdleSpeed{2}:=Tempnum{3}/60;
        !
        IF sLateralSpeed{1}<>NumToStr(nLateralSpeed{1},0) sLateralSpeed{1}:=NumToStr(nLateralSpeed{1},0);
        IF sLateralSpeed{2}<>NumToStr(nLateralSpeed{2},3) sLateralSpeed{2}:=NumToStr(nLateralSpeed{2},3);
        !
        IF sTemperingSpeed{1}<>NumToStr(nTemperingSpeed{1},0) sTemperingSpeed{1}:=NumToStr(nTemperingSpeed{1},0);
        IF sTemperingSpeed{2}<>NumToStr(nTemperingSpeed{2},3) sTemperingSpeed{2}:=NumToStr(nTemperingSpeed{2},3);
        !
        IF sIdleSpeed{1}<>NumToStr(nIdleSpeed{1},0) sIdleSpeed{1}:=NumToStr(nIdleSpeed{1},0);
        IF sIdleSpeed{2}<>NumToStr(nIdleSpeed{2},3) sIdleSpeed{2}:=NumToStr(nIdleSpeed{2},3);
        !
    ENDPROC

    PROC rReceiveTCPData()
        VAR num TempNum{2};
        !
        TempNum{1}:=nUnsignedToSigned_INT_TS000_50(GInput(giTCPOffsetX));
        TempNum{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giTCPOffsetZ));
        !
        IF Tempnum{1}<-25 Tempnum{1}:=-25;
        IF Tempnum{1}>25 Tempnum{1}:=25;
        !
        IF Tempnum{2}<-25 Tempnum{2}:=-25;
        IF Tempnum{2}>25 Tempnum{2}:=25;
        !
        nTCP_Trans.x:=Tempnum{1};
        nTCP_Trans.z:=Tempnum{2};
        !
        IF sTCP_Trans{1}<>NumToStr(nTCP_Trans.x,0) sTCP_Trans{1}:=NumToStr(nTCP_Trans.x,0);
        IF sTCP_Trans{2}<>NumToStr(nTCP_Trans.z,0) sTCP_Trans{2}:=NumToStr(nTCP_Trans.z,0);
        !
    ENDPROC

    PROC rInit_Data_Special()
        !***********************************************************
        ! Procedure: rInit_Data_Special
        ! Machine: - / -
        ! Description:
        ! - Initialize Destrapper variables
        !
        ! Author: rob_lentz
        ! Version: 1.0 
        ! Date: 13-07-2017
        !***********************************************************
        !
    ENDPROC

    PROC rReceiveLsrData()
        VAR num TempNum{4};
        !
        TempNum{1}:=nUnsignedToSigned_INT_TS000_50(GInput(giLsrDist_1))/10;
        TempNum{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giLsrDist_2))/10;
        TempNum{3}:=nUnsignedToSigned_INT_TS000_50(GInput(giLsrDist_3))/10;
        TempNum{4}:=nUnsignedToSigned_INT_TS000_50(GInput(giLsrDist_4))/10;
        !
        nLsrMon{1}:=TempNum{1};
        nLsrMon{2}:=TempNum{2};
        nLsrMon{3}:=TempNum{3};
        nLsrMon{4}:=TempNum{4};
        !
        IF sLsrMon{1}<>NumToStr(nLsrMon{1},1) sLsrMon{1}:=NumToStr(nLsrMon{1},1);
        IF sLsrMon{2}<>NumToStr(nLsrMon{2},1) sLsrMon{2}:=NumToStr(nLsrMon{2},1);
        IF sLsrMon{3}<>NumToStr(nLsrMon{3},1) sLsrMon{3}:=NumToStr(nLsrMon{3},1);
        IF sLsrMon{4}<>NumToStr(nLsrMon{4},1) sLsrMon{4}:=NumToStr(nLsrMon{4},1);
    ENDPROC

    PROC rWrite_Data_Special()
        !***********************************************************
        ! Procedure: rWrite_Data_Special
        ! Machine: - / -
        ! Description:
        ! - Write measured data to local varibles to use on HMI
        !***********************************************************
        !
        rReceiveLsrData;
        rReceiveSpeedData;
        rReceiveTCPData;
        !
    ENDPROC

    PROC rWrite_Options_Special()
        !***********************************************************
        ! Procedure: rWrite_Options_Special
        ! Machine: - / -
        ! Description:
        ! - Write the destrapper options (from customer) to local booleans
        !   to use during the cycle 
        !***********************************************************
        !
        IF RobOS()=TRUE Setdo doCP2_Selected_SIM,diCP2_Selected;
        IF RobOS()=TRUE Setdo doHMI_Select_CP2,diCP2_Selected;
        !
        IF RobOS()=FALSE Setdo doCP2_Selected_SIM,DOutput(doHMI_Select_CP2);
        !
    ENDPROC
ENDMODULE
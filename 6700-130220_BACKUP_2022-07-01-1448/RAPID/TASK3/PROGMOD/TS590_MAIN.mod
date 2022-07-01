MODULE TS590_MAIN
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Module Name: TS590_MAIN
    !Version:     1.0
    !Description:  
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    !
    !****** MODIFY TO CUSTOMER SPECS ****** 
    CONST string sStandardLanguage:="en";
    !****** MODIFY TO CUSTOMER SPECS ******
    !
    PERS num nRobotPosition:=16;
    ! Used for writing the robot position to the plc  
    PERS string sHMI_EventLine{2};
    PERS string sHMI_Control;
    ! Used to display the control location on the HMI
    CONST string sPNInternalDevice:="PNCP1";
    !
    TASK PERS tooldata tTool:=[TRUE,[[1.2727,10,813.858],[5.65694372839885E-17,0.923879532511287,2.34318281388422E-17,-0.38268343236509]],[128.5,[-12.8,2.9,350],[1,0,0,0],35.394,36.232,9.127]];

    PROC main()
        !***********************************************************
        ! Procedure: main 
        ! Machine: BL8
        ! Description:
        ! - Main routine task 2
        !***********************************************************
        !
        Waittime 0.2;
        !
        bMHS_RobotToManualMode:=TRUE;
        !
        rInit_Data_Special;
        !
        rInit_Data_TS050_9;
        !
        WHILE TRUE DO
            !
            Waittime 0.5;
            !
            rMachineIndependent_TS050_1(sStandardLanguage);
            !
            rWrite_Robot_Events;
            !
            rStart_Conditions_MD;
            !
            rWrite_Data_Special;
            !
            rRequestStop;
            !
            rCalcTCP;
            !
            IF (DOutPut(doInCycle)=0) THEN
                Set doDataReceiving;
                !
                rWrite_Options_Special;
                !
                Reset doDataReceiving;
            ENDIF
            !
            rCheckRobotToManualMode;
            !
        ENDWHILE
    ERROR
        IF (ERRNO=ERR_NORUNUNIT) THEN
            SKIPWARN;
            TRYNEXT;
        ENDIF
    ENDPROC

    PROC rCalcTCP()
        VAR pos tcp;
        VAR num tcpspeed;
        VAR jointtarget joints;
        !
        joints:=CJointT(\TaskName:="T_ROB1");
        !
        Setgo goROB1JOINT1,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_1*100));
        Setgo goROB1JOINT2,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_2*100));
        Setgo goROB1JOINT3,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_3*100));
        Setgo goROB1JOINT4,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_4*100));
        Setgo goROB1JOINT5,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_5*100));
        Setgo goROB1JOINT6,nSignedToUnSigned_INT_TS000_51(Round(joints.robax.rax_6*100));
        Setgo goTRACKJOINT1,nSignedToUnSigned_INT_TS000_51(Round(joints.extax.eax_a));
        !
        tcp:=CPos(\Tool:=tTool\WObj:=wobj0);
        !
        SetGO goTCPX,nSignedToUnSigned_INT_TS000_51(tcp.x);
        SetGO goTCPY,nSignedToUnSigned_INT_TS000_51(tcp.y);
        SetGO goTCPZ,nSignedToUnSigned_INT_TS000_51(tcp.z);
        !
        tcpspeed:=AOutput(AOSpeed);
        tcpspeed:=tcpspeed*1000*60;
        SetGO goTCPSPEED,nSignedToUnSigned_INT_TS000_51(tcpspeed);
        !
    ENDPROC

    PROC rRequestStop()
        !***********************************************************
        ! Procedure: rRequestStop        
        ! Machine:  
        ! Description:
        ! -Check if there is an request for stop current cycle
        !***********************************************************
        VAR bool StopAllowed;
        !
        StopAllowed:=diInCycle=1 AND (doRobotInTCPCenteringCycle=1 OR doRobotIdentificationCycle=1 OR doRobotTemperingCycle=1 OR doRobotFreerunCycleBWD=1 OR doRobotFreerunCycleFWD=1);
        IF ((StopAllowed=FALSE AND diRequestStopReceived=1) OR bMHS_RobotToManualMode) Reset doRequestStopReceived;
        IF ((diRequestStopCycle=1 OR doHMI_RequestStop=1)) Set doRequestStopReceived;
        Reset doHMI_RequestStop;
    ENDPROC

    PROC rCheckRobotToManualMode()
        !
        IF OpMode()=OP_MAN_PROG OR siAutoOn=0 bMHS_RobotToManualMode:=TRUE;
        !    
    ENDPROC

    PROC rWrite_Robot_Events()
        !***********************************************************
        ! Procedure: rWrite_Robot_Events        
        ! Machine: Destrapper 
        ! Description:
        ! - Write robot positions to a integer for the plc
        !*********************************************************** 
        VAR num CurrentPos;
        !
        CurrentPos:=49;
        !Robot Undefined Posotion
        !
        IF ((DOutPut(doInCycle)=0) AND (DOutPut(doRobotInHomePosition)=1)) CurrentPos:=00;
        IF (DOutPut(doRobotMovingToHomePosition)=1) CurrentPos:=06;
        IF (DOutPut(doRobotInMaintenancePos)=1) CurrentPos:=07;
        IF (DOutPut(doMovingToMaintenancePos)=1) CurrentPos:=08;
        !
        IF (DOutPut(doRobotMoveToInitPosCP1)=1) CurrentPos:=11;
        IF (DOutPut(doRobotInInitposCP1)=1) AND nMovementcounter<5 CurrentPos:=12;
        IF (DOutPut(doRobotInInitposCP1)=1) AND nMovementcounter>=5 CurrentPos:=13;

        IF (DOutPut(doRobotMoveToInitPosCP2)=1) CurrentPos:=14;
        IF (DOutPut(doRobotInInitposCP2)=1) AND nMovementcounter<5 CurrentPos:=15;
        IF (DOutPut(doRobotInInitposCP2)=1) AND nMovementcounter>=5 CurrentPos:=16;
        !
        IF (DOutPut(doRobotInTCPCenteringCycle)=1) CurrentPos:=21;
        IF (DOutPut(doRobotIdentificationCycle)=1) CurrentPos:=24;
        IF (DOutPut(doRobotTemperingCycle)=1) CurrentPos:=27;
        IF (DOutPut(doRobotFreerunCycleFWD)=1) CurrentPos:=30;
        IF (DOutPut(doRobotFreerunCycleBWD)=1) CurrentPos:=33;
        !
        IF (DOutPut(doRobotInCalibrationPos)=1) CurrentPos:=46;
        IF (DOutPut(doMovingToCalibrationPos)=1) CurrentPos:=47;
        IF (DOutPut(doSC1_CBC_Active)=1) CurrentPos:=48;
        !
        nRobotPosition:=CurrentPos;
        !
        SetGO goRobotPosition,nRobotPosition;
        !*****MACHINE POSITIONS HMI**** 
        sHMI_EventLine{1}:=sHMI_Events{nRobotPosition+1};
        !
        !
    ERROR
        IF (ERRNO=ERR_NORUNUNIT) THEN
            SKIPWARN;
            TRYNEXT;
        ENDIF
    ENDPROC

    PROC rStart_Conditions_MD()
        !***********************************************************
        ! Procedure: rStart_Conditions_MD        
        ! Machine: -/-
        ! Description:
        ! - Display start conditions
        !***********************************************************
        !
        IF (diLocal_SIM=0) THEN
            sHMI_Control:=sHMI_START_CONDITIONS{21};
        ELSE
            sHMI_Control:=sHMI_START_CONDITIONS{23};
        ENDIF
        !
    ERROR
        IF (ERRNO=ERR_NORUNUNIT) THEN
            SKIPWARN;
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
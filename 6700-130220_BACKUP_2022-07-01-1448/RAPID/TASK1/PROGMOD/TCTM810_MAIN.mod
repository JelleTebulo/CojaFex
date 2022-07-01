MODULE TCTM810_MAIN
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Version:     1.0
    !Description:  
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    PERS wztemporary WZ_Robot_In_Home_Position;
    PERS wztemporary WZ_Robot_In_Home_Box;
    PERS wztemporary WZ_Robot_WorldZone_Active;

    !
    PROC Main()
        !****************************************************************
        !* PROC: Main 	  
        !* Description:
        !* - Main procedure 
        !* - Robotprogram starts here 
        !****************************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC Main(), Project: "+sProject;
        !
        rInit;
        !
        WHILE TRUE DO
            !
            WaitTime 0.2;
            !
            Velset 100,250;
            !
            IF (sdoRobotInHomeBox=1 AND sdoRobotInHomePos=1) THEN
                IF (((Doutput(doSC1_CBC_OK)=0) OR (DOutput(doSC1_CBC_Request)=1) OR (DOutput(doSC1_CBC_Prewarning)=1)) AND sdoSynchronized=1 AND bSC1_CBC_Allowed=TRUE AND RobOS()) THEN
                    !                
                    rRunCyclicBrakeCheck;
                    !
                ELSEIF (diRobotToInitPos=1 OR (RobOS()=FALSE AND Doutput(doHMI_RobotToInitPos)=1)) THEN
                    !
                    rMoveToClamp;
                    rResetCycleBoolean;
                    !  
                ENDIF
            ELSE
                !
                bTCPCenteringAllowed:=(doRobotInInitposCP1=1 OR doRobotInInitposCP2=1) AND bTCPCentered=FALSE;
                bFreerunBWD_Allowed:=(bTCPCentered AND bPipeIdentified AND nMovementCounter>1);
                bFreerunFWD_Allowed:=(bTCPCentered AND bPipeIdentified AND nMovementCounter<nLastIdentifiedTarget);
                !
                        bTCPCenteringAllowed:=TRUE;
        bTCPCentered:=TRUE;
        bPipeIdentified:=TRUE;
        bFreerunFWD_Allowed:=TRUE;
        bFreerunBWD_Allowed:=TRUE;

                !
                IF (doRobotInInitposCP1=1 OR doRobotInInitposCP2=1) THEN
                    !
                    IF (bTCPCenteringAllowed=TRUE AND (diTCPCentering_Cycle=1 OR (Doutput(doHMI_TCPCentering_Cycle)=1))) THEN
                        !
                        rTcpCenteringCycle;
                        bTCPCentered:=TRUE;
                        !
                    ELSEIF (bTCPCentered=TRUE AND (diIdentification_Cycle=1 OR (Doutput(doHMI_Identification_Cycle)=1))) THEN
                        !
                        rIdentificationCycle;
                        bPipeIdentified:=TRUE;
                        !        
                    ELSEIF (bFreerunBWD_Allowed=TRUE AND (diTempering_Cycle=1 OR (Doutput(doHMI_Tempering_Cycle)=1))) THEN
                        !
                        rTemperingCycle;
                        !
                    ELSEIF (bFreerunFWD_Allowed=TRUE AND (diFreeRun_FWD=1 OR (Doutput(doHMI_FreeRun_FWD)=1))) THEN
                        !
                        rFreerun_FWD;
                        !
                    ELSEIF (bFreerunBWD_Allowed=TRUE AND (diFreeRun_BWD=1 OR (Doutput(doHMI_FreeRun_BWD)=1))) THEN
                        !
                        rFreerun_BWD;
                        !
                    ELSEIF (diHome=1 OR (Doutput(doHMI_Home)=1)) THEN
                        !
                        rResetCycleBoolean;
                        rMoveToHome;
                        !
                    ENDIF
                ENDIF
            ENDIF
        ENDWHILE
        ! 
    ERROR
        IF (ERRNO=ERR_UISHOW_FATAL OR ERRNO=ERR_TP_NO_CLIENT OR ERRNO=ERR_UISHOW_FULL OR ERRNO=ERR_NORUNUNIT) THEN
            SKIPWARN;
            TRYNEXT;
        ELSE
            EXIT;
        ENDIF
    ENDPROC

    PROC rResetCycleBoolean()
        !****************************************************************
        !* PROC: rResetCycleBoolean 	  
        !* Description:
        !* - Reset booleans for cycles
        !****************************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC rResetCycleBoolean()";
        !
        bTCPCenteringAllowed:=FALSE;
        bTCPCentered:=FALSE;
        bPipeIdentified:=FALSE;
        bFreerunFWD_Allowed:=FALSE;
        bFreerunBWD_Allowed:=FALSE;
        nLastIdentifiedTarget:=0;
        nMovementCounter:=0;
    ENDPROC

    PROC rInit()
        !***********************************************************
        ! Procedure: rInit
        ! Machine: Destrapper
        ! Description:
        ! - initilize the program, restart worldzone's reset commands 
        !   robot to home, reset all errors and start the HMI.
        !***********************************************************
        !
        VAR num mystatus:=0;
        !
        rLog_Event_Message_TS000_3\Message:="PROC rInit()";
        !
        IF diWZ_Active=0 rWorldZones;
        !
        rMoveHomeSafe;
        !
        IF bRobotContinueFromCurrentPos=FALSE THEN
            !
            rResetCommands;
            !
            rResetCycleBoolean;
            !
            rResetposition_in_home_box;
            !
            rResetCommand_After_MoveHomeSafe;
            !
            rResetposition_in_Coil_box;
            !
        ENDIF
        !
        rResetErrors;
        !
        MotionSup\On\TuneValue:=200;
        !
        UIShow sScreenMaker,"ABB.Robotics.SDK.Views.MainScreen"\Status:=mystatus\NoCloseBtn;
        !
        Velset 100,MAX_SPEED;
        !
        bSC1_CBC_Allowed:=TRUE;
        !
        CornerPathWarning(FALSE);
        !
        rLog_Clear_Log_TS000_5;
    ERROR
        !
        IF (ERRNO=ERR_UISHOW_FATAL OR ERRNO=ERR_TP_NO_CLIENT OR ERRNO=ERR_UISHOW_FULL) THEN
            SKIPWARN;
            TRYNEXT;
        ENDIF
    ENDPROC

    PROC rWorldZones()
        !***********************************************************
        ! Procedure: rWorldZones
        ! Description:
        ! - zones, used to determine where the robot is in the cell.
        !***********************************************************
        VAR shapedata SH_Robot_In_Home_Box;
        VAR shapedata SH_Robot_In_Home_Position;
        VAR shapedata SH_Robot_WorldZone_Active;

        VAR bool Time_Out;
        !
        rLog_Event_Message_TS000_3\Message:="PROC rWorldZones()";
        !
        !WZSphDef\Inside,SH_Robot_In_Home_Position,[pRobHomePosition.trans.x,pRobHomePosition.trans.y,pRobHomePosition.trans.z],50;
        !WZDOSet\Temp,WZ_Robot_In_Home_Position\Inside,SH_Robot_In_Home_Position,doWZ_RobotInHome,1;
        !
        WZBoxDef\Inside,SH_Robot_In_Home_Box,WZ_Robot_Home_Box_MIN,WZ_Robot_Home_Box_MAX;
        WZDOSet\Temp,WZ_Robot_In_Home_Box\Inside,SH_Robot_In_Home_Box,doWZ_Robot_In_Home_Box,0;
        !
        WZBoxDef\Inside,SH_Robot_WorldZone_Active,WZ_Robot_Inside_Reach_MIN,WZ_Robot_Inside_Reach_MAX;
        WZDOSet\Temp,WZ_Robot_WorldZone_Active\Inside,SH_Robot_WorldZone_Active,doWZ_Active,1;
        !
        WaitUntil diWZ_Active=1\MaxTime:=10\TimeFlag:=Time_Out;
        !
    ENDPROC

    PROC rResetCommands()
        !***********************************************************
        ! Procedure: rResetCommands
        ! Machine: Destrapper
        ! Description:
        ! - reset all command's which should be reset at the start
        !   of the robot program. For example the paint nozzle.
        !***********************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC rResetCommands()";
        !
        !
    ENDPROC

    PROC rResetposition_in_home_box()
        !****************************************************************
        !*  PROC		: rResetposition_in_home_box 	 
        !*  Description:
        !*	- Reset all possible positions inside the home box
        !****************************************************************
        rLog_Event_Message_TS000_3\Message:="PROC rResetposition_in_home_box()";
        !
        Reset doRobotInMaintenancePos;
        Reset doMovingToMaintenancePos;
        Reset doMovingToCalibrationPos;
        Reset doRobotInCalibrationPos;
        !
    ENDPROC

    PROC rResetCommand_After_MoveHomeSafe()
        !****************************************************************
        !* PROC: rResetCommand_After_MoveHomeSafe 	 
        !* Description:
        !* - Reset these commands after move home safe
        !****************************************************************
        rLog_Event_Message_TS000_3\Message:="PROC rResetCommand_After_MoveHomeSafe()";
        !
    ENDPROC

    PROC rResetposition_in_Coil_box()
        !****************************************************************
        !*  PROC		: rResetposition_in_Coil_box 	 
        !*  Description:
        !*	- Reset all possible positions inside the coil box
        !****************************************************************
        rLog_Event_Message_TS000_3\Message:="PROC rResetposition_in_Coil_box()";
        !
        Reset doInCycle;
        !
        Reset doRobotInInitposCP1;
        Reset doRobotInInitposCP2;
        Reset doRobotMoveToInitPosCP1;
        Reset doRobotMoveToInitPosCP2;
        Reset doRobotInTCPCenteringCycle;
        Reset doRobotIdentificationCycle;
        Reset doRobotTemperingCycle;
        Reset doRobotFreerunCycleFWD;
        Reset doRobotFreerunCycleBWD;
        !
        Reset doRobotMovingToHomePosition;
        !
    ENDPROC

    PROC rResetErrors()
        !****************************************************************
        !*  PROC		: rResetErrors 	 
        !*  Description:
        !*	- Reset all Errors  
        !****************************************************************
        rLog_Event_Message_TS000_3\Message:="PROC rResetErrors()";
        !
        Reset doRobotNotAbleToGoHome;
    ENDPROC

    PROC rClear_HMI()
        !***********************************************************
        ! Procedure: rClear_HMI 
        ! Description:
        ! - reset values on the HMI at the start of a new cycle
        !***********************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC rClear_HMI()";
        !
        !
    ERROR
        IF ERRNO=ERR_GO_LIM THEN
            SkipWarn;
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
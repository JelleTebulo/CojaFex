MODULE TCTM839_CYCLE_INIT
    !***********************************************************
    ! Project: P21-100538
    ! Description:
    !   Routines/functions can be modified for the customer
    !
    ! Author: RLZ
    ! Version: 1.0
    ! Date: 05-2022
    !***********************************************************    
    PERS jointtarget jToolRotate{2}:=[[[0,-30,30,180,0,0],[6236.68,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,-30,30,180,-60,0],[6236.68,9E+09,9E+09,9E+09,9E+09,9E+09]]];

    PERS robtarget pMoveToClamp{10}:=[[[0,500,0],[1,0,0,0],[0,1,0,1],[6236.68,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.996618,0,0.0821716,0],[0,1,0,1],[6236.68,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]],
                                        [[0,0,0],[0.989781,5.48241E-07,-0.142594,-6.64395E-07],[0,0,0,0],[2253.54,9E+09,9E+09,9E+09,9E+09,9E+09]]];

    !
    PERS bool bTCPCentered:=TRUE;
    PERS bool bPipeIdentified:=TRUE;
    !
    PERS bool bFreerunFWD_Allowed:=TRUE;
    PERS bool bFreerunBWD_Allowed:=TRUE;
    PERS bool bTCPCenteringAllowed:=TRUE;
    !
    PERS wobjdata wTempClamp:=[FALSE,TRUE,"",[[8065,1500,-518],[0.707107,0,0,0.707107]],[[1185.66,828.317,760],[0.693997,0.135528,-0.693997,0.135528]]];
    PERS robtarget pInitClamp11:=[[-33.30,470.72,67.01],[0.999675,-0.0141032,0.0129427,-0.016822],[-1,0,-2,0],[3956.63,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PERS robtarget pInitClamp12:=[[-33.29,470.71,66.99],[0.999675,-0.0141031,0.0129379,-0.0168198],[-1,0,-2,0],[3956.63,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR num Initial_angle;

    PERS num nClampPozX{2}:=[1460,1522];
    PERS num nClampAngle{2}:=[0,22.1];


    PROC rMoveToClamp()
        !***********************************************************
        ! Procedure: rMoveToClamp
        ! Description:
        ! - Robot moves save to the clamp, and positions the tool acc 
        !   to the received position Translation X, and Rotation Z
        !***********************************************************
        !
        VAR robtarget Dummy;
        VAR robtarget pActualPos;
        !
        rLog_Time_Date_TS000_1;
        !
        bAbort_Cycle:=FALSE;
        !
        Velset 100,MAX_SPEED;
        !
        rResetCommands;
        !
        rClear_HMI;
        !

        Set doInCycle;
        Waituntil Doutput(doDataReceiving)=0;
        rReceived_client_info;
        !
        IF bAbort_Cycle=FALSE THEN
            IF (diCP2_Selected_SIM=1) THEN
                !
                Set doRobotMoveToInitPosCP2;
                rLog_Event_Message_TS000_3\Message:="PROC rMoveToClamp(2)";
                !
                rWobjClamp2;
                !
                jRobHomePos.extax.eax_a:=wClamp2.uframe.trans.x-wClamp2.oframe.trans.y-1000;
                MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wobj0;
                !
                jToolRotate{1}:=jRobHomePos;
                jToolRotate{1}.robax.rax_4:=180;
                MoveAbsJ jToolRotate{1}\NoEOffs,vHomeBoxSpeed,z5,tInductionRing\WObj:=wClamp2;
                !
                jToolRotate{2}:=jRobHomePos;
                jToolRotate{2}.robax.rax_4:=180;
                jToolRotate{2}.robax.rax_5:=-60;
                MoveAbsJ jToolRotate{2}\NoEOffs,vHomeBoxSpeed,z5,tInductionRing\WObj:=wClamp2;
                !
                pMoveToClamp{1}.trans:=[0,500,0];
                pMoveToClamp{1}.robconf:=[0,1,0,1];
                pMoveToClamp{1}.rot:=[1,0,0,0];
                pMoveToClamp{1}.extax.eax_a:=jToolRotate{2}.extax.eax_a;
                MoveL pMoveToClamp{1},vSlowSpeed,fine,tInductionRing\WObj:=wClamp_NonRotate;
                !
                pActualPos:=CRobT(\Tool:=tInductionRing\WObj:=wobj0);
                Initial_angle:=-1*((pActualPos.trans.y-3000)/(1500/45));
                ! Initial angle is -45 at y=1500mm and 0 at y=3000mm
                IF pActualPos.trans.y<1500 Initial_angle:=45;
                IF pActualPos.trans.y>3000 Initial_angle:=0;
                pMoveToClamp{2}:=pMoveToClamp{1};
                pMoveToClamp{2}.rot:=pMoveToClamp{2}.rot*OrientZYX(0,Initial_angle,0);
                pMoveToClamp{2}.trans.y:=0;
                MoveL reltool(pMoveToClamp{2},0,500,0),vSlowSpeed,fine,tInductionRing\WObj:=wClamp2;
                MoveL reltool(pMoveToClamp{2},0,0,0),vSlowSpeed,fine,tInductionRing\WObj:=wClamp2;
                !
                wTempClamp:=wClamp2;
                TargetNew{1}:=CRobT(\Tool:=tInductionRing\WObj:=wTempClamp);
                MoveL TargetNew{1},v50,fine,tInductionRing\WObj:=wTempClamp;
                !
                Rtcpy_Offset:=EulerZYX(\Y,TargetNew{1}.rot);
                !
                Reset doRobotMoveToInitPosCP2;
                Set doRobotInInitposCP2;
                !
            ELSE
                !
                Set doRobotMoveToInitPosCP1;
                rLog_Event_Message_TS000_3\Message:="PROC rMoveToClamp(1)";
                !
                rWobjClamp1;
                !
                jRobHomePos.extax.eax_a:=wClamp1.uframe.trans.x-wClamp1.oframe.trans.y+1000;
                MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wobj0;
                !
                jToolRotate{1}:=jRobHomePos;
                jToolRotate{1}.robax.rax_4:=0;
                MoveAbsJ jToolRotate{1}\NoEOffs,vHomeBoxSpeed,z5,tInductionRing\WObj:=wClamp1;
                !
                jToolRotate{2}:=jRobHomePos;
                jToolRotate{2}.robax.rax_4:=0;
                jToolRotate{2}.robax.rax_5:=60;
                MoveAbsJ jToolRotate{2}\NoEOffs,vHomeBoxSpeed,z5,tInductionRing\WObj:=wClamp1;
                !
                pMoveToClamp{1}.trans:=[0,500,0];
                pMoveToClamp{1}.robconf:=[0,0,0,0];
                pMoveToClamp{1}.rot:=[1,0,0,0];
                pMoveToClamp{1}.extax.eax_a:=jToolRotate{2}.extax.eax_a;
                MoveL pMoveToClamp{1},vSlowSpeed,fine,tInductionRing\WObj:=wClamp_NonRotate;
                !
                pActualPos:=CRobT(\Tool:=tInductionRing\WObj:=wobj0);
                Initial_angle:=(pActualPos.trans.y-3000)/(1500/45);
                ! Initial angle is -45 at y=1500mm and 0 at y=3000mm
                IF pActualPos.trans.y<1500 Initial_angle:=-45;
                IF pActualPos.trans.y>3000 Initial_angle:=0;
                pMoveToClamp{2}:=pMoveToClamp{1};
                pMoveToClamp{2}.rot:=pMoveToClamp{2}.rot*OrientZYX(0,Initial_angle,0);
                pMoveToClamp{2}.trans.y:=0;
                MoveL reltool(pMoveToClamp{2},0,500,0),vSlowSpeed,fine,tInductionRing\WObj:=wClamp1;
                MoveL reltool(pMoveToClamp{2},0,0,0),vSlowSpeed,fine,tInductionRing\WObj:=wClamp1;
                !
                wTempClamp:=wClamp1;
                TargetNew{1}:=CRobT(\Tool:=tInductionRing\WObj:=wTempClamp);
                MoveL TargetNew{1},v50,fine,tInductionRing\WObj:=wTempClamp;
                !
                Rtcpy_Offset:=EulerZYX(\Y,TargetNew{1}.rot);
                !
                Reset doRobotMoveToInitPosCP1;
                Set doRobotInInitposCP1;
                !
            ENDIF
        ENDIF
    ENDPROC

    PROC rTcpCenteringCycle()
        !***********************************************************
        ! Procedure: rTcpCenteringCycle
        ! Description:
        ! - Measure pipe for 100mm and position tool in measured 
        !   orientation
        !***********************************************************
        !
        Set doRobotInTCPCenteringCycle;
        !
        rLog_Time_Date_TS000_1;
        !
        IF RobOS() THEN

            IF (diCP2_Selected_SIM=1) THEN
                !
                rLog_Event_Message_TS000_3\Message:="PROC rTcpCenteringCycle(2)";
                !
                !**** ACTION ****
                rPathFollowSystemPreStart;
                !**** ACTION ****
            ELSE
                !
                rLog_Event_Message_TS000_3\Message:="PROC rTcpCenteringCycle(1)";
                !
                !**** ACTION ****
                rPathFollowSystemPreStart;
                !**** ACTION ****
            ENDIF
        ELSE
            FOR i FROM 1 TO 100 DO
                TargetNew{i+1}:=TargetNew{i};
                TargetNew{i+1}.trans.y:=TargetNew{i+1}.trans.y+1;
                MoveL TargetNew{i+1},v10,z1,tInductionRing\WObj:=wTempClamp;
            ENDFOR
            MoveL TargetNew{1},v100,Fine,tInductionRing\WObj:=wTempClamp;
            nMovementCounter:=1;
        ENDIF
        !
        Reset doRobotInTCPCenteringCycle;
        !
    ENDPROC

    PROC rIdentificationCycle()
        !***********************************************************
        ! Procedure: rIdentificationCycle
        ! Description:
        ! - Measure pipe and store the targets in an array. After
        !   the cycle sent the data (Translations and orientations 
        !   to the PLC
        !***********************************************************
        !
        Set doRobotIdentificationCycle;
        !
        rLog_Time_Date_TS000_1;
        !
        IF RobOS() THEN
            !
            IF (diCP2_Selected_SIM=1) THEN
                !
                rLog_Event_Message_TS000_3\Message:="PROC rIdentificationCycle(2)";
                !
                !**** ACTION ****
                rPathFollowSystem;
                !
                !**** ACTION ****
            ELSE
                !
                rLog_Event_Message_TS000_3\Message:="PROC rIdentificationCycle(1)";
                !
                !**** ACTION ****
                rPathFollowSystem;
                !**** ACTION ****
            ENDIF
        ELSE
            !
            rRSO_Identification;
            !
        ENDIF
        !
                nLastIdentifiedTarget:=nMovementCounter;

        Reset doRobotIdentificationCycle;
        !
    ENDPROC

    PROC rTemperingCycle()
        !***********************************************************
        ! Procedure: rTemperingCycle
        ! Description:
        ! - Robot moves backwards along the measured robotagets while
        !   the pipe is given a heat treatment.
        !***********************************************************
        !
        Set doRobotTemperingCycle;
        !
        rLog_Time_Date_TS000_1;
        !
        !IF RobOS() THEN
        IF (diCP2_Selected_SIM=1) THEN
            !
            rLog_Event_Message_TS000_3\Message:="PROC rTemperingCycle(2)";
            !
            !**** ACTION ****
            rFreeRunBwd(TRUE);
            !**** ACTION ****
        ELSE
            !
            rLog_Event_Message_TS000_3\Message:="PROC rTemperingCycle(1)";
            !
            !**** ACTION ****
            rFreeRunBwd(TRUE);
            !
            !**** ACTION ****
        ENDIF
        !ELSE
        !    rRSO_MoveBWD;
        !ENDIF
        !
        Reset doRobotTemperingCycle;
        !
    ENDPROC

    PROC rFreerun_FWD()
        !***********************************************************
        ! Procedure: rFreerun_FWD
        ! Description:
        ! - Robot moves forwards along the measured robotagets at 
        !   high speed, this is only possible if the current target
        !   is smaller then the last measured target.
        !***********************************************************
        !
        Set doRobotFreerunCycleFWD;
        !
        rLog_Time_Date_TS000_1;
        !
        IF (diCP2_Selected_SIM=1) THEN
            !
            rLog_Event_Message_TS000_3\Message:="PROC rFreerun_FWD(2)";
            !
            !**** ACTION ****
            !**** 
            rFreeRunFwd;
            !**** 
            !****
            !**** ACTION ****
        ELSE
            !
            rLog_Event_Message_TS000_3\Message:="PROC rFreerun_FWD(1)";
            !
            !**** ACTION ****
            !**** 
            !****
            rFreeRunFwd;
            !****
            !**** ACTION ****
        ENDIF
        !
        Reset doRobotFreerunCycleFWD;
        !
    ENDPROC

    PROC rFreerun_BWD()
        !***********************************************************
        ! Procedure: rFreerun_BWD
        ! Description:
        ! - Robot moves backwards along the measured robotagets at 
        !   high speed, this is only possible if the current target
        !   is bigger then 1
        !***********************************************************
        !
        Set doRobotFreerunCycleBWD;
        !
        rLog_Time_Date_TS000_1;
        !
        IF (diCP2_Selected_SIM=1) THEN
            !
            rLog_Event_Message_TS000_3\Message:="PROC rFreerun_BWD(2)";
            !
            !**** ACTION ****
            !**** 
            rFreeRunBwd(FALSE);
            !
            !****
            !**** ACTION ****
        ELSE
            !
            rLog_Event_Message_TS000_3\Message:="PROC rFreerun_BWD(1)";
            !
            !**** ACTION ****
            !**** 
            !****
            rFreeRunBwd(FALSE);
            !****
            !**** ACTION ****
        ENDIF
        !
        Reset doRobotFreerunCycleBWD;
        !
    ENDPROC

    PROC rMoveToHome()
        !***********************************************************
        ! Procedure: rMoveToHome
        ! Description:
        ! - Robot moves back to the home position.
        !   1. Robot moves 100mm in the direction of the tool
        !   2. Robot moves up
        !   3. Robot moves back to home.
        !
        !   Ask operator if the tool is free to leave the pipe
        !   Otherwise abort the request.
        !  
        !***********************************************************
        !
        VAR btnres answer;
        VAR robtarget pActualPos;
        !
        rLog_Time_Date_TS000_1;
        !
        Reset doRobotInInitposCP1;
        Reset doRobotInInitposCP2;
        !
        Set doRobotMovingToHomePosition;
        !
        !
        rLog_Event_Message_TS000_3\Message:="PROC rMoveToHome()";
        !
        answer:=0;
        !
        UIMsgBox\Header:="UIMsgBox Header","Robot to home"\MsgLine2:="Is the robot free to move home?"\MsgLine3:="Can the robot move safe of the pipe"\MsgLine4:=""\MsgLine5:=""\Buttons:=btnOKCancel\Icon:=iconWarning\Result:=answer;
        !
        IF answer=resOK THEN
            !
            velset 100,100;
            pActualPos:=CRobT(\Tool:=tInductionRing\WObj:=wobj0);
            MoveL RelTool(pActualPos,0,250,0),v100,fine,tInductionRing;
            pActualPos:=CRobT(\Tool:=tInductionRing\WObj:=wobj0);
            pActualPos.extax.eax_a:=pActualPos.trans.x;
            reg1:=EulerZYX(\X,pActualPos.rot);
            reg2:=EulerZYX(\Y,pActualPos.rot);
            reg3:=EulerZYX(\Z,pActualPos.rot);
            !
            IF diCP2_Selected_SIM=0 THEN
                pActualPos.rot:=OrientZYX(-90,reg2,reg1);
                pActualPos.trans.x:=pActualPos.trans.x+1000;
                pActualPos.extax.eax_a:=pActualPos.extax.eax_a+1000;
            ELSE
                pActualPos.rot:=OrientZYX(90,reg2,reg1);
                pActualPos.trans.x:=pActualPos.trans.x-1000;
                pActualPos.extax.eax_a:=pActualPos.extax.eax_a-1000;
            ENDIF
            !
            MoveJ pActualPos,v100,fine,tInductionRing;
            !
            jToolRotate{2}.extax.eax_a:=pActualPos.extax.eax_a;
            jToolRotate{1}.extax.eax_a:=pActualPos.extax.eax_a;
            jRobHomePos.extax.eax_a:=pActualPos.extax.eax_a;
            !
            MoveAbsJ jToolRotate{2}\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wTempClamp;
            MoveAbsJ jToolRotate{1}\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wTempClamp;
            MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wobj0;
            !
            Reset doInCycle;
        ELSE
            ErrWrite "Request to home aborted by operator",""\RL2:="Call service";
        ENDIF
        !
        rLog_Clear_Log_TS000_5;
        !
        Reset doRobotMovingToHomePosition;
        !
    ENDPROC

    PROC rWobjClamp1()
        CONST num nClamp1Radius:=494+400;
        CONST num nClamp1PozZ:=760;
        !
        wClamp1.uframe.trans:=[390,720,-255];
        wClamp1.uframe.trans.x:=2000+wClamp1.uframe.trans.x;
        wClamp1.uframe.trans.z:=-303+wClamp1.uframe.trans.z;
        wClamp1.uframe.rot:=OrientZYX(90,0,0);
        wClamp1.oframe.trans.x:=nClampPozX{1}+nClamp1Radius*Sin(nClampAngle{1});
        wClamp1.oframe.trans.y:=-nClamp1Radius*Cos(nClampAngle{1});
        wClamp1.oframe.trans.z:=nClamp1PozZ;
        wClamp1.oframe.rot:=OrientZYX(0,0,0);
        wClamp_NonRotate:=wClamp1;
        wClamp_NonRotate.oframe.rot:=wClamp1.oframe.rot*OrientZYX(180,0,0);
        wClamp1.oframe.rot:=wClamp1.oframe.rot*OrientZYX(180+nClampAngle{1},0,0);
        !
    ENDPROC

    PROC rWobjClamp2()
        CONST num nClamp2Radius:=494+400;
        CONST num nClamp2PozZ:=760;
        !
        wClamp2.uframe.trans:=[10065,1500,-215];

        !wClamp2.uframe.trans:=[10065,725,-215];
        wClamp2.uframe.trans.x:=-2000+wClamp2.uframe.trans.x;
        wClamp2.uframe.trans.z:=-303+wClamp2.uframe.trans.z;
        !
        wClamp2.uframe.rot:=OrientZYX(90,0,0);
        wClamp2.oframe.trans.x:=nClampPozX{2}-nClamp2Radius*Sin(nClampAngle{2});
        wClamp2.oframe.trans.y:=nClamp2Radius*Cos(nClampAngle{2});
        wClamp2.oframe.trans.z:=nClamp2PozZ;
        wClamp2.oframe.rot:=OrientZYX(0,-90,0);
        wClamp_NonRotate:=wClamp2;
        !
        wClamp2.oframe.rot:=wClamp2.oframe.rot*OrientZYX(0,0,nClampAngle{2});
    ENDPROC

    PROC rReceived_client_info()
        !***********************************************************
        ! Procedure: rReceived_client_info
        ! Machine: Destrapper
        ! Description:
        ! - write the received information to local variable's. And use
        !   the local VAR in the software!
        !***********************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC rInit_Received_information()";
        !
        IF diCP2_Selected_SIM=1 THEN
            rLog_Event_Message_TS000_3\Message:="diCP2_Selected_SIM = 1";
            !
            IF RobOS() THEN
                nClampAngle{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giClamp2Angle))/100;
                nClampPozX{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giClamp2PosX));
            ELSE
                nClampAngle{2}:=nUnsignedToSigned_INT_TS000_50(-10);
                nClampPozX{2}:=nUnsignedToSigned_INT_TS000_50(1490);
            ENDIF
            !
            rLog_Numeric_Value_TS000_2\Message:="nClampPozX{2} = ",nClampPozX{2};
            rLog_Numeric_Value_TS000_2\Message:="nClampAngle{2} = ",nClampAngle{2};
            !
            IF (nClampPozX{2}<400 OR nClampPozX{2}>2500) bAbort_Cycle:=TRUE;
            IF (nClampAngle{2}<-45 OR nClampAngle{2}>45) bAbort_Cycle:=TRUE;
        ELSE
            rLog_Event_Message_TS000_3\Message:="diCP2_Selected_SIM = 0";
            !
            IF RobOS() THEN
                nClampAngle{2}:=nUnsignedToSigned_INT_TS000_50(GInput(giClamp1Angle))/100;
                nClampPozX{1}:=nUnsignedToSigned_INT_TS000_50(GInput(giClamp1PosX));
            ELSE
                nClampAngle{2}:=nUnsignedToSigned_INT_TS000_50(15);
                nClampPozX{1}:=nUnsignedToSigned_INT_TS000_50(1460);
            ENDIF
            !
            rLog_Numeric_Value_TS000_2\Message:="nClampPozX{1}  = ",nClampPozX{1};
            rLog_Numeric_Value_TS000_2\Message:="nClampAngle{1} = ",nClampAngle{1};
            !
            IF (nClampPozX{1}<400 OR nClampPozX{1}>2500) bAbort_Cycle:=TRUE;
            IF (nClampAngle{1}<-45 OR nClampAngle{1}>45) bAbort_Cycle:=TRUE;
        ENDIF
    ENDPROC
ENDMODULE
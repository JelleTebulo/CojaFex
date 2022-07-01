MODULE TCTM840_HOMING
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Module Name: TBL840_HOMING
    !Version:     1.0
    !Description: 
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    !
    PERS bool bRobotContinueFromCurrentPos:=TRUE;
    PERS jointtarget jRobHomePos:=[[0,-30,30,90,0,0],[6236.68,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PROC rMoveHomeSafe()
        !***********************************************************
        ! Procedure: rMoveHomeSafe 
        ! Description:
        ! - robot moves safe from every position to the homeposition 
        !   in case the robot don't konw how, alarm 10 will be made
        !***********************************************************
        !
        VAR robtarget pActualPos;
        VAR jointtarget pActualJoint;
        !
        VelSet 100,500;
        ConfL\On;
        ConfJ\On;
        !
        rLog_Event_Message_TS000_3\Message:="PROC rMoveHomeSafe()";
        !
        rLog_Current_Rob_Pos_TS000_9;
        !
        tCurrentTool:=CTool();
        pActualPos:=CRobT(\Tool:=tCurrentTool\WObj:=wobj0);
        pActualJoint:=CalcJointT(pActualPos,tCurrentTool\WObj:=wobj0);
        !
        rCalc_Joint_Target_For_MHS;
        !
        bRobotContinueFromCurrentPos:=FALSE;
        !
        IF (Doutput(sdoRobotInHomeBox)=1 OR sdoRobotInHomePos=1) THEN
            jRobHomePos.extax.eax_a:=pActualPos.extax.eax_a;
            MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wPipe;
            !
            IF pActualPos.extax.eax_a<1500 pActualPos.extax.eax_a:=1500;
            IF pActualPos.extax.eax_a>7500 pActualPos.extax.eax_a:=7500;
            !
            jRobHomePos.extax.eax_a:=pActualPos.extax.eax_a;
            MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wPipe;
            !
        ELSE
            rOperatorJogRobotHome;
        ENDIF
        !
        ConfJ\On;
        !
        bMHS_RobotToManualMode:=FALSE;
        !
        Reset doRobotMovingToHomePosition;
        Reset doRobotMovingInHomeBox;
        !
    ENDPROC

    PROC rOperatorJogRobotHome()
        CONST listitem list{2}:=[["","ROBOT TO HOME"],["","CONTINUE CYCLE"]];
        VAR num list_item;
        VAR btnres button_answer;
        !
        rLog_Event_Message_TS000_3\Message:="PROC rOperatorJogRobotHome";
        !
        list_item:=UIListView(\Result:=button_answer\Header:="ROBOT PROGRAM STARTED IN MAIN",list\Buttons:=btnOK\Icon:=iconInfo\DefaultIndex:=1);
        !
        IF list_item=1 THEN
            !
            bRobotContinueFromCurrentPos:=FALSE;
            !
            Set doRobotNotAbleToGoHome;
            UIMsgBox\Header:="ROBOT TO HOME","..."\MsgLine2:="Is the area free for the robot to move home?"\MsgLine3:="Otherwise jog robot to home!"\MsgLine4:=""\MsgLine5:=""\Buttons:=btnOKCancel\Icon:=iconInfo\Result:=button_answer;
            !
            IF button_answer=resOK THEN
                jRobHomePos.extax.eax_a:=5100;
                MoveAbsJ jRobHomePos\NoEOffs,v7000,Fine,tInductionRing\WObj:=wPipe;
            ELSE
                ExitCycle;
            ENDIF
        ELSEIF list_item=2 THEN
            ! Continue cycle from current position
            bRobotContinueFromCurrentPos:=TRUE;
        ENDIF
    ENDPROC

    FUNC bool bCurrentJoint(jointtarget ComparePos,num Accuracy)
        !***********************************************************
        ! Function: bCurrentJoint
        ! Machine: -
        ! Description:
        ! - Function to read and compare the actual robot joints
        !   with the given joint
        !***********************************************************
        !
        VAR num Counter:=0;
        VAR jointtarget ActualJoint;
        !
        ActualJoint:=CJointT();
        Counter:=0;
        !
        IF ActualJoint.robax.rax_1>ComparePos.robax.rax_1-Accuracy AND ActualJoint.robax.rax_1<ComparePos.robax.rax_1+Accuracy Counter:=Counter+1;
        IF ActualJoint.robax.rax_2>ComparePos.robax.rax_2-Accuracy AND ActualJoint.robax.rax_2<ComparePos.robax.rax_2+Accuracy Counter:=Counter+1;
        IF ActualJoint.robax.rax_3>ComparePos.robax.rax_3-Accuracy AND ActualJoint.robax.rax_3<ComparePos.robax.rax_3+Accuracy Counter:=Counter+1;
        IF ActualJoint.robax.rax_4>ComparePos.robax.rax_4-Accuracy AND ActualJoint.robax.rax_4<ComparePos.robax.rax_4+Accuracy Counter:=Counter+1;
        IF ActualJoint.robax.rax_5>ComparePos.robax.rax_5-Accuracy AND ActualJoint.robax.rax_5<ComparePos.robax.rax_5+Accuracy Counter:=Counter+1;
        IF ActualJoint.robax.rax_6>ComparePos.robax.rax_6-Accuracy AND ActualJoint.robax.rax_6<ComparePos.robax.rax_6+Accuracy Counter:=Counter+1;
        !        
        RETURN Counter=6;
    ENDFUNC
ENDMODULE
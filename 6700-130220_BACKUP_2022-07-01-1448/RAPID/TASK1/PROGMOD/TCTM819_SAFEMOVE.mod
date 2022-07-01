MODULE TCTM819_SAFEMOVE
    !***********************************************************
    ! Project: P21-100538
    ! Description:
    !   Routines/functions can be modified for the customer
    !
    ! Author: RLZ
    ! Version: 1.0
    ! Date: 04-2022
    !***********************************************************    
    !
    PERS bool bSC1_CBC_Allowed:=TRUE;

    PROC rRunCyclicBrakeCheck()
        !****************************************************************
        !* PROC: rRunCyclicBrakeCheck
        !* Description:
        !* - Run cyclic brake check  (720 hours)
        !****************************************************************
        VAR num mystatus:=0;
        !
        rLog_Time_Date_TS000_1;
        !
        rLog_Event_Message_TS000_3\Message:="PROC rRunCyclicBrakeCheck()";
        !
        Set doRobotMovingInHomeBox;
        !
        CyclicBrakeCheck;
        !
        !MoveL pRobHomeposition,vHomeBoxSpeed,fine,tDestrapper;
        !
        Reset doRobotMovingInHomeBox;
        !        
        UIShow sScreenMaker,"ABB.Robotics.SDK.Views.MainScreen"\Status:=mystatus\NoCloseBtn;
        IF (DOutput(doSC1_CBC_ERROR)=1) bSC1_CBC_Allowed:=FALSE;
        !
    ERROR
        IF (ERRNO=ERR_UISHOW_FATAL OR ERRNO=ERR_TP_NO_CLIENT OR ERRNO=ERR_UISHOW_FULL OR ERRNO=ERR_NORUNUNIT) THEN
            SKIPWARN;
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
MODULE TCTM850_MAINTENANCE
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
    CONST jointtarget jStand_Norm_Pos:=[[0,0,0,0,0,0],[0,0,0,0,0,0]];
    CONST jointtarget jOutsingulair_2:=[[0,0,0,0,0,0],[0,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget jStand_Norm_Pos10:=[[0,0,0,0,15,0],[9182.5,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget jStand_Norm_Pos20:=[[0,0,0,0,0,0],[9153,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget jStand_Norm_Pos30:=[[-3.02001E-05,0.000567028,0.00058996,0.000252867,0.000193243,-0.000165295],[5153,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST jointtarget jStand_Norm_Pos40:=[[0.000834197,8.41553E-05,0.000633499,0.00153461,15.0002,-0.00204184],[9181.69,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget pMaintenance:=[[4499.96,3100,0],[0.624985,0.332774,-0.33752,0.62027],[-1,-1,2,0],[4500,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget pMoveToMaintenance:=[[4500.00,1500.00,2000.00],[0.5,0.5,-0.500001,0.5],[-1,0,1,0],[4500,9E+09,9E+09,9E+09,9E+09,9E+09]];


    PROC rCalibration_Position_IRB()
        !***********************************************************
        ! Procedure: rCalibration_Position_IRB
        ! Machine: - 
        ! Description:
        ! - robot to calibration position
        !***********************************************************
        ! 
        rLog_Event_Message_TS000_3\Message:="PROC rCalibration_Position_IRB()";
        ! 
        rResetCommands;
        rResetCommand_After_MoveHomeSafe;
        ! 
        Set doRobotMovingInHomeBox;
        Set doMovingToCalibrationPos;
        !
        MoveAbsJ jOutsingulair_2\NoEOffs,vHomeBoxSpeed,z50,tInductionRing;
        MoveAbsJ jStand_Norm_Pos10\NoEOffs,vHomeBoxSpeed,fine,tInductionRing;
        MoveAbsJ jStand_Norm_Pos40\NoEOffs,vHomeBoxSpeed,fine,tInductionRing;
        MoveL [[4499.98,1249.99,999.97],[0.717983,-0.0551986,-0.0772038,-0.68956],[-1,0,-1,0],[4499.99,9E+09,9E+09,9E+09,9E+09,9E+09]],vHomeBoxSpeed,fine,tInductionRing;
        !    
        Reset doMovingToCalibrationPos;
        Set doRobotInCalibrationPos;
        Reset doRobotMovingInHomeBox;
        !   
        WaitUntil(diLocal_Sim=0 OR diHMI_Home=1);
        ! 
        Reset doRobotInCalibrationPos;
        Set doRobotMovingInHomeBox;
        Set doRobotMovingToHomePosition;
        ! 
        MoveAbsJ jOutsingulair_2\NoEOffs,vHomeBoxSpeed,z50,tInductionRing;
        MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,z50,tInductionRing;
        Reset doRobotMovingToHomePosition;
        !
    ENDPROC

    PROC rRobotToMaintPos()
        !***********************************************************
        ! Procedure: rRobotToMaintPos
        ! Machine: Destrapper
        ! Description:
        ! - Robot to maintenance position
        !***********************************************************
        !
        rLog_Event_Message_TS000_3\Message:="PROC rRobotToMaintPos()";
        !
        Set doRobotMovingInHomeBox;
        !
        rResetCommands;
        rResetCommand_After_MoveHomeSafe;
        !
        Set doRobotMovingInHomeBox;
        !
        jRobHomePos.extax.eax_a:=pMoveToMaintenance.extax.eax_a;
        MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wobj0;
        MoveJ pMoveToMaintenance,vHomeBoxSpeed,fine,tInductionRing;
        MoveL pMaintenance,vHomeBoxSpeed,fine,tInductionRing;
        !
        WaitUntil(diLocal_Sim=0 OR diHMI_Home=1 OR doHMI_Home=1);
        !
        MoveL pMoveToMaintenance,vHomeBoxSpeed,fine,tInductionRing;
        jRobHomePos.extax.eax_a:=pMoveToMaintenance.extax.eax_a;
        MoveAbsJ jRobHomePos\NoEOffs,vHomeBoxSpeed,Fine,tInductionRing\WObj:=wobj0;

        !
    ENDPROC


ENDMODULE
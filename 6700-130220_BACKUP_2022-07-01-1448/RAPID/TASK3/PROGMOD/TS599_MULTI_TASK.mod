MODULE TS599_MULTI_TASK
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Module Name: TS599_MULTI_TASK
    !Version:     1.0
    !Description: 
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    !
    PERS num nHMI_Required_Angle_Axis{6}:=[0,-30,30,90,0,0];
    PERS bool bMHS_RobotToManualMode:=TRUE;
    !
    PERS pos nTCP_Trans:=[0,0,0];
    PERS num nLateralSpeed{2}:=[1,0.0166667];
    PERS num nTemperingSpeed{2}:=[25,0.416667];
    PERS num nIdleSpeed{2}:=[1000,16.6667];
    PERS num nMovementCounter:=653;         ! Actual tcp heating ring position
    PERS num nLastIdentifiedTarget:=653;
ENDMODULE
MODULE TCTM860_ROBOTSTUDIO
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Version:     1.0
    !Description: 
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    !

    PROC rRSO_Identification()
        !
        WHILE nMovementCounter<2000 AND diRequestStopReceived=0 DO
            TargetNew{nMovementCounter+1}:=TargetNew{nMovementCounter};
            TargetNew{nMovementCounter+1}.trans.y:=TargetNew{nMovementCounter+1}.trans.y+1;

            IF diCP2_Selected_SIM=1 THEN
                TargetNew{nMovementCounter+1}.extax.eax_a:=TargetNew{nMovementCounter+1}.extax.eax_a-1;
            ELSE
                TargetNew{nMovementCounter+1}.extax.eax_a:=TargetNew{nMovementCounter+1}.extax.eax_a+1;
            ENDIF
            MoveL TargetNew{nMovementCounter+1},v10,z1,tInductionRing\WObj:=wTempClamp;
            Incr nMovementCounter;
        ENDWHILE
        !
        nLastIdentifiedTarget:=nMovementCounter;
        !
    ENDPROC
ENDMODULE

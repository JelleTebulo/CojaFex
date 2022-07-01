MODULE SmartSensor
    PERS pos posLB1:=[-148.418,2092.04,677.303];
    PERS pos posLB2:=[-210.705,2002.84,568.934];
    PERS pos posLB3:=[-148.418,2092.04,459.907];
    PERS pos posLB4:=[-85.7482,2181.79,568.934];
    !
    TASK PERS tooldata LB_1:=[TRUE,[[-28.28,-68,781.72],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    TASK PERS tooldata LB_2:=[TRUE,[[ 28.28,-68,781.72],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    TASK PERS tooldata LB_3:=[TRUE,[[ 28.28,-68,838.28],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    TASK PERS tooldata LB_4:=[TRUE,[[-28.28,-68,838.28],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    !
    PERS num Lasers{4}:=[-3449.93,-3441.27,-3504.93,-3514.26];
    !
    PERS pos posSPLB1:=[2694.16,-68,1648.63];
    PERS pos posSPLB2:=[2665.88,-68,1599.65];
    PERS pos posSPLB3:=[2714.86,-68,1571.37];
    PERS pos posSPLB4:=[2743.14,-68,1620.35];
    !
    PROC main()
        !****************************************************************
        !* PROC: Main 	  
        !* Description:
        !* - TASK 4
        !* - Robotprogram starts here
        !****************************************************************
        !
        WHILE TRUE DO
            !
            IF RobOS()=FALSE rDistanceLaser;
            WaitTime 0.1;
        ENDWHILE
    ENDPROC

    PROC rDistanceLaser()
        !****************************************************************
        !* PROC: rDistanceLaser 	  
        !* Description:
        !* - Every 0.01 second the distance between the tool and the 
        !*   sensed object is measured.
        !****************************************************************
        posSPLB1:=CPos(\Tool:=LB_1\WObj:=wobj0);
        posSPLB2:=CPos(\Tool:=LB_2\WObj:=wobj0);
        posSPLB3:=CPos(\Tool:=LB_3\WObj:=wobj0);
        posSPLB4:=CPos(\Tool:=LB_4\WObj:=wobj0);
        !
        Lasers{1}:=250-(Distance(posLB1,posSPLB1));
        Lasers{2}:=250-(Distance(posLB2,posSPLB2));
        Lasers{3}:=250-(Distance(posLB3,posSPLB3));
        Lasers{4}:=250-(Distance(posLB4,posSPLB4));
        !
    ENDPROC
ENDMODULE
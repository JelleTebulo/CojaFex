MODULE TCTM820_CALCULATE
    !*****************************************************
    !Project    : P21-100538 COJAFEX
    !Version:     1.0
    !Description:  
    !Date:        04-2022
    !Author:      RLZ
    !*****************************************************
    
    PERS robtarget pConfigCopy;
    VAR num TotalFor{4} := [0,0,0,0];
    VAR num MeanFor{4} := [0,0,0,0];
    VAR num TotalAfter{4} := [0,0,0,0];
    VAR num MeanAfter{4} := [0,0,0,0];
    VAR num Ncycle:=0;
    VAR num doErrorTrans{7,4}:=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
    VAR num doErrorRot{7,4}:=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
    VAR bool ErrorAs{7} := [FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE];     ! Error state whether the joint angles are within acceptable range or not. If not, the allowable joint angles are reduced    
    VAR bool Error_OOR := FALSE;          ! Error state wether the robot configuration remains within its work area
    VAR num posY_Offset := 1000;          ! Offset for positioning as 7 w.r.t the x-position of the tool.
    PERS num Rtcpy_Offset:=43.6404;            ! Offset for rotating the y axis of the tool.
    PERS num posY;
	PERS num As{7};     
    CONST num e_m1 := 1; ! margin for cmaxult_as
    CONST num e_m2 := 9; ! margin for cmax_as
    CONST num ErrorHyst := 2;   ! This extra error is used to narrow the acceptable joint angles (i.e. acceptable joint limits -50 to 50 become -40 to 40)
    VAR num cmaxult_as{6,2};
    VAR num cmax_as_const{6,2};  ! Minimum and maximum axis angles per joint respectively in order to steer ax7 or ytcp
    VAR num cmax_as{6,2};  ! Variable version in order to apply hysteresis method      
     
    VAR num AxisError{6} := [0,0,0,0,0,0]; ! Errors of the axes
    CONST num dy := 2;              ! Stap in y-direction van as 7
    CONST num dRtcpy := 0.15;       ! Stap om de y as van tcp    
    VAR num pgain{6} := [1,1,1,1,1,1];              ! Gain for adjusting As{7,1} (i.e. As{7,1}(k+1) = As{7,1}(k) + jointlimiterrors*pgain)
    VAR num rgain{6} := [1,1,1,1,1,1];              ! Gain for adjusting rotation y-tcp (i.e. y-tcp(k+1) = y-tcp(k) + jointlimiterrors*rgain)
    VAR num p_wrist_r_gain := 1;                  ! Gain for adjusting As{7,1} to account for the distance between the wrist center and the robot origin
    CONST num k:=20;                                ! nMovementCounter + 1 (k=1)
    VAR num ErrorAs6 := 0;                          ! Relative error of as 6 wrt 270 or 90  degrees for in configuration method cfx=0 or cfx=1
    VAR num ErrorAs5 := 0;                          ! Relative error of as 5 wrt 0
    VAR num ErrorAs1 := 0;                          ! Relative error of as 1 wrt 0
    PERS num As6OptVal := -90;                      ! The optimum value to rotate to when using logic to give room for axis 5 
    VAR num As3MaxOOR := -45;                      ! Threshold when to use rPreventOutOfReach procedure   
    CONST num As3cMaxOOR := -45;                    ! Constant threshold when to use rPreventOutOfReach procedure   
    VAR num As3ErrorOOR := 0;                       ! Error of as3 wrt to As3MaxOOR
    VAR num OOR_Counter := 0;                       ! Out of reach counter       
    
    VAR jointtarget JointConf;                        ! Joints configuration for TargetNew at nMovementCounter+k 
    VAR jointtarget JointConf1;                        ! Joints configuration for TargetNew at nMovementCounter+k+1
    VAR jointtarget JointConfStart;
    VAR num Axis7PartSize;
    VAR robtarget StartRob;
    VAR robtarget FutureRob;
    VAR jointtarget JointC;
    
    PROC rCalcCenterPointTube()
        VAR robtarget Dummy{3};
        !
        Dummy{1} := [[0,0,0],[1,0,0,0],[0,0,-1,0],[0,9E+9,9E+9,9E+9,9E+9,9E+9]];
        Dummy{2} := [[0,0,0],[1,0,0,0],[0,0,-1,0],[0,9E+9,9E+9,9E+9,9E+9,9E+9]];
        Dummy{3} := [[0,0,0],[1,0,0,0],[0,0,-1,0],[0,9E+9,9E+9,9E+9,9E+9,9E+9]];
        !
        nLaserData := Lasers;
        nLaserData{1} := GInput(giLsrDist_1)/10;
        nLaserData{2} := GInput(giLsrDist_2)/10;
        nLaserData{3} := GInput(giLsrDist_3)/10;
        nLaserData{4} := GInput(giLsrDist_4)/10;
        !
        IF (nLaserData{1}*10)>65000 THEN
            nLaserData{1} := -1;
        ENDIF
        IF (nLaserData{2}*10)>65000 THEN
            nLaserData{2} := -1;
        ENDIF
        IF (nLaserData{3}*10)>65000 THEN
            nLaserData{3} := -1;
        ENDIF
        IF (nLaserData{4}*10)>65000 THEN
            nLaserData{4} := -1;
        ENDIF
        !        Als de laser uit bereik is: waarde 0, dan de vorige overnemen.
        IF (CALC_COUNTER+nMovementCounter-50)<1200 AND nMovementCounter>0 THEN
            LaserDataAll{CALC_COUNTER+nMovementCounter-50} := [nLaserData{1},nLaserData{2},nLaserData{3},nLaserData{4}];
        ENDIF
        IF nLaserData{1}<=5 THEN
            nLaserData{1} := nLaserDataprev{1};
            nLaserOutRange{1} := nLaserOutRange{1}+1;
        ELSE
            nLaserOutRange{1} := 0;
        ENDIF
        IF nLaserData{2}<=5 THEN
            nLaserData{2} := nLaserDataprev{2};
            nLaserOutRange{2} := nLaserOutRange{2}+1;
        ELSE
            nLaserOutRange{2} := 0;
        ENDIF
        IF nLaserData{3}<=5 THEN
            nLaserData{3} := nLaserDataprev{3};
            nLaserOutRange{3} := nLaserOutRange{3}+1;
        ELSE
            nLaserOutRange{3} := 0;
        ENDIF
        IF nLaserData{4}<=5 THEN
            nLaserData{4} := nLaserDataprev{4};
            nLaserOutRange{4} := nLaserOutRange{4}+1;
        ELSE
            nLaserOutRange{4} := 0;
        ENDIF
        !
        SaveLaserValue{CALC_COUNTER+nMovementCounter} := [nLaserData{1},nLaserData{2},nLaserData{3},nLaserData{4}];
        !
        posDelta.x := ((nLaserData{2}+nLaserData{4})/2)-nLaserData{2};
        posDelta.z := ((nLaserData{1}+nLaserData{3})/2)-nLaserData{1};
        TempTool := tLasers;
        !
        Dummy{1}.trans := [0,0,0];
        Dummy{2}.trans := posDelta;
        !
        rUpdateToolTcp TempTool,(Dummy{2}),(Dummy{1});
        !
        Dummy{3} := (CRobT(\Tool := TempTool\WObj := wTempClamp));
        !
        pConfigCopy.robconf := Dummy{3}.robconf;
        !
        posCenterTube := Dummy{3}.trans;
        !
        nLaserDataprev := nLaserData;
        !
    ENDPROC

    PROC rTubeEnd()
        VAR num Aantal{4} := [0,0,0,0];
        TotalFor{1} := 0;
        TotalAfter{1} := 0;
        TotalFor{2} := 0;
        TotalAfter{2} := 0;
        TotalFor{3} := 0;
        TotalAfter{3} := 0;
        TotalFor{4} := 0;
        TotalAfter{4} := 0;
        Aantal := [0,0,0,0];
        
        FOR i FROM 1 TO LaserDistMean DO
            IF SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q1 > 5 THEN
                TotalFor{1} := TotalFor{1} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q1;
                Aantal{1} := Aantal{1} + 1;
            ENDIF
            IF SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q1 > 5 THEN
                TotalFor{2} := TotalFor{2} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q2;
                Aantal{2} := Aantal{2} + 1;
            ENDIF
            IF SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q1 > 5 THEN
                TotalFor{3} := TotalFor{3} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q3;
                Aantal{3} := Aantal{3} + 1;
            ENDIF
            IF SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q1 > 5 THEN
                TotalFor{4} := TotalFor{4} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i - LaserDistMean - 1}.q4;
                Aantal{4} := Aantal{4} + 1;
            ENDIF

            TotalAfter{1} := TotalAfter{1} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i + 1}.q1;
            TotalAfter{2} := TotalAfter{2} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i + 1}.q2;     
            TotalAfter{3} := TotalAfter{3} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i + 1}.q3;     
            TotalAfter{4} := TotalAfter{4} + SaveLaserValue{CALC_COUNTER + nMovementCounter - i + 1}.q4;     
        ENDFOR
        
        IF Aantal{1}>0 THEN MeanFor{1} := TotalFor{1}/Aantal{1}; ENDIF
        IF Aantal{2}>0 THEN MeanFor{2} := TotalFor{2}/Aantal{2}; ENDIF
        IF Aantal{3}>0 THEN MeanFor{3} := TotalFor{3}/Aantal{3}; ENDIF
        IF Aantal{4}>0 THEN MeanFor{4} := TotalFor{4}/Aantal{4}; ENDIF
        MeanAfter{1} := TotalAfter{1}/LaserDistMean;
        MeanAfter{2} := TotalAfter{2}/LaserDistMean;
        MeanAfter{3} := TotalAfter{3}/LaserDistMean;
        MeanAfter{4} := TotalAfter{4}/LaserDistMean;
        IF MeanFor{1} > 4 AND MeanFor{2} > 5 AND MeanFor{3} > 5 AND MeanFor{4} > 5 THEN
            IF ((MeanAfter{1} - MeanFor{1}) > EndTubeMinDistance) AND ((MeanAfter{2} - MeanFor{2}) > EndTubeMinDistance) AND ((MeanAfter{3} - MeanFor{3}) > EndTubeMinDistance) AND ((MeanAfter{4} - MeanFor{4}) > EndTubeMinDistance) THEN
                END_TUBE_REACHED{1} := TRUE; 
            ENDIF
        ENDIF
        IF nLaserOutRange{1}>=MaxLaserOutRange OR nLaserOutRange{2}>=MaxLaserOutRange OR nLaserOutRange{3}>=MaxLaserOutRange OR nLaserOutRange{4}>=MaxLaserOutRange THEN
            END_TUBE_REACHED{2} := TRUE;
        ENDIF
    ENDPROC
    
    PROC rUpdateToolTcp(PERS tooldata Tool,robtarget Ref_Point,robtarget Cur_Point)
        VAR pose ppnt := [[0,0,0],[1,0,0,0]];
        VAR pose ppntnew := [[0,0,0],[1,0,0,0]];
        VAR pose pa := [[0,0,0],[1,0,0,0]];
        !
        ppnt.trans := Ref_Point.trans;
        ppnt.rot := Ref_Point.rot;
        ppntnew.trans := Cur_Point.trans;
        ppntnew.rot := Cur_Point.rot;
        pa := PoseMult(ppntnew,PoseInv(Tool.tframe));
        Tool.tframe := PoseMult(PoseInv(pa),ppnt);
        ! 
    ENDPROC

    PROC rLinearFilter()
        !Linialiseren van de beweging. Lineaire lijn over de laatst opgelagen 21 data punten, datapunt 11 wordt waarde overeenstemmend met de lineaire lijn.
        VAR pos a := [0,0,0];
        ! Y = a*x + b
        VAR pos b := [0,0,0];
        ! Y = a*x + b
        VAR num startPos := 0;
        ! eerste data punt voor pos filter
        VAR num midPos := 0;
        ! middelste data punt van filter
        VAR num datanr := 0;
        ! som van datanr
        VAR num datanr_2 := 0;
        ! som van datanr^2
        VAR pos positie := [0,0,0];
        ! som van de posities [x, y, z]
        VAR pos datanr_positie := [0,0,0];
        ! som van datanr*positie [x, y, z]
        VAR num aantal := 0;
        !
        reg4 := 0;
        reg5 := 0;
        reg6 := 0;
        datanr := 0;
        datanr_2 := 0;
        positie := [0,0,0];
        datanr_positie := [0,0,0];
        aantal := 0;
        !
        nPosFilterCounter := CALC_COUNTER+nMovementCounter-((POS_FILTER_SIZE-1)*0.5);
        startPos := nPosFilterCounter-((POS_FILTER_SIZE-1)*0.5);
        midPos := (POS_FILTER_SIZE+1)*0.5;
        !
        FOR idx FROM 1 TO POS_FILTER_SIZE DO
            incr aantal;
            datanr := datanr+idx;
            ! X
            datanr_2 := datanr_2+(pow(idx,2));
            ! X^2
            positie.x := positie.x+TargetNew{startPos-1+idx}.trans.x;
            ! Y = X
            positie.y := positie.y+TargetNew{startPos-1+idx}.trans.y;
            ! Y = Y
            positie.z := positie.z+TargetNew{startPos-1+idx}.trans.z;
            ! Y = Z
            datanr_positie.x := datanr_positie.x+(idx*TargetNew{startPos-1+idx}.trans.x);
            ! X * Y
            datanr_positie.y := datanr_positie.y+(idx*TargetNew{startPos-1+idx}.trans.y);
            datanr_positie.z := datanr_positie.z+(idx*TargetNew{startPos-1+idx}.trans.z);
        ENDFOR
        !
        a.x := ((POS_FILTER_SIZE*datanr_positie.x)-(datanr*positie.x))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.x := ((positie.x*datanr_2)-(datanr*datanr_positie.x))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        a.y := ((POS_FILTER_SIZE*datanr_positie.y)-(datanr*positie.y))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.y := ((positie.y*datanr_2)-(datanr*datanr_positie.y))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        a.z := ((POS_FILTER_SIZE*datanr_positie.z)-(datanr*positie.z))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.z := ((positie.z*datanr_2)-(datanr*datanr_positie.z))/((POS_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        !
        reg4 := (a.x*midPos)+b.x;
        ! y=ax+b
        reg5 := (a.y*midPos)+b.y;
        reg6 := (a.z*midPos)+b.z;
        !
        TargetNew{nPosFilterCounter}.trans := [reg4,reg5,reg6];
        !
    ENDPROC

    PROC rCalcStartPos()
        !Linialiseren van de beweging. Lineaire lijn over de laatst opgelagen 21 data punten, datapunt 11 wordt waarde overeenstemmend met de lineaire lijn.
        VAR pos a := [0,0,0];
        ! Y = a*x + b
        VAR pos b := [0,0,0];
        ! Y = a*x + b
        VAR num startPos := 0;
        ! eerste data punt voor pos filter
        VAR num eindPos := 0;
        ! laatste data punt voor pos filter
        VAR num datanr := 0;
        ! som van datanr
        VAR num datanr_2 := 0;
        ! som van datanr^2
        VAR pos positie := [0,0,0];
        ! som van de posities [x, y, z]
        VAR pos datanr_positie := [0,0,0];
        ! som van datanr*positie [x, y, z]
        VAR num aantal := 0;
        !
        reg4 := 0;
        reg5 := 0;
        reg6 := 0;
        aantal := 0;
        datanr := 0;
        datanr_2 := 0;
        positie := [0,0,0];
        datanr_positie := [0,0,0];
        !
        startPos := CALC_COUNTER+(0.5*(POS_FILTER_SIZE+1));
        ! CALC_COUNTER + 0.5*22
        eindPos := nMovementCounter+CALC_COUNTER-(0.5*(POS_FILTER_SIZE+1));
        ! nMovementCounter + CALC_COUNTER - 0.5*22
        !
        IF eindPos<(startPos+POS_FILTER_SIZE) THEN
            TPWrite "StartError";
            stop;
        ENDIF
        !
        FOR idx FROM startPos TO eindPos DO
            incr aantal;
            datanr := datanr+idx;
            ! X
            datanr_2 := datanr_2+(pow(idx,2));
            ! X^2
            positie.x := positie.x+TargetNew{idx}.trans.x;
            ! Y
            positie.y := positie.y+TargetNew{idx}.trans.y;
            positie.z := positie.z+TargetNew{idx}.trans.z;
            datanr_positie.x := datanr_positie.x+(idx*TargetNew{idx}.trans.x);
            ! X*Y
            datanr_positie.y := datanr_positie.y+(idx*TargetNew{idx}.trans.y);
            datanr_positie.z := datanr_positie.z+(idx*TargetNew{idx}.trans.z);
        ENDFOR
        !
        a.x := ((aantal*datanr_positie.x)-(datanr*positie.x))/((aantal*datanr_2)-(pow(datanr,2)));
        b.x := ((positie.x*datanr_2)-(datanr*datanr_positie.x))/((aantal*datanr_2)-(pow(datanr,2)));
        a.y := ((aantal*datanr_positie.y)-(datanr*positie.y))/((aantal*datanr_2)-(pow(datanr,2)));
        b.y := ((positie.y*datanr_2)-(datanr*datanr_positie.y))/((aantal*datanr_2)-(pow(datanr,2)));
        a.z := ((aantal*datanr_positie.z)-(datanr*positie.z))/((aantal*datanr_2)-(pow(datanr,2)));
        b.z := ((positie.z*datanr_2)-(datanr*datanr_positie.z))/((aantal*datanr_2)-(pow(datanr,2)));
        !
        reg4 := (a.x*1)+b.x;
        reg5 := (a.y*1)+b.y;
        reg6 := (a.z*1)+b.z;
        !
        TargetNew{1}.trans := [reg4,reg5,reg6];
        !
        ! Voor alle voorliggende datapunten de theoretisch lineaire positie berekenen
        FOR i FROM 1 TO (nMovementCounter+CALC_COUNTER-1) DO
            TargetNew{i}.trans.x := (a.x*i)+b.x;
            TargetNew{i}.trans.y := (a.y*i)+b.y;
            TargetNew{i}.trans.z := (a.z*i)+b.z;
            IF (a.y*i)+b.y<0
                TargetNew{i}.trans.y := 0;
        ENDFOR
    ENDPROC

    PROC rLinearFilterRot(num startpos,num eindpos)
        ! Bereken de richting van de vectoren met behulp van een lineaire lijn door de datapunten
        VAR pos a := [0,0,0];        ! Y = a*x + b
        VAR pos b := [0,0,0];        ! Y = a*x + b
        VAR num datanr := 0;        ! som van datanr
        VAR num datanr_2 := 0;        ! som van datanr^2
        VAR pos positie := [0,0,0];        ! som van de posities [x, y, z]
        VAR pos datanr_positie := [0,0,0];        ! som van datanr*positie [x, y, z]
        VAR num aantal := 0;        !
        aantal := 0;
        datanr := 0;
        datanr_2 := 0;
        positie := [0,0,0];
        datanr_positie := [0,0,0];
        !
        FOR idx FROM 1 TO ORIENT_FILTER_SIZE DO
            incr aantal;
            datanr := datanr+idx;
            ! X
            datanr_2 := datanr_2+(pow(idx,2));
            ! X^2
            positie.x := positie.x+TargetNew{startPos-1+idx}.trans.x;
            ! Y
            positie.y := positie.y+TargetNew{startPos-1+idx}.trans.y;
            positie.z := positie.z+TargetNew{startPos-1+idx}.trans.z;
            datanr_positie.x := datanr_positie.x+(idx*TargetNew{startPos-1+idx}.trans.x);
            ! X*Y
            datanr_positie.y := datanr_positie.y+(idx*TargetNew{startPos-1+idx}.trans.y);
            datanr_positie.z := datanr_positie.z+(idx*TargetNew{startPos-1+idx}.trans.z);
        ENDFOR
        !
        a.x := ((ORIENT_FILTER_SIZE*datanr_positie.x)-(datanr*positie.x))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.x := ((positie.x*datanr_2)-(datanr*datanr_positie.x))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        a.y := ((ORIENT_FILTER_SIZE*datanr_positie.y)-(datanr*positie.y))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.y := ((positie.y*datanr_2)-(datanr*datanr_positie.y))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        a.z := ((ORIENT_FILTER_SIZE*datanr_positie.z)-(datanr*positie.z))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        b.z := ((positie.z*datanr_2)-(datanr*datanr_positie.z))/((ORIENT_FILTER_SIZE*datanr_2)-(pow(datanr,2)));
        !
        ! Bereken startpunt en eindpunt van de lineaire vector om de orientatie mee te berekenen
        ! y=ax+b
        OrientVectorStart.x := (a.x*1)+b.x;
        OrientVectorEind.x := (a.x*ORIENT_FILTER_SIZE)+b.x;
        OrientVectorStart.y := (a.y*1)+b.y;
        OrientVectorEind.y := (a.y*ORIENT_FILTER_SIZE)+b.y;
        OrientVectorStart.z := (a.z*1)+b.z;
        OrientVectorEind.z := (a.z*ORIENT_FILTER_SIZE)+b.z;
    ENDPROC

    PROC rCalcOrient()
        VAR num reg_vector_xyz := 0;
        ! actuele y-vector lengte
        !
        VAR pos y_tcp := [0,0,0];
        ! actuele y-vector genormaliseerd
        VAR pos x_tcp := [0,0,0];
        ! berekende x-vector
        VAR pos z_tcp := [0,0,0];
        ! berekende z-vector
        VAR pos z_g := [0,0,1];
        ! gewenste tool orientatie Z-as verticaal naar boven (global frame)
        !
        VAR num rX_Calc := 0;
        ! berekende rX benodigd [degrees]
        VAR num rY_Calc := 0;
        ! berekende rY benodigd [degrees]
        VAR num rZ_Calc := 0;
        ! berekende rZ benodigd [degrees]
        !
        VAR num startPos := 0;
        ! eerste data punt voor orient filter
        VAR num eindPos := 0;
        ! laatste data punt voor orient filter
        !
        nOrientFilterCounter := nPosFilterCounter-((ORIENT_FILTER_SIZE-1)*0.5);
        startPos := nOrientFilterCounter-((ORIENT_FILTER_SIZE-1)*0.5);
        eindPos := nOrientFilterCounter+((ORIENT_FILTER_SIZE-1)*0.5);
        !
        rLinearFilterRot(startPos),(eindPos);
        !
        reg_vector_xyz := sqrt(pow((OrientVectorEind.x-OrientVectorStart.x),2)+pow((OrientVectorEind.y-OrientVectorStart.y),2)+pow((OrientVectorEind.z-OrientVectorStart.z),2));
        !
        ! Genormaliseerde y-vector bepalen (tool richting)
        y_tcp.x := (OrientVectorEind.x-OrientVectorStart.x)/reg_vector_xyz;
        y_tcp.y := (OrientVectorEind.y-OrientVectorStart.y)/reg_vector_xyz;
        y_tcp.z := (OrientVectorEind.z-OrientVectorStart.z)/reg_vector_xyz;
        !
        ! kruisproduct van de vectoren om de vector haaks op het vlak te berekenen
        ! kruisproduct van y_tcp met z_g om de x_tcp te bepalen
        ! kruisproduct van x_tcp met y_tcp om de z_tcp te bepalen
        ! tesamen is het: Rbase_tcp_g = [x_tcp, y_tcp, z_tcp]
        x_tcp.x := ((y_tcp.y*z_g.z)-(z_g.y*y_tcp.z));
        x_tcp.y := -((y_tcp.x*z_g.z)-(z_g.x*y_tcp.z));
        x_tcp.z := ((y_tcp.x*z_g.y)-(z_g.x*y_tcp.y));
        z_tcp.x := ((x_tcp.y*y_tcp.z)-(y_tcp.y*x_tcp.z));
        z_tcp.y := -((x_tcp.x*y_tcp.z)-(y_tcp.x*x_tcp.z));
        z_tcp.z := ((x_tcp.x*y_tcp.y)-(y_tcp.x*x_tcp.y)); 
        !
        ! oplossing van de rotatiematrix Rbase_tcp_g
        ! Array: [x_tcp , y_tcp, z_tcp]
        rX_Calc := asin(y_tcp.z);
        IF z_tcp.z<>0 THEN
            rY_Calc := atan2(-x_tcp.z,z_tcp.z);
        ELSE
            rY_Calc := 0;
        ENDIF
        IF y_tcp.y<>0 THEN
            rZ_Calc := atan2(-y_tcp.x,y_tcp.y);
        ELSE
            rZ_Calc := 0;
        ENDIF
        
        TargetNew{nOrientFilterCounter}.rot := OrientZYX(rZ_Calc,rY_Calc,rX_Calc)*OrientZYX(0,Rtcpy_Offset,0);
        
        pnewCalc := [rZ_Calc,rY_Calc,rX_Calc];
    ENDPROC

PROC rTrackPosition()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Procedure for controlling axis 7 and the rotation around the y-tcp axis.
    !**********************************************************************************************************************************************
    
    Ncycle:= Ncycle+1;
    JointConf := CalcJointT(TargetNew{nMovementCounter+k},tInductionRing\WObj := wTempClamp);
    
    FutureRob := CalcRobT (JointConf, tInductionRing\WObj:=wClamp_NonRotate);  ! Compute robtarget of timestep k
    
    IF diCP2_Selected_SIM=1     posY := -FutureRob.trans.y + wClamp_NonRotate.uframe.trans.x + wClamp_NonRotate.oframe.trans.x;
    IF diCP2_Selected_SIM=0     posY := FutureRob.trans.y + wClamp_NonRotate.uframe.trans.x + wClamp_NonRotate.oframe.trans.x;
    !posY := -FutureRob.trans.y + wClamp_NonRotate.uframe.trans.x;
    
    rCompute_As6OptVal;  ! Compute the optimum value for axis 6      
    
    if Ncycle=1 THEN 
        ! Initialize the offset of the 7th axis wrt the tool
        JointConfStart := CalcJointT(TargetNew{1},tInductionRing\WObj := wTempClamp); 
        StartRob := CalcRobT (JointConfStart, tInductionRing\WObj:=wClamp_NonRotate);
        !posY_Offset:= TargetNew{1}.extax.eax_a + StartRob.trans.y - wClamp_NonRotate.uframe.trans.x; 
!        IF diCP2_Selected_SIM=0     posY_Offset:= TargetNew{1}.extax.eax_a - StartRob.trans.y + wClamp_NonRotate.uframe.trans.x; 
!        IF diCP2_Selected_SIM=1     posY_Offset:= TargetNew{1}.extax.eax_a - wClamp_NonRotate.uframe.trans.x - wClamp_NonRotate.oframe.trans.x; 
         posY_Offset:= TargetNew{1}.extax.eax_a - posY; 
        
        ! Initialize the maximum axes boundaries
       !cmaxult_as := [[-40+e_m1,90-e_m1],[-60+e_m1,90-e_m1],[-90+e_m1,70-e_m1],[-180+e_m1,180-e_m1],[0+e_m1,90-e_m1],[-350+e_m1,350-e_m1]];
        cmaxult_as := [[-40+e_m1,-35-e_m1],[-60+e_m1,30-e_m1],[-90+e_m1, 70-e_m1],[-300+e_m1,-60-e_m1],[0+e_m1,90-e_m1],[-150+e_m1,150-e_m1]];  ! Ultimum minimum and maximum axis angles per joint respectively in order to stop the robot        
        IF TargetNew{1}.robconf.cfx=1 THEN
            cmaxult_as{5,1} := -90+e_m1;  ! Ultimum minimum and maximum axis angles per joint respectively in order to stop the robot 
            cmaxult_as{5,2} :=  0-e_m1;
            cmaxult_as{4,1} := 60+e_m1;  
            cmaxult_as{4,2} := 300-e_m1;
        ENDIF
        JointC := CJointT();
        IF JointC.robax.rax_4>90 THEN ! As 4 lays around 180 degrees
            cmaxult_as{4,1} := 0+e_m1;  
            cmaxult_as{4,2} := 300-e_m1;
        ELSEIF JointC.robax.rax_4<-90 THEN ! As 4 lays around -180 degrees
            cmaxult_as{4,1} := -300+e_m1;  
            cmaxult_as{4,2} :=  0-e_m1;       
        ENDIF    
        
        cmax_as_const := [[cmaxult_as{1,1}+e_m2,cmaxult_as{1,2}-e_m2],
                          [cmaxult_as{2,1}+e_m2,cmaxult_as{2,2}-e_m2],
                          [cmaxult_as{3,1}+e_m2,cmaxult_as{3,2}-e_m2],
                          [cmaxult_as{4,1}+e_m2,cmaxult_as{4,2}-e_m2],
                          [cmaxult_as{5,1}+e_m2,cmaxult_as{5,2}-e_m2],
                          [cmaxult_as{6,1}+e_m2,cmaxult_as{6,2}-e_m2]];  ! Minimum and maximum axis angles per joint respectively in order to steer ax7 or ytcp
        cmax_as := cmax_as_const;  ! Variable version in order to apply hysteresis method   
    ENDIF        
    
    ! Compute new target following the tool plus offset for axis 7
    TargetNew{nMovementCounter+k+1}.extax.eax_a := posY + posY_Offset;
        
    TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0);
    
    IF Ncycle = 1 THEN ! Initialize the first k points of the 7th axis
        Axis7PartSize := (TargetNew{nMovementCounter+k+1}.extax.eax_a - TargetNew{nMovementCounter}.extax.eax_a) / (k+1);
        FOR i FROM 1 TO (k+1) DO
            TargetNew{i+1}.extax.eax_a := TargetNew{i}.extax.eax_a + Axis7PartSize;
        ENDFOR
    ENDIF
        
    ! Calculate axis angles on new track position
    JointConf1 := CalcJointT(TargetNew{nMovementCounter+k+1},tInductionRing\WObj := wTempClamp);
    As := [JointConf1.robax.rax_1, JointConf1.robax.rax_2, JointConf1.robax.rax_3, JointConf1.robax.rax_4,
           JointConf1.robax.rax_5, JointConf1.robax.rax_6, JointConf1.extax.eax_a]; ! Axes at time step (nMovementCounter+k+1)
    
    rPreventOutOfReach; ! Prevent outside reach error (Check the distance between wrist center and robot base)
    
!######################################################### Method 2: Intuitive rules for axis 7 ################################################################################
    IF As{1}>cmax_as{1,2}       AxisError{1} := abs(cmax_as{1,2}-As{1});
    IF As{1}<cmax_as{1,1}       AxisError{1} := abs(cmax_as{1,1}-As{1});
    IF As{2}>cmax_as{2,2}       AxisError{2} := abs(cmax_as{2,2}-As{2});
    IF As{2}<cmax_as{2,1}       AxisError{2} := abs(cmax_as{2,1}-As{2});
    IF As{3}>cmax_as{3,2}       AxisError{3} := abs(cmax_as{3,2}-As{3});
    IF As{3}<cmax_as{3,1}       AxisError{3} := abs(cmax_as{3,1}-As{3});
    IF As{4}>cmax_as{4,2}       AxisError{4} := abs(cmax_as{4,2}-As{4});
    IF As{4}<cmax_as{4,1}       AxisError{4} := abs(cmax_as{4,1}-As{4});
    IF As{5}>cmax_as{5,2}       AxisError{5} := abs(cmax_as{5,2}-As{5});
    IF As{5}<cmax_as{5,1}       AxisError{5} := abs(cmax_as{5,1}-As{5});
    IF As{6}>cmax_as{6,2}       AxisError{6} := abs(cmax_as{6,2}-As{6});
    IF As{6}<cmax_as{6,1}       AxisError{6} := abs(cmax_as{6,1}-As{6});
        
    rIntuitiveRulesAxis7;
    rIntuitiveRulesYtcp;
    
! Bounds As7=[0, 9180], dy=[-2, 2]    
    IF (TargetNew{nMovementCounter+k+1}.extax.eax_a-TargetNew{nMovementCounter+k}.extax.eax_a)>dy THEN  ! limit the maximum speed by dy
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k}.extax.eax_a+dy;
    ELSEIF (TargetNew{nMovementCounter+k+1}.extax.eax_a-TargetNew{nMovementCounter+k}.extax.eax_a)<-dy THEN
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k}.extax.eax_a-dy;
    ENDIF

    IF TargetNew{nMovementCounter+k+1}.extax.eax_a<0 TargetNew{nMovementCounter+k+1}.extax.eax_a := 0;  ! Axis 7 may not exceed 0 
    IF TargetNew{nMovementCounter+k+1}.extax.eax_a>9180 TargetNew{nMovementCounter+k+1}.extax.eax_a := 9180;    ! Axis 7 may not exceed 8600 
    rApplyHysteresis;         
ENDPROC
        
PROC rComputeErrorAxis1()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Compute relative error of axis 1 with respect to 0 degrees (error lays between 0 and 1)
    ! Subsequently this error is multiplied by the amount of mm axis 7 needs to rotate.
    ! The relative error is always positive, 1 when the absolute error is higher than 10 deg,
    ! linear declining to zero when between 2 and 10 deg and zero when lower than 2 deg.
    !**********************************************************************************************************************************************
    ErrorAs1 := 1;  
    IF abs(As{1})<10     ErrorAs1:=(abs(As{1}))/8-2/8;   
    IF abs(As{1})<2      ErrorAs1:=0;     
ENDPROC

PROC rComputeErrorAxis5()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Compute relative error of axis 5 with respect to 0 degrees (error lays between 0 and 1)
    ! Subsequently this error is multiplied by the amount of degrees ytcp needs to rotate.
    ! The relative error is always positive, 1 when the absolute error is higher than 10 deg,
    ! linear declining to zero when between 2 and 10 deg and zero when lower than 2 deg.
    !**********************************************************************************************************************************************
    ErrorAs5 := 1;  
    IF abs(As{5})<10     ErrorAs5:=(abs(As{5}))/8-2/8;   
    IF abs(As{5})<2      ErrorAs5:=0;     
ENDPROC

PROC rCompute_As6OptVal()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Determine the optimum value for axis 6 to rotate ytcp to when using logic for axis 5.
    !**********************************************************************************************************************************************
    IF As{6}<=180  AND As{6}>=0        As6OptVal := 90;
    IF As{6}<=-360 AND As{6}>=-180     As6OptVal := -270;
    IF As{6}<=0    AND As{6}>=-180     As6OptVal := -90;
    IF As{6}<=360  AND As{6}>=180      As6OptVal := 270;
ENDPROC

PROC  rComputeErrorAxis6()       
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Compute relative error of axis 6 with respect to 270 or 90, depending on cfx. (error lays between 0 and 1)
    ! Subsequently this error is multiplied by the amount of degrees ytcp needs to rotate.
    ! The relative error is always positive, 1 when the absolute error is higher than 10 deg,
    ! linear declining to zero when between 2 and 10 deg and zero when lower than 2 deg.   
!    !**********************************************************************************************************************************************
!    IF TargetNew{1}.robconf.cfx=0 THEN
!        ErrorAs6 := 1;   
!        IF abs(As6OptVal-As{6})<10     ErrorAs6:=(abs(As6OptVal-As{6}))/8-2/8;
!        IF abs(As6OptVal-As{6})<2      ErrorAs6:=0;
!    ELSEIF TargetNew{1}.robconf.cfx=1  THEN 
!        ErrorAs6 := 1;  
!        IF abs(As6OptVal-As{6})<10     ErrorAs6:=(abs(As6OptVal-As{6}))/8-2/8;   
!        IF abs(As6OptVal-As{6})<2      ErrorAs6:=0;     
!    ENDIF
    IF TargetNew{1}.robconf.cfx=0 THEN
        ErrorAs6 := 1;   
        IF abs(As6OptVal-As{6})<10     ErrorAs6:=(abs(As6OptVal-As{6}))/8-2/8;
        IF abs(As6OptVal-As{6})<2      ErrorAs6:=0;
    ELSEIF TargetNew{1}.robconf.cfx=1  THEN 
        ErrorAs6 := 1;  
        IF abs(As6OptVal-As{6})<10     ErrorAs6:=0;   
        IF abs(As6OptVal-As{6})<2      ErrorAs6:=0;     
    ENDIF
ENDPROC

PROC rCheckJointAxes()       
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Checks the 6 axes whether they are permissable. Else stop the robot motion and write error on pendant.       
    !*********************************************************************************************************************************************    
    VAR string str1;
    VAR string str2;
    FOR i FROM 1 TO 6 DO
        IF As{i} <= cmaxult_as{i,1} OR As{i} >= cmaxult_as{i,2} THEN
            TPErase;
            str1 := NumToStr(i,0);
            str2 := NumToStr(As{i},0);
            TPWrite "Axis " + str1 + " is out of bounds.";
            TPWrite "Axis Angle = " + str2;
            Stop;
            WHILE TRUE DO 
                WaitTime 0.2; 
            ENDWHILE
        ENDIF
    ENDFOR
ENDPROC

PROC rApplyHysteresis()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Narrow the bounds of the ith axis when they are exceeded, reset them again when the hysteresis value (ExtraError) is met.       
    !**********************************************************************************************************************************************
    ! Do (not) apply Hysteresis for 6 joints
    FOR i FROM 1 TO 6 DO
        IF ErrorAs{i}=TRUE AND (As{i}<=cmax_as{i,1} OR As{i}>=cmax_as{i,2}) THEN 
            cmax_as{i,1} := cmax_as_const{i,1}+ErrorHyst;
            cmax_as{i,2} := cmax_as_const{i,2}-ErrorHyst;            
            posY_Offset := TargetNew{nMovementCounter+k+1}.extax.eax_a - posY;
        ELSEIF ErrorAs{i}=TRUE AND (As{i}>cmax_as{i,1} AND As{i}<cmax_as{i,2}) THEN
            ErrorAs{i} := FALSE;
            cmax_as{i,1} := cmax_as_const{i,1};
            cmax_as{i,2} := cmax_as_const{i,2};
        ENDIF    
    ENDFOR
    
    IF Error_OOR=TRUE AND JointConf1.robax.rax_3<=As3MaxOOR THEN 
        As3MaxOOR := As3cMaxOOR - 10;            
        posY_Offset := TargetNew{nMovementCounter+k+1}.extax.eax_a - posY;
    ELSEIF Error_OOR=TRUE AND JointConf1.robax.rax_3>As3MaxOOR THEN 
        Error_OOR := FALSE;
        As3MaxOOR := As3cMaxOOR;     
    ENDIF 
ENDPROC

PROC rCalc_Joint_Target_For_MHS()
    !***********************************************************
    ! Procedure: rCalc_Joint_Target_For_MHS
    ! Machine: Destrapper
    ! Description:
    ! - calculate the required robot axix for display on the HMI
    !   when the robot ain't able to return home automaticly
    !***********************************************************
    VAR jointtarget jointpos1;
    !
    rLog_Event_Message_TS000_3\Message := "PROC rCalc_Joint_Target_For_MHS()";
    !
    jointpos1 := jRobHomePos;
    !
    nHMI_Required_Angle_Axis{1} := Round(jointpos1.robax.rax_1);
    nHMI_Required_Angle_Axis{2} := Round(jointpos1.robax.rax_2);
    nHMI_Required_Angle_Axis{3} := Round(jointpos1.robax.rax_3);
    nHMI_Required_Angle_Axis{4} := Round(jointpos1.robax.rax_4);
    nHMI_Required_Angle_Axis{5} := Round(jointpos1.robax.rax_5);
    nHMI_Required_Angle_Axis{6} := Round(jointpos1.robax.rax_6);
ENDPROC

PROC rPreventOutOfReach()
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Prevent outside reach error.
    ! Use axis 3 to determine whether the robot is getting out of reach. 
    ! Bring axis 7 towards tcp position and rotate ytcp to the optimal orientation when this distance is higher than allowed.
    !**********************************************************************************************************************************************
    
    JointConf1 := CalcJointT(TargetNew{nMovementCounter+k+1},tInductionRing\WObj := wTempClamp);
    IF JointConf1.robax.rax_3<As3MaxOOR THEN
        As3ErrorOOR := abs(As3MaxOOR-JointConf1.robax.rax_3); ! Compute error of the difference between the threshold and the current angle of axis 3
        Error_OOR := TRUE;
        OOR_Counter := OOR_Counter+1;
        IF (As{7}-posY) > 0 THEN
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - As3ErrorOOR;
            doErrorTrans{7,1} := doErrorTrans{7,1}+1;    
        ENDIF
        IF (As{7}-posY) < 0 THEN
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + As3ErrorOOR;
            doErrorTrans{7,2} := doErrorTrans{7,2}+1;    
        ENDIF
        ! Rotate y_tcp to the optimal orientation
        rComputeErrorAxis6;       
        IF TargetNew{1}.robconf.cfx=0 THEN
            IF As{6} >= As6OptVal THEN 
                Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*min(p_wrist_r_gain*As3ErrorOOR, dRtcpy);
                TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*min(p_wrist_r_gain*As3ErrorOOR, dRtcpy));
                doErrorRot{7,1} := doErrorRot{7,1}+1;      
            ELSEIF As{6} < As6OptVal THEN       
                Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*max(-p_wrist_r_gain*As3ErrorOOR, -dRtcpy);
                TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*max(-p_wrist_r_gain*As3ErrorOOR, -dRtcpy));
                doErrorRot{7,2} := doErrorRot{7,2}+1;           
            ENDIF
        ELSEIF TargetNew{1}.robconf.cfx=1 THEN
            IF  abs(As{6}) >= abs(As6OptVal) THEN    
                Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*min(p_wrist_r_gain*As3ErrorOOR, dRtcpy);
                TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*min(p_wrist_r_gain*As3ErrorOOR, dRtcpy));
                doErrorRot{7,3} := doErrorRot{7,3}+1; 
            ELSEIF abs(As{6}) < abs(As6OptVal) THEN   
                Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*max(-p_wrist_r_gain*As3ErrorOOR, -dRtcpy);
                TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*max(-p_wrist_r_gain*As3ErrorOOR, -dRtcpy));
                doErrorRot{7,4} := doErrorRot{7,4}+1;                        
            ENDIF
        ENDIF           
        rApplyHysteresis;  
    ENDIF            
ENDPROC

PROC rIntuitiveRulesAxis7()
    
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Look per joint angle whether it lays within the bounds.
    ! If not, control the 7th axis towards the right direction.
    ! The amount of movement of axis 7 is determined by
    ! - Gain of specific axis
    ! - The error of the joint
    ! - Robot configuration cfx 
    !**********************************************************************************************************************************************
    
    ! Joint angle 1         
    IF As{1} > cmax_as{1,2}  THEN
        ErrorAs{1} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{1}*AxisError{1};
        doErrorTrans{1,1} := doErrorTrans{1,1} + 1;
    ELSEIF As{1} < cmax_as{1,1} THEN
        ErrorAs{1} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{1}*AxisError{1};
        doErrorTrans{1,2} := doErrorTrans{1,2} + 1;
    ENDIF

    ! Joint angle 2
    IF As{2} > cmax_as{2,2} AND (As{7}-posY) >= 0 THEN
        ErrorAs{2} := TRUE;
        rComputeErrorAxis1;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - ErrorAs1*pgain{2}*AxisError{2};
        doErrorTrans{2,1} := doErrorTrans{2,1} + 1;
    ELSEIF As{2} > cmax_as{2,2} AND (As{7}-posY) < 0 THEN
        ErrorAs{2} := TRUE;   
        rComputeErrorAxis1;            
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + ErrorAs1*pgain{2}*AxisError{2};
        doErrorTrans{2,2} := doErrorTrans{2,2} + 1;
    ENDIF    
    IF As{2} < cmax_as{2,1} AND (As{7}-posY) >= 0 THEN
        ErrorAs{2} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{2}*AxisError{2};
        doErrorTrans{2,3} := doErrorTrans{2,3} + 1;
    ELSEIF As{2} < cmax_as{2,1} AND (As{7}-posY) < 0 THEN
        ErrorAs{2} := TRUE;               
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{2}*AxisError{2};
        doErrorTrans{2,4} := doErrorTrans{2,4} + 1;
    ENDIF

    ! Joint angle 3 
    IF As{3} < cmax_as{3,1} AND (As{7}-posY) >= 0 THEN
        ErrorAs{3} := TRUE;
        rComputeErrorAxis1;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - ErrorAs1*pgain{3}*AxisError{3};
        doErrorTrans{3,1} := doErrorTrans{3,1} + 1;
    ELSEIF As{3} < cmax_as{3,1} AND (As{7}-posY) < 0 THEN
        ErrorAs{3} := TRUE;
        rComputeErrorAxis1;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + ErrorAs1*pgain{3}*AxisError{3};
        doErrorTrans{3,2} := doErrorTrans{3,2} + 1;
    ENDIF
    IF As{3} > cmax_as{3,2} AND (As{7}-posY) >= 0 THEN
        ErrorAs{3} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{3}*AxisError{3};
        doErrorTrans{3,3} := doErrorTrans{3,3} + 1;
    ELSEIF As{3} > cmax_as{3,2} AND (As{7}-posY) < 0 THEN
        ErrorAs{3} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{3}*AxisError{3};
        doErrorTrans{3,4} := doErrorTrans{3,4} + 1;
    ENDIF
    
    ! Joint angle 4
    IF As{4} < cmax_as{4,1} THEN
        ErrorAs{4} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{4}*AxisError{4};
        doErrorTrans{4,1} := doErrorTrans{4,1} + 1;
    ENDIF 
    IF As{4} > cmax_as{4,2} THEN
        ErrorAs{4} := TRUE;
        TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{4}*AxisError{4};
        doErrorTrans{4,2} := doErrorTrans{4,2} + 1;
    ENDIF

    ! Joint angle 5 
    IF TargetNew{1}.robconf.cfx=0 THEN  ! start quadrant as 5 [0,90]
        IF As{5} > cmax_as{5,2} AND (As{7}-posY) >= 0 THEN
            ErrorAs{5} := TRUE;
            rComputeErrorAxis1;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - ErrorAs1*pgain{5}*AxisError{5};
            doErrorTrans{5,1}:= doErrorTrans{5,1} + 1;
        ELSEIF As{5} > cmax_as{5,2} AND (As{7}-posY) < 0 THEN
            ErrorAs{5} := TRUE;
            rComputeErrorAxis1; 
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + ErrorAs1*pgain{5}*AxisError{5};
            doErrorTrans{5,2}:= doErrorTrans{5,2} + 1;
        ENDIF
        IF As{5} < cmax_as{5,1} AND (As{7}-posY) >= 0 THEN
            ErrorAs{5} := TRUE;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{5}*AxisError{5};
            doErrorTrans{5,3}:= doErrorTrans{5,3} + 1;
        ELSEIF As{5} < cmax_as{5,1} AND (As{7}-posY) < 0 THEN
            ErrorAs{5} := TRUE;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{5}*AxisError{5};
            doErrorTrans{5,4}:= doErrorTrans{5,4} + 1;
        ENDIF
    ELSEIF TargetNew{1}.robconf.cfx=1 THEN   ! start quadrant as 5 [-90, 0]
        IF As{5} <= cmax_as{5,1} AND (As{7}-posY) >= 0 THEN
            ErrorAs{5} := TRUE;
            rComputeErrorAxis1;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - ErrorAs1*pgain{5}*AxisError{5};
            doErrorTrans{5,1}:= doErrorTrans{5,1} + 1;
        ELSEIF As{5} <= cmax_as{5,1} AND (As{7}-posY) < 0 THEN
            ErrorAs{5} := TRUE;
            rComputeErrorAxis1; 
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + ErrorAs1*pgain{5}*AxisError{5};
            doErrorTrans{5,2}:= doErrorTrans{5,2} + 1;
        ENDIF
        IF As{5} >= cmax_as{5,2} AND (As{7}-posY) >= 0 THEN
            ErrorAs{5} := TRUE;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a + pgain{5}*AxisError{5};
            doErrorTrans{5,3}:= doErrorTrans{5,3} + 1;
        ELSEIF As{5} >= cmax_as{5,2} AND (As{7}-posY) < 0 THEN
            ErrorAs{5} := TRUE;
            TargetNew{nMovementCounter+k+1}.extax.eax_a := TargetNew{nMovementCounter+k+1}.extax.eax_a - pgain{5}*AxisError{5};
            doErrorTrans{5,4}:= doErrorTrans{5,4} + 1;
        ENDIF
    ENDIF     
    
ENDPROC


PROC rIntuitiveRulesYtcp()    
    !**********************************************************************************************************************************************
    ! Date:        24-06-2022
    ! Author:      Jelle Kelbling
    ! Description: Look per joint angle whether it lays within the bounds.
    ! If not, control the orientation around the y-axis of the tcp towards the right direction.
    ! The amount of rotation of y-tcp is determined by
    ! - Gain of specific axis
    ! - The error of the joint, error of axis 5 and/or axis 6
    ! - Robot configuration cfx 
    ! - Maximum bound of the rotation
    !**********************************************************************************************************************************************
    
!######################################################### Method 2: Intuitive rules for rotating y-tcp ################################################################################
!     Joint angle 2: 
!     When As2 becomes too large, the tool should rotate around the tube such that As2 has optimal moving space
!     As2 has optimal moving space when As6OptVal is reached, accounting for As5 as well.
    IF TargetNew{1}.robconf.cfx=1 THEN
    	IF (As{2} > cmax_as{2,2} AND abs(As{6}) >= abs(As6OptVal)) OR (As{2} < cmax_as{2,1} AND abs(As{6}) < abs(As6OptVal)) THEN 
    	   ErrorAs{2} := TRUE;
           rComputeErrorAxis5; rComputeErrorAxis6;
           Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*ErrorAs5*min(rgain{2}*abs(AxisError{2}), dRtcpy);
           TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry :=  ErrorAs6*ErrorAs5*min(rgain{2}*abs(AxisError{2}), dRtcpy)); 
    	   doErrorRot{2,1}:= doErrorRot{2,1} + 1;
    	ENDIF  
    	IF (As{2} > cmax_as{2,2} AND abs(As{6}) < abs(As6OptVal)) OR (As{2} < cmax_as{2,1} AND abs(As{6}) >= abs(As6OptVal)) THEN 
    	   ErrorAs{2} := TRUE;
           rComputeErrorAxis5; rComputeErrorAxis6;
    	   Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*ErrorAs5*max(-rgain{2}*abs(AxisError{2}), -dRtcpy);
    	   TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*ErrorAs5*max(-rgain{2}*abs(AxisError{2}), -dRtcpy)); 
    	   doErrorRot{2,2}:= doErrorRot{2,2} + 1;
    	ENDIF   
!    ELSEIF TargetNew{1}.robconf.cfx=0 THEN
!    	IF (As{2} > cmax_as{2,2} AND As{6} >= As6OptVal) OR (As{2} < cmax_as{2,1} AND As{6} < As6OptVal) THEN 
!    	   ErrorAs{2} := TRUE;
!           rComputeErrorAxis5; rComputeErrorAxis6;
!    	   Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*ErrorAs5*max(-rgain{2}*abs(AxisError{2}), -dRtcpy);
!    	   TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*ErrorAs5*max(-rgain{2}*abs(AxisError{2}), -dRtcpy)); 
!    	   doErrorRot{2,3}:= doErrorRot{2,3} + 1;
!    	ENDIF  
!    	IF (As{2} > cmax_as{2,2} AND As{6} < As6OptVal) OR (As{2} < cmax_as{2,1} AND As{6} >= As6OptVal) THEN 
!    	   ErrorAs{2} := TRUE;
!           rComputeErrorAxis5; rComputeErrorAxis6;
!           Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*ErrorAs5*min(rgain{2}*abs(AxisError{2}), dRtcpy);
!           TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry :=  ErrorAs6*ErrorAs5*min(rgain{2}*abs(AxisError{2}), dRtcpy)); 
!    	   doErrorRot{2,4}:= doErrorRot{2,4} + 1;
!    	ENDIF   
    ENDIF
    
!     Joint angle 3: 
!     When As3 becomes too large, the tool should rotate around the tube such that As3 has optimal moving space
!     As3 has optimal moving space when As6OptVal is reached, accounting for As5 as well.
	IF (As{3} < cmax_as{3,1} AND As{6} < As6OptVal) OR (As{3} > cmax_as{3,2} AND As{6} >= As6OptVal) THEN 
	   ErrorAs{3} := TRUE;
       rComputeErrorAxis5; rComputeErrorAxis6;
	   Rtcpy_Offset := Rtcpy_Offset  +ErrorAs6*ErrorAs5*max(-rgain{3}*abs(AxisError{3}), -dRtcpy);
	   TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*ErrorAs5*max(-rgain{3}*abs(AxisError{3}), -dRtcpy)); 
	   doErrorRot{3,1}:= doErrorRot{3,1} + 1;
	ENDIF  
	IF (As{3} < cmax_as{3,1} AND As{6} >= As6OptVal) OR (As{3} > cmax_as{3,2} AND As{6} < As6OptVal) THEN 
	   ErrorAs{3} := TRUE;
       rComputeErrorAxis5; rComputeErrorAxis6;
       Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*ErrorAs5*min(rgain{3}*abs(AxisError{3}), dRtcpy);
       TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry :=  ErrorAs6*ErrorAs5*min(rgain{3}*abs(AxisError{3}), dRtcpy)); 
	   doErrorRot{3,2}:= doErrorRot{3,2} + 1;
	ENDIF   
    
    !Joint 4
    IF  As{4} < cmax_as{4,1} OR As{4} > cmax_as{4,2} THEN
        IF TargetNew{1}.robconf.cfx=1 THEN
            ErrorAs{4} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + min(rgain{4}*abs(AxisError{4}), dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := min(rgain{4}*abs(AxisError{4}), dRtcpy));
            doErrorRot{4,1} := doErrorRot{4,1}+1; 
        ELSEIF TargetNew{1}.robconf.cfx=0 THEN
            ErrorAs{4} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + max(-rgain{4}*abs(AxisError{4}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := max(-rgain{4}*abs(AxisError{4}), -dRtcpy));
            doErrorRot{4,2} := doErrorRot{4,2}+1;
        ENDIF
    ENDIF
            
    ! Joint 5
    IF TargetNew{1}.robconf.cfx=0 THEN
        IF  As{5} < cmax_as{5,1} AND abs(As{6}) >= abs(As6OptVal) THEN ! Optimum/Minimum is located at ax6=As6OptVal degrees. 
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + max(-rgain{4}*abs(AxisError{4}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := max(-rgain{4}*abs(AxisError{4}), -dRtcpy)); 
            doErrorRot{5,1} := doErrorRot{5,1}+1;            
        ELSEIF  As{5} < cmax_as{5,1} AND abs(As{6}) < abs(As6OptVal) THEN              
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + min(rgain{5}*abs(AxisError{5}), dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := min(rgain{5}*abs(AxisError{5}), dRtcpy)); 
            doErrorRot{5,2} := doErrorRot{5,2}+1;   
        ENDIF       
        IF  As{5} > cmax_as{5,2} AND abs(As{6}) >= abs(As6OptVal) THEN 
            rComputeErrorAxis6; 
            ErrorAs{5} := TRUE;        
            Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*min(rgain{5}*abs(AxisError{5}), dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*min(rgain{5}*abs(AxisError{5}), dRtcpy));
            doErrorRot{5,3} := doErrorRot{5,3}+1;      
        ELSEIF  As{5} > cmax_as{5,2} AND abs(As{6}) < abs(As6OptVal) THEN            
            rComputeErrorAxis6; 
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*max(-rgain{5}*abs(AxisError{5}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*max(-rgain{5}*abs(AxisError{5}), -dRtcpy));
            doErrorRot{5,4} := doErrorRot{5,4}+1;           
        ENDIF
    ELSEIF TargetNew{1}.robconf.cfx=1 THEN
        IF  As{5} > cmax_as{5,2} AND abs(As{6}) >= abs(As6OptVal) THEN
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + max(-rgain{5}*abs(AxisError{5}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := max(-rgain{5}*abs(AxisError{5}), -dRtcpy));  
            doErrorRot{5,1} := doErrorRot{5,1}+1;            
        ELSEIF  As{5} > cmax_as{5,2} AND abs(As{6}) < abs(As6OptVal) THEN              
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + min(rgain{5}*abs(AxisError{5}), dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := min(rgain{5}*abs(AxisError{5}), dRtcpy));
            doErrorRot{5,2} := doErrorRot{5,2}+1;     
        ENDIF          
        IF  As{5} < cmax_as{5,1} AND abs(As{6}) >= abs(As6OptVal) THEN
            rComputeErrorAxis6; 
            ErrorAs{5} := TRUE;             
            Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*min(rgain{5}*abs(AxisError{5}), dRtcpy);
            doErrorRot{5,3} := doErrorRot{5,3}+1; 
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*min(rgain{5}*abs(AxisError{5}), dRtcpy));
        ELSEIF  As{5} < cmax_as{5,1} AND abs(As{6}) < abs(As6OptVal) THEN            
            rComputeErrorAxis6;  
            ErrorAs{5} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + ErrorAs6*max(-rgain{5}*abs(AxisError{5}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := ErrorAs6*max(-rgain{5}*abs(AxisError{5}), -dRtcpy));
            doErrorRot{5,4} := doErrorRot{5,4}+1;           
        ENDIF
    ENDIF
    
    !Joint 6
    IF  As{6} < cmax_as{6,1} OR As{6} > cmax_as{6,2} THEN
        IF TargetNew{1}.robconf.cfx=1 THEN
            ErrorAs{6} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + min(rgain{6}*abs(AxisError{6}), dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := min(rgain{6}*abs(AxisError{6}), dRtcpy));
            doErrorRot{6,1} := doErrorRot{6,1}+1; 
        ELSEIF TargetNew{1}.robconf.cfx=0 THEN
            ErrorAs{6} := TRUE;
            Rtcpy_Offset := Rtcpy_Offset + max(-rgain{6}*abs(AxisError{6}), -dRtcpy);
            TargetNew{nMovementCounter+k+1} := RelTool(TargetNew{nMovementCounter+k+1}, 0, 0, 0, \Ry := max(-rgain{6}*abs(AxisError{6}), -dRtcpy));
            doErrorRot{6,2} := doErrorRot{6,2}+1; 
        ENDIF
    ENDIF
    
    !Bound the maximal rotation around the tool between 45 and 90 degrees depending on cfx
    IF TargetNew{1}.robconf.cfx=1 THEN
        IF Rtcpy_Offset<-45   Rtcpy_Offset:=-45;
        IF Rtcpy_Offset>90   Rtcpy_Offset:=90;
    ELSEIF TargetNew{1}.robconf.cfx=0 THEN
        IF Rtcpy_Offset>45   Rtcpy_Offset:=45;
        IF Rtcpy_Offset<=-90   Rtcpy_Offset:=-90;
    ENDIF
        
ENDPROC


ENDMODULE
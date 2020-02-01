SET phase TO -1.

function launch_circ {
    if not hasnode {
        set m_time to time:seconds + eta:apoapsis.
        set v0 to velocityat(ship,m_time):orbit:mag.
        set v1 to sqrt(body:mu/(body:radius + apoapsis)).
        set circularize to node(m_time, 0, 0, v1 - v0).
        add circularize.
    }
    lock steering to circularize:deltav:direction.
    lock max_acc to ship:maxthrust / ship:mass.
    lock burn_duration to circularize:deltav:mag/max_acc.

    WARPTO(circularize:eta - burn_duration).
    wait until circularize:eta <= burn_duration.
    lock throttle to 1.

    wait until circularize:deltav:mag < 1.
    lock throttle to 0.

    remove circularize.
}

function ish{
  PARAMETER a.
  PARAMETER b.
  PARAMETER ishyness.

  RETURN a-ishyness < b AND a+ishyness > b.
}

set Targetbody to duna.

// Ttransfer = 2 * pi SQRT ((rtransfer)^3/(mu sun))
set radius1 to body:orbit:semimajoraxis.
set radius2 to Targetbody:orbit:semimajoraxis.
set newmu   to  ship:body:body:mu.                        // GM of the planet you are orbiting's parent
set radius3 to ((radius1 + radius2)/2).                   // transfer semimajoraxis
set Ttrans  to  2* constant:pi * SQRT((radius3^3)/(newmu)).  // Time of transfer orbit

// phase angle = 180 - SQRT(mu/targetradius) * (Ttrans/targetradius) * 180/pi
// mu is the mu of the sun in most cases. targetradius is distance planet and sun.
set PrePhaseAngle to ((.5 * ttrans)/Targetbody:orbit:period) *360.
set phaseangle to (180 - PrePhaseAngle). // this is the phase angle we need between the two planets

// SOI exit velocity = SQRT(mu/currentRadius) * (SQRT((2*targetradius)/(targetradius + currentradius)) - 1)
// mu is the mu of the sun in most cases. targetradius is distance planet and sun. currentradius is distance current planet and sun.
// SOI exit velocity is the speed we will be going at the very edge of our body's SOI.
set SOIvel to SQRT(newmu/radius1) * (SQRT((2 * radius2)/(radius1 + radius2)) -1).


// ejection velocity = (currentheight * (SOIradius * SOIexit^2 - 2*mu) + 2 * SOIradius * mu)
// !! mu is now mu of the planet you're orbiting (exmpl Kerbin). currentheight is now height above center of orbiting planet.
// SOIradius is radius of SOI.
// ejection velocity = amount of delta v we need to burn to reach our target.

set curheight to Body:radius + ship:altitude.
set SOIradius to body:SOIradius.
set bodymu to body:mu.
set ejectvel to SQRT((curheight * (SOIradius * SOIvel^2 -2*bodymu) + 2 * SOIradius * bodymu)/(curheight * SOIradius)).


set epsilon to (((ejectvel^2)/2)-(bodymu/curheight)).
set heightx to (curheight * ejectvel).
set ecc to SQRT(1 + (2*epsilon*heightx^2)/(bodymu^2)).
set theta to ARCCOS(1/ecc).
set ejectangle to 180- theta.
// angle in orbit we need to burn at.


set oldangle to vang(body:position-sun:position,targetBody:position-sun:position).
WARPTO(time:seconds + 100).
wait until ship:unpacked.
set newangle to vang(body:position-sun:position,targetBody:position-sun:position).
set anglepersec to ((newangle - oldangle)/100).
set onedegree to (1/(ABS(anglepersec))).

set curangle to vang(body:position-sun:position,targetBody:position-sun:position).

function print_trtansit_info{
    print "Degrees per second:   " +  anglepersec.
    print "One degree takes:      " + onedegree.
    print "Current angle:         " + curangle.
    print "current time:          " + TIME:SECONDS.
    print "seconds till angle:    " + ABS((curangle -PhaseAngle)/anglepersec).
    print "seconds till angle UT: " + (TIME:SECONDS + ABS((curangle -PhaseAngle)/anglepersec)).
    print "-----------------------------------".
    Print "Phase angle needed:    " + PhaseAngle.
    print "-----------------------------------".
    Print "Ejection angle needed: " + ejectangle.
    Print "Dv needed:             " + SOIvel.
    Print "Ejection velocity:     " + ejectvel.
    print "Eject angle:           " + ejectangle.
}.

wait 2.

SET phase TO 0.


// MISSION CODE

// MISSION CODE

// MISSION CODE

// MISSION CODE

// MISSION CODE




wait until phase = 0.

print_trtansit_info().
print "wait input phase angle".

CLEARSCREEN.

WARPTO(time:seconds +  ABS((curangle -PhaseAngle)/anglepersec)).
wait until ship:unpacked.

SET phase TO 1.




wait until phase = 1.

//Next, we'll lock our throttle to 100%.
LOCK THROTTLE TO 0.8.   // 1.0 is the max, 0.0 is idle.

//This is our countdown loop, which cycles from 10 to 0
PRINT "Counting down:".
FROM {local countdown is 3.} UNTIL countdown = 0 STEP {SET countdown to countdown - 1.} DO {
    PRINT "..." + countdown.
    WAIT 1. // pauses the script here for 1 second.
}

STAGE.

//This is a trigger that constantly checks to see if our thrust is zero.
//If it is, it will attempt to stage and then return to where the script
//left off. The PRESERVE keyword keeps the trigger active even after it
//has been triggered.
WHEN MAXTHRUST = 0 THEN {
    PRINT "Staging".
    STAGE.
    PRESERVE.
}.

SET MYSTEER TO HEADING(90,90).
LOCK STEERING TO MYSTEER. // from now on we'll be able to change steering by just assigning a new value to MYSTEER
UNTIL SHIP:APOAPSIS > 100000 {
    wait 1.

    print (SHIP:VELOCITY:SURFACE:MAG / 300).
    print 90 / (SHIP:VELOCITY:SURFACE:MAG / 300).

    if ((SHIP:VELOCITY:SURFACE:MAG / 300) > 0.9) {
        SET MYSTEER TO HEADING(90, 90 / (SHIP:VELOCITY:SURFACE:MAG / 300)).
    }

}.

PRINT "100km apoapsis reached, cutting throttle".

LOCK THROTTLE TO 0.

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

SET phase to 2.




wait until phase = 2.

launch_circ().

SET phase TO 3.




wait until phase = 3.

set ejection to vang(TargetBody:velocity:orbit, ship:body:position).
set proret to 999. // basically a null value but will activate until loop

until ProRet = 1 and ish(ejection, ejectangle, 0.5) {
    set oldejection to vang(TargetBody:velocity:orbit, ship:body:position).
    wait 0.
    set ejection to vang(TargetBody:velocity:orbit, ship:body:position).

    If oldejection > ejection { // if decreasing you come closer to the planets prograde
        print "Angle to prograde: " + ejection.
        set ProRet to 1.
    }

    If ejection > oldejection {
        print "Angle to retrograde: " + (180 - ejection).
        set ProRet to 0.
    }

    wait 1.
}

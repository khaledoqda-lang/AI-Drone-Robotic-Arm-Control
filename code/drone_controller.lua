
sim=require'sim'

function sysCall_init() 
    particlesAreVisible=true
    simulateParticles=true
    fakeShadow=true
    
    particleCountPerSecond=430
    particleSize=0.005
    particleDensity=8500
    particleScatteringAngle=30
    particleLifeTime=0.5
    maxParticleCount=50

    -- Detatch the manipulation sphere:
    targetObj=sim.getObject('../target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
    d=sim.getObject('../base')

    propellerHandles={}
    jointHandles={}
    particleObjects={-1,-1,-1,-1}
    local ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
    if not particlesAreVisible then
        ttype=ttype+sim.particle_invisible
    end
    for i=1,4,1 do
        propellerHandles[i]=sim.getObject('../propeller['..(i-1)..']/respondable')
        jointHandles[i]=sim.getObject('../propeller['..(i-1)..']/joint')
        if simulateParticles then
            particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,{2,1,0.2,3,0.4},particleLifeTime,maxParticleCount,{0.3,0.7,1})
        end
    end
    heli=sim.getObject('..')

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0


    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpts+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
end 

function sysCall_actuation() 
    pos=sim.getObjectPosition(d)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,0,1,0.2}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj)
    pos=sim.getObjectPosition(d)
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.45+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    
    -- Decide of the motor velocities:
    handlePropeller(1,thrust*(1-alphaCorr+betaCorr+rotCorr))
    handlePropeller(2,thrust*(1-alphaCorr-betaCorr-rotCorr))
    handlePropeller(3,thrust*(1+alphaCorr-betaCorr+rotCorr))
    handlePropeller(4,thrust*(1+alphaCorr+betaCorr-rotCorr))
end 


function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x=(math.random()-0.5)*2
        y=(math.random()-0.5)*2
        if (x*x+y*y<=1) then
            if (simulateParticles) then
                pos[1]=x*0.08
                pos[2]=y*0.08
                pos[3]=-particleSize*0.6
                dir[1]=pos[1]+(math.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]+(math.random()-0.5)*maxParticleDeviation*2
                dir[3]=pos[3]-particleVelocity*(1+0.2*(math.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData={pos[1],pos[2],pos[3],dir[1],dir[2],dir[3]}
                sim.addParticleObjectItem(particleObject,itemData)
            end
            particleCnt=particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    torque={0,0,rotDir*0.002*particleVelocity}
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
end
sim=require'sim'

function _computeAnglesFromPosition(absolutePosition,verticalOffset,radialOffset)
    local voff=0
    local rOff=0
    if verticalOffset then
        voff=verticalOffset
    end
    if radialOffset then
        rOff=radialOffset
    end
    local m=sim.getObjectMatrix(motorHandles[1])
    m=sim.getMatrixInverse(m)
    local p=sim.multiplyVector(m,absolutePosition)
    local jointAngles={0,0,0,0}
    jointAngles[1]=math.atan2(p[2],p[1])+math.pi
    if jointAngles[1]>math.pi*3/2 then
        jointAngles[1]=jointAngles[1]-math.pi*2
    end
    local v=p[3]-0.0895+0.0592+voff
    local r=math.sqrt(p[1]*p[1]+p[2]*p[2])-0.0204-0.0346+rOff
    local h=math.sqrt(r*r+v*v)
    local l1=0.148
    local l2=0.16
    if math.abs((l1*l1+l2*l2-(h*h))/(2*l1*l2))>1 then
        return nil
    end
    local t2=math.pi-math.acos((l1*l1+l2*l2-(h*h))/(2*l1*l2))
    if math.abs((l1*l1+h*h-l2*l2)/(2*l1*h))>1 then
        return nil
    end
    local al=math.acos((l1*l1+h*h-l2*l2)/(2*l1*h))
    local t1=math.atan2(v,r)+al
    t2=0.6484-(t1-t2)
    t1=t1+0.281
    jointAngles[2]=t1
    jointAngles[3]=t2
    return jointAngles
end

function computeAnglesFromSuctionCupPosition(absolutePosition,verticalOffset,radialOffset)
    return _computeAnglesFromPosition(absolutePosition,verticalOffset,radialOffset)
end

function computeAnglesFromGripperPosition(absolutePosition,verticalOffset,radialOffset)
    return _computeAnglesFromPosition(absolutePosition,verticalOffset-0.039,radialOffset-0.048)
end

function initialize(maxVel,maxAccel,maxJerk,maxTorque)
    motorHandles={-1,-1,-1,-1}
    for i=1,4,1 do
        motorHandles[i]=sim.getObject('../motor'..i)
        sim.setJointTargetForce(motorHandles[i],maxTorque)
        sim.setObjectFloatParam(motorHandles[i],sim.jointfloatparam_maxvel,maxVel)
    end
    auxMotor1=sim.getObject('../auxMotor1')
    auxMotor2=sim.getObject('../auxMotor2')

    maxVelVect={maxVel,maxVel,maxVel,maxVel}
    maxAccelVect={maxAccel,maxAccel,maxAccel,maxAccel}
    maxJerkVect={maxJerk,maxJerk,maxJerk,maxJerk}
end

function moveToPosition(newMotorPositions,synchronous,maxVel,maxAccel,maxJerk)
    local _maxVelVect={}
    local _maxAccelVect={}
    local _maxJerkVect={}
    if not maxVel then
        maxVel=maxVelVect[1]
    end
    if not maxAccel then
        maxAccel=maxAccelVect[1]
    end
    if not maxJerk then
        maxJerk=maxJerkVect[1]
    end
    for i=1,4,1 do
        _maxVelVect[i]=math.max(1*math.pi/180,math.min(maxVel,maxVelVect[i]))
        _maxAccelVect[i]=math.max(1*math.pi/180,math.min(maxAccel,maxAccelVect[i]))
        _maxJerkVect[i]=math.max(1*math.pi/180,math.min(maxJerk,maxJerkVect[i]))
    end
    local op=sim.ruckig_nosync
    if synchronous then
        op=-1
    end
    local params = {
        joints = motorHandles,
        targetPos = newMotorPositions,
        maxVel = _maxVelVect,
        maxAccel = _maxAccelVect,
        maxJerk = _maxJerkVect,
        flags = op,
    }
    sim.moveToConfig(params)
end

function enableGripper(enable)
    local fakeOperation=false
    local dat={}
    dat.fakeOperation=fakeOperation
    dat.enabled=enable
    sim.setBufferProperty(gripperHandle, 'customData.activity',sim.packTable(dat))
end

function sysCall_thread()
    modelBase=sim.getObject('..')
    gripperHandle=sim.getObject('../uarmGripper')
    pickupPart=sim.getObject('../pickupCylinder')
    sim.setObjectParent(pickupPart,-1,true)
    local maxVelocity=45*math.pi/180 -- rad/s
    local maxAcceleration=40*math.pi/180 -- rad/s^2
    local maxJerk=80*math.pi/180 -- rad/s^3
    local maxTorque=10 -- kg*m^2/s^2

    initialize(maxVelocity,maxAcceleration,maxJerk,maxTorque)

    while true do
        local p=sim.getObjectPosition(pickupPart)
        local angles1=computeAnglesFromGripperPosition(p,0.07,-0.05)
        enableGripper(false)
        -- Synchronous operation of the individual joints:
        moveToPosition(angles1,true)
        local angles2=computeAnglesFromGripperPosition(p,0,0)
        moveToPosition(angles2,true)
        enableGripper(true)
        sim.wait(2)
        moveToPosition(angles1,true)
        moveToPosition({90*math.pi/180,104*math.pi/180,60*math.pi/180,90*math.pi/180},true)
        moveToPosition(angles1,true)
        moveToPosition(angles2,true)
        enableGripper(false)
        sim.wait(2)
        moveToPosition(angles1,true)
        moveToPosition({90*math.pi/180,104*math.pi/180,60*math.pi/180,90*math.pi/180},false)
    end
end

function sysCall_joint(inData)
    if inData.handle==auxMotor1 then
        local t2=-sim.getJointPosition(motorHandles[2])+104*math.pi/180
        local t3=sim.getJointPosition(motorHandles[3])-59.25*math.pi/180
        error=t3-t2-inData.pos
    end
    if inData.handle==auxMotor2 then
        local t3=sim.getJointPosition(motorHandles[3])-59.25*math.pi/180
        error=-t3-inData.pos
    end
    local ctrl=error*20
    
    local maxVelocity=ctrl
    if (maxVelocity>inData.maxVel) then
        maxVelocity=inData.maxVel
    end
    if (maxVelocity<-inData.maxVel) then
        maxVelocity=-inData.maxVel
    end
    local forceOrTorqueToApply=inData.maxForce

    local outData={}
    outData.vel=maxVelocity
    outData.force=forceOrTorqueToApply
    return outData
end

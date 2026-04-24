
# ===== Kalman Filter State =====
class Kalman1D:
    def __init__(self, q=0.01, r=0.1):
        self.x = 0.0
        self.p = 1.0
        self.q = q
        self.r = r

    def update(self, measurement):
        # Prediction
        self.p = self.p + self.q

        # Kalman Gain
        k = self.p / (self.p + self.r)

        # Update
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p
        return self.x
   
 import sim
except ImportError:
    sim = None 

# Global configuration variables
particlesAreVisible = True
simulateParticles = True
fakeShadow = True
particleCountPerSecond = 430
particleSize = 0.005
particleDensity = 8500
particleScatteringAngle = 30
particleLifeTime = 0.5
maxParticleCount = 50

# Handle storage
propellerHandles = []
jointHandles = []
particleObjects = [-1, -1, -1, -1]
motorHandles = [-1, -1, -1, -1]

# Control variables
cumul = 0
lastE = 0
pAlphaE = 0
pBetaE = 0
psp2 = 0
psp1 = 0
prevEuler = 0
notFullParticles = 0

def sysCall_init():
    global targetObj, d, heli, shadowCont, propellerHandles, jointHandles, particleObjects
    global pParam, iParam, dParam, vParam
    
    # Detach the manipulation sphere:
    targetObj = sim.getObject('../target')
    sim.setObjectParent(targetObj, -1, True)

    # Base object for control reference
    d = sim.getObject('../base')
# ===== Estimators =====
alt_kalman = Kalman1D()
roll_kalman = Kalman1D()
pitch_kalman = Kalman1D()

# ===== LSTM placeholder (simulation prediction layer) =====
def lstm_predict(state_vector):
    # (placeholder for trained model)
    return [0.0, 0.0, 0.0]    
    # Particle type bitmask
    ttype = (sim.particle_roughspheres + sim.particle_cyclic + 
             sim.particle_respondable1to4 + sim.particle_respondable5to8 + 
             sim.particle_ignoresgravity)
    
    if not particlesAreVisible:
        ttype += sim.particle_invisible

    # Initialize propellers and particles
    for i in range(4):
        # Lua index propeller[0..3] corresponds to range(4)
        propellerHandles.append(sim.getObject(f'../propeller[{i}]/respondable'))
        jointHandles.append(sim.getObject(f'../propeller[{i}]/joint'))
        if simulateParticles:
            particleObjects[i] = sim.addParticleObject(ttype, particleSize, particleDensity, 
                                                       [2, 1, 0.2, 3, 0.4], particleLifeTime, 
                                                       maxParticleCount, [0.3, 0.7, 1])
    
    heli = sim.getObject('..')
    
    # PID and velocity parameters
    pParam, iParam, dParam, vParam = 2, 0, 0, -2

    if fakeShadow:
        shadowCont = sim.addDrawingObject(sim.drawing_discpts + sim.drawing_cyclic + 
                                         sim.drawing_25percenttransparency + 
                                         sim.drawing_50percenttransparency + 
                                         sim.drawing_itemsizes, 0.2, 0, -1, 1)

def sysCall_actuation():
    global cumul, lastE, pAlphaE, pBetaE, psp1, psp2, prevEuler
    
    # Update shadow position
    pos = sim.getObjectPosition(d, -1)
    if fakeShadow:
        itemData = [pos[0], pos[1], 0.002, 0, 0, 0, 1, 0.2]
        sim.addDrawingObjectItem(shadowCont, itemData)
    
    # Vertical control:
    targetPos = sim.getObjectPosition(targetObj, -1)
    l = sim.getVelocity(heli) # returns [linearVelocity, angularVelocity]
    e = (targetPos[2] - pos[2])
    cumul += e
    pv = pParam * e
    # Thrust calculation (5.45 is the base weight compensation)
    thrust = 5.45 + pv + iParam * cumul + dParam * (e - lastE) + l[0][2] * vParam
    lastE = e
    
    def geometric_control(pos, target_pos, vel):
    """
    Simplified SE(3) geometric control for quadrotor
    """

    # position error
    e_pos = np.array(target_pos) - np.array(pos)

    # velocity damping
    e_vel = -np.array(vel[0])

    # desired acceleration (control law)
    acc = 2.5 * e_pos + 1.2 * e_vel

    # thrust (Z-axis dominant)
    thrust = 5.45 + acc[2]

    # attitude commands
    roll_cmd  = acc[1] * 0.05
    pitch_cmd = -acc[0] * 0.05
    yaw_cmd   = 0

    return thrust, roll_cmd, pitch_cmd, yaw_cmd    
    # Stability corrections based on target distance
    alphaCorr += sp[1] * 0.005 + 1 * (sp[1] - psp2)
    betaCorr -= sp[0] * 0.005 - 1 * (sp[0] - psp1)
    psp2, psp1 = sp[1], sp[0]
    
    # Rotational control (Yaw):
    euler = sim.getObjectOrientation(d, targetObj)
    rotCorr = euler[2] * 0.1 + 2 * (euler[2] - prevEuler)
    prevEuler = euler[2]
    
    # Motor velocity mixing
    handlePropeller(0, thrust * (1 - alphaCorr + betaCorr + rotCorr))
    handlePropeller(1, thrust * (1 - alphaCorr - betaCorr - rotCorr))
    handlePropeller(2, thrust * (1 + alphaCorr - betaCorr + rotCorr))
    handlePropeller(3, thrust * (1 + alphaCorr + betaCorr - rotCorr))

def handlePropeller(index, particleVelocity):
    global notFullParticles
    propellerRespondable = propellerHandles[index]
    propellerJoint = jointHandles[index]
    propeller = sim.getObjectParent(propellerRespondable)
    particleObject = particleObjects[index]
    
    # Scattering calculation
    maxParticleDeviation = math.tan(particleScatteringAngle * 0.5 * math.pi / 180) * particleVelocity
    
    t = sim.getSimulationTime()
    sim.setJointPosition(propellerJoint, t * 10)
    ts = sim.getSimulationTimeStep()
    
    m = sim.getObjectMatrix(propeller, -1)
    particleCnt = 0
    
    # Calculate how many particles to spawn this step
    requiredParticleCnt = particleCountPerSecond * ts + notFullParticles
    notFullParticles = requiredParticleCnt % 1
    requiredParticleCnt = int(math.floor(requiredParticleCnt))
    
    while particleCnt < requiredParticleCnt:
        # Uniform distribution in a circle
        x = (random.random() - 0.5) * 2
        y = (random.random() - 0.5) * 2
        if (x*x + y*y <= 1):
            if simulateParticles:
                p_pos = [x * 0.08, y * 0.08, -particleSize * 0.6]
                p_dir = [
                    p_pos[0] + (random.random() - 0.5) * maxParticleDeviation * 2,
                    p_pos[1] + (random.random() - 0.5) * maxParticleDeviation * 2,
                    p_pos[2] - particleVelocity * (1 + 0.2 * (random.random() - 0.5))
                ]
handlePropeller(0, thrust + roll_cmd - pitch_cmd + yaw_cmd)
handlePropeller(1, thrust - roll_cmd - pitch_cmd - yaw_cmd)
handlePropeller(2, thrust + roll_cmd + pitch_cmd - yaw_cmd)
handlePropeller(3, thrust - roll_cmd + pitch_cmd + yaw_cmd)
                # Transform to world coordinates
                actual_pos = sim.multiplyVector(m, p_pos)
                actual_dir = sim.multiplyVector(m, p_dir)
                sim.addParticleObjectItem(particleObject, actual_pos + actual_dir)
            particleCnt += 1

    # Apply reactive force (Newton's 3rd Law)
    totalExertedForce = (particleCnt * particleDensity * particleVelocity * math.pi * (particleSize**3)) / (6 * ts)
    force = [0, 0, totalExertedForce]
    
    # Matrix for rotation only (zeroing out translation)
    m_rot = list(m)
    m_rot[3], m_rot[7], m_rot[11] = 0, 0, 0
    
    force = sim.multiplyVector(m_rot, force)
    
    # Alternating rotation direction for torque compensation
    rotDir = 1 - (index % 2) * 2
    torque = [0, 0, rotDir * 0.002 * particleVelocity]
    torque = sim.multiplyVector(m_rot, torque)
    
    sim.addForceAndTorque(propellerRespondable, force, torque)
pos = sim.getObjectPosition(d, -1)
vel = sim.getVelocity(heli)

targetPos = sim.getObjectPosition(targetObj, -1)

# ===== Filtering =====
alt = alt_kalman.update(pos[2])

# optional: IMU simulation
roll  = roll_kalman.update(sim.getObjectOrientation(d, -1)[0])
pitch = pitch_kalman.update(sim.getObjectOrientation(d, -1)[1])            sim.removeParticleObject(p_obj)

state = [pos[0], pos[1], pos[2], vel[0][0], vel[0][1], vel[0][2]]
pred = lstm_predict(state)

# future correction (anticipation)
targetPos[0] += pred[0]
targetPos[1] += pred[1]
targetPos[2] += pred[2]

thrust, roll_cmd, pitch_cmd, yaw_cmd = geometric_control(
    [pos[0], pos[1], alt],
    targetPos,
    vel
)

#!/usr/bin/python

import time, math
import numpy as np
import matplotlib.pyplot as plt
import yaml
import scipy.ndimage as ndimage

#******************************************************************************
# Tube
#  Manages simulation of tube environment.
#******************************************************************************

class Tube:

    def __init__(self, tubeConfig, debug=True):
        self.length = tubeConfig['tubeLength']
        self.fiducialSpacing = tubeConfig['fiducialSpacing']
        self.fiducialWidth = tubeConfig['fiducialWidth']
        self.fiducialFirstStart = tubeConfig['fiducialFirstStart']
        self.baseReflectivity = tubeConfig['baseReflectivity']
        self.fiducialReflectivity = tubeConfig['fiducialReflectivity']
        
        self.pushingEnd = tubeConfig['pushingEnd']
        self.pushingAccel = tubeConfig['pushingAccel']

        self.reflectivityResolution = 0.001
        self.generateReflectivityImage()

    def generateReflectivityImage(self):
        numEntries = self.length/self.reflectivityResolution
        self.reflectivityMap = np.zeros((numEntries), dtype=np.uint8)
        self.reflectivityMap += self.baseReflectivity
        # insert fiducials
        i = int(self.fiducialFirstStart/self.reflectivityResolution)
        while (i < numEntries):
            for j in range(i, min(numEntries, i+int(self.fiducialWidth/self.reflectivityResolution))):
                self.reflectivityMap[j] = self.fiducialReflectivity
            i += int(self.fiducialSpacing/self.reflectivityResolution)

    def sampleReflectivity(self, y):
        i = min(max(int(y / self.reflectivityResolution), 0), self.reflectivityMap.shape[0])
        return self.reflectivityMap[i]

    def samplePushingAccel(self, y):
        if (y < self.pushingEnd):
            return self.pushingAccel
        else:
            return 0.0


#******************************************************************************
# Pod Plant
#   Implements pod process and measurement dynamics (for simulation)
#******************************************************************************

class PodPlant:
    # pod state:
    # [
    #   x  \
    #   y   \
    #   z    |
    #   r    | floating base
    #   p   /
    #   y  /
    #   dx  \
    #   dy   \
    #   dz    |
    #   dr    | floating base
    #   dp   /
    #   dy  /
    #   b   # actuation of brake
    # ]

    # input frame:
    # [
    #   brakinginput
    # ]

    mass=250. #kg
    mu=0.5 #brake-on-rail coeff of friction
    brakingDelay=0.0 # RC time constant in braking system impulse response
    nu=1. #friction amount

    def __init__(self, tube, dt, debug=True):
        self.debug = debug
        self.dt = dt
        self.tube = tube
        self.imu_bias = -0.5

    def update(self, t, x, u):
        # implement process dynamics for a single timestep of size self.dt

        # for now, very simple double integrator dynamics with friction 
        # prop to velocity
        # control input applied with factor of mu in -y direction
        xd = np.zeros((12))
        xd[0:6] = x[6:12]
        xd[6:12] = (-self.nu*x[6:12])/self.mass
        xd[7] += self.tube.samplePushingAccel(x[1])

        xn = np.zeros((13))
        xn[0:12] = x[0:12] + self.dt*xd + np.array([np.random.normal(0.00, 0.001) for i in range(12)])*xd

        # friction cone has to be dealt with slightly differently.
        # first delay the brakes
        alpha = self.dt / (self.dt + self.brakingDelay)
        xn[12] = x[12]*(1.-alpha) + alpha*u[0]
        brakingF = self.mu*xn[12]
        if xn[7] > 0:
            xn[7] = max(0.0, xn[7] - brakingF*self.dt/self.mass)
        else:
            xn[7] = min(0.0, xn[7] + brakingF*self.dt/self.mass)

        return xn

    def output(self, t, x, u):
        # implement measurements given current state and input

        # IMU:
        xdd = (self.update(t, x, u)[6:12] - x[6:12]) / self.dt + np.random.normal(0.00, 0.01) + self.imu_bias
        ref = self.tube.sampleReflectivity(x[1])
        return (xdd, ref)

class PodStateEstimatorDoubleIntegrator:
    # input from pod

    # state is the estimated state of the pod

    # output estimate is of the state frame of the pod

    def __init__(self, pod, tube, dt, debug=True):
        self.debug = debug
        self.dt = dt
        self.pod = pod
        self.tube = tube

    def update(self, t, x, z):
        # double integrate the IMU
        xn = np.zeros((12))
        xn[0:12] = x[0:12]
        xn[1] += self.dt * xn[7]
        xn[7] += self.dt * z[0][1]
        return xn

    def output(self, t, x, z):
        # implement measurements given current state and input
        return x


class PodStateEstimatorDiscreteBayes:
    # input from pod

    # state is the estimated state of the pod:
    # foh on all elems other than y
    # but for y, maintains a discrete bayes filter
    # by discretized pdf using self.nBins bins

    # output estimate is of the state frame of the pod
    def __init__(self, pod, tube, dt, binSize, debug=True):
        self.debug = debug
        self.dt = dt
        self.pod = pod
        self.tube = tube
        self.binSize = binSize
        self.nBins = int(tube.length / binSize)

        self.TP = 0.99 # see lines 50% of the time
        self.FN = 0.01

        self.FP = 1/599. # see ghost line once per line period
        self.TN = 598/598.

        # for every bin, calculate percent area of bin
        # that line covers
        self.binLineProps = np.zeros((self.nBins))
        binLineCounts = np.zeros((self.nBins))
        y = 0.
        ratio = int(self.binSize / self.tube.reflectivityResolution)
        for i in range(self.nBins):
            self.binLineProps[i] = np.sum(self.tube.reflectivityMap[i*ratio:(i+1)*ratio]/255.)/ ratio

    def getInitialState(self, x0):
        ''' Expects 12-dim input, allocates the big np array
        for discrete bayes'''
        state = [x0]
        pdf = np.zeros((self.nBins))
        pdf[self.getBin(x0[1])] = 1.
        state.append(pdf)
        return state

    def getBin(self, y):
        return max(0, min(self.nBins-1, y / self.binSize))

    def update(self, t, x, z):
        # double integrate the IMU
        xn_di = np.zeros((12))
        xn_di[0:12] = x[0][0:12]
        xn_di[1] += self.dt * xn_di[7]
        xn_di[7] += self.dt * z[0][1]

        xn_bayes = x[1]
        
        # process update, relies on stochasticity to
        # advance forwards below resolution of grid
        vel_step = self.dt*xn_di[7] / self.binSize
        if np.random.random() < (vel_step % 1):
            shift = math.ceil(vel_step)
        else:
            shift = math.floor(vel_step)

        xn_bayes_new = np.zeros((self.nBins + abs(shift) +1))
        if (shift >= 0.0):
            xn_bayes_new[shift:(self.nBins+shift)] = xn_bayes
            xn_bayes[0:self.nBins] = xn_bayes_new[0:self.nBins]
        else:
            xn_bayes_new[0:self.nBins] = xn_bayes
            xn_bayes[0:self.nBins] = xn_bayes_new[-shift:(self.nBins-shift)]
    
        # (fractional) right-shift by velocity timestep
        # nope, this isn't right: it causes slow
        # dissolution of the state
        # todo: vectorize
    #    vel_step = self.dt*xn_di[7] / self.binSize
    #    shift = math.floor(vel_step)
    #    xn_bayes_new = np.zeros((self.nBins + abs(shift) +1))
    #    if (vel_step >= 0.0):
    #        xn_bayes_new[shift:(self.nBins+shift)] = (1 - (vel_step % 1))*xn_bayes
    #        xn_bayes_new[(shift+1):(self.nBins+shift+1)] += (vel_step % 1)*xn_bayes
    #        xn_bayes[0:self.nBins] = xn_bayes_new[0:self.nBins]
    #    else:
    #        xn_bayes_new[0:self.nBins] = (1. - (vel_step % 1))*xn_bayes
    #        xn_bayes_new[1:(self.nBins+1)] += (vel_step % 1)*xn_bayes
    #        xn_bayes[0:self.nBins] = xn_bayes_new[(shift-self.nBins):(-shift+1)]
        
        # uncertainty update:
        # todo: determine sigma somehow
        xn_bayes = ndimage.filters.gaussian_filter(xn_bayes, 1, order=0, output=None, mode='constant')

        # measurement update
        if z[1] > 127: # we saw a line
            xn_bayes *= (self.binLineProps * self.TP + (1. - self.binLineProps) * self.FP)
        else: # we didn't see a line
            xn_bayes *= (self.binLineProps * self.FN + (1. - self.binLineProps) * self.TN)

        # normaliiiiize
        if np.sum(xn_bayes) > 0.00001:
            xn_bayes /= np.sum(xn_bayes)
        return [xn_di, xn_bayes]

    def output(self, t, x, z):
        # implement measurements given current state and input
        xout = np.zeros((12))
        xout[:] = x[0][:]
        xout[1] = np.argmax(x[1])*self.binSize
        return xout

class PodBrakingController:
    # input is estimated state of pod and measured acceleration

    # state is latest commanded braking force (kinda weird)

    # output is commanded braking force

    mass=250. #kg

    def __init__(self, stoppingDistance, dt, debug=True):
        self.debug = debug
        self.dt = dt
        self.stoppingDistance = stoppingDistance

    def update(self, t, x, z):
        y = z[0][1]
        yd = z[0][7]

        # calculate mu from last commanded braking force,
        # measured acceleration response
        ydd = z[1][1]
        if abs(x[0]) >= 0.1 and abs(ydd) > 0.01:
            mu = self.mass * abs(ydd) / x[0]
        else:
            mu = 1.

        distToEnd = max(self.stoppingDistance - y, 0.0)
        desiredyd = distToEnd*0.5 # start braking at 80 m

        # if we're slow, stop
        if (distToEnd < self.stoppingDistance/2 and yd < 1):
            #time.sleep(0.1)
            return [2.4*9.81*self.mass]
        elif (desiredyd < yd):
            #time.sleep(0.1)
            err = yd - desiredyd
            return [10.*self.mass*abs(err)/mu]
        else:
            return [0.]


    def output(self, t, x, z):
        return x


# Test code if this file is being run and not imported
if __name__ == "__main__":

    tubeConfig_s = 'tubeConfig.yaml'
    tubeConfig_f = open(tubeConfig_s)
    tubeConfig = yaml.safe_load(tubeConfig_f)
    tubeConfig_f.close()

    tube = Tube(tubeConfig, debug=True)

    dt = 0.001
    x = np.array([0., 10., 0., 0., 0., 0., #position
                   0., 0.0, 0., 0., 0., 0., #velocity
                   0.]) # braking force
    xse = np.array([0., 10., 0., 0., 0., 0.,
                   0., 0.0, 0., 0., 0., 0.])
    xse_di = np.array([0., 10., 0., 0., 0., 0.,
                   0., 0.0, 0., 0., 0., 0.])
    xcon = np.array([0.])
    u = np.array([0.])

    # Simulation speed control
    realtimefactor = 5.

    # Drawing setup
    drawfps = 30
    lastdrawtime = time.time()
    drawtwindow = 40
    drawtstep = 0.01
    drawMinlb = -30
    drawMinub = tubeConfig['tubeLength']/2.
    fig = plt.figure(1)
    ax_y = fig.add_subplot(211)
    ax_y.grid(True)
    ax_y.set_title("POD!")
    ax_y.set_xlabel("Time")
    ax_y.set_ylabel("Position")
    ax_y.axis([0-drawtwindow*0.75,0+drawtwindow*0.25,-drawMinlb,drawMinub])


    ax_yse = fig.add_subplot(212)
    ax_yse.grid(True)
    ax_yse.set_title("POD SE!")
    ax_yse.set_xlabel("Position")
    ax_yse.set_ylabel("Probability mass")
    ax_yse.axis([-10,10,-0.25,1.25])

    fig.hold(True)
    line_y=ax_y.plot([0],[0],'-k')
    line_yhat=ax_y.plot([0],[0],'-r')
    line_yhat_di=ax_y.plot([0],[0],'--c')

    line_yse=ax_yse.plot([0],[0],'-r')

    fig.show(False)

    # Mostly for plotting
    state_vec = [[0], [0], [0]]
    se_vec = [[0], [0]]
    se_di_vec = [[0], [0]]

    # Spawn the simulation 
    pod = PodPlant(tube = tube, dt = dt, debug=True)
    se = PodStateEstimatorDiscreteBayes(pod = pod, tube=tube, dt = dt, binSize=0.1, debug=True)
    xse = se.getInitialState(xse);

    se_di = PodStateEstimatorDoubleIntegrator(pod=pod, tube=tube, dt=dt, debug=True)
    
    controller = PodBrakingController(stoppingDistance = tubeConfig['tubeLength']-50, dt = dt, debug=True)

    # Run simulation main loop
    t = 0.
    starttime = time.time()
    ti = 0
    while (1):
        # Update the pod and generate sensor outputs
        xn = pod.update(t, x, u)     
        z = pod.output(t, x, u)
        x[0:12] = xn[0:12]

        # Pass sensor outputs to state estimator
        xsen = se.update(t, xse, z)
        xhat = se.output(t, xse, z)
        xse[0][:] = xsen[0][:]
        xse[1][:] = xsen[1][:]

        # Run double integrator in parallel
        xse_din = se_di.update(t, xse_di, z)
        xhat_di = se_di.output(t, xse_di, z)
        xse_di[:] = xse_din[:]

        # Pass state estimate output to controller
        xconn = controller.update(t, xcon, (xhat, z))
        u = controller.output(t, xcon, (xhat, z))
        xcon[0] = xconn[0]

        # Simulation infrastructure
        t += dt
        ti += 1
        
        # Logging and plotting logic
        if (ti % (drawtstep / dt) == 0):
            if len(state_vec[0]) > drawtwindow/drawtstep:
                state_vec[0].pop(0)
                state_vec[1].pop(0)
                state_vec[2].pop(0)
                se_vec[0].pop(0)
                se_vec[1].pop(0)
                se_di_vec[0].pop(0)
                se_di_vec[1].pop(0)
            state_vec[0].append(t)
            state_vec[1].append(x[1])
            state_vec[2].append(x[7])
            se_vec[0].append(xhat[1])
            se_vec[1].append(xhat[7])
            se_di_vec[0].append(xhat_di[1])
            se_di_vec[1].append(xhat_di[7])

        if (time.time() - lastdrawtime > 1./drawfps):
            fig.hold(False)
            line_y[0].set_data(state_vec[0], state_vec[1])
            fig.hold(True)
            line_yhat[0].set_data(state_vec[0], se_vec[0])
            line_yhat_di[0].set_data(state_vec[0], se_di_vec[0])
            ax_y.axis([t-drawtwindow*0.75, t+drawtwindow*0.25, drawMinlb, max(drawMinub, max(state_vec[1])+100)])

            line_yse[0].set_data(np.arange(0., tube.length, se.binSize), xse[1])
            ax_yse.axis([0, tube.length, -0.01, np.max(xse[1])+0.01])

            fig.canvas.draw()
            fig.canvas.flush_events()
            lastdrawtime = time.time()

        # Simulation rate control
        #if (time.time() - starttime < t/realtimefactor):
        #    time.sleep(t/realtimefactor + starttime - time.time())

        # Simulation halt control
        if (x[1] > tubeConfig['tubeLength']):
            print "CRASH"
            break
        if (x[1] > 0 and abs(x[7]) < 0.001):
            print "STOP"
            break

plt.pause(1000)

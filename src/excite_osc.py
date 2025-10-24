import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the Oscillator class
class Oscillator:
    

    # Define the constructor
    def __init__(self, mass2, stiffness12, lenth12, excitation=10.0, damping = 0.0):
        """
        Constructor for the oscillator class"""
        self.m2 = mass2;
        self.k12 = stiffness12
        self.stVec = np.zeros((1, 2))
        self.l12 = lenth12
        self.dmp = damping
        self.t = np.zeros(1)
        self.cmd = np.zeros(1) # [x1]
        self.frqExi = excitation # Hz
        self.resFreq = (self.k12/self.m2)**.5/(2*np.pi) # Hz
        self.cmdTmp = np.zeros(1)
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the drone
        """
        return  "Oscillator with inertia1 {} kg*m^2, inertia2 {} kg*m^2, stifness10 {} N/m, stifness23 {} N/m, and ratio {}".format(self.inertia1, self.inertia2, self.stifness10, self.stifness23, self.ratio)

    def setConditions(self, x2, xdot2):
        """
        Sets the initial conditions of the drone
        
        args:
            x0: initial x position in m
            y0: initial y position in m
            theta0: initial angle in radians
            xdot0: initial x velocity in m/s
            ydot0: initial y velocity in m/s
            thetadot0: initial angular velocity in rad/s
        """
        self.stVec[-1, :] = [x2, xdot2]


    
    def control(self, t, y):
        self.cmd[0] = 0.2*self.l12*np.sin(2*np.pi*self.frqExi*t)

    def eqGenerator(self):
        """
        Generates the equations of motion for the oscillator
        
        Args:
            None
        Returns:
            None
        """
        def eq(t, y):
            """
            Returns the equations of motion for the drone
            args:
                t: time in s
                y: state vector
            returns:
                ydot: derivative of the state vector
            """

            # y = [x, y, theta, xdot, ydot, thetadot]
            # ydot = [xdot, ydot, thetadot, xddot, yddot, thetaddot]
            #self.pid(t, y)
            self.control(t, y)
            ydot = 0.*self.stVec[-1, :]#np.zeros(4)
            ydot[0] = y[1] # define theta1dot
            ydot[1] = self.k12/self.m2*(self.l12 + self.cmd[0] - y[0]) - self.dmp*y[1] # define theta3dot
            return ydot
        self.eq = eq
    
    def updateState(self, t, y):
        """
        Updates the state vector and time vector
        args:
            t: time in s
            y: state vector
        """
        self.stVec = np.vstack((self.stVec, y))
        self.cmd = np.vstack((self.cmd, self.cmdTmp))
        self.t = np.append(self.t, t)
    
    def plot(self):
        """
        Plots the state vector
        args:
            None
        """
        fig, axs = plt.subplots(2, 1)
        # Make the figure large
        fig.set_size_inches(18.5, 10.5)

        # Separate the plots
        fig.tight_layout(pad=3.0)
        ymax = 1.1*max(np.max(self.stVec[:, 0]),np.max(self.stVec[:, 1]))

        axs[0].plot(self.t, self.stVec[:, 0],'r')
        axs[0].set_title('Position')
        axs[0].set_ylabel('Position (m)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylim(-ymax, ymax)

        axs[1].plot(self.t, self.stVec[:, 1],'b')
        axs[1].set_title('Velocity')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylim(-ymax, ymax)

        plt.show()

    def plotControl(self):
        pass

    def animate(self):
        pass

    def solve(self, t0, tf, dt):
        """
        Solves the equations of motion for the drone
        args:
            t0: initial time in s
            tf: final time in s
            dt: time step in s
        """
        self.eqGenerator()
        r = RK45(self.eq, t0, self.stVec[-1, :], tf, max_step=dt)
        while r.status == 'running':
            r.step()
            self.updateState(r.t, r.y)
        #self.plot()
    
    def integrate(tVect,varVect):
        """
        Integrates the equations of motion for the drone
        args:
            tVect: time vector
            varVect: state vector
        returns:
            integrated state vector
        """
        return np.trapz(varVect,tVect)
    
    def energy(self):
        """
        Calculates the energy of the oscillator
        args:
            None
        returns:
            energy: energy of the oscillator
        """
        pass
    
def main():
    osc = Oscillator(1.0,1.0,1.0,excitation=0.0,damping=-0.0010)
    osc.eqGenerator()
    osc.setConditions(1.5,0.0)
    print('Starting simulation')
    osc.solve(0, 500, 0.01)
    osc.plot()
    # m = 1.0
    # k = 1.0
    # l = 1.0

    # freq = np.array([0.1]) # Hz
    # freq = (k/m)**.5*np.linspace(0.99, 1.01, 101)/(2*np.pi)
    # print(freq)
    # #for n in range(1, 100):
    # #   freq = np.append(freq, freq[-1]*1.01)
    # print(freq)
    
    # ampl = np.array([])
    # for freq_tmp in freq:
    #    osc = Oscillator(m,k,l,excitation=freq_tmp)
    #    osc.eqGenerator()
    #    osc.setConditions(1.0,0.0)
    #    print('Starting simulation freq = {}'.format(freq_tmp))
    #    osc.solve(0, 1000, 0.1)
    #    ampl = np.append(ampl, np.max(osc.stVec[:, 0]))
    
    # plt.plot(freq, ampl)
    # plt.plot([(osc.k12/osc.m2)**.5/(2*np.pi), (osc.k12/osc.m2)**.5/(2*np.pi)], [0, 1.1*np.max(ampl)], 'r--')
    # plt.show()

    
if __name__ == "__main__":
    main()
    

    

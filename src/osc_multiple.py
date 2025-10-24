import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the Oscillator class
class Oscillator:
    

    # Define the constructor
    def __init__(self, mass0, mass1, stiffness0, stiffness1, length0,nb_masses=5):
        """
        Constructor for the oscillator class"""
        self.m0 = mass0;
        self.m1 = mass1;
        self.k0 = stiffness0
        self.k1 = stiffness1
        self.nbm = nb_masses
        self.stVec = np.zeros((1, 2*(self.nbm+1)))
        self.enVec = np.zeros((1, self.nbm+1))
        self.enTotVec = np.zeros((1, 2*(self.nbm+1)))
        self.stifness10 = stiffness0
        self.fric0 = 0.0
        self.fric1 = 0.0
        self.l0 = length0
        self.t = np.zeros(1)
        self.cmd = np.zeros(1) # [x1]
        self.cmdTmp = np.zeros(1)
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the drone
        """
        return  "Oscillator with {} masses, mass 0 {} kg*m^2, mass 1{} kg*m^2, stifness0 {} N/m, stifness1 {} N/m, friction 0 {} and friction 1{}".format(self.nbm, self.m0, self.m1, self.k0, self.k1, self.fric0, self.fric1,self.fric1)

    def setConditions(self, xInit, xdotInit):
        """
        Sets the initial conditions of the drone
        
        Args:
            xInit: initial position of all masses in m
            xdotInit: initial velocity of all masses in m/s
        Returns:
            None
        """

        self.stVec[-1, :] = np.concatenate((xInit, xdotInit))


    
    def control(self, t, y):
        return None
    
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

            self.control(t, y)
            # Initialise the state vector

            ydot = 0.*self.stVec[-1, :]#np.zeros(4)
            ydot[0:self.nbm+1] = y[self.nbm+1:2*(self.nbm+1)]
            ydot[self.nbm+1] = (-self.k0*y[0] - self.nbm*self.k1*(y[0]+self.l0) + self.k1*np.sum(y[1:self.nbm+1]) - self.fric0*y[self.nbm+1])/self.m0

            for indy in range(self.nbm+2,2*(self.nbm+1)):
                ydot[indy] = (self.k1*(-y[indy-self.nbm -1]  +self.l0 + y[0]) - self.fric1*y[indy])/self.m1
            
            return ydot
        self.eq = eq
    
    def computeEnergy(self):
        """
        Computes the energy of the oscillator
        args:
            None
        returns:
            energy: energy of the oscillator
        """
        # Compute the energy of the oscillator
        self.enVec = np.zeros((len(self.t), self.nbm+1))
        self.enVec[:,0] = .5*self.m0*(self.stVec[:,self.nbm+1]**2) + self.k0*(self.stVec[:,0]**2)/2
        for indx in range(1,self.nbm+1):
            self.enVec[:, indx] = .5*self.m1*(self.stVec[:,indx+self.nbm+1]**2) + .5*self.k1*((self.stVec[:,indx]- self.stVec[:,0] -self.l0)**2)
        self.enTotVec = np.sum(self.enVec, axis=1)
        
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
        fig.tight_layout(pad=4.0)

        axs[0].plot(self.t, self.stVec[:, 0],'b')
        axs[0].set_title('Position')
        axs[0].set_ylabel('Position (m)')
        axs[0].set_xlabel('Time (s)')
        for indx in range(1, self.nbm+1):
            axs[0].plot(self.t, self.stVec[:, indx],'r')

        axs[1].plot(self.t, self.stVec[:, self.nbm+1],'k')
        axs[1].set_title('Velocity')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].set_xlabel('Time (s)')
        for indxdot in range(self.nbm+2, 2*(self.nbm+1)):
            axs[1].plot(self.t, self.stVec[:, indxdot],'g')

        plt.show()

    def plotEnergy(self):
        """
        Plots the energy of the oscillator
        args:
            None
        """
        fig = plt.figure()
        # Make the figure large

        # Separate the plots
        fig.tight_layout(pad=3.0)
        #ymax = 1.1*max(np.max(self.stVec[:, 0]),np.max(self.stVec[:, 1]))
        plt.plot(self.t, self.enTotVec,'k')
        plt.plot(self.t, self.enVec[:, 0],'b')
        plt.title('Energy')
        plt.ylabel('Energy (J)')
        plt.xlabel('Time (s)')

        for indx in range(1, self.nbm+1):
            plt.plot(self.t, self.enVec[:, indx],'r')
            plt.title('Energy')
            plt.ylabel('Energy (J)')
            plt.xlabel('Time (s)')
        plt.show()

    def plotPhase(self):
        """
        plot the phase portrait of the oscillator
        args:
            None
        """
        plt.figure()
        plt.plot(self.stVec[:, 0], self.stVec[:, self.nbm+1], 'k')
        plt.title('Phase portrait')
        plt.xlabel('Position (m)')
        plt.ylabel('Velocity (m/s)')
        for indx in range(1, self.nbm+1):
            plt.plot(self.stVec[:, indx]-self.l0, self.stVec[:, indx+self.nbm+1], 'r')
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
def main():
    osc = Oscillator(1.0, # m0
                     1.0, # m1
                     .0, # k0
                     10.0, # k1
                     1.0, # l0
                     nb_masses=3)
    print(osc)
    osc.eqGenerator()
    #osc.setConditions([0.0, 10., 1.0, 1.0, 1.0, 1.0], [0.0, .0, .0, .0, .0, .0])
    osc.setConditions([0.0, 1.10, 1.0, 1.2], [0., 0., 0., 0.])
    osc.fric0 = 0.
    osc.fric1 = 0.
    print('Starting simulation')
    osc.solve(0, 50, 0.01)
    osc.plot()
    osc.computeEnergy()
    osc.plotEnergy()
    osc.plotPhase()
    
if __name__ == "__main__":
    main()
    

    

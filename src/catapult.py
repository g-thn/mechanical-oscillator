import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the Catapult class
class Catapult:
    

    # Define the constructor
    def __init__(self, mass1, mass2, gravity):
        """
        Constructor for the Catapult class"""
        
        self.m1 = mass1
        self.m2 = mass2
        self.g = gravity
        # Ration function parameters
        self.k1 = 10.
        self.k2 = .10
        self.k3 = .0
        self.lengthRamp = 3.0
        self.heightFall = 100.
        self.leftRamp = False
        self.tStop = 0.0
        self.stVec = np.zeros((1, 4))
        self.t = np.zeros(1)
        self.cmd = np.zeros((1,2)) # [F, theta]
        self.cmdTmp = np.zeros(2)
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the catapult
        """
        return  "Catapult with mass 1 {} kg and mass 2 {} kg*m^2".format(self.mass1, self.inertia2, self.stifness10, self.stifness23, self.ratio)

    def setConditions(self, z1, z2, zdot1, zdot2):
        """
        Sets the initial conditions of the catapult
        
        args:
            x0: initial x position in m
            y0: initial y position in m
            theta0: initial angle in radians
            xdot0: initial x velocity in m/s
            ydot0: initial y velocity in m/s
            thetadot0: initial angular velocity in rad/s
        """
        self.stVec[-1, :] = [z1, z2, zdot1, zdot2]


    
    def control(self, t, y):
        pass
    
    def ratio(self,z):
        return self.k1*z**10 + self.k2 
    
    def ratiodot(self, z):
        return 10*self.k1*z**9
    
    def ratioddot(self, z):
        return 0.

    def phi(self, z2, zdot2):
        return self.ratio(z2)
    
    def psi(self, z2, zdot2):
        return (self.ratiodot(z2))*zdot2**2
    
    def eqGenerator(self):
        """
        Generates the equations of motion for the Catapult
        
        Args:
            None
        Returns:
            None
        """
        def eq(t, y):
            """
            Returns the equations of motion for the catapult
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
            f = self.ratio(y[1])
            phi_ = self.phi(y[1], y[3])
            psi_ = self.psi(y[1], y[3])
            ydot = 0.*self.stVec[-1, :]#np.zeros(4)
            if (y[0] < self.lengthRamp) & (not self.leftRamp) & (y[1] > -self.heightFall): #(y[1] > -self.heightFall) & 
                ydot[0] = y[2] # define z1dot
                ydot[1] = y[3] # define z2dot
                ydot[3] = -(self.g*(self.m2 - f*self.m1) + self.m1*f*psi_)/(self.m1*f*phi_ + self.m2) #define z1ddot
                ydot[2] = -(ydot[3]*phi_ + psi_)# define z2ddot
                if ydot[2] < 0:
                    self.leftRamp = True
                    self.tStop = t
            else:
                ydot[0] = y[2] # define z1dot
                ydot[1] = 0.#y[3]#0. # define z2dot
                ydot[2] = -self.m1*self.g# define z2ddot
                ydot[3] = 0.#-self.m2*self.g#0. #define z1ddot
                
                self.leftRamp = True
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
        if not self.leftRamp:
            self.tStop = t
            
    def computeEnergy(self):
        """
        Computes the energy of the catapult
        args:
            None
        returns:
            ep1 = potential energy of mass 1
            ep2 = potential energy of mass 2
            ek1 = kinetic energy of mass 1
            ek2 = kinetic energy of mass 2
        """
        ep1 = self.m1*self.g*self.stVec[:, 0]
        ep2 = self.m2*self.g*self.stVec[:, 1]
        ek1 = 0.5*self.m1*self.stVec[:, 2]**2
        ek2 = 0.5*self.m2*self.stVec[:, 3]**2
        return ep1, ep2, ek1, ek2
        
    
    def plot(self):
        """
        Plots the state vector
        args:
            None
        """
        fig, axs = plt.subplots(2, 2)
        # Make the figure large
        fig.set_size_inches(18.5, 10.5)

        # Separate the plots
        fig.tight_layout(pad=3.0)
        ymax = 1.1*max(np.max(self.stVec[:, 0]),np.max(-self.stVec[:, 1]))
        vmax = 1.1*max(np.max(self.stVec[:, 2]),np.max(-self.stVec[:, 3]))
        accmax = 1.1*max(np.max(np.gradient(self.stVec[:, 2], self.t)),np.max(-np.gradient(self.stVec[:, 3], self.t)))
        axs[0, 0].plot(self.t, self.stVec[:, 0],'r', label='mass 1')
        axs[0, 0].plot(self.t, -self.stVec[:, 1],'b', label='mass 2')
        axs[0, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 0].grid()
        axs[0, 0].set_title('Position')
        axs[0, 0].set_ylabel('Height (m)')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylim(-ymax, ymax)
        axs[0, 0].legend()

        axs[0, 1].plot(self.t, self.stVec[:, 2],'r', label='mass 1')
        axs[0, 1].plot(self.t, -self.stVec[:, 3],'b', label='mass 2')
        axs[0, 1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 1].grid()
        axs[0, 1].set_title('Velocity')
        axs[0, 1].set_ylabel('Velocity (m/s)')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylim(-vmax, vmax)
        axs[0, 1].legend()

        axs[1, 0].plot(self.t, np.gradient(self.stVec[:, 2], self.t),'r', label='mass 1')
        axs[1, 0].plot(self.t, -np.gradient(self.stVec[:, 3], self.t),'b', label='mass 2')  
        axs[1, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1, 0].grid()
        axs[1, 0].set_title('Acceleration')
        axs[1, 0].set_ylabel('Acceleration (m/s^2)')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylim(-accmax, accmax)
        axs[1, 0].legend()

        plt.show()

    def plotEnergy(self):
        ep1, ep2, ek1, ek2 = self.computeEnergy()
        print('Max potential energy: {}'.format(np.max(ep1+ep2)))
        print('Max kinetic energy: {}'.format(np.max(ek1+ek2)))
        #del fig, axs
        fig, axs = plt.subplots(2, 2)
        # Make the figure large
        fig.set_size_inches(18.5, 10.5)

        # Separate the plots
        fig.tight_layout(pad=3.0)
        epmax = 1.1*max(np.max(ep1), np.max(ep2))
        epmin = 1.1*min(np.min(ep1), np.min(ep2))
        ekmax = 1.1*max(np.max(ek1), np.max(ek2))
        ekmin = 1.1*min(np.min(ek1), np.min(ek2))

        axs[0, 0].plot(self.t, ep1,'r', label='mass 1')
        axs[0, 0].plot(self.t, ep2,'b', label='mass 2')
        axs[0, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 0].grid()
        axs[0, 0].set_title('Potential Energy')
        axs[0, 0].set_ylabel('Energy (J)')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylim(epmin, epmax)
        axs[0, 0].legend()

        axs[0, 1].plot(self.t, ek1,'r', label='mass 1')
        axs[0, 1].plot(self.t, ek2,'b', label='mass 2')
        axs[0, 1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 1].grid()
        axs[0, 1].set_title('Kinetic Energy')
        axs[0, 1].set_ylabel('Energy (J)')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylim(ekmin, ekmax)
        axs[0, 1].legend()

        axs[1, 0].plot(self.t, 100*ek1/np.max(-ep2),'r', label='mass 1')
        # axs[1, 0].plot(self.t, ek2,'b', label='mass 2')
        #axs[1, 0].plot(self.t, ek1/1000,'r', label='mass 1')
        axs[1, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1, 0].grid()
        axs[1, 0].set_title('Energy efficiency')
        axs[1, 0].set_ylabel('Energy used (%)')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylim(0,1.1*max(100*ek1/np.max(-ep2)))
        axs[1, 0].legend()

        axs[1, 1].plot(self.t, ep1+ek1,'r', label='mass 1 total')
        axs[1, 1].plot(self.t, ek1,'r-.', linewidth = .75, label='mass 1 kinetic')
        axs[1, 1].plot(self.t, ep1,'r--', linewidth = .75, label='mass 1 potential')
        axs[1, 1].plot(self.t, ep2+ek2,'b', label='mass 2')
        axs[1, 1].plot(self.t, ek2,'b-.', linewidth = .75, label='mass 2 kinetic')
        axs[1, 1].plot(self.t, ep2,'b--', linewidth = .75, label='mass 2 potential')
        axs[1, 1].plot(self.t, ep1+ep2+ek1+ek2,'k', label='total')
        axs[1, 1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1, 1].grid()
        axs[1, 1].set_title('Total Energy')
        axs[1, 1].set_ylabel('Energy (J)')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylim(1.1*min(np.min(ep1+ek1),np.min(ep2+ek2),np.min(ep1),np.min(ep2),np.min(ek1),np.min(ek2)), 1.1*max(np.max(ep1+ek1), np.max(ep2+ek2),np.max(ep1), np.max(ep2), np.max(ek1), np.max(ek2)))
        axs[1, 1].legend()

        plt.show()
    
    def plotPhasePortraits(self,accelOnly = True):
        if accelOnly:
            indStop = np.where(self.t >= self.tStop)[0][0]
            plt.plot(self.stVec[0:indStop, 0], self.stVec[0:indStop, 2], 'r', label='mass 1')
            plt.plot(-self.stVec[0:indStop, 1], -self.stVec[0:indStop, 3], 'b', label='mass 2')
        else:
            plt.plot(self.stVec[:, 0], self.stVec[:, 2], 'r', label='mass 1')
            plt.plot(-self.stVec[:, 1], -self.stVec[:, 3], 'b', label='mass 2')
        plt.axvline(0, color='k', linestyle='--')
        plt.axhline(0, color='k', linestyle='--')
        plt.grid()
        plt.title('Phase Portraits')
        plt.xlabel('Position (m)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.show()
    
    def plotControl(self):
        pass
    
    def plotRatio(self):
        z2 = np.linspace(0, self.heightFall, 100)
        ratio = self.ratio(z2)
        plt.plot(z2, ratio)
        plt.title('Ratio function')
        plt.xlabel('z2 (m)')
        plt.ylabel('Ratio')
        plt.show()

    def animate(self):
        pass

    def solve(self, t0, tf, dt):
        """
        Solves the equations of motion for the catapult
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
        Integrates the equations of motion for the catapult
        args:
            tVect: time vector
            varVect: state vector
        returns:
            integrated state vector
        """
        return np.trapz(varVect,tVect)
    
    def energy(self):
        """
        Calculates the energy of the Catapult
        args:
            None
        returns:
            energy: energy of the Catapult
        """
        pass
    
def main():
    cata = Catapult(1., 100., 9.81)
    cata.eqGenerator()
    cata.setConditions(0., 0., 0., 0.)
    print('Starting simulation')
    cata.solve(0, 1., 0.001)
    cata.plotRatio()
    cata.plot()
    cata.plotEnergy()
    cata.plotPhasePortraits()
    
if __name__ == "__main__":
    main()
    

    

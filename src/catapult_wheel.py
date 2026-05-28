import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the Catapult class
class CatapultWheel:
    
    # Define the constructor
    def __init__(self, inertia1, mass2, gravity):
        """
        Constructor for the Catapult class"""
        
        self.in1 = inertia1
        self.m2 = mass2
        self.g = gravity
        # Ration function parameters
        self.k1 = -1.
        self.k2 = .10
        self.k3 = .0
        self.lengthRamp = 3.0
        self.heightFall = 100.0
        self.maxAngle = 10.*2*np.pi
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

    def setConditions(self, theta0, z0, thetaot0, zdot0):
        """
        Sets the initial conditions of the catapult
        
        args:
            theta0: initial x position in m
            z0: initial y position in m
            thetaot0: initial x velocity in m/s
            zdot0: initial y velocity in m/s
        """
        self.stVec[-1, :] = [theta0, z0, thetaot0, zdot0]


    
    def control(self, t, y):
        pass
    
    def ratio(self,theta):
        """
        Returns the ratio function for the catapult
        args:
            theta: angle in radians
        returns:
            ratio: the ratio function value
        """
        return self.k1*theta #*theta#self.k1*theta**10 + self.k2 
    
    def ratiodot(self, theta):
        """
        Returns the derivative of the ratio function for the catapult
        args:
            theta: angle in radians
        returns:
            ratiodot: the derivative of the ratio function value
        """
        return self.k1 + 0.*theta # 100.#10*self.k1*theta**9
    
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
            #self.control(t, y)
            f = self.ratio(y[0])
            fdot = self.ratiodot(y[0])
            #phi_ = self.phi(y[1], y[3])
            #psi_ = self.psi(y[1], y[3])
            ydot = 0.*self.stVec[-1, :]#np.zeros(4)
            if (y[1] < self.lengthRamp) & (not self.leftRamp):
                ydot[0] = y[2] # define z1dot
                ydot[1] = y[3] # define z2dot
                ydot[2] = self.m2*f*(self.g - fdot*y[2]**2)/(self.in1 + self.m2*f**2) #define thetaddot
                ydot[3] = -(fdot*y[2]**2 + f*ydot[2])# define z2ddot
                # if ydot[2] < 0:
                #     self.leftRamp = True
                #     self.tStop = t
            else:
                ydot[0] = y[2] # define z1dot
                ydot[1] = y[3] #y[3]#0. # define z2dot
                ydot[2] = 0.#-self.m2*self.g#0. #define z1ddot
                ydot[3] = -self.m2*self.g# define z2ddot
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
            ep1 = potential energy of wheel (zero)
            ep2 = potential energy of mass 2
            ek1 = rotational energy of wheel
            ek2 = kinetic energy of mass 2
        """
        ep1 = 0*self.stVec[:, 0]
        ep2 = self.m2*self.g*self.stVec[:, 1]
        ek1 = 0.5*self.in1*self.stVec[:, 2]**2
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

        axs[0, 0].plot(self.t, self.stVec[:, 0],'r', label='wheel')
        axs[0, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 0].grid()
        axs[0, 0].set_title('Position')
        axs[0, 0].set_ylabel('angle (rad)')
        axs[0, 0].set_xlabel('Time (s)')
        ##axs[0, 0].set_ylim(-ymax, ymax)
        axs[0, 0].legend()

        axs[0, 1].plot(self.t, self.stVec[:, 1],'b', label='mass 2')
        axs[0, 1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0, 1].grid()
        axs[0, 1].set_title('Position')
        axs[0, 1].set_ylabel('Height (m)')
        axs[0, 1].set_xlabel('Time (s)')
        # axs[0, 1].set_ylim(-ymax, ymax)
        axs[0, 1].legend()

        axs[1, 0].plot(self.t, self.stVec[:, 2],'r', label='wheel')
        axs[1, 0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1, 0].grid()
        axs[1, 0].set_title('Angular Velocity')
        axs[1, 0].set_ylabel('Angular Velocity (rad/s)')
        axs[1, 0].set_xlabel('Time (s)')
        # axs[1, 0].set_ylim(-vmax, vmax)
        axs[1, 0].legend()

        axs[1, 1].plot(self.t, self.stVec[:, 3],'b', label='mass 2')
        axs[1, 1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1, 1].grid()
        axs[1, 1].set_title('Velocity')
        axs[1, 1].set_ylabel('Velocity (m/s)')
        axs[1, 1].set_xlabel('Time (s)')
        # axs[1, 1].set_ylim(-vmax, vmax)
        axs[1, 1].legend()

        plt.show()

    def plotEnergy(self):
        """
        Plots the energy of the catapult
        args:
            None
        returns:
            None
        
        """

        ep1, ep2, ek1, ek2 = self.computeEnergy()
        print('Max potential energy: {}'.format(np.max(ep1+ep2)))
        print('Max kinetic energy: {}'.format(np.max(ek1+ek2)))
        #del fig, axs
        fig, axs = plt.subplots(3, 1)
        # Make the figure large
        fig.set_size_inches(18.5, 10.5)

        # Separate the plots
        fig.tight_layout(pad=3.0)
        epmax = 1.1*max(np.max(ep1), np.max(ep2))
        epmin = 1.1*min(np.min(ep1), np.min(ep2))
        ekmax = 1.1*max(np.max(ek1), np.max(ek2))
        ekmin = 1.1*min(np.min(ek1), np.min(ek2))
        axs[0].plot(self.t, ep1,'r', label='wheel')
        axs[0].plot(self.t, ep2,'b', label='mass 2')
        axs[0].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[0].grid()
        axs[0].set_title('Potential Energy')
        axs[0].set_ylabel('Energy (J)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylim(epmin, epmax)
        axs[0].legend()

        axs[1].plot(self.t, ek1,'r', label='wheel')
        axs[1].plot(self.t, ek2,'b', label='mass 2')
        axs[1].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[1].grid()
        axs[1].set_title('Kinetic Energy')
        axs[1].set_ylabel('Energy (J)')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylim(ekmin, ekmax)
        axs[1].legend()

        axs[2].plot(self.t, ep1+ek1,'r', label='wheel')
        axs[2].plot(self.t, ep2+ek2,'b', label='mass 2')
        axs[2].plot(self.t, ep1+ep2+ek1+ek2,'k', label='total')
        axs[2].axvline(self.tStop, color='k', linestyle='--', label='end of ramp')
        axs[2].grid()
        axs[2].set_title('Total Energy')
        axs[2].set_ylabel('Energy (J)')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylim(1.1*min(np.min(ep1+ek1), np.min(ep2+ek2)), 1.1*max(np.max(ep1+ek1), np.max(ep2+ek2)))
        axs[2].legend()

        plt.show()
    
    def plotPhasePortraits(self):
        """
        Plots the phase portraits of the catapult
        args:
            None
        returns:
            None
        """
        plt.plot(self.stVec[:, 0], self.stVec[:, 2], 'r', label='wheel')
        plt.plot(-self.stVec[:, 1], -self.stVec[:, 3], 'b', label='mass 2')
        plt.grid()
        plt.title('Phase Portraits')
        plt.xlabel('Position (m)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.show()
    
    def plotControl(self):
        pass
    
    def plotRatio(self):
        """Plots the ratio function of the catapult
        args:
            None
        returns:
            None"""
        theta = np.linspace(0, self.maxAngle, 100)
        ratio = self.ratio(theta)
        plt.plot(theta, ratio)
        plt.title('Ratio function')
        plt.xlabel('theta (rad)')
        plt.ylabel('Ratio')
        plt.show()

    def plotRatioPolar(self):
        """Plots the ratio function of the catapult in polar coordinates
        args:
            None
        returns:
            None"""
        
        theta = np.linspace(0, self.maxAngle, 1000)
        ratio = self.ratio(theta)
        plt.polar(theta, ratio)
        plt.title('Ratio function in polar coordinates')
        plt.xlabel('theta (rad)')
        plt.ylabel('Ratio')
        #plt.axis('equal')
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
    cata = CatapultWheel(100., 1., 9.81)
    cata.eqGenerator()
    cata.setConditions(0., 0., 100*2*np.pi/60., 0.)
    print('Starting simulation')
    cata.solve(0, 10., 0.001)
    cata.plotRatio()
    cata.plotRatioPolar()
    cata.plot()
    cata.plotEnergy()
    cata.plotPhasePortraits()
    
if __name__ == "__main__":
    main()
    

    

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
        self.k1 = 0.1 
        self.k2 = 0.1
        self.k3 = 10.0
        self.stVec = np.zeros((1, 4))
        self.t = np.zeros(1)
        self.cmd = np.zeros((1,2)) # [F, theta]
        self.cmdTmp = np.zeros(2)
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the drone
        """
        return  "Catapult with mass 1 {} kg and mass 2 {} kg*m^2".format(self.mass1, self.inertia2, self.stifness10, self.stifness23, self.ratio)

    def setConditions(self, z1, z2, zdot1, zdot2):
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
        self.stVec[-1, :] = [z1, z2, zdot1, zdot2]


    
    def control(self, t, y):
        pass
    
    def ratio(self,z):
        return None
    
    def ratiodot(self, z):
        return None
    
    def ratioddot(self, z):
        return None
    


    def phi(z2, zdot2):
        return 0.5*Catapult.m2*zdot2**2 + Catapult.m2*Catapult.g*z2
    
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
            ydot[0] = y[2] # define z1dot
            ydot[1] = y[3] # define z2dot
            ydot[2] = -self.k10/self.m1*y[0] - self.k23*self.r*(-self.r*y[0]-y[1])/self.m1 # -self.k10/self.m1*y[0] + self.r*self.k23/self.m1*(y[1]-y[0]) #define z1ddot
            ydot[3] = self.k23/self.m2*(-self.r*y[0] - y[1])#-self.k23/self.m2*(y[1]-y[0])# define z2ddot
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
        axs[0].set_title('Theta 1')
        axs[0].set_ylabel('angle (rad)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylim(-ymax, ymax)

        axs[1].plot(self.t, self.stVec[:, 1],'b')
        axs[1].set_title('Theta 2')
        axs[1].set_ylabel('angle (rad)')
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
        Calculates the energy of the Catapult
        args:
            None
        returns:
            energy: energy of the Catapult
        """
        pass
    
def main():
    osc = Catapult(10., .1, .1, 10., 5.)
    osc.eqGenerator()
    osc.setConditions(100., 0., 0., 0.)
    print('Starting simulation')
    osc.solve(0, 100, 0.01)
    osc.plot()
    
if __name__ == "__main__":
    main()
    

    

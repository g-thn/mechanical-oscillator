import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the Oscillator class
class Oscillator:
    

    # Define the constructor
    def __init__(self, inertia1, inertia3, stifness10, stifness23, ratio ):
        """
        Constructor for the oscillator class"""
        self.i1 = inertia1
        self.i3 = inertia3
        self.k10 = stifness10
        self.k23 = stifness23
        self.r = ratio
        self.stVec = np.zeros((1, 4))
        self.t = np.zeros(1)
        self.cmd = np.zeros((1,2)) # [F, theta]
        self.cmdTmp = np.zeros(2)
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the drone
        """
        return  "Oscillator with inertia1 {} kg*m^2, inertia2 {} kg*m^2, stifness10 {} N/m, stifness23 {} N/m, and ratio {}".format(self.inertia1, self.inertia2, self.stifness10, self.stifness23, self.ratio)

    def setConditions(self, theta1, theta3, thetadot1, thetadot3):
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
        self.stVec[-1, :] = [theta1, theta3, thetadot1, thetadot1]


    
    def control(self, t, y):
        pass


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
            ydot[0] = y[2] # define theta1dot
            ydot[1] = y[3] # define theta3dot
            ydot[2] = -self.k10/self.i1*y[0] - self.k23*self.r*(-self.r*y[0]-y[1])/self.i1 # -self.k10/self.i1*y[0] + self.r*self.k23/self.i1*(y[1]-y[0]) #define theta1ddot
            ydot[3] = self.k23/self.i3*(-self.r*y[0] - y[1])#-self.k23/self.i3*(y[1]-y[0])# define theta3ddot
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
        Calculates the energy of the oscillator
        args:
            None
        returns:
            energy: energy of the oscillator
        """
        pass
    
def main():
    osc = Oscillator(10., .1, .1, 10., 5.)
    osc.eqGenerator()
    osc.setConditions(100., 0., 0., 0.)
    print('Starting simulation')
    osc.solve(0, 100, 0.01)
    osc.plot()
    
if __name__ == "__main__":
    main()
    

    

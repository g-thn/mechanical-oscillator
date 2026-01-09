import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import RK45
from scipy.integrate import quad

# Define the CarBumper class
class CarBumper:
    

    # Define the constructor
    def __init__(self, mass1, mass2, k, kt,damping,bump_height,bump_width,car_velocity,bumper_position=0.0):
        """
        Constructor for the oscillator class"""
        self.m1 = mass1 # mass of the car body
        self.m2 = mass2 # mass of the wheel
        self.k = k  # stiffness of the spring
        self.kt = kt  # stiffness of the tire
        self.g = 9.81
        self.stVec = np.zeros((1, 4)) # [x1, x2, x1dot, x2dot]
        self.envec = np.zeros((1, 4)) # [kinetic energy mass1, kinetic energy mass2, potential energy spring, potential energy tire]
        self.dmp = damping
        self.bh = bump_height
        self.bw = bump_width
        self.bp = bumper_position
        self.cv = car_velocity
        self.t = np.zeros(1)
        self.cmd = np.zeros(1) # [x1]
        self.cmdTmp = np.zeros(1)
        self.xt_func = self.bumbTimeProfile # function to describe the bump profile in time
        self.eq = None # equations of motion


    def __str__(self):
        """
        Returns a string representation of the drone
        """
        return  "Car with mass {} kg, wheel mass {} kg, stiffness of the spring {} N/m, stiffness of the tire {} N/m, damping {} ".format(self.m1, self.m2, self.k, self.kt, self.dmp)

    def setConditions(self, x1_0, x2_0, x1dot_0, x2dot_0):
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
        self.stVec[-1, :] = [x1_0, x2_0, x1dot_0, x2dot_0]


    
    def control(self, t, y):
        pass

    def bumbSpaceProfile(self, x):
        """
        Returns the bump height at position x
        args:
            x: position in m
        returns:
            y: bump height at position x in m
        """
        if (x >= self.bp) and (x < (self.bp+self.bw)):
            return self.bh*(1-np.cos((x-self.bp)*2*np.pi/(self.bw)))/2
        else:
            return 0
    
    def bumbTimeProfile(self, t):
        """
        Returns the bump height at time t
        args:
            t: time in s
        returns:
            y: bump height at time t in m
        """
        x = self.cv*t
        return self.bumbSpaceProfile(x)

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
            ydot[0] = y[2] # define x1dot
            ydot[1] = y[3] # define x2dot
            ydot[2] = 1./self.m1*(-self.k*(y[0]-y[1]) - self.dmp*(y[2] - y[3])) - self.g # define x3dot
            ydot[3] = 1./self.m2*(self.k*(y[0]-y[1]) + self.dmp*(y[2] - y[3]) - self.kt*(y[1] - self.xt_func(t))) - self.g # define x4dot
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
        First line of graph is position, second line is velocity
        args:
            None
        returns:
            None
        """
        fig, axes = plt.subplots(2, 2)
        fig.set_size_inches(18.5, 10.5)
        axes[0, 0].plot(self.t, self.stVec[:, 0], 'r')
        axes[0, 0].plot(self.t, [self.xt_func(ti) for ti in self.t], 'k--')
        axes[0, 0].set_title('Position x1')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[1, 0].plot(self.t, self.stVec[:, 2], 'b')
        axes[1, 0].set_title('Velocity x1dot')
        axes[1, 0].set_ylabel('Velocity (m/s)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[0, 1].plot(self.t, self.stVec[:, 1], 'r')
        axes[0, 1].plot(self.t, [self.xt_func(ti) for ti in self.t], 'k--')
        axes[0, 1].set_title('Position x2')
        axes[0, 1].set_ylabel('Position (m)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[1, 1].plot(self.t, self.stVec[:, 3], 'b')
        axes[1, 1].set_title('Velocity x2dot')
        axes[1, 1].set_ylabel('Velocity (m/s)')
        axes[1, 1].set_xlabel('Time (s)')

        plt.show()
    
    def plotEnergy(self):
        """
        Plots the energy of the oscillator
        args:
            None
        returns:
            None
        """
        self.energy()
        fig, axes = plt.subplots(2, 3)
        fig.set_size_inches(18.5, 10.5)
        axes[0, 0].plot(self.t, self.envec[:, 0], 'r')
        axes[0, 0].set_title('Kinetic Energy mass 1')
        axes[0, 0].set_ylabel('Energy (J)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[1, 0].plot(self.t, self.envec[:, 1], 'b')
        axes[1, 0].set_title('Kinetic Energy mass 2')
        axes[1, 0].set_ylabel('Energy (J)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[0, 1].plot(self.t, self.envec[:, 2], 'r')
        axes[0, 1].set_title('Potential Energy spring')
        axes[0, 1].set_ylabel('Energy (J)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[1, 1].plot(self.t, self.envec[:, 3], 'b')
        axes[1, 1].set_title('Potential Energy tire')
        axes[1, 1].set_ylabel('Energy (J)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[0, 2].plot(self.t, self.envec[:, 0] + self.envec[:, 2], 'k')
        axes[0, 2].set_title('Total Energy of the car body')
        axes[0, 2].set_ylabel('Energy (J)')
        axes[0, 2].set_xlabel('Time (s)') 
        axes[1, 2].plot(self.t, self.envec[:, 1] + self.envec[:, 3], 'k')
        axes[1, 2].set_title('Total Energy of the tire')
        axes[1, 2].set_ylabel('Energy (J)')
        axes[1, 2].set_xlabel('Time (s)') 

        plt.show()

    def plotBumpProfile(self,t_sim = 50,nb_points=1000):
        """
        Plot the bump profile in space and time
        args:
            None
        returns:
            None
        """
        t = np.linspace(0, t_sim, nb_points)
        x = t*self.cv
        y = [self.bumbSpaceProfile(x) for x in x]
        fig, axes = plt.subplots(2, 1)
        fig.set_size_inches(18.5, 10.5)
        axes[0].plot(x, y,'r')
        axes[0].set_title('Bump profile in space')
        axes[0].set_xlabel('Position (m)')
        axes[0].set_ylabel('Bump height (m)')
        y = [self.bumbTimeProfile(t) for t in t]
        axes[1].plot(t, y,'b')
        axes[1].set_title('Bump profile in time')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Bump height (m)')
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
        ke1 = 0.5*self.m1*self.stVec[:,2]**2
        ke2 = 0.5*self.m2*self.stVec[:,3]**2
        pe_spring = 0.5*self.k*(self.stVec[:,0]-self.stVec[:,1])**2
        pe_tire = 0.5*self.kt*(self.stVec[:,1]-np.array([self.xt_func(ti) for ti in self.t]))**2
        self.envec = np.vstack((ke1,ke2,pe_spring,pe_tire)).T
        return self.envec
    
def main():
    t_sim = 10.0
    bumper = CarBumper(mass1=465., mass2=50., k=5700., kt=135.e3, damping=450.,bump_height=.2,bump_width=.50,car_velocity=5.,bumper_position=20.0)
    print(bumper)
    bumper.g = 0.0  # set gravity to zero for this simulation
    bumper.setConditions(x1_0=0.0, x2_0=0.0, x1dot_0=0.0, x2dot_0=0.0)
    #bumper.plotBumpProfile(t_sim=t_sim,nb_points=1000)   
    bumper.solve(t0=0.0, tf=t_sim, dt=0.01)
    bumper.plot()
    bumper.plotEnergy()


    
if __name__ == "__main__":
    main()
    

    

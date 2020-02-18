import numpy as np
class KalmanFilterEq():

    '''
        Initialize the required terms used in the Extended Kalman Filter equations.
        Set P - Uncertainity covariance matrix of state(x)
        Q - Covariance matrix of noise
        F - Update matrix
        n - Number states to estimate
        x - state vector
    '''
    def __init__(self,n):
        self.n=n
        self.identity= np.matrix(np.eye(n))
        self.P=None
        self.Q=None
        self.F=None
        self.x=None

    '''
        Prediction step using the extended kalman equations:
        x(n)= F * x(n-1)
        P(n)= F * P(n-1) * F^T + Q
    '''
    def predict(self):
        self.x = self.F * self.x
        self.P= self.F * self.P * self.F.T + self.Q


    '''
        Updation of x and P using Kalman equations:
        H - Extraction Matrix to extract the measurement of x if sensor is perfect
        y - Difference between actual measurements and predicted measurements
        PHt= S * H^T
        S = H * P * H^T + R (Innovation term)
        K = P * H^T * S^-1 (Kalman Gain)
        x(n) = x(n-1) + K * y
        P(n) = (I - K*H) * P(n-1)
    '''
    def update(self,z,H,Hx,R):
        y=z-Hx
        PHt= self.P *H.T
        S=  H*PHt + R
        K= PHt * (S.I)
        self.x = self.x + K * y
        self.P = (self.I - K * H) * self.P



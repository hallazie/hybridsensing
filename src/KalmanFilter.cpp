#include "KalmanFilter.h"
#include "OdomModel.h"

namespace hybridnavi {
namespace kalmanfilter {
bool KalmanFilter::setAngleMask(const State& mask) {
    this->angleMask = mask;
    return true;
}
bool KalmanFilter::angleModify(State& a, State& b, double half_period) {
    for(int i = 0;i<angleMask.size();i++) {
        if(angleMask[i]>0.5) {
double t = a[i];
double tb= b[i];
	 while(t<=-half_period) t+=(2*half_period);
	 while(t>half_period) t-=(2*half_period);
	 while(tb<=-half_period) tb+=(2*half_period);
	 while(tb>half_period) tb-=(2*half_period);
            while((t-tb)> half_period) tb+=(2* half_period);
            while((tb-t)>= half_period) tb-=(2* half_period);
		b[i] = tb;
		a[i] = t;
        }
    }
    return true;
}
bool KalmanFilter::controlUpdate(const ProbaLocation& lastX, const State& u,
                                 ProbaLocation& X_hat) {
    if(NULL == om) return false;
    om->predictLocation(lastX,u[0],u[1],X_hat.X );
    Covariance Q;
    Covariance A;
    om->getQMatrix(lastX, u[0],u[1],Q);
    om->getAMatrix(lastX, u[0],u[1],A);
    X_hat.P =A*lastX.P*A.transpose() + Q;// A*P*AT + Q
    return true;
}

bool KalmanFilter::controlUpdate(const ProbaLocation& lastX,
                                 const State& u,
                                 const Covariance& A,
                                 const Covariance& Q,
                                 ProbaLocation& X_hat) {
    if(NULL == om) return false;
    om->predictLocation(lastX,u[0],u[1],X_hat.X );
    X_hat.P =A*lastX.P*A.transpose() + Q;// A*P*AT + Q
    return true;
}
bool KalmanFilter::observationUpdate(const ProbaLocation& x_hat,
                                     const ProbaLocation& y,
                                     ProbaLocation& x) {
    this->gaussianFusion(x_hat, y, x);
    return true;
}
bool KalmanFilter::gaussianFusion(const ProbaLocation& x_hat,
                                     const ProbaLocation& y,
                                     ProbaLocation& x) {
    State yy = y.X;
    State xx = x_hat.X;
    this->angleModify(xx,yy,M_PI);
    Covariance K = x_hat.P*((x_hat.P + y.P).inverse());
	for(int ii=0;ii<3;ii++) {
if(K(ii,ii)<0) K(ii,ii) = 0;
if(K(ii,ii)>1) K(ii,ii) = 1;
}
    x.X = xx + K*(yy - xx);
    x.P = x_hat.P - K*x_hat.P;
    return true;
}
bool KalmanFilter::changeOM(OdomModel* o) {
    if (NULL == this->om) {
        this->om = o;
        return false;
    } else {
        delete this->om;
        this->om = o;
        return true;
    }
}
}
}

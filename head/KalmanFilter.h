#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include "ProbaLocation.h"
#include "OdomModel.h"

namespace hybridnavi {
namespace kalmanfilter {
class KalmanFilter {
public:
    KalmanFilter():om(NULL){};
    bool controlUpdate(const ProbaLocation&,const State&, const Covariance&, const Covariance&, ProbaLocation& );
    bool controlUpdate(const ProbaLocation&,const State&, ProbaLocation& );
    bool observationUpdate(const ProbaLocation&, const ProbaLocation&, ProbaLocation&);
    bool changeOM(OdomModel* om );
    bool setAngleMask(const State &);
    ~KalmanFilter(){
        if(NULL != om) {
            delete om;
            om = NULL;
        }
    }
private:
    OdomModel* om;
    State angleMask;
    bool gaussianFusion(const ProbaLocation&, const ProbaLocation&, ProbaLocation&);
    bool angleModify(State&, State&,double );
};
}
}
#endif//KALMANFILTER_H_

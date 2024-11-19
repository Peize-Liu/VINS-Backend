/* solver.h
    wrapper for different sovlers
    input feature manager. and system status

*/
#include <stdio.h>

namespace OdomBackendSovler{

enum SolverType{
    NLS,
    Filter
};

enum SolverStatus{
    INIT,
    OPTIMIZATION,
    MARGINALIZATION,
    RESET
};

class NLS
{
private:
    /* data */
public:
    NLS(/* args */);
    ~NLS();
    void Optimization(); //input feature manager, sliding window status
    void Marginalization();//
    void Reset();
};

class Filter{

};

} // namespace OdomBackendSovler
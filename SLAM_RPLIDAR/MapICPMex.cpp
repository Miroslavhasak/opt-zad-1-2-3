#include "mex.h"
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "ICP.h"

using MapHandle = std::unique_ptr<Map>;

//------------------------------------------------------------------------------
// Helpers
//------------------------------------------------------------------------------
static MapHandle* getMapHandle(const mxArray* mx) {
    return reinterpret_cast<MapHandle*>(*reinterpret_cast<uint64_t*>(mxGetData(mx)));
}

static void setHandle(mxArray*& out, MapHandle* ptr) {
    out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *reinterpret_cast<uint64_t*>(mxGetData(out)) = reinterpret_cast<uint64_t>(ptr);
}

//------------------------------------------------------------------------------
// Commands
//------------------------------------------------------------------------------
static void cmdNew(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {

    double voxel_size = mxGetScalar(prhs[1]);
    double prob_hit = mxGetScalar(prhs[2]);
    double prob_miss = mxGetScalar(prhs[3]);
    setHandle(plhs[0], new MapHandle(new Map(voxel_size,prob_hit,prob_miss)));
}

static void cmdDelete(MapHandle* mapPtr) {
    delete mapPtr;
}


static void cmdErase(Map& map, int nrhs, const mxArray* prhs[]) {

    map.eraseBelowProb(mxGetScalar(prhs[2]));
}

static void cmdIsEmpty(Map& map, mxArray* plhs[]) {
    plhs[0] = mxCreateLogicalScalar(map.isEmpty());
}

static void cmdAlignScan(Map& map, int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {

plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
plhs[1] = mxCreateDoubleMatrix(3, 3, mxREAL);

mwSize n = mxGetM(prhs[2]);

Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor>> scan(mxGetPr(prhs[2]), n, 3);

Eigen::Map<Eigen::Vector3d> t_init(
  mxGetPr(prhs[4])); 

Eigen::Map<Eigen::Matrix3d> R_init(
   mxGetPr(prhs[5]));

Eigen::Map<Eigen::Vector3d> t_hat_map(
  mxGetPr(plhs[0]));

Eigen::Map<Eigen::Matrix3d> R_hat_map(
  mxGetPr(plhs[1]));    


 Eigen::Vector3d t_hat=t_init;
 Eigen::Matrix3d R_hat=R_init;
    

if (map.isEmpty())
{   
    std::vector<Point> first_scan;
    int N = scan.rows();
    for (int i = 0; i < N; ++i) {
        first_scan.emplace_back(R_hat * (scan.row(i).transpose())+t_hat);
    }

    map.addPoints(t_hat,first_scan);
    t_hat_map=t_hat;    
    R_hat_map=R_hat; 
    return;
}   

double max_corr_dist = mxGetScalar(prhs[3]);

double GN_tol = mxGetScalar(prhs[6]);
double ICP_tol_pos = mxGetScalar(prhs[7]);   
double ICP_tol_ori = mxGetScalar(prhs[8]);   

 std::vector<Point> pA_aligned= ICP_map_estimate(
    scan,  
    map,
    R_hat,
    t_hat,
    max_corr_dist,
    GN_tol,
    ICP_tol_pos,
    ICP_tol_ori);

    map.addPoints(t_hat,pA_aligned);

    t_hat_map=t_hat;    
    R_hat_map=R_hat; 

}


static void cmdGetPoints(Map& map, mxArray* plhs[], int nrhs) {

mwSize n = map.numOccupiedPoints();

plhs[0] = mxCreateDoubleMatrix(3, n, mxREAL);
plhs[1] = mxCreateDoubleMatrix(1, n, mxREAL);

Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> outPts(mxGetPr(plhs[0]), 3, n);
Eigen::Map<Eigen::RowVectorXd> outProbs(mxGetPr(plhs[1]), n);

mwSize i = 0;
for (auto it = map.begin(); it != map.end(); ++it) {
    const Point& pt = *it;   
    outPts.col(i) = pt;     
    outProbs(i)   = it.occupancy();
    i++;
}

}    
//------------------------------------------------------------------------------
// Entry point
//------------------------------------------------------------------------------
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
   
    std::string cmd = mxArrayToString(prhs[0]);

    if (cmd == "new") {
        cmdNew(nlhs, plhs, nrhs, prhs);
        return;
    }

    auto* mapPtr = reinterpret_cast<MapHandle*>(*reinterpret_cast<uint64_t*>(mxGetData(prhs[1])));
    if (!mapPtr || !(*mapPtr))
        mexErrMsgIdAndTxt("MapMex:InvalidHandle", "Invalid Map handle.");

    Map& map = **mapPtr;

    if (cmd == "delete") cmdDelete(mapPtr);
    else if (cmd == "erase") cmdErase(map, nrhs, prhs);
    else if (cmd == "isEmpty") cmdIsEmpty(map, plhs);
    else if (cmd == "getPoints") cmdGetPoints(map, plhs, nrhs);
    else if (cmd == "alignScan") cmdAlignScan(map, nlhs,plhs,nrhs,prhs);
    else mexErrMsgIdAndTxt("MapMex:InvalidCommand", "Unknown command.");
}
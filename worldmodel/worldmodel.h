#ifndef WORLDMODEL_H
#define WORLDMODEL_H

#include "../headers/headers.h"
#include "../math/Geometry.h"
#include "../math/hctmatrix.h"
#include "WorldObject.h"
#include "../headers/Field.h"
#include "../rvdraw/rvdraw.h"

class BallKF;
class PlayerKF;

using namespace std;

class WorldModel {

private:

    unsigned long cycle;
    int scoreLeft;
    int scoreRight;
    double time;
    double gameTime;
    int playMode;
    int lastPlayMode;
    int lastDifferentPlayMode;
    int uNum;
    int side;
    

    bool uNumSet;
    bool sideSet;

    
    WorldObject worldObjects[NUM_WORLD_OBJS];
    VecPosition myPosition;
    SIM::AngDeg myAngDegrees;
    bool confident;

    VecPosition myLastPosition;
    SIM::AngDeg myLastAngDegrees;
    RVSender *rvsend;
    bool fUseGroundTruthDataForLocalization;
    // TODO: comment it out if we don't want ground truth
#define GROUND_TRUTH_SERVER
#ifdef GROUND_TRUTH_SERVER
    VecPosition myPositionGroundTruth;
    SIM::AngDeg myAngGroundTruth;
    VecPosition ballGroundTruth;
#endif

    double lastBallSightingTime;
    double lastLineSightingTime;

    // This is where we actually saw the ball ourself
    vector<VecPosition> lastBallSeenPosition;
    vector<double> lastBallSeenTime;

    // remember last two skills
    vector<SkillType> lastSkills;
    vector<SkillType> executedSkillsForOdometry;

    // record last odometry from particle filter
    SIM::Point2D lastOdometryPos;
    double lastOdometryAngDeg;

    bool fLocalized;
    bool fFallen;

    //cx 2018-5-1
    bool turnEasy;
    vector<bool> turnEasyTeammate;
    //cx 2018-5-6
    int chooseCF;
    //cx 2018-5-16
    int chooseCF_noBall;
    int chooseFF;
    int chooseStopper;
    int chooseWL;
    int chooseWR;
    vector<int> CFVoteResult;
    vector<int> CF_noBallVoteResult;
    vector<int> FFVoteResult;
    vector<int> StopperVoteResult;
    vector<int> WLVoteResult;
    vector<int> WRVoteResult;
    
    vector<VecPosition> ballPosTeammate;
    
    HCTMatrix localToGlobal, globalToLocal;

    vector<bool> fallenTeammate;
    vector<bool> fallenOpponent;

    string opponentTeamName;

    BallKF* ballKalmanFilter;
    PlayerKF* opponentKalmanFilters;

public:

    WorldModel();
    ~WorldModel();

    inline void setMyConfidence(bool confidence) {
        confident = confidence;
    }
    inline bool getMyConfidence() const {
        return confident;
    }

    inline WorldObject* getWorldObject( int index ) {
        return &worldObjects[index];
    }
    void updateGoalPostsAndFlags();
    void updateMatricesAndMovingObjs( VecPosition& fieldXPlusYPlus,
                                      VecPosition& fieldXPlusYMinus,
                                      VecPosition& fieldXMinusYPlus,
                                      VecPosition& fieldXMinusYMinus );


    inline void setMyPosition( const VecPosition& newPos ) {
        myPosition = newPos;
    }
    inline void setMyPosition( const SIM::Point2D& newPos ) {
        myPosition.setX( newPos.getX() );
        myPosition.setY( newPos.getY() );
        // Z is not changed, stays at the default
    }
    inline VecPosition getMyPosition() const {
        return myPosition;
    }


    inline void setMyAngDeg( SIM::AngDeg newAng ) {
        myAngDegrees = newAng;
    }
    inline void setMyAngRad( SIM::AngRad newAng ) {
        myAngDegrees = Rad2Deg( newAng );
    }
    inline SIM::AngDeg getMyAngDeg() const {
        return myAngDegrees;
    }
    inline SIM::AngRad getMyAngRad() const {
        return Deg2Rad( myAngDegrees );
    }


    inline void setMyLastPosition( const VecPosition& newPos ) {
        myLastPosition = newPos;
    }
    inline VecPosition getMyLastPosition() const {
        return myLastPosition;
    }


    inline void setMyLastAngDeg( SIM::AngDeg newAng ) {
        myLastAngDegrees = newAng;
    }
    inline void setMyLastAngRad( SIM::AngRad newAng ) {
        myLastAngDegrees = Rad2Deg( newAng );
    }
    inline SIM::AngDeg getMyLastAngDeg() const {
        return myLastAngDegrees;
    }
    inline SIM::AngRad getMyLastAngRad() const {
        return Deg2Rad( myLastAngDegrees );
    }

    inline RVSender *getRVSender() const {
        return rvsend;
    }

    inline void setUseGroundTruthDataForLocalization(bool fUseGroundTruthDataForLocalization) {
        this->fUseGroundTruthDataForLocalization = fUseGroundTruthDataForLocalization;
    }
    inline bool useGroundTruthDataForLocalization() {
        return fUseGroundTruthDataForLocalization;
    }

#ifdef GROUND_TRUTH_SERVER
    inline void setMyPositionGroundTruth( const VecPosition& newPos ) {
        myPositionGroundTruth = newPos;
    }
    inline VecPosition getMyPositionGroundTruth() const {
        return myPositionGroundTruth;
    }

    inline void setMyAngDegGroundTruth( double angDeg ) {
        myAngGroundTruth = angDeg;
    }
    inline double getMyAngDegGroundTruth() const {
        return myAngGroundTruth;
    }

    inline void setBallGroundTruth( VecPosition newBall ) {
        ballGroundTruth = newBall;
    }
    inline VecPosition getBallGroundTruth() const {
        return ballGroundTruth;
    }
#endif

    inline void setLastBallSightingTime(double lastTime) {
        lastBallSightingTime = lastTime;
    }
    inline double getLastBallSightingTime() const {
        return lastBallSightingTime;
    }

    inline void setLastLineSightingTime(double lastTime) {
        lastLineSightingTime = lastTime;
    }
    inline double getLastLineSightingTime() const {
        return lastLineSightingTime;
    }

    inline void setLastBallSeenPosition(VecPosition position) {
        lastBallSeenPosition.insert(lastBallSeenPosition.begin(),position);
        lastBallSeenPosition.pop_back();
    }
    inline vector<VecPosition> getLastBallSeenPosition() const {
        return lastBallSeenPosition;
    }

    inline void setLastBallSeenTime(double time) {
        lastBallSeenTime.insert(lastBallSeenTime.begin(),time);
        lastBallSeenTime.pop_back();
    }

    inline vector<double> getLastBallSeenTime() const {
        return lastBallSeenTime;
    }

    inline unsigned long getCycle() const {
        return cycle;
    }
    inline void incrementCycle() {
        cycle++;
    }
    inline void setCycle(const unsigned long &cycle) {
        this->cycle = cycle;
    }

    inline int getScoreLeft() const {
        return scoreLeft;
    }
    inline void setScoreLeft(const int &scoreLeft) {
        this->scoreLeft = scoreLeft;
    }

    inline int getScoreRight() const {
        return scoreRight;
    }
    inline void setScoreRight(const int &scoreRight) {
        this->scoreRight = scoreRight;
    }

    inline double getTime() const {
        return time;
    }
    inline void setTime(const double &time) {
        this->time = time;
    }

    inline double getGameTime() const {
        return gameTime;
    }
    inline void setGameTime(const double &gameTime) {
        this->gameTime = gameTime;
    }

    inline int getPlayMode() const {
        return playMode;
    }
    inline void setPlayMode(const int &playMode) {
        this->playMode = playMode;
    }

    inline int getLastPlayMode() const {
        return lastPlayMode;
    }
    inline void setLastPlayMode(const int &lastPlayMode) {
        this->lastPlayMode = lastPlayMode;
    }

    inline int getLastDifferentPlayMode() const {
        return lastDifferentPlayMode;
    }
    inline void setLastDifferentPlayMode(const int &lastDifferentPlayMode) {
        this->lastDifferentPlayMode = lastDifferentPlayMode;
    }

    inline int getUNum() const {
        return uNum;
    }
    inline void setUNum(const int &uNum) {
        this->uNum = uNum;
    }

    inline bool getUNumSet() const {
        return uNumSet;
    }
    inline void setUNumSet(const bool &uNumSet) {
        this->uNumSet = uNumSet;
    }


    inline SkillType getLastSkill() const {
        return lastSkills[0];
    }
    inline SkillType getPreviousLastSkill() const {
        return lastSkills[1];
    }
    inline void setLastSkill(const SkillType &lastSkill) {
        this->lastSkills[1] = this->lastSkills[0];
        this->lastSkills[0] = lastSkill;
    }



    // functions for odometry
    inline void addExecutedSkill(const SkillType &skill) {
        executedSkillsForOdometry.push_back( skill );
    }
    inline const vector<SkillType>& getExecutedSkills() const {
        return executedSkillsForOdometry;
    }
    inline void resetExecutedSkills() {
        executedSkillsForOdometry.clear();
    }

    // tracking odometry
    inline SIM::Point2D& getLastOdometryPos() {
        return lastOdometryPos;
    }
    inline void setLastOdometryPos(const SIM::Point2D& pos) {
        lastOdometryPos = pos;
    }

    inline double& getLastOdometryAngDeg() {
        return lastOdometryAngDeg;
    }
    inline void setLastOdometryAngDeg(const double& ang) {
        lastOdometryAngDeg = ang;
    }

    // THIS IS A SINGLE POINT DETERMINES WHETHER USING BALL KALMAN FILTER
    inline bool useKalmanFilter() {
        return true;
    }


    inline int getSide() {
        return side;
    }
    inline void setSide(const int &side) {
        this->side = side;
        updateGoalPostsAndFlags();
    }

    inline bool getSideSet() {
        return sideSet;
    }
    inline void setSideSet(const bool &sideSet) {
        this->sideSet = sideSet;
    }

    inline VecPosition getGoalPost(const int &i) const {
        return worldObjects[GOALPOST_1_L  + i].pos;
    };

    inline VecPosition getMyLeftGoalPost() {
        return ((side == SIDE_LEFT)? worldObjects[GOALPOST_1_L].pos : worldObjects[GOALPOST_2_R].pos );
    }
    inline VecPosition getMyRightGoalPost() {
        return ((side == SIDE_LEFT)? worldObjects[GOALPOST_2_L].pos : worldObjects[GOALPOST_1_R].pos);
    }
    inline VecPosition getOppLeftGoalPost() {
        return ((side == SIDE_LEFT)? worldObjects[GOALPOST_1_R].pos : worldObjects[GOALPOST_2_L].pos);
    }
    inline VecPosition getOppRightGoalPost() {
        return ((side == SIDE_LEFT)? worldObjects[GOALPOST_2_R].pos : worldObjects[GOALPOST_1_L].pos );
    }

    inline double distanceToOppGoal(VecPosition &p) {

        VecPosition oppLeftGoalPost = getOppLeftGoalPost();
        VecPosition oppRightGoalPost = getOppRightGoalPost();

        if(p.getY() > oppLeftGoalPost.getY()) {
            return p.getDistanceTo(oppLeftGoalPost);
        }
        else if(p.getY() < oppRightGoalPost.getY()) {
            return p.getDistanceTo(oppRightGoalPost);
        }
        else {
            return fabs(oppLeftGoalPost.getX() - p.getX());
        }
    }

    inline double distanceToMyGoal(VecPosition &p) {

        VecPosition myLeftGoalPost = getMyLeftGoalPost();
        VecPosition myRightGoalPost = getMyRightGoalPost();

        if(p.getY() > myLeftGoalPost.getY()) {
            return p.getDistanceTo(myLeftGoalPost);
        }
        else if(p.getY() < myRightGoalPost.getY()) {
            return p.getDistanceTo(myRightGoalPost);
        }
        else {
            return fabs(myLeftGoalPost.getX() - p.getX());
        }
    }


    inline VecPosition getBall() const {
        return worldObjects[WO_BALL].pos;
    };
    inline void setBall(const VecPosition &ball) {
        worldObjects[WO_BALL].pos = ball;
    }

    inline VecPosition getBallWrtTorso() const {
        return g2l(getBall());
    }

    inline VecPosition getTeammate(const int &i) const {
        return worldObjects[i].pos;
    };
    inline void setTeammate(const int &i, const VecPosition &teammate) {
        worldObjects[i].pos = teammate;
    }

    inline VecPosition getOpponent(const int &i) const {
        return worldObjects[i].pos;
    };
    inline void setOpponent(const int &i, const VecPosition &opponent) {
        worldObjects[i].pos = opponent;
    }

    inline void setObjectPosition(const int &i, const VecPosition &pos) {
        worldObjects[i].pos = pos;
    }


    inline void setGlobalToLocal(const int &i, const int &j, const double &v) {
        this->globalToLocal.setCell(i, j, v);
    }

    inline void setLocalToGlobal(const int &i, const int &j, const double &v) {
        this->localToGlobal.setCell(i, j, v);
    }


    inline bool isLocalized() const {
        return fLocalized;
    }
    inline void setLocalized(bool fLocalized) {
        this->fLocalized = fLocalized;
    }

    inline bool isFallen() const {
        return fFallen;
    }
    inline void setFallen(bool fFallen) {
        this->fFallen = fFallen;
    }
    
    inline bool isTurnEasy() const{
        return turnEasy;
    }
    inline void setTurnEasy(bool turnEasy){
        this->turnEasy=turnEasy;
    }
    inline int getChooseCF() const{
        return chooseCF;
    }
    inline void setChooseCF(int chooseCF){
        this->chooseCF=chooseCF;
    }
    inline int getChooseCF_noBall() const{
        return chooseCF_noBall;
    }
    inline void setChooseCF_noBall(int chooseCF_noBall){
        this->chooseCF_noBall=chooseCF_noBall;
    }
    inline int getChooseFF() const{
        return chooseFF;
    }
    inline void setChooseFF(int chooseFF){
        this->chooseFF=chooseFF;
    }
    //------------cx, batch
    inline int getChooseStopper() const{
        return chooseStopper;
    }
    inline int getChooseWL() const{
        return chooseWL;
    }
    inline int getChooseWR() const{
        return chooseWR;
    }
    inline void setChooseStopper(int chooseStopper){
        this->chooseStopper=chooseStopper;
    }
    inline void setChooseWL(int chooseWL){
        this->chooseWL=chooseWL;
    }
    inline void setChooseWR(int chooseWR){
        this->chooseWR=chooseWR;
    }
    //-----------
    
    inline VecPosition g2l(const VecPosition &global) const {
        return globalToLocal.transform(global);
    }
    inline VecPosition l2g(const VecPosition &local) const {
        return localToGlobal.transform(local);
    }


    inline bool canTrustVision() {
        return fLocalized && getMyPosition().getDistanceTo(getMyLastPosition()) < .2;
    }

    bool getTurnEasyTeammate(int index) const{
        return turnEasyTeammate[index];
    }
    void setTurnEasyTeammate(int index, bool turnEasy){
        turnEasyTeammate[index]=turnEasy;
    }
    int getCFVoteResult(int index) const{
        return CFVoteResult[index];
    }
    void setCFVoteResult(int index, int chooseCF){
        CFVoteResult[index]=chooseCF;
    }
    //----cx,batch
    int getCF_noBall_VoteResult(int index) const{
        return CF_noBallVoteResult[index];
    }
    int getFFVoteResult(int index) const{
        return FFVoteResult[index];
    }
    int getStopperVoteResult(int index) const{
        return StopperVoteResult[index];
    }
    int getWLVoteResult(int index) const{
        return WLVoteResult[index];
    }
    int getWRVoteResult(int index) const{
        return WRVoteResult[index];
    }
    void setCF_noBallVoteResult(int index, int chooseCF_noBall){
        CF_noBallVoteResult[index]=chooseCF_noBall;
    }
    void setFFVoteResult(int index, int chooseFF){
        FFVoteResult[index]=chooseFF;
    }
    void setStopperVoteResult(int index, int chooseStopper){
        StopperVoteResult[index]=chooseStopper;
    }
    void setWLVoteResult(int index, int chooseWL){
        WLVoteResult[index]=chooseWL;
    }
    void setWRVoteResult(int index, int chooseWR){
        WRVoteResult[index]=chooseWR;
    }
    //----
    VecPosition getBallPosTeammate(int index) const{
        return ballPosTeammate[index];
    }
    void setBallPosTeammate(int index, VecPosition ballpos){
        ballPosTeammate[index]=ballpos;
    }
    void setBallPosTeammate_X(int index, double bPosX){
        ballPosTeammate[index].setX(bPosX);
    }
    void setBallPosTeammate_Y(int index, double bPosY){
        ballPosTeammate[index].setY(bPosY);
    }
    
    bool getFallenTeammate(int index) const {
        return fallenTeammate[index];
    }
    void setFallenTeammate(int index, bool fFallen) {
        fallenTeammate[index] = fFallen;
    }
    bool getFallenOpponent(int index) const {
        return fallenOpponent[index];
    }
    void setFallenOpponent(int index, bool fFallen) {
        fallenOpponent[index] = fFallen;
    }

    string getOpponentTeamName() const {
        return opponentTeamName;
    }
    void setOpponentTeamName(string name) {
        opponentTeamName = name;
    }

    BallKF* getBallKalmanFilter() const {
        return ballKalmanFilter;
    }
    PlayerKF* getOpponentKalmanFilters() const {
        return opponentKalmanFilters;
    }

    int lastCF; //cx
    int myRole;
    
    void display();
};

#endif // WORLDMODEL_H


#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
extern int agentBodyType;

/*
 * Real game beaming.
 * Filling params x y angle
 */
//void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
//    beamX = -HALF_FIELD_X + worldModel->getUNum();
//    beamY = 0;
//    beamAngle = 0;
//}
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    switch(worldModel->getUNum()){
    case 1:
        beamX=-HALF_FIELD_X+0.2;
        beamY=0;
        beamAngle=0;
        break;
    case 2:
        beamX=-HALF_FIELD_X+2;
        beamY=GOAL_Y/2;
        beamAngle=0;
        break;
    case 3:
        beamX=-HALF_FIELD_X+2;
        beamY=-GOAL_Y/2;
        beamAngle=0;
        break;
    case 4:
        beamX=-HALF_FIELD_X+5;
        beamY=0;
        beamAngle=0;
        break;
    case 5:
        beamX=-HALF_FIELD_X+7;
        beamY=1.5;
        beamAngle=0;
        break;
    case 6:
        beamX=-HALF_FIELD_X+7;
        beamY=-3;
        beamAngle=0;
        break;
    case 7:
        beamX=-HALF_FIELD_X+11;
        beamY=4.5;
        beamAngle=0;
        break;
    case 8:
        beamX=-HALF_FIELD_X+11;
        beamY=-7.5;
        beamAngle=0;
	break;
    case 9:
        beamX=-HALF_FIELD_X+14;
        beamY=6.3;
        beamAngle=0;
        break;
    case 10:
        beamX=-HALF_FIELD_X+12.6;
        beamY=0;
        beamAngle=0;
        break;
    case 11:
        beamX=-HALF_FIELD_X+14.5;
        beamY=-2.4;
        beamAngle=0;
        break;

    }
}

SkillType NaoBehavior::selectSkill() {
    // My position and angle
    //cout << worldModel->getUNum() << ": " << worldModel->getMyPosition() << ",\t" << worldModel->getMyAngDeg() << "\n";

    // Position of the ball
    //cout << worldModel->getBall() << "\n";

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Agents draw the position of where they think the ball is
    // Also see example in naobahevior.cc for drawing agent position and
    // orientation.
    /*
    worldModel->getRVSender()->clear(); // erases drawings from previous cycle
    worldModel->getRVSender()->drawPoint("ball", ball.getX(), ball.getY(), 10.0f, RVSender::MAGENTA);
    */

    // ### Demo Behaviors ###

    // Walk in different directions
    //return goToTargetRelative(VecPosition(1,0,0), 0); // Forward
    //return goToTargetRelative(VecPosition(-1,0,0), 0); // Backward
    //return goToTargetRelative(VecPosition(0,1,0), 0); // Left
    //return goToTargetRelative(VecPosition(0,-1,0), 0); // Right
    //return goToTargetRelative(VecPosition(1,1,0), 0); // Diagonal
    //return goToTargetRelative(VecPosition(0,1,0), 90); // Turn counter-clockwise
    //return goToTargetRelative(VecPdosition(0,-1,0), -90); // Turn clockwise
    //return goToTargetRelative(VecPosition(1,0,0), 15); // Circle

    // Walk to the ball
    //return goToTarget(ball);

    // Turn in place to face ball
    /*double distance, angle;
    getTargetDistanceAndAngle(ball, distance, angle);
    if (abs(angle) > 10) {
      return goToTargetRelative(VecPosition(), angle);
    } else {
      return SKILL_STAND;
    }*/

    // Walk to ball while always facing forward
    //return goToTargetRelative(worldModel->g2l(ball), -worldModel->getMyAngDeg());

    // Dribble ball toward opponent's goal
    //return kickBall(KICK_DRIBBLE, VecPosition(HALF_FIELD_X, 0, 0));

    // Kick ball toward opponent's goal
    //return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0)); // Basic kick
    //return kickBall(KICK_IK, VecPosition(HALF_FIELD_X, 0, 0)); // IK kick

    // Just stand in place
    //return SKILL_STAND;

    // Demo behavior where players form a rotating circle and kick the ball
    // back and forth
    //return demoKickingCircle();
    return demoDynamicPlanning();
}


/*
 * Demo behavior where players form a rotating circle and kick the ball
 * back and forth
 */
SkillType NaoBehavior::demoKickingCircle() {
    //worldModel->getRVSender()->drawCircle("zero",0,0,2.0,RVSender::MAGENTA);
    worldModel->getRVSender()->drawPoint("ball", ball.getX(), ball.getY(), 10.0f, RVSender::MAGENTA);
    worldModel->getRVSender()->drawAgentText("CF",9, RVSender::RED);
    // Parameters for circle
    VecPosition center = VecPosition(-HALF_FIELD_X/2.0, 0, 0);
    double circleRadius = 5.0;
    double rotateRate = 2.5;

    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }

    if (playerClosestToBall == worldModel->getUNum()) {
        // Have closest player kick the ball toward the center
        return kickBall(KICK_FORWARD, center);
    } else {
        // Move to circle position around center and face the center
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());

        // Our desired target position on the circle
        // Compute target based on uniform number, rotate rate, and time
        VecPosition target = center + VecPosition(circleRadius,0,0).rotateAboutZ(360.0/(NUM_AGENTS-1)*(worldModel->getUNum()-(worldModel->getUNum() > playerClosestToBall ? 1 : 0)) + worldModel->getTime()*rotateRate);

        // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    }
}

SkillType NaoBehavior::demoDynamicPlanning(){
    //cout<<"=========="<<worldModel->getGameTime()<<endl;

    //目标点
    VecPosition center = VecPosition(HALF_FIELD_X, 0, 0);
   // worldModel->getRVSender()->drawPoint("target",center.getX(),center.getY(),10.0f, RVSender::VIOLET);
    //worldModel->getRVSender()->drawLine("TtoB",center.getX(),center.getY(),ball.getX(),ball.getY(),RVSender::GREEN);
    //worldModel->getRVSender()->drawCircle("b",ball.getX(),ball.getY(),1,RVSender::BLUE);
    //get the closest opponent to ball 
    int opponentClosestToBall=-1;
    closestDistanceOpponentToBall(&opponentClosestToBall);
    worldModel->getRVSender()->drawCircle("1",-HALF_FIELD_X,0,2,RVSender::VIOLET);
//    worldModel->getRVSender()->drawCircle("my",worldModel->getWorldObject( opponentClosestToBall )->pos.getX(),
//					  worldModel->getWorldObject(opponentClosestToBall)->pos.getY(),0.2f, RVSender::MAGENTA);
    
//    worldModel->getRVSender()->drawLine("line",-7,15,-7,-15,RVSender::BLUE);
//    worldModel->getRVSender()->drawLine("line",-15,0,-7,0,RVSender::BLUE);
    // Find closest player to ball
    
    /////////////////////////////////////////////4-18 00:54
    //int playerClosestToBall = -1;
    //double closestDistanceToBall = closestDistanceTeammateToBall(&playerClosestToBall);
    
    //cout<<closestDistanceToBall<<endl;
    /*
    if(startFlag){
	clo1=closestDistanceToBall;
	cout<<"开始最近距离："<<clo1<<endl;
	startFlag=false;
    }
    */
//     if (playerClosestToBall == worldModel->getUNum()) {
//         double myX=worldModel->getMyPosition().getX();
//         double myY=worldModel->getMyPosition().getY();
//         worldModel->getRVSender()->drawCircle("my",myX,myY,0.2f, RVSender::MAGENTA);
//         worldModel->getRVSender()->
//         drawLine("MtoB",myX,myY,ball.getX(),ball.getY(),RVSender::GREEN);
// 	
// 	//worldModel->getMyPosition.getDistanceTo(ball);
// 	/*
// 	if(closestDistanceToBall<1){
// 	    cout<<"距离："<<clo1-1<<endl;
// 	    cout<<"消耗时间："<<worldModel->getGameTime()<<endl;
// 	    double myVelocity=(clo1-1)/worldModel->getGameTime();
// 	    cout<<myVelocity<<endl;
// 	    //startFlag=true;
// 	}
// 	*/
// // 	if(worldModel->getGameTime()<30){
// // 	    cout<<opponentVelocity()<<endl;
// // 	}
// 	
// 	cout<<opponentVelocity()<<endl;
    VecPosition localCenter = worldModel->g2l(ball);
    SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
    
    if(worldModel->getPlayMode()==PM_KICK_OFF_RIGHT)
        return SKILL_STAND;
    
    int mynum;
    closestDistanceTeammateToBall(mynum);
    if(worldModel->getUNum()==mynum ){
        if(worldModel->getPlayMode()==PM_KICK_OFF_LEFT){
            return kickBall(KICK_FORWARD,VecPosition(0,15,0));
        }
        return kickBall(KICK_IK,center);
    }else {
        if(worldModel->getPlayMode()==PM_KICK_OFF_LEFT)
            return SKILL_STAND;
        if(worldModel->getUNum() ==11 || worldModel->getUNum()==10 || worldModel->getUNum()==9 ||           worldModel->getUNum()==8 ||worldModel->getUNum()==7 ){
//         //printD(roleMap);
            map<vector<int>, map<int,VecPosition> >roleMap;
            vector<int> agents_4;
            agents_4.push_back(11);
            agents_4.push_back(10);
            agents_4.push_back(9);
            agents_4.push_back(8);
            vector<int> agents_5(agents_4);
            agents_5.push_back(7);
            roleMap=dynamicPlanningFunction(agents_5,printPoints_5()); 
            print_goToTargetAllPlayer(roleMap,5); //TODO:optimizition
            return goToTargetAllPlayer(roleMap,5);
        }else if(worldModel->getUNum()==6){
            VecPosition vp6(closestDistanceOpponentToMyGoal_InDefenceArea2());
            if(vp6==VecPosition(0,0,0)){
                return goToTarget(VecPosition(-3,-5,0));
            }
            else{
                worldModel->getRVSender()->drawLine("1",worldModel->getMyPosition().getX(),worldModel->getMyPosition().getY(),vp6.getX()-1,vp6.getY(),RVSender::ORANGE);
                worldModel->getRVSender()->drawLine("1",vp6.getX(),vp6.getY(),vp6.getX()-1,vp6.getY(),RVSender::DARKBLUE);
                return goToTarget(VecPosition(vp6.getX()-1,vp6.getY(),0));
            }
        }else if(worldModel->getUNum()==2){
            return goToTarget(DefenceLeft());
        }else if(worldModel->getUNum()==3){
            VecPosition targetDR=DefenceRight();
            if(me.getDistanceTo(targetDR)<0.5)
                return goToTargetRelative(worldModel->g2l(targetDR),localCenterAngle);
            else
                return goToTarget(DefenceRight());
        }else if(worldModel->getUNum()==4){
            VecPosition targetBML=BackMidfieldLeft();
            if(me.getDistanceTo(targetBML)<0.5)
                return goToTargetRelative(worldModel->g2l(targetBML),localCenterAngle);
            else
                return goToTarget(BackMidfieldLeft());
        }else if(worldModel->getUNum()==5){
            VecPosition targetBMR=BackMidfieldRight();
            if(me.getDistanceTo(targetBMR)<0.5)
                return goToTargetRelative(worldModel->g2l(targetBMR),localCenterAngle);
            else
                return goToTarget(targetBMR);
        }
    }
//     if(worldModel->getPlayMode()==PM_PLAY_ON){
//         
//     }
    //printPlayerNum returns 2-11 ,,,,,,,,,no error
    //vector<int> testvi=printPlayerNum();
    
    //printPoints return ten points,,,,,,,,no error
    //vector<VecPosition> vvp=printPoints();

    

    
//     three agents dPF3---------------------------
//     vector<VecPosition> vecPostions;
//     VecPosition vp1=VecPosition(ball.getX()+2,ball.getY(),0);
//     VecPosition vp2up=VecPosition(ball.getX()-2,ball.getY(),0);
//     VecPosition vp3down=VecPosition(ball.getX(),ball.getY()+2,0);
//     VecPosition vp4center=VecPosition(ball.getX(),ball.getY()-2,0);
//     VecPosition vp5=VecPosition(ball.getX()+4,ball.getY(),0);
//     VecPosition vp6=VecPosition(ball.getX()-4,ball.getY(),0);
//     VecPosition vp7=VecPosition(ball.getX(),ball.getY()-4,0);
//     VecPosition vp8=VecPosition(ball.getX(),ball.getY()+4,0);
//     
//     worldModel->getRVSender()->drawCircle("my",vp1.getX(),vp1.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp2up.getX(),vp2up.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp3down.getX(),vp3down.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp4center.getX(),vp4center.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp8.getX(),vp8.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp5.getX(),vp5.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp6.getX(),vp6.getY(),0.2f, RVSender::MAGENTA);
//     worldModel->getRVSender()->drawCircle("my",vp7.getX(),vp7.getY(),0.2f, RVSender::MAGENTA);
//     vecPostions.push_back(vp1);
//     vecPostions.push_back(vp2up);
//     vecPostions.push_back(vp3down);
//     vecPostions.push_back(vp4center);
//     vecPostions.push_back(vp5);
//     vecPostions.push_back(vp6);
//     vecPostions.push_back(vp7);
//     //vecPostions.push_back(vp8);
//     
//     vector<int> vecAgents;
//     vecAgents.push_back(9);
//     //vecAgents.push_back(2);
//     vecAgents.push_back(8);
//     vecAgents.push_back(7);
//     vecAgents.push_back(6);
//     vecAgents.push_back(5);
//     vecAgents.push_back(4);
//     vecAgents.push_back(3);
// 
//     map<vector<int>, map<int,VecPosition> >roleMap;
//     roleMap=dynamicPlanningFunction(vecAgents,vecPostions); 
//     if(worldModel->getUNum() !=1 || worldModel->getUNum()!=2 || worldModel->getUNum()!=10 || worldModel->getUNum()!=11  ){
//         //printD(roleMap);
//         print_goToTargetAllPlayer(roleMap,7); //TODO:optimizition
//         return goToTargetAllPlayer(roleMap,7);
//     }
    //three agents dPF3---------------------------
    
//     //like die Circle ---stiff
//     map<vector<int>, map<int,VecPosition> >roleMap;
//     vector<int> tenPlayers=printPlayerNum();
//     vector<VecPosition> tenPositions=printPoints();
//     roleMap=dPF3(tenPlayers,tenPositions);
//     //cout<<"after roleMap"<<endl;
//     if(worldModel->getUNum() != 1){
//         print_goToTargetAllPlayer(roleMap,10);
//         //return goToTargetAllPlayer(roleMap);
//         return SKILL_STAND;
//     }
//     cout<<"=================="<<endl;
    return SKILL_STAND;
}
void NaoBehavior::printD(map<vector<int>, map<int,VecPosition> > roleMap){
    map<int, VecPosition> rolePlanning;
    map<vector<int>, map<int,VecPosition> >::iterator i=roleMap.begin();
    while(i!=roleMap.end()){
        if((i->first).size()==3){ ///10
            rolePlanning=roleMap[i->first];
        }
        i++;
    }
    map<int, VecPosition>::iterator miv=rolePlanning.begin();
    while(miv != rolePlanning.end()){
        cout<<miv->first<<"----> ("<<miv->second.getX()<<", "<<miv->second.getY()<<"}\n";
        ++miv;
    }
}


double NaoBehavior::closestDistanceTeammateToBall(int &pNum){
    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    pNum=playerClosestToBall;
    return closestDistanceToBall;
}

double NaoBehavior::closestDistanceOpponentToBall(int *pNum){
    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        VecPosition temp;
        WorldObject* opponent = worldModel->getWorldObject( i );
        if (opponent->validPosition) {
            temp = opponent->pos;
        } else {
            continue;
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            closestDistanceToBall = distanceToBall;
	    playerClosestToBall=i;
        }
    }
    //cout<<"opponent closest====== "<<playerClosestToBall<<endl;
    *pNum=playerClosestToBall;
    return closestDistanceToBall;
}
vector<VecPosition> NaoBehavior::printPoints_4(){
    vector<VecPosition> positions;
    //1 Front Forward
    VecPosition FF=VecPosition(ball.getX()+5,ball.getY()+0.1,0);
    if(ball.getX()>10){
        if(ball.getY()>0){
            double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
            double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
            FF.setX(x);
            FF.setY(y);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),ball.getX(),ball.getY(),RVSender::GREEN);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),HALF_FIELD_X,ball.getY(),RVSender::GREEN);
            }else{
                double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
                double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
                y=2*ball.getY()-y;
                FF.setX(x);
                FF.setY(y);
            }
  }
  worldModel->getRVSender()->drawPoint("1",FF.getX(),FF.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(FF);
  
  //2 Wing Forward Left
  VecPosition WFL=VecPosition(ball.getX()-0.5,ball.getY()+3,0);
  if(ball.getY()>6.5){
    double y;
    y=9.6;
    WFL.setY(y);
  }
  worldModel->getRVSender()->drawPoint("1",WFL.getX(),WFL.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(WFL);
  
  //3 Wing Forward Right
  VecPosition WFR=VecPosition(ball.getX()-0.5,ball.getY()-3,0);
  if(ball.getY()<-6.5){
    WFR.setY(-9.6);
  }
  worldModel->getRVSender()->drawPoint("1",WFR.getX(),WFR.getY(),10.0f,RVSender::GREEN);
  positions.push_back(WFR);
    
  //4 Center Back
  VecPosition CBL=VecPosition(ball.getX()-4,ball.getY(),0);
  if(ball.getX()<-11){
    CBL.setX(-11);
    //worldModel->getRVSender()->drawLine("1",oppoPos1.getX(),oppoPos1.getY(),ball.getX(),ball.getY(),RVSender::YELLOW);
  }
  worldModel->getRVSender()->drawPoint("1",CBL.getX(),CBL.getY(),10.0f,RVSender::RED);
  positions.push_back(CBL);
  
  return positions;
}
vector<VecPosition> NaoBehavior::printPoints_5(){
    vector<VecPosition> positions;
    //1 Front Forward
    VecPosition FF=VecPosition(ball.getX()+5,ball.getY()+0.1,0);
    if(ball.getX()>10){
        if(ball.getY()>0){
            double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
            double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
            FF.setX(x);
            FF.setY(y);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),ball.getX(),ball.getY(),RVSender::GREEN);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),HALF_FIELD_X,ball.getY(),RVSender::GREEN);
            }else{
                double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
                double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
                y=2*ball.getY()-y;
                FF.setX(x);
                FF.setY(y);
            }
  }
  worldModel->getRVSender()->drawPoint("1",FF.getX(),FF.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(FF);
  
  //2 Wing Forward Left
  VecPosition WFL=VecPosition(ball.getX()-0.5,ball.getY()+3,0);
  if(ball.getY()>6.5){
    double y;
    y=9.6;
    WFL.setY(y);
  }
  worldModel->getRVSender()->drawPoint("1",WFL.getX(),WFL.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(WFL);
  
  //3 Wing Forward Right
  VecPosition WFR=VecPosition(ball.getX()-0.5,ball.getY()-3,0);
  if(ball.getY()<-6.5){
    WFR.setY(-9.6);
  }
  worldModel->getRVSender()->drawPoint("1",WFR.getX(),WFR.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(WFR);
    
  //4 Center Forward Back
  VecPosition CFB=VecPosition(ball.getX()-4,ball.getY(),0);
  if(ball.getX()<-11){
    CFB.setX(-11);
    //worldModel->getRVSender()->drawLine("1",oppoPos1.getX(),oppoPos1.getY(),ball.getX(),ball.getY(),RVSender::YELLOW);
  }
  worldModel->getRVSender()->drawPoint("1",CFB.getX(),CFB.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(CFB);
  
  //5 Feild Center Offence
  VecPosition FCO=VecPosition(ball.getX()-2,ball.getY(),0);
  worldModel->getRVSender()->drawPoint("1",FCO.getX(),FCO.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(FCO);

  return positions;
}
vector<VecPosition> NaoBehavior::printPoints(){
    vector<VecPosition> positions;
    //1 Front Forward
    VecPosition FF=VecPosition(ball.getX()+5,ball.getY()+0.1,0);
    if(ball.getX()>10){
        if(ball.getY()>0){
            double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
            double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
            FF.setX(x);
            FF.setY(y);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),ball.getX(),ball.getY(),RVSender::GREEN);
            //worldModel->getRVSender()->
            //drawLine("TtoB",FF.getX(),FF.getY(),HALF_FIELD_X,ball.getY(),RVSender::GREEN);
            }else{
                double x=(sqrt(3)*(ball.getX()+HALF_FIELD_X*3))/(4*sqrt(3));
                double y=sqrt(3)*x-sqrt(3)*HALF_FIELD_X+ball.getY();
                y=2*ball.getY()-y;
                FF.setX(x);
                FF.setY(y);
            }
  }
  worldModel->getRVSender()->drawPoint("1",FF.getX(),FF.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(FF);
  
  //2 Wing Forward Left
  VecPosition WFL=VecPosition(ball.getX()-3,ball.getY()+0.1,0);
  if(ball.getX()<-11.8){
    double x0=-HALF_FIELD_X, y0=GOAL_Y/4,x,y;
    x=(x0+ball.getX())/2;
    y=(y0+ball.getY())/2;
    WFL.setX(x);
    WFL.setY(y);
    worldModel->getRVSender()->drawLine("1",x0,y0,ball.getX(),ball.getY(),RVSender::BLUE);
  }
  worldModel->getRVSender()->drawPoint("1",WFL.getX(),WFL.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(WFL);
  
  //3 Wing Forward Right
  VecPosition WFR=VecPosition(ball.getX()-2,ball.getY()-1.1,0);
  if(ball.getY()<-8.9){
    WFR.setY((-HALF_FIELD_Y+ball.getY())/2);
  }if(ball.getX()<-12.8){
    double x0=-HALF_FIELD_X, y0=-GOAL_Y/4,x,y;
    x=(x0+ball.getX())/3;
    y=(y0+ball.getY())/3;
    WFR.setX(x);
    WFR.setY(y);
    //worldModel->getRVSender()->drawLine("1",x0,y0,ball.getX(),ball.getY(),RVSender::YELLOW);
  }
  worldModel->getRVSender()->drawPoint("1",WFR.getX(),WFR.getY(),10.0f,RVSender::GREEN);
  positions.push_back(WFR);
  
  //4 Side Forward Left
  VecPosition SFL=VecPosition(ball.getX()+2,ball.getY()+3.2,0);
  if(ball.getY()>4){
    SFL.setY((HALF_FIELD_Y+ball.getY())/2);
  }
  if(ball.getX()>11.9){
    SFL.setX((HALF_FIELD_X+ball.getX())/2);
  }
  worldModel->getRVSender()->drawPoint("1",SFL.getX(),SFL.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(SFL);
  
  //5 Side Forward Right
  VecPosition SFR=VecPosition(ball.getX(),ball.getY()-4,0);
  if(ball.getY()<-4){
    SFR.setY((-HALF_FIELD_Y+ball.getY())/2);
  }
  worldModel->getRVSender()->drawPoint("1",SFR.getX(),SFR.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(SFR);
  
  //6 Center Attack Midfield
  VecPosition CAM=VecPosition(1,5+0.1,0);
  worldModel->getRVSender()->drawPoint("1",CAM.getX(),CAM.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(CAM);
  
  //7 Center Defence Midfield
  VecPosition CDM=VecPosition(-2,-4,0);
  worldModel->getRVSender()->drawPoint("1",CDM.getX(),CDM.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(CDM);
  
  //8 Center Back Left
  VecPosition CBL=VecPosition(-HALF_FIELD_X+2,HALF_GOAL_Y,0);
  VecPosition oppoPos1=closestDistanceOpponentToMyGoal_InDefenceArea1();
  if(oppoPos1.getDistanceTo(ball)>6){
    double x=oppoPos1.getX()-1;
    double y=oppoPos1.getY()-1;
    CBL.setX(x);
    CBL.setY(y);
    //worldModel->getRVSender()->drawLine("1",oppoPos1.getX(),oppoPos1.getY(),ball.getX(),ball.getY(),RVSender::YELLOW);
  }
  worldModel->getRVSender()->drawPoint("1",CBL.getX(),CBL.getY(),10.0f,RVSender::RED);
  positions.push_back(CBL);
  
  //9 Center Back Right
  VecPosition CBR=VecPosition(-HALF_FIELD_X+2,-1,0);
  VecPosition oppoPos2=closestDistanceOpponentToMyGoal_InDefenceArea2();
  if(oppoPos2.getDistanceTo(ball)>6){
    double x=oppoPos2.getX()-1;
    double y=oppoPos2.getY()+1;
    CBR.setX(x);
    CBR.setY(y);
  }
  worldModel->getRVSender()->drawPoint("1",CBR.getX(),CBR.getY(),10.0f,RVSender::ORANGE);
  positions.push_back(CBR);
  
  VecPosition CF=VecPosition(ball.getX(),ball.getY(),0);
  worldModel->getRVSender()->drawCircle("1",CF.getX(),CF.getY(),0.2f,RVSender::ORANGE);
  positions.push_back(CF);
  //
  //cout<<"there are "<<positions.size()<<endl;
  return positions;
}
vector<int> NaoBehavior::printPlayerNum(){
    //player number
    vector<int> pn;
    for(int i=9;i>=0;--i){
        //number 2-11
        pn.push_back(i+2);
    }
    return pn;
}

VecPosition NaoBehavior::closestDistanceOpponentToMyGoal(){
    int playerClosestToMyGoal=-1;
    double closestDistanceToMyGoal=1000;
    VecPosition vp;
    VecPosition myGoal=VecPosition(-HALF_FIELD_X,0,0);
    for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        VecPosition temp;
        WorldObject* opponent = worldModel->getWorldObject( i );
        if (opponent->validPosition) {
            temp = opponent->pos;
        } else {
            continue;
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(myGoal);
        if (distanceToBall < closestDistanceToMyGoal) {
            closestDistanceToMyGoal = distanceToBall;
	    playerClosestToMyGoal=i;
	    vp=temp;
        }
    }
    return vp;
}

VecPosition NaoBehavior::closestDistanceOpponentToMyGoal_InDefenceArea1(){
  double closestDistanceToMyGoal=1000;
    VecPosition vp;
    VecPosition myGoal=VecPosition(-HALF_FIELD_X,0,0);
    for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        VecPosition temp;
        WorldObject* opponent = worldModel->getWorldObject( i );
        if (opponent->validPosition) {
            temp = opponent->pos;
        } else {
            continue;
        }
        temp.setZ(0);
        if(temp.getX()<-2 && temp.getY()>0){
            double distanceToMyGoal = temp.getDistanceTo(myGoal);
            if (distanceToMyGoal < closestDistanceToMyGoal) {
                closestDistanceToMyGoal = distanceToMyGoal;
                vp=temp;
            }
        }
    }
    if(vp!=VecPosition(0,0,0))
        worldModel->getRVSender()->drawCircle("1",vp.getX(),vp.getY(),0.2f,RVSender::YELLOW);
    return vp;
}

VecPosition NaoBehavior::closestDistanceOpponentToMyGoal_InDefenceArea2(){
  double closestDistanceToMyGoal=1000;
    VecPosition vp;
    VecPosition myGoal=VecPosition(-HALF_FIELD_X,0,0);
    for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        VecPosition temp;
        WorldObject* opponent = worldModel->getWorldObject( i );
        if (opponent->validPosition) {
            temp = opponent->pos;
        } else {
            continue;
        }
        temp.setZ(0);
        if(temp.getX()<-2 && temp.getY()<0){
            double distanceToMyGoal = temp.getDistanceTo(myGoal);
            if (distanceToMyGoal < closestDistanceToMyGoal) {
                closestDistanceToMyGoal = distanceToMyGoal;
                vp=temp;
            }
        }
    }
    if(vp!=VecPosition(0,0,0))
        worldModel->getRVSender()->drawCircle("1",vp.getX(),vp.getY(),0.2f,RVSender::YELLOW);
    return vp;
}

VecPosition NaoBehavior::DefenceLeft(){
    VecPosition myGoalUp=VecPosition(-HALF_FIELD_X,0.5,0);
    int r=2;
    //Defence Left
    VecPosition DL;
    if(ball.getY()==0.5){
        DL.setX(-13);
        DL.setY(0.5);
    }else{
        double xDL,yDL;
        xDL=r*(ball.getX()-myGoalUp.getX())/myGoalUp.getDistanceTo(ball)+myGoalUp.getX();
        yDL=r*(ball.getY()-myGoalUp.getY())/myGoalUp.getDistanceTo(ball)+myGoalUp.getY();
        DL.setX(xDL);
        DL.setY(yDL);
    }
    worldModel->getRVSender()->drawPoint("1",DL.getX(),DL.getY(),10.0f,RVSender::YELLOW);
    return DL;
}
VecPosition NaoBehavior::DefenceRight(){
    VecPosition myGoalDown=VecPosition(-HALF_FIELD_X,-0.5,0);
    int r=2;
    //Defence Right
    VecPosition DR;
    double xDR,yDR;
    xDR=r*(ball.getX()-myGoalDown.getX())/myGoalDown.getDistanceTo(ball)+myGoalDown.getX();
    yDR=r*(ball.getY()-myGoalDown.getY())/myGoalDown.getDistanceTo(ball)+myGoalDown.getY();
    DR.setX(xDR);
    DR.setY(yDR);
    worldModel->getRVSender()->drawPoint("1",DR.getX(),DR.getY(),10.0f,RVSender::YELLOW);
    return DR;
}
VecPosition NaoBehavior::BackMidfieldLeft(){
    VecPosition myGoalCenter=VecPosition(-HALF_FIELD_X,0,0);
    int r=5;
    //Defence Left
    VecPosition BML;
    if(ball.getY()==0.5){
        BML.setX(-13);
        BML.setY(0.5);
    }else{
        double xDL,yDL;
        xDL=r*(ball.getX()-myGoalCenter.getX())/myGoalCenter.getDistanceTo(ball)+myGoalCenter.getX();
        yDL=r*(ball.getY()-myGoalCenter.getY())/myGoalCenter.getDistanceTo(ball)+myGoalCenter.getY();
        BML.setX(xDL);
        BML.setY(yDL);
    }
    worldModel->getRVSender()->drawPoint("1",BML.getX(),BML.getY(),10.0f,RVSender::YELLOW);
    return BML;
}
VecPosition NaoBehavior::BackMidfieldRight(){
    VecPosition myGoalDown=VecPosition(-HALF_FIELD_X,0,0);
    int r=7;
    //Defence Right
    VecPosition DR;
    double xDR,yDR;
    xDR=r*(ball.getX()-myGoalDown.getX())/myGoalDown.getDistanceTo(ball)+myGoalDown.getX();
    yDR=r*(ball.getY()-myGoalDown.getY())/myGoalDown.getDistanceTo(ball)+myGoalDown.getY();
    DR.setX(xDR);
    DR.setY(yDR);
    worldModel->getRVSender()->drawPoint("1",DR.getX(),DR.getY(),10.0f,RVSender::YELLOW);
    return DR;
}


double NaoBehavior::opponentVelocity(){
    if(worldModel->getGameTime()/12.0==1||worldModel->getGameTime()/14.0==1){
      cout<<"我是cx"<<endl;
      for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
	  VecPosition temp;
	  WorldObject* opponent = worldModel->getWorldObject( i );
	  if (opponent->validPosition) {
	      temp = opponent->pos;
	  } else {
	      continue;
	  }
	  temp.setZ(0);
	  preVecPosition.push_back(temp);
      }
      if(preVecPosition.size()>=22){
	  for(unsigned int i=preVecPosition.size();i>preVecPosition.size()-11;--i){
	      tempVelocity.push_back(preVecPosition[i-1].getDistanceTo(preVecPosition[i-12])/2);
	      //tempVelocity.push_back(1);
	      //cout<<"===================="<<endl;
	  }
      }
      if(tempVelocity.size()>=11){
	  sort(tempVelocity.begin(),tempVelocity.end());
	  return tempVelocity[8];
      }
    }
    if(tempVelocity.size()>=11){
	cout<<"========="<<tempVelocity.size();
	return tempVelocity[8];
    }
  return 0.5;
}

//Cij choose j in i, num==j, 
void NaoBehavior::Cij(int i, int j,vector<int> &r,int num,vector<vector<int> > & result){  
    if (j == 1){
        for (int k = 0; k < i; k++){  
            vector<int> temp(num);  
            r[num - 1] = k;  
            for (int i = 0; i < num;i++){  
                temp[i]=r[i];  
                //cout << r[i] << ' ';  
            }  
            result.push_back(temp);  
            //cout << endl;  
        }  
    }  
    else if (j == 0)  {  
        //do nothing!  
    }  
    else{  
        for (int k = i; k >= j; k--){  
            r[j-2] = k-1;  
            Cij(k - 1, j - 1,r,num,result);  
            }
    }
}
vector<vector<int> >  NaoBehavior::Cij_result(int i, int j, vector<int> &r, int num, vector<vector<int> > &result){
	vector<int> tempR(r);
//	cout<<"111111==============="<<endl;
	Cij(i,j,r,num,result);
//	cout<<"22222222============"<<endl;
//	cout<<"tempR size : "<<tempR.size()<<endl;
	//cout<<"rr size: "<<rr.size()<<endl;
	
	vector<vector<int> > rr(result);
	for(unsigned int ii=0;ii<result.size();++ii){
		vector<int> t;
		for(unsigned int jj=0;jj<result[ii].size();++jj){
			rr[ii][jj]=tempR[result[ii][jj]];
//			cout<<rr[ii][jj]<<' ';
			
		}
//		cout<<endl;
	}
	return rr;
}
void NaoBehavior::combination(int m, int k,int sum, vector<vector<int> > &result){
    if(k==0)
		return;	
	for(int i=m; i>=k; --i){
		a[k]=i-1;
		if(k>1){
			combination(i-1,k-1,sum,result);
		}else{
			vector<int> temp;
			for(int j=sum; j>0; j--){
				temp.push_back(a[j]);
			}
			result.push_back(temp);
		}
	}
}
vector<vector<int> >  NaoBehavior::comb_result(int i, int j,vector<int> &r,  int num, vector<vector<int> > &result){
		combination(i,j,num,result);
	    vector<vector<int> > rr(result);
		for(unsigned int i=0;i<result.size();++i){
			 //vector<int> t;
			 for(unsigned int j=0;j<result[i].size();++j){
				 rr[i][j]=r[result[i][j]];
														                
		        }
    }   
    return rr; 
}
double NaoBehavior::sumDistance(map<int,VecPosition> miv){
    double result=0;
    //
//     for(pair<int,VecPosition> i:miv){
// 	int playerNum=i.first;
// 	WorldObject *playerObj=worldModel->getWorldObject(playerNum);
// 	//need process??? 2018-4-10
// 	VecPosition playerPos=playerObj->pos;
// 	double d=playerPos.getDistanceTo(i.second);
// 	result+=d;
//     }
    map<int,VecPosition>::iterator mit=miv.begin();
    while(mit!=miv.end()){
        int playerNum=mit->first;
        VecPosition temp;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( playerNum );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                //cout<<"invalid Position << "<<endl;
                //continue;
                temp = teammate->pos;
            }
        }
        temp.setZ(0);
        //WorldObject *playerObj=worldModel->getWorldObject(playerNum);
        //VecPosition playerPos=playerObj->pos;
        double d=temp.getDistanceTo(mit->second);
        result+=d;
        
        ++mit;
    }
    //cout<<"distance result: "<<result<<endl;
    return result;
    
}

SkillType NaoBehavior::goToTargetAllPlayer(map<vector<int>, map<int,VecPosition> > roleMap, unsigned int num){
    map<int, VecPosition> rolePlanning;
    for(map<vector<int>, map<int,VecPosition> >::iterator i=roleMap.begin();i!=roleMap.end();i++){
        if((i->first).size()==num){ ////////10
            rolePlanning=roleMap[i->first];
        }
    }
    for(map<int,VecPosition>::iterator it=rolePlanning.begin();it!=rolePlanning.end();++it){
        if(worldModel->getUNum() ==it->first){
            return goToTarget(it->second);
        }
    }
    return SKILL_STAND;
}
//===================================

void NaoBehavior::print_goToTargetAllPlayer(map<vector<int>, map<int,VecPosition> > roleMap,unsigned int num){
    map<int, VecPosition> rolePlanning;
    map<vector<int>, map<int,VecPosition> >::iterator i=roleMap.begin();
    while(i!=roleMap.end()){
        if((i->first).size()==num){ ///10
            rolePlanning=roleMap[i->first];
        }
        i++;
    }
    for(map<int,VecPosition>::iterator it=rolePlanning.begin();it!=rolePlanning.end();++it){
        VecPosition temp;
        if(worldModel->getUNum() ==it->first){
            //return goToTarget(it->second);
            // This is us
            temp = worldModel->getMyPosition();
        } else {
                WorldObject* teammate = worldModel->getWorldObject( it->first );
                if (teammate->validPosition) {
                    temp = teammate->pos;
                } else {
                    continue;
                }
        }
        temp.setZ(0);
        worldModel->getRVSender()->drawLine("pr",temp.getX(),temp.getY(),
                                            (it->second).getX(),(it->second).getY(),RVSender::BLUEGREEN);
        
    }
    //cout<<"print line over !!!!"<<endl;
}
//===================================


map<vector<int>, map<int,VecPosition> > NaoBehavior::dynamicPlanningFunction(vector<int> agents, vector<VecPosition> positions){
    map<vector<int>,map<int,VecPosition> > roleMap;
    int n=agents.size();
    for(int k=1;k<=n;++k){
        vector<int>::iterator ita=agents.begin();
        vector<vector<int> > S;
        //for each a in agents
        while(ita !=agents.end()){
            vector<vector<int> > resultS;
            vector<int> temp(agents);
            //delete a from temp, S exclude a;
            vector<int>::iterator it_temp=temp.begin();
            while(it_temp != temp.end()){
                if(*it_temp == *ita){
                    temp.erase(it_temp);
                    break;
                }
                ++it_temp;
            }
            S=comb_result(n-1,k-1,temp,k-1,resultS); //when k==2 , fixed !
            //S = sets of agents, or null? 
            if(S.size()!=0){
                vector<vector<int> >::iterator sit1=S.begin();
                //for each s in S do...
                while(sit1 !=S.end()){  // in while
                    vector<int> s=*sit1;
                    //sort(s.begin(),s.end());
                    map<int,VecPosition> mTemp = roleMap[s];
                    map<int,VecPosition> m(mTemp.begin(),mTemp.end()) ;
                    m.insert(make_pair(*ita,positions[k-1]));
                    //v = aUs
                    vector<int> v(s.begin(),s.end());
                    v.push_back(*ita);
                    sort(v.begin(),v.end());
                    if(roleMap.count(v) != 0){ //exist v in roleMap
                        if(sumDistance(roleMap[v])>sumDistance(m)){ //sumDistance error: fixed
                            roleMap[v]=m;
                        }
                        //else
                            //roleMap[v]=roleMap[v];
                    }
                    else{
                        roleMap[v]=m;
                    }
                    ++sit1;
                }
            }
            else{
                map<int, VecPosition> first;
                first[*ita]=positions[k-1];
                vector<int> v;
                v.push_back(*ita);
                //sort(v.begin(),v.end());
                roleMap.insert(make_pair(v,first));
            }
            ++ita;
        }
	    
    }
    return roleMap;
}

map<vector<int>, map<int,VecPosition> > NaoBehavior::dPF3(vector<int> agents, vector<VecPosition> positions){
    map<vector<int>,map<int,VecPosition> > roleMap;
    int n=agents.size();
    for(int k=1;k<=n;++k){
        vector<int>::iterator ita=agents.begin();
        //where S is better!!!!!!!!!!!!!!!!!!!!!!!
        vector<vector<int> > S;
        //for each a in agents
        while(ita !=agents.end()){
            vector<vector<int> > resultS;
            vector<int> temp(agents);
            //temp= agents - a
            vector<int>::iterator it_temp=temp.begin();
            while(it_temp != temp.end()){
                if(*it_temp == *ita){
                    temp.erase(it_temp);
                    break;
                }
                ++it_temp;
            }
            S=Cij_result(n-1,k-1,temp,k-1,resultS); //when k==2 ,there cause error , fixed !
            //
//             cout<<"a = "<<*ita<<endl;
//             vector<vector<int> >::iterator testS=S.begin();
//             while(testS != S.end()){
//                 vector<int>::iterator testSi=testS->begin();
//                 while(testSi != testS->end()){
//                     cout<<"S = "<<*testSi<<' ';
//                     ++testSi;
//                 }
//                 ++testS;
//                 cout<<"--------"<<endl;
//             }
//             cout<<"================="<<endl;
            //S = sets of agents, or null? 
            //if S=null,the line(for(vector<int> &s : S)) will not be executed
            if(S.size()!=0){
                vector<vector<int> >::iterator sit1=S.begin();
                //for each s in S do...
                while(sit1 !=S.end()){  // in while
                    vector<int> s=*sit1;
                    sort(s.begin(),s.end());
                    //set<int> s(sit1->begin(),sit1->end());
                    map<int,VecPosition> mTemp = roleMap[s]; //////////*sit1
                    map<int,VecPosition> m(mTemp.begin(),mTemp.end()) ;
                    m.insert(make_pair(*ita,positions[k-1]));
//                     map<int,VecPosition>::iterator testm=m.begin();
//                     while(testm != m.end()){
//                         cout<<"m : "<<testm->first<<" --> "<<testm->second;
//                         ++testm;
//                     }
//                     cout<<endl;
                    //v = aUs
                    vector<int> v(s.begin(),s.end());//////////*sit1
                    v.push_back(*ita);
                    sort(v.begin(),v.end());
                    if(roleMap.find(v) != roleMap.end()){ //exist v in roleMap
                        if(sumDistance(roleMap[v])>sumDistance(m)){ //sumDistance error: fixed
                            roleMap[v]=m;
                        }
                        
                        //else
                            //roleMap[v]=roleMap[v];
                    }
                    else{
                        
                        roleMap[v]=m;
//                         cout<<"+++"<<endl;
//                         printD(roleMap);
//                         cout<<"+++"<<endl;
                    }
                    ++sit1;
                    
                }
            }
            else{
                map<int, VecPosition> first;
                first[*ita]=positions[k-1];
                vector<int> v;
                v.push_back(*ita);
                //sort(v.begin(),v.end());
                roleMap.insert(make_pair(v,first));
            }
            ++ita;
        }
	    
    }
//     cout<<"+++"<<endl;
//     printD(roleMap);
//     cout<<"+++"<<endl;
    return roleMap;
}

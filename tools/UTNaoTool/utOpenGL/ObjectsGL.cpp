#include "ObjectsGL.h"
#include <common/Field.h>

#include <iostream>
using namespace std;

void ObjectsGL::drawGreenCarpet() {
  glBegin(GL_POLYGON); 
  basicGL.colorRGB(0,100,0);
  glVertex3f (-HALF_GRASS_X/FACT,-HALF_GRASS_Y/FACT,0);
  glVertex3f (-HALF_GRASS_X/FACT,HALF_GRASS_Y/FACT,0);
  glVertex3f (HALF_GRASS_X/FACT,HALF_GRASS_Y/FACT,0);
  glVertex3f (HALF_GRASS_X/FACT,-HALF_GRASS_Y/FACT,0); 
  glEnd();  
}

void ObjectsGL::drawFieldLine(Point2D start, Point2D end) {
  basicGL.useFieldLineWidth();
  basicGL.colorRGB(basicGL.whiteRGB);
  basicGL.translate(0,0,2);
  basicGL.drawLine(start, end);
}

void ObjectsGL::drawIntersection(Point2D p, float alpha) {
  glPushMatrix();
  basicGL.colorRGBAlpha(basicGL.pinkRGB,alpha);  
  basicGL.translate(p,0.0);
  basicGL.drawSphere(BALL_RADIUS);
  glPopMatrix();;
}

void ObjectsGL::drawPenaltyCross(Point2D p, float alpha){
  glPushMatrix();
  basicGL.colorRGBAlpha(basicGL.whiteRGB,alpha);
  Point2D px1 = p;
  px1.x -= 50;
  Point2D px2 = p;
  px2.x += 50;
  Point2D py1 = p;
  py1.y -= 50;
  Point2D py2 = p;
  py2.y += 50;
  basicGL.drawLine(px1, px2);
  basicGL.drawLine(py1, py2);
  glPopMatrix();
}

void ObjectsGL::drawCenterCircle(Point2D p, float alpha){
  glPushMatrix();
  basicGL.useFieldLineWidth();
  basicGL.colorRGB(basicGL.whiteRGB);
  basicGL.colorRGBAlpha(basicGL.whiteRGB,alpha);  
  basicGL.translate(p,0.0);
  basicGL.drawCircle(CIRCLE_RADIUS);
  glPopMatrix();
}

void ObjectsGL::drawLinePoint(Point2D p, float alpha) {
  glPushMatrix();
  basicGL.colorRGBAlpha(basicGL.whiteRGB,alpha);  
  basicGL.translate(p,0.0);
  basicGL.drawSphere(BALL_RADIUS);
  glPopMatrix();;
}

void ObjectsGL::drawBall(Point2D p, float alpha) {
  drawBallColor(p,alpha,basicGL.orangeRGB);
}

void ObjectsGL::drawBallColor(Point2D p, float alpha, RGB color){
  glPushMatrix();
  basicGL.colorRGBAlpha(color,alpha);
  basicGL.translate(p,BALL_RADIUS);
  basicGL.drawSphere(BALL_RADIUS);
  glPopMatrix();
}
  

void ObjectsGL::drawBallVel(Point2D p, Vector2D vel, float alpha) {
  drawBallVelColor(p, vel, alpha, basicGL.redRGB);
}

void ObjectsGL::drawBallVelColor(Point2D p, Vector2D vel, float alpha, RGB color) {
  glPushMatrix();
  basicGL.colorRGBAlpha(color,alpha);
  basicGL.setLineWidth(50);
  basicGL.drawLine(p, p+vel);
  glPopMatrix();
}

void ObjectsGL::drawYellowGoal(Point2D p1, float alpha) {
  basicGL.colorRGBAlpha(basicGL.yellowRGB, alpha);
  drawGoal(p1);
}

void ObjectsGL::drawYellowPost(Point2D p1, float alpha) {
  basicGL.colorRGBAlpha(basicGL.yellowRGB, alpha);
  drawGoalPost(p1);
}

void ObjectsGL::drawGoalPost(Point2D p) {
 glPushMatrix();  
 basicGL.translate(p);
 basicGL.drawCylinder(50.0f,GOAL_HEIGHT);
 glPopMatrix();  
}

void ObjectsGL::drawCrossBar(Point2D goalCenter) {
 glPushMatrix();  

 basicGL.translate(goalCenter,0.95*GOAL_HEIGHT);
 basicGL.rotateXDeg(90.0);
 basicGL.drawCylinder(30.0f,GOAL_Y/2.0);
 basicGL.rotateXDeg(-180.0);
 basicGL.drawCylinder(39.0f,GOAL_Y/2.0);
  
 glPopMatrix();  
}

void ObjectsGL::drawGoal(Point2D goalCenter) { 
  Point2D post;
  post.x=goalCenter.x;
  post.y=goalCenter.y+GOAL_Y/2.0;
  drawGoalPost(post); //left

  post.y=goalCenter.y-GOAL_Y/2.0;
  drawGoalPost(post); //right

  drawCrossBar(goalCenter);
}

void ObjectsGL::drawBeacon(Point2D center, RGB tColor, RGB bColor, float alpha) {
  glPushMatrix();
  basicGL.translate(center);
  basicGL.colorRGBAlpha(bColor, alpha);
  basicGL.drawCylinder(50.0f, 100.0f);
  basicGL.translate(Vector3<float>(0,0,100.0f));
  basicGL.colorRGBAlpha(tColor, alpha);
  basicGL.drawCylinder(50.0f, 100.0f);
  glPopMatrix();
}

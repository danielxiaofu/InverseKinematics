////////////////////////////////////////////////////
// // Template code for  CS 174C
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <shared/defs.h>

#include "shared/opengl.h"

#include <string.h>
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"
#include "SampleParticle.h"
#include "SampleGravitySimulator.h"
#include "SkeletonSystem.h"
#include "SkeletonSimulator.h"
#include "Joint.h"
#include "Hermite.h"
#include <util/jama/tnt_stopwatch.h>
#include <util/jama/jama_lu.h>

// register a sample variable with the shell.
// Available types are:
// - TCL_LINK_INT 
// - TCL_LINK_FLOAT

int g_testVariable = 10;

void initializeJoints(SkeletonSystem* skeleton);

SETVAR myScriptVariables[] = {
	"testVariable", TCL_LINK_INT, (char *) &g_testVariable,
	"",0,(char *) NULL
};


//---------------------------------------------------------------------------------
//			Hooks that are called at appropriate places within anim.cpp
//---------------------------------------------------------------------------------

// start or end interaction
void myMouse(int button, int state, int x, int y)
{

	// let the global resource manager know about the new state of the mouse 
	// button
	GlobalResourceManager::use()->setMouseButtonInfo( button, state );

	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		animTcl::OutputMessage(
			"My mouse received a mouse button press event\n");

	}
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		animTcl::OutputMessage(
			"My mouse received a mouse button release event\n") ;
	}
}	// myMouse

// interaction (mouse motion)
void myMotion(int x, int y)
{

	GLMouseButtonInfo updatedMouseButtonInfo = 
		GlobalResourceManager::use()->getMouseButtonInfo();

	if( updatedMouseButtonInfo.button == GLUT_LEFT_BUTTON )
	{
		animTcl::OutputMessage(
			"My mouse motion callback received a mousemotion event\n") ;
	}

}	// myMotion


void MakeScene(void)
{

	/* 
	
	This is where you instantiate all objects, systems, and simulators and 
	register them with the global resource manager

	*/

	GlobalResourceManager::use()->clearAll();

	bool success;

	// register a skeleton system
	SkeletonSystem* bob = new SkeletonSystem("bob");

	success = GlobalResourceManager::use()->addSystem(bob, true);

	// make sure it was registered successfully
	assert(success);

	// register a simulator
	SkeletonSimulator* iksim =
		new SkeletonSimulator("iksim", bob);

	success = GlobalResourceManager::use()->addSimulator(iksim);

	// make sure it was registered successfully
	assert(success);

	initializeJoints(bob);

	bob->initialize(0, 0, 0, 0, 0, 0, 0);

	// regiseter a hermite
	Hermite * hermiteSystem = new Hermite("hermite");
	success = GlobalResourceManager::use()->addSystem(hermiteSystem, true);
	assert(success);
	iksim->setHermite(hermiteSystem);

	glutPostRedisplay();

}	// MakeScene

// OpenGL initialization
void myOpenGLInit(void)
{
	animTcl::OutputMessage("Initialization routine was called.");

}	// myOpenGLInit

void myIdleCB(void)
{
	
	return;

}	// myIdleCB

void myKey(unsigned char key, int x, int y)
{
	 animTcl::OutputMessage("My key callback received a key press event\n");
	return;

}	// myKey

static int testGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	 animTcl::OutputMessage("This is a test command!");
	return TCL_OK;

}	// testGlobalCommand

static int partOneGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	GlobalResourceManager::use()->clearAll();

	bool success;

	// register a skeleton system
	SkeletonSystem* bob = new SkeletonSystem("bob");

	success = GlobalResourceManager::use()->addSystem(bob, true);

	// make sure it was registered successfully
	assert(success);

	// register a simulator
	SkeletonSimulator* iksim =
		new SkeletonSimulator("iksim", bob);

	success = GlobalResourceManager::use()->addSimulator(iksim);

	// make sure it was registered successfully
	assert(success);

	initializeJoints(bob);

	bob->initialize(0, 0, 0, 0, 0, 0, 0);

	// regiseter a hermite
	Hermite * hermiteSystem = new Hermite("hermite");
	success = GlobalResourceManager::use()->addSystem(hermiteSystem , true);
	assert(success);
	iksim->setHermite(hermiteSystem);

	glutPostRedisplay();

	return TCL_OK;

}	// testGlobalCommand

void mySetScriptCommands(Tcl_Interp *interp)
{

	// here you can register additional generic (they do not belong to any object) 
	// commands with the shell

	Tcl_CreateCommand(interp, "test", testGlobalCommand, (ClientData) NULL,
					  (Tcl_CmdDeleteProc *)	NULL);

}	// mySetScriptCommands

void initializeJoints(SkeletonSystem* skeleton)
{
	Joint* root = new Joint("root");
	root->initialize(0.0, 0.0, 2.0);
	GlobalResourceManager::use()->addObject(root, true);
	skeleton->addJoint(root);

	Joint* spine = new Joint("spine");
	spine->initialize(0.0, 3.0, 0.0);
	GlobalResourceManager::use()->addObject(spine, true);
	skeleton->addJoint(spine);
	skeleton->setSpine(spine);

	Joint* leftCollar = new Joint("leftCollar");
	leftCollar->initialize(-0.5, 0.2, 0.0);
	GlobalResourceManager::use()->addObject(leftCollar, true);
	skeleton->addJoint(leftCollar);
	skeleton->setLeftCollar(leftCollar);

	Joint* leftShoulder = new Joint("leftShoulder");
	leftShoulder->initialize(-3, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(leftShoulder, true);
	skeleton->addJoint(leftShoulder);
	skeleton->setLeftShoulder(leftShoulder);

	Joint* leftElbow = new Joint("leftElbow");
	leftElbow->initialize(-3, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(leftElbow, true);
	skeleton->addJoint(leftElbow);
	skeleton->setLeftElbow(leftElbow);

	Joint* leftWrist = new Joint("leftWrist");
	leftWrist->initialize(-2, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(leftWrist, true);
	skeleton->addEndEffector(leftWrist);

	skeleton->traverseUp(); // go back to leftWrist
	skeleton->traverseUp(); // go back to leftElbow
	skeleton->traverseUp(); // go back to leftShoulder
	skeleton->traverseUp(); // go back to spine

	Joint* rightCollar = new Joint("rightCollar");
	rightCollar->initialize(0.5, 0.2, 0.0);
	GlobalResourceManager::use()->addObject(rightCollar, true);
	skeleton->addJoint(rightCollar);
	skeleton->setRightCollar(rightCollar);

	Joint* rightShoulder = new Joint("rightShoulder");
	rightShoulder->initialize(3, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(rightShoulder, true);
	skeleton->addJoint(rightShoulder);

	Joint* rightElbow = new Joint("rightElbow");
	rightElbow->initialize(3, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(rightElbow, true);
	skeleton->addJoint(rightElbow);

	Joint* rightWrist = new Joint("rightWrist");
	rightWrist->initialize(2.0, 0.0, 0.0);
	GlobalResourceManager::use()->addObject(rightWrist, true);
	skeleton->addJoint(rightWrist);

	// go back to spine
	skeleton->traverseUp();
	skeleton->traverseUp(); 
	skeleton->traverseUp();
	skeleton->traverseUp();

	Joint* head = new Joint("head");
	head->initialize(0.0, 1.0, 0.0);
	GlobalResourceManager::use()->addObject(head, true);
	skeleton->addJoint(head);

	// go back to root
	skeleton->traverseUp();
	skeleton->traverseUp();

	Joint* leftHip = new Joint("leftHip");
	leftHip->initialize(-0.5, -0.2, 0.0);
	GlobalResourceManager::use()->addObject(leftHip, true);
	skeleton->addJoint(leftHip);

	Joint* leftThigh = new Joint("leftThigh");
	leftThigh->initialize(0.0, -1.0, 0.0);
	GlobalResourceManager::use()->addObject(leftThigh, true);
	skeleton->addJoint(leftThigh);

	Joint* leftLeg = new Joint("leftLeg");
	leftLeg->initialize(0.0, -1.0, 0.0);
	GlobalResourceManager::use()->addObject(leftLeg, true);
	skeleton->addJoint(leftLeg);

	Joint* leftFoot = new Joint("leftFoot");
	leftFoot->initialize(0.0, -0.5, 0.0);
	GlobalResourceManager::use()->addObject(leftFoot, true);
	skeleton->addJoint(leftFoot);

	// go back to root
	skeleton->traverseUp();
	skeleton->traverseUp();
	skeleton->traverseUp();
	skeleton->traverseUp();

	Joint* rightHip = new Joint("rightHip");
	rightHip->initialize(0.5, -0.2, 0.0);
	GlobalResourceManager::use()->addObject(rightHip, true);
	skeleton->addJoint(rightHip);

	Joint* rightThigh = new Joint("rightThigh");
	rightThigh->initialize(0.0, -1.0, 0.0);
	GlobalResourceManager::use()->addObject(rightThigh, true);
	skeleton->addJoint(rightThigh);

	Joint* rightLeg = new Joint("rightLeg");
	rightLeg->initialize(0.0, -1.0, 0.0);
	GlobalResourceManager::use()->addObject(rightLeg, true);
	skeleton->addJoint(rightLeg);

	Joint* rightFoot = new Joint("rightFoot");
	rightFoot->initialize(0.0, -0.5, 0.0);
	GlobalResourceManager::use()->addObject(rightFoot, true);
	skeleton->addJoint(rightFoot);
}
//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,z
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 
    \author   <http://www.chai3d.org>
    \author   Francois Conti
    \autor    Michael Berger
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#if defined(C_ENABLE_ALUMINUM_DEVICE_SUPPORT)
#include "devices/CAluminumDevice.h"
#endif
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;

#include <iostream>
#include <fstream> 
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// show points other than the tip of the arm
bool showJoints = true;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing parts of the haptic device 
cShapeSphere* cursor_1;
cShapeSphere* cursor_2; 
cShapeSphere* cursor_3;
cShapeSphere* cursor_4;

// a transparent sphere representing the trajectory start point
cShapeSphere* startPoint;

// a haptic device handler
cHapticDeviceHandler* handler;

// create a line segment object to represent target trajectory
cMultiSegment* guidePath = new cMultiSegment();

// create a line sgement object to connect joints of arms
cMultiSegment* jointRelations = new cMultiSegment();

// a pointer to the current haptic device
cAluminumDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a label to display the linear velocity [m] of the haptic device
cLabel* labelHapticDeviceLinearVelocity;

// a global variable to store the linear velocity [m] of the haptic device
string hapticeDeviceLinearVelocity;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a label to display position axes
cLabel* labelPosAxes;

// a global variable to store the forces [N] on the haptic device
cVector3d hapticDeviceForces;

// a label to display forces at joint
cLabel* labelForces;

//a label to display force axes
cLabel* labelForcesAxes;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a flag for trajectory creation (ON/OFF)
bool useRecording = false;

// a flag for trajectory pathway (ON/OFF)
bool useTrajectory = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// a vector representing origin
cVector3d original (0,0,0);

// global coordinate arrays used in trajectory writing
std::vector<double> x_vec = {0};
std::vector<double> y_vec = {0};
std::vector<double> z_vec = {0};

// variable for trajectory input file name
std::string inputfileName;

// variable for trajectory output file name
std::string outputFileName;

// variable for proportional force feedback
double Kp;

// variable for how many points taken from trajectory file
int length;
int lines;
int counter;

// save_log variable
std::vector<cVector3d> positions_unique;
cVector3d a_position;

// parameter for max linear velocity allowed
double maxLinearVelocity = 5;

// parameter for where haptic feedback kicks in
double distanceTolerance = 0.01;

// global position vectors of the tip of arm and it's join
cVector3d position, position_2, position_3, position_4;

// global vectors used to erase joint connection segments upon update
cVector3d vertex1, vertex2, vertex3, vertex4;
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// function used to read trajectory data
void trajectoryRead(void);

// function used to record trajectory data
void trajectoryWrite(void);

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    Laryngoscopy Tracking Device
    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.
    An OpenGL window is opened and displays 3D cursors representing the device connected to your computer. 
    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status are read at each haptic cycle. 
    Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Laryngoscopy Tracker" << endl;
    cout << "ver01" << endl;
    cout << "Team 24" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[t] - Enable/Disable guide path" << endl;
    cout << "[r] - Enable/Disable recording a trajectory" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    //std::cout << " " << std::to_string(length) << endl;
    // // query user for input inputfileName
	// cout << "Enter the name of the file to read trajectory from (without extensions): ";
	// cin >> inputfileName;

    // query user for output fileName
	 //cout << "Enter the name of the file to record trajectory to (without extensions): ";
	 //cin >> outputFileName;
    outputFileName = "please";

    // // query user for proportional feedback constant
    // cout << "Enter proportional force constant (from 0 to 50) ";
	// cin >> Kp;



    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

   // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // // position and orient the camera front view
    // camera->set( cVector3d (0.0, 0.0, 0.375),    // camera position (eye)
    //              cVector3d (1.0, 0.0, 0.375),    // look at position (target)
    //              cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // position and orient the camera side view
    camera->set( cVector3d (0.5, -0.3, 0.25),    // camera position (eye)
                 cVector3d (0.5, 0.0, 0.375),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // position and orient the camera side view
    camera->set( cVector3d (1, -0.5, 0.375),    // camera position (eye)
                 cVector3d (0.5, 0.0, 0.375),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    cursor_1 = new cShapeSphere(0.015);
    cursor_2 = new cShapeSphere(0.012);
    cursor_3 = new cShapeSphere(0.012);
    cursor_4 = new cShapeSphere(0.02);

    // create a sphere to represent the start point
    startPoint = new cShapeSphere(0.02);

    // insert cursor inside world
    world->addChild(cursor_1);
    world->addChild(cursor_2);
    world->addChild(cursor_3);
    world->addChild(cursor_4);

    // insert start point to world
    world->addChild(startPoint);

    // add guide path trajectory into world
    world->addChild(guidePath);

    // add joint relations object into world
    world->addChild(jointRelations);

    // assign color properties to both line objects
    cColorf color;
    color.setYellowGold();
    guidePath->setLineColor(color);
    jointRelations->setLineColor(color);

    // assign line width
    guidePath->setLineWidth(4.0);
    jointRelations->setLineWidth(4.0);

    // use display list for faster rendering
    guidePath->setUseDisplayList(true);
    //jointRelations->setUseDisplayList(true);

  	// set cursor colors
    cursor_1->m_material->setBlue();
    cursor_2->m_material->setYellow();
    cursor_3->m_material->setRed();
    cursor_4->m_material->setOrange(); 

    // set start point color and transparency
    startPoint->m_material->setGreen();
    startPoint->setTransparencyLevel(0.4);

   
    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    //handler->getDevice(hapticDevice, 0);
    cGenericHapticDevicePtr ptrRetrieval;
	handler->getDevice(ptrRetrieval, 0);
    hapticDevice = dynamic_pointer_cast<cAluminumDevice>(ptrRetrieval);
    
    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
        cursor_1->setShowFrame(true);

        // set the size of the reference frame
        cursor_1->setFrameSize(0.05);

        // display reference frame
        cursor_4->setShowFrame(true);

        // set the size of the reference frame
        cursor_4->setFrameSize(0.05);
    }
   
    //--------------------------------------------------------------------------
    // CREATE PLANE
    //--------------------------------------------------------------------------
	
    // create mesh
    cMesh* plane = new cMesh();

    // add mesh to world
    world->addChild(plane);

    // create plane primitive
    cCreateMap(plane, 5, 5, 10, 10);

    // compile object
    plane->setUseDisplayList(true);

    // set color properties
    plane->m_material->setGreen();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI32();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    labelHapticDeviceModel->m_fontColor.setBlack();
    labelHapticDeviceModel->setText(info.m_modelName);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    
    // create a label for pos axes
    labelPosAxes = new cLabel(font);
    labelPosAxes->m_fontColor.setBlack();
    labelPosAxes->setText("X Pos  Y Pos   Z Pos");
    camera->m_frontLayer->addChild(labelPosAxes);

    // create a label for pos axes
    labelForcesAxes = new cLabel(font);
    labelForcesAxes->m_fontColor.setBlack();
    labelForcesAxes->setText("X Force Y Force Z Force");
    camera->m_frontLayer->addChild(labelForcesAxes);

    // create a label to display the forces on haptic device
    labelForces = new cLabel(font);
    labelForces->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelForces);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    labelHapticDevicePosition->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticDevicePosition);

    // create a label to display the linearVelocity of haptic device
    labelHapticDeviceLinearVelocity = new cLabel(font);
    labelHapticDeviceLinearVelocity->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticDeviceLinearVelocity);
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


	//--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // reads trajectory data
    if (useTrajectory){
        trajectoryRead();
    }

    // create start point cSphere
    startPoint->setLocalPos(x_vec[0], y_vec[0], z_vec[0]);
    // Create guidePath line segment object
    double index0;
    double index1;
    for (int i=0;i<(counter-2);i++){
        // create vertex 0
            index0 = guidePath->newVertex(x_vec[i], y_vec[i], z_vec[i]);
            
            // create vertex 1
            index1 = guidePath->newVertex(x_vec[i+1], y_vec[i+1], z_vec[i+1]);

            // create segment by connecting both vertices together
            guidePath->newSegment(index0, index1);
     } 

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // save log
    trajectoryWrite();

    // exit
    return 0;
}

//------------------------------------------------------------------------------
void trajectoryRead(void)
{
 // open and load trajectory file
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {

        homedir = getpwuid(getuid())->pw_dir;

    }
    
    std::string inputfileName = "logRead";
    ifstream trajectoryFile;
    trajectoryFile.open(string(homedir) + "/chai3d/" + inputfileName + ".m");
    if (trajectoryFile.fail()) {
        cerr << "Error Opening Trajectory File, Check File Name" <<endl;
        exit(1);
    }
    double xPoint;
    double yPoint;
    double zPoint;
    counter = 0;    
    lines = 1500;
    string line;

    for(int c=4;c<7;++c){
        for(int i=0;i<lines;i++){   
            if (c==4 && i==0) {
                trajectoryFile >> xPoint;
                x_vec[0] = xPoint;
            }
            if (c==4 && i!=0) {
                trajectoryFile >> xPoint;
                x_vec.push_back(xPoint);
            }
            if (c==5 && i==0) {
                trajectoryFile >> yPoint;
                y_vec[0] = yPoint;
            }
            if (c==5 && i!=0) {
                trajectoryFile >> yPoint;
                y_vec.push_back(yPoint);
            }
            if (c==6 && i==0) {
                trajectoryFile >> zPoint;
                z_vec[0] = zPoint;
            }
            if (c==6 && i!=0) {
                trajectoryFile >> zPoint;
                z_vec.push_back(zPoint);
            }
            counter = counter + 1;

            //i = i + 1;
        }
    }
    counter = counter / 3;
    trajectoryFile.close();
    std::cout << "Number of points in trajectory input file " << counter << std::endl;

}

void trajectoryWrite(void)
{
    using namespace std;
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    std::ofstream myfile;
    myfile.open (std::string(homedir) + "/chai3d/" + outputFileName + ".m");
    length = positions_unique.size(); 
    // defining variable for trajectory use
    std::cout << "Number of points in trajectory output file " << length << std::endl;
    string channel[3] = {"Test1","Test2","Test3"}; 
    for(int c=4;c<7;++c){
        myfile << channel[c];
        for(int i=0;i<length;++i){
            if(c==4) myfile << positions_unique[i].x();
            if(c==5) myfile << positions_unique[i].y();
            if(c==6) myfile << positions_unique[i].z();
            myfile << " ";
        }
        myfile << "\n";
    }
    myfile.close();
}

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of pos axes label
    labelPosAxes->setLocalPos(20, height - 70, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 100, 0);

    // update position of label
    labelHapticDeviceLinearVelocity->setLocalPos(20, height - 130, 0);
  
    //update position of force axes label
    labelForcesAxes->setLocalPos(width - 260, height - 70, 0);

    // update position of force label
    labelForces->setLocalPos(width - 260, height - 100, 0);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField){
            cout << "> Enable force field     \n";
        }
        else{
            cout << "> Disable force field    \n";
        }
    }

    // option - enable/disable trajectory recording
    else if (a_key == GLFW_KEY_R)
    {
        useRecording = !useRecording;
        if (useRecording){
            cout << "> Enable trajectory recording     \n";
            startPoint->setTransparencyLevel(0);
        }
        else{
            cout << "> Disable trajectory recording    \n";
            startPoint->setTransparencyLevel(0.4);

        }
    }

    // option - enable/disable trajectory on screen
    else if (a_key == GLFW_KEY_T)
    {
        useTrajectory = !useTrajectory;
        if (useTrajectory)
            cout << "> Enable trajectory      \n";
        else
            cout << "> Disable trajectory     \n";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \n";
        else
            cout << "> Disable damping        \n";
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update velocity data
    labelHapticDeviceLinearVelocity->setText("Linear Velocity: " + hapticeDeviceLinearVelocity);

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // update force data
    labelForces->setText(hapticDeviceForces.str(3));

    if (useTrajectory){
         guidePath->setTransparencyLevel(1.0,useTrajectory,useTrajectory,useTrajectory);
    }
    else {
         guidePath->setTransparencyLevel(0,!useTrajectory,!useTrajectory,!useTrajectory);
    }
    /////////////////////////////////////////////////////////////////////
    // AluminumHAPTICS DEBUG INFO
    /////////////////////////////////////////////////////////////////////

#if defined(C_ENABLE_ALUMINUM_DEVICE_SUPPORT)
    if(cAluminumDevice* w = dynamic_cast<cAluminumDevice*>(hapticDevice.get())){
        cAluminumDevice::aluminumhaptics_status s = w->getStatus();
        //std::cout << s.toJSON() << std::endl;
    }
#endif





    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;
    int loopCount = 0;
    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // grabs position and rotation data and creates vectors to assign it to  
        hapticDevice->getPosition(position, position_2, position_3, position_4);
        cMatrix3d rotation, rotation_2, rotation_3, rotation_4;
        hapticDevice->getRotation(rotation, rotation_2, rotation_3, rotation_4);

        // update position and orientation of cursor_1 (arm tip)
        cursor_1->setLocalPos(position);
        cursor_1->setLocalRot(rotation);

        if (showJoints){
        // update position and orientation of cursor_2 (last joint)
            cursor_2->setLocalPos(position_2);
            // cursor_2->setLocalRot(rotation_2);

            // update position and orientation of middle joint
            cursor_3->setLocalPos(position_3);
            // cursor_3->setLocalRot(rotation_3);

            // update position and orientation of base 
            cursor_4->setLocalPos(position_4);
            // cursor_4->setLocalRot(rotation_4);

            // call to function which creates and updates jointRelations object at slower tick rate than updateHaptics loop
            if (loopCount % 16 == 0){
                int n = jointRelations->getNumSegments();;
                
                //Create jointRelations line segment object
                if (n > 0){
                    jointRelations->clear();
                }
                jointRelations->newSegment(position, position_2);
                jointRelations->newSegment(position_2, position_3);
                jointRelations->newSegment(position_3, position_4);
                
            }
        }

        //read linear velocity 
        cVector3d linearVelocity; 
        hapticDevice->getLinearVelocity(linearVelocity);       
        double currentVelocity = pow(linearVelocity.x(), 2) + pow(linearVelocity.y(), 2) +pow(linearVelocity.z(),2);
        hapticeDeviceLinearVelocity = std::to_string(currentVelocity);

        // safety shutdown
        if (loopCount > 100 && (currentVelocity > maxLinearVelocity)) {
        cerr << "Linear Velocity Too High, System Shutdown" <<endl;
        GLFWwindow* a_window;
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
        exit(1);
        }

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);
     
        // /////////////////////////////////////////////////////////////////////
        // // UPDATE 3D CURSOR MODEL
        // /////////////////////////////////////////////////////////////////////
       
        // update global variable for graphic display update
        hapticDevicePosition = position;


        /////////////////////////////////////////////////////////////////////
        // CONTINUOUSLY UPDATING NEAREST POINT
        ////////////////////////////////////////////////////////////////////

        // initializing minimum distance from haptic device
        double min = sqrt(pow(x_vec[0]-position.x(),2) + pow(y_vec[0]-position.y(),2) + pow(z_vec[0]-position.z(),2));

        // index of mininum-distance point
        int minIndex = 0;

        // Loop through every point on trajectory and change min and minIndex if a smaller distance is found
        for (int i=1; i<length; i++) {
                double nextDistance = sqrt(pow(x_vec[i]-position.x(),2) + pow(y_vec[i]-position.y(),2) + pow(z_vec[i]-position.z(),2));
                if (nextDistance < min) {
                    min = nextDistance;
                    minIndex = i;
                }
        }
           
            //cVector3d currentPosition (position.x(),position.y(),position.z());
            double x_hold = position.x();
            double y_hold = position.y();
            double z_hold = position.z();
            a_position.set(x_hold,y_hold,z_hold);

            // only record
            if (useRecording){
                positions_unique.push_back(a_position);
            }
           
       
        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        if (min > distanceTolerance){
            desiredPosition.set(x_vec[minIndex], y_vec[minIndex], z_vec[minIndex]);
        }
        else {
            desiredPosition.set(position.x(),position.y(),position.z());
        }

        // desired orientation
        //cMatrix3d desiredRotation;
        //desiredRotation.identity();
        
        // variables for forces
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);

        // apply force field
        if (useForceField)
        {
            // compute linear force
            Kp = 25; // [N/m]
            cVector3d forceField = Kp * (desiredPosition - position);
            force.add(forceField);

            // compute angular torque
            //double Kr = 0.05; // [N/m.rad]
            //cVector3d axis;
            //double angle;
            //cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
            //deltaRotation.toAxisAngle(axis, angle);
            //torque = rotation * ((Kr * angle) * axis);
        }
    
        // apply damping term
        if (useDamping)
        {
            cHapticDeviceInfo info = hapticDevice->getSpecifications();

            // compute linear damping force
            double Kv = 1.0 * info.m_maxLinearDamping;
            cVector3d forceDamping = -Kv * linearVelocity;
            //force.add(forceDamping);

            // compute angular damping force
            double Kvr = 1.0 * info.m_maxAngularDamping;
            cVector3d torqueDamping = -Kvr * angularVelocity;
            //torque.add(torqueDamping);

        }
       
        // update global variable for graphic display update
         hapticDeviceForces = force;

        // send computed force, torque, and gripper force to haptic device
        //hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // signal frequency counter
        freqCounterHaptics.signal(1);
        loopCount = loopCount + 1;

        // sleep to set update rate at approximately 1000Hz
         usleep(925);
    }
    
    // exit haptics thread
    simulationFinished = true;
}


//------------------------------------------------------------------------------

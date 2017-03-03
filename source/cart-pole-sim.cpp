/*
*	cart-pole-sim.cpp
*	
*	Simulates a basic cart-pole system balanced with a number of controllers, using MuJoCo.
* 
*
*/

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"

/**************************** FORWARD DECLARATION ****************************/

// Utils
void autoscale(GLFWwindow* window);
void loadmodel(GLFWwindow* window, const char* filename);

// GLFW Callbacks
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

// Simulate + Render
void makeoptionstring(const char* name, char key, char* buf);
void render(GLFWwindow* window);
void simulation(void);

// Controls
void ctrlManual(const mjModel* m, mjData* d);
void ctrlLQR(const mjModel* m, mjData* d);
void ctrlNonlinear(const mjModel* m, mjData* d);
void setInitialConditions(mjData* d,mjtNum *ICpos, mjtNum *ICvel);

/**************************** GLOBAL VARS ****************************/

// model
mjModel* m = 0;
mjData* d = 0;
char lastfile[1000] = "";

// user state
bool paused = false;
bool showoption = false;
bool showinfo = true;
bool showfullscreen = false;
bool slowmotion = false;
int showhelp = 1;                   // 0: none; 1: brief; 2: full

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
char status[1000] = "";

// OpenGL rendering
int refreshRate;
const int fontscale = mjFONTSCALE_150;
mjrContext con;


// selection and perturbation
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
int needselect = 0;                 // 0: none, 1: select, 2: center, 3: center and track 
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// Select controller
mjfGeneric mjcb_control = ctrlManual;
// Select initial conditions
mjtNum ICpos[2] = {0,mjPI/32};
mjtNum ICvel[2] = {0,0};
// K linear gain vector corresponding to [x theta xDot thetaDot]
mjtNum K[4] = {5000,100000,100,100};

// Control output
mjtNum u;

// help strings
const char help_title[] = 
"Help\n"
"Option\n"
"Info\n"
"Full screen\n"
"Stereo\n"
"Slow motion\n"
"Pause\n"
"Reset\n"
"Forward\n"
"Back\n"
"Forward 100\n"
"Back 100\n"
"Autoscale\n"
"Reload\n"
"Geoms\n"
"Sites\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Camera\n"
"Frame\n"
"Label";


const char help_content[] = 
"F1\n"
"F2\n"
"F3\n"
"F5\n"
"F6\n"
"Enter\n"
"Space\n"
"BackSpace\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Ctrl A\n"
"Ctrl L\n"
"0 - 4\n"
"Shift 0 - 4\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"[ ]\n"
"; '\n"
". /";

char opt_title[1000] = "";
char opt_content[1000];

/**************************** MAIN ****************************/

// Where all the magic happens
int main(int argc, const char** argv) {

	// Check that the correct simulator version is used
	if( mjVERSION_HEADER!=mj_version() ) {
		mju_error("Headers and library have different versions");
	}

	// Activate the simulator license
	mj_activate("mjkey.txt");

	// Initialize GLFW
	if (!glfwInit()) return 1;

	// Set to native refresh rate
	refreshRate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;

	// Enable multisampling
	glfwWindowHint(GLFW_SAMPLES,4);

	// Set up mono visualization
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Cart-Pole Simulation", NULL, NULL);
    
    // Die if something fucked up with creating a window
    if( !window ) {
        glfwTerminate();
        return 1;
    }

    // Some more default GLFW stuff

    // make context current, request v-sync on swapbuffers
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // save window-to-framebuffer pixel scaling (needed for OSX scaling)
    int width, width1, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width1, &height);
    window2buffer = (double)width1 / (double)width;

    // init MuJoCo rendering, get OpenGL info
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, fontscale);

    // set GLFW callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetWindowRefreshCallback(window, render);

    // Load the cart-pole model description
    loadmodel(window,"../model/cart-pole.xml");

    // Set initial conditions
    setInitialConditions(d,ICpos,ICvel);

    // Main loop
    while( !glfwWindowShouldClose(window) )
    {
        // simulate and render
        render(window);

        // handle events (this calls all callbacks)
        glfwPollEvents();
    }

    // Clean up and terminate

    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);

    // terminate
    glfwTerminate();
    mj_deactivate();
    return 0;
}


/**************************** HELPER FUNCTIONS ****************************/

// center and scale view
void autoscale(GLFWwindow* window){
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}

// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename){

    // load and compile
    char error[1000] = "Could not load binary model!";
    mjModel* mnew = 0;
	
	mnew = mj_loadXML(filename, 0, error, 1000);

    if( !mnew )
    {
        printf("%s\n", error);
        return;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
    mj_forward(m, d);

    // TODO: Might want to get rid of lastfile if not needed?
    strcpy(lastfile, filename);

    // re-create custom context
    mjr_makeContext(m, &con, fontscale);

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    needselect = 0;

    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if( window && m->names )
        glfwSetWindowTitle(window, m->names);
}

/**************************** GLFW CALLBACKS ****************************/

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    int n;

    // require model
    if( !m )
        return;

    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    switch( key )
    {
    case GLFW_KEY_F1:                   // help
        showhelp++;
        if( showhelp>2 )
            showhelp = 0;
        break;

    case GLFW_KEY_F2:                   // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3:                   // info
        showinfo = !showinfo;
        break;

    case GLFW_KEY_F5:                   // toggle full screen
        showfullscreen = !showfullscreen;
        if( showfullscreen )
            glfwMaximizeWindow(window);
        else
            glfwRestoreWindow(window);
        break;

    case GLFW_KEY_F6:                   // stereo
        scn.stereo = (scn.stereo==mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
        break;

    case GLFW_KEY_ENTER:                // slow motion
        slowmotion = !slowmotion;
        break;

    case GLFW_KEY_SPACE:                // pause
        paused = !paused;
        break;

    case GLFW_KEY_BACKSPACE:            // reset
        mj_resetData(m, d);
        setInitialConditions(d,ICpos,ICvel);
        mj_forward(m, d);
        break;

    case GLFW_KEY_RIGHT:                // step forward
        if( paused )
            mj_step(m, d);
        break;

    case GLFW_KEY_LEFT:                 // step back
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            mj_step(m, d);
            m->opt.timestep = -m->opt.timestep;
        }
        break;

    case GLFW_KEY_DOWN:                 // step forward 100
        if( paused )
            for( n=0; n<100; n++ )
                mj_step(m,d);
        break;

    case GLFW_KEY_UP:                   // step back 100
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            for( n=0; n<100; n++ )
                mj_step(m,d);
            m->opt.timestep = -m->opt.timestep;
        }
        break;

    case GLFW_KEY_ESCAPE:               // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '[':                           // previous fixed camera or free
        if( m->ncam && cam.type==mjCAMERA_FIXED )
        {
            if( cam.fixedcamid>0 )
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']':                           // next fixed camera
        if( m->ncam )
        {
            if( cam.type!=mjCAMERA_FIXED )
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if( cam.fixedcamid<m->ncam-1 )
                cam.fixedcamid++;
        }
        break;

    case ';':                           // cycle over frame rendering modes
        vopt.frame = mjMAX(0, vopt.frame-1);
        break;

    case '\'':                          // cycle over frame rendering modes
        vopt.frame = mjMIN(mjNFRAME-1, vopt.frame+1);
        break;

    case '.':                           // cycle over label rendering modes
        vopt.label = mjMAX(0, vopt.label-1);
        break;

    case '/':                           // cycle over label rendering modes
        vopt.label = mjMIN(mjNLABEL-1, vopt.label+1);
        break;

    default:                            // toggle flag
        // control keys
        if( mods & GLFW_MOD_CONTROL )
        {
            if( key==GLFW_KEY_A )
                autoscale(window);
            else if( key==GLFW_KEY_L && lastfile[0] )
                loadmodel(window, lastfile);

            break;
        }

        // toggle visualization flag
        for( int i=0; i<mjNVISFLAG; i++ )
            if( key==mjVISSTRING[i][2][0] )
                vopt.flags[i] = !vopt.flags[i];

        // toggle rendering flag
        for( int i=0; i<mjNRNDFLAG; i++ )
            if( key==mjRNDSTRING[i][2][0] )
                scn.flags[i] = !scn.flags[i];

        // toggle geom/site group
        for( int i=0; i<mjNGROUP; i++ )
            if( key==i+'0')
            {
                if( mods & GLFW_MOD_SHIFT )
                    vopt.sitegroup[i] = !vopt.sitegroup[i];
                else
                    vopt.geomgroup[i] = !vopt.geomgroup[i];
            }
    }
}

// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods){
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if( !m )
        return;

    // set perturbation
    int newperturb = 0;
    if( act==GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select>0 )
    {
        // right: translate;  left: rotate
        if( button_right )
            newperturb = mjPERT_TRANSLATE;
        else if( button_left )
            newperturb = mjPERT_ROTATE;

        // perturbation onset: reset reference
        if( newperturb && !pert.active )
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if( act==GLFW_PRESS && glfwGetTime()-lastclicktm<0.25 && button==lastbutton )
    {
        if( button==GLFW_MOUSE_BUTTON_LEFT )
            needselect = 1;
        else if( mods & GLFW_MOD_CONTROL )
            needselect = 3;
        else
            needselect = 2;

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if( act==GLFW_PRESS )
    {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}

// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // require model
    if( !m )
        return;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move perturb or camera
    if( pert.active )
        mjv_movePerturb(m, d, action, dx/height, dy/height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset){
    // require model
    if( !m )
        return;

    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

/**************************** SIMULATION + RENDERING ****************************/

// make option string
void makeoptionstring(const char* name, char key, char* buf){
    int i=0, cnt=0;

    // copy non-& characters
    while( name[i] && i<50 )
    {
        if( name[i]!='&' )
            buf[cnt++] = name[i];

        i++;
    }

    // finish
    buf[cnt] = ' ';
    buf[cnt+1] = '(';
    buf[cnt+2] = key;
    buf[cnt+3] = ')';
    buf[cnt+4] = 0;
}

// render
void render(GLFWwindow* window) {
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);

    // advance simulation
    simulation();

    // update simulation statistics
    if( !paused )
    {
        // camera string
        char camstr[20];
        if( cam.type==mjCAMERA_FREE )
            strcpy(camstr, "Free");
        else if( cam.type==mjCAMERA_TRACKING )
            strcpy(camstr, "Tracking");
        else
            sprintf(camstr, "Fixed %d", cam.fixedcamid);

        // status
        sprintf(status, "%-20.1f\n%d (%d)\n%.0f\n%.2f\n%.2f (%02d it)\n%.1f %.1f\n%s\n%s\n%s",
                d->time, 
                d->nefc, 
                d->ncon, 
                1.0/(glfwGetTime()-lastrendertm),
                d->energy[0]+d->energy[1],
                mju_log10(mju_max(mjMINVAL, d->solver_trace[mjMAX(0,mjMIN(d->solver_iter-1,mjNTRACE-1))])),
                d->solver_iter, 
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[1])),
                camstr, 
                mjFRAMESTRING[vopt.frame], 
                mjLABELSTRING[vopt.label] );
    }

    // timing satistics
    lastrendertm = glfwGetTime();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // selection
    if( needselect )
    {
        // find selected geom
        mjtNum pos[3];
        int selgeom = mjr_select(rect, &scn, &con, 
            (int)(window2buffer*lastx), (int)(rect.height-window2buffer*lasty), pos, NULL);

        // find corresponding body if any
        int selbody = 0;
        if( selgeom>=0 && selgeom<scn.ngeom && scn.geoms[selgeom].objtype==mjOBJ_GEOM )
        {
            selbody = m->geom_bodyid[scn.geoms[selgeom].objid];
            if( selbody<0 || selbody>=m->nbody )
                selbody = 0;
        }

        // set lookat point, start tracking is requested
        if( needselect==2 || needselect==3 )
        {
            if( selgeom>=0 )
                mju_copy3(cam.lookat, pos);

            // switch to tracking camera
            if( needselect==3 && selbody )
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if( selbody )
            {
                // record selection
                pert.select = selbody;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, pos, d->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        needselect = 0;
    }

    // render
    mjr_render(rect, &scn, &con);

    // show overlays
    if( showhelp==1 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Help  ", "F1  ", &con);
    else if( showhelp==2 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &con);

    // show info
    if( showinfo )
    {
        if( paused )
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, "PAUSED", 0, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, 
                "Time\nSize\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nLabel", status, &con);
    }

    // show options
    if( showoption )
    {
        int i;
        char buf[100];

        // fill titles on first pass
        if( !opt_title[0] )
        {
            for( i=0; i<mjNRNDFLAG; i++)
            {
                makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                strcat(opt_title, "\n");
            }
            for( i=0; i<mjNVISFLAG; i++)
            {
                makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                if( i<mjNVISFLAG-1 )
                    strcat(opt_title, "\n");
            }
        }

        // fill content
        opt_content[0] = 0;
        for( i=0; i<mjNRNDFLAG; i++)
        {
            strcat(opt_content, scn.flags[i] ? " + " : "   ");
            strcat(opt_content, "\n");
        }
        for( i=0; i<mjNVISFLAG; i++)
        {
            strcat(opt_content, vopt.flags[i] ? " + " : "   ");
            if( i<mjNVISFLAG-1 )
                strcat(opt_content, "\n");
        }

        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, opt_title, opt_content, &con);
    }

    // swap buffers
    glfwSwapBuffers(window); 
}

// advance simulation
void simulation(void) {
    // no model
    if( !m )
        return;
        
    // paused
    else if( paused )
    {
        // apply pose perturbations, run mj_forward
        mjv_applyPerturbPose(m, d, &pert, 1);       // move mocap and dynamic bodies
        mj_forward(m, d);
    }

    // running
    else
    {
        // slow motion factor: 10x
        mjtNum factor = (slowmotion ? 10 : 1);

        // advance effective simulation time by 1/refreshRate
        mjtNum startsimtm = d->time;
        while( (d->time-startsimtm)*factor < 1.0/refreshRate )
        {
            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            if( pert.select>0 )
            {
                mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                mjv_applyPerturbForce(m, d, &pert);
            }

            // run mj_step and count
            mj_step(m, d);
        }
    }
}

/**************************** CONTROLLERS ****************************/
void setInitialConditions(mjData *d,mjtNum *ICpos,mjtNum *ICvel) {
    mju_copy(d->qpos,ICpos,2);
    mju_copy(d->qvel,ICvel,2);
}

void ctrlManual(const mjModel* m, mjData* d) {
    
    u = -(K[0]*d->qpos[0] + K[1]*d->qpos[1] + K[2]*d->qvel[0] + K[3]*d->qvel[1]);
    
    printf("Control Output: %f\n",u);
    printf("State: [%f,%f,%f,%f]\n\n",d->qpos[0],d->qpos[1],d->qvel[0],d->qvel[1]);

    mju_copy(d->ctrl,&u,1);
}

void ctrlLQR(const mjModel* m, mjData* d) {

}

void ctrlNonlinear(const mjModel* m, mjData* d) {

}

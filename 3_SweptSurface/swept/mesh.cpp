#include <iostream>
#include "math.h"
#include <GL/glut.h>
using namespace std;
#include <sstream>
#include <math.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <fstream>
#include <cstring>
using namespace std;
using namespace glm;


static unsigned int width = 1000;
static unsigned int height = 1000;

static bool mouseRotatePressed = false;
static bool mouseMovePressed   = false;
static bool mouseZoomPressed   = false;
static bool mouseDollyPressed   = false;
static bool seekPlz   = false;

static int	lastX = 0;
static float theta=0.0, lastTheta=0.0;
static int	lastY = 0;
static float beta=0.0, lastBeta=0.0;

static int	lastX_t = 0;
static float theta_t=0.0, lastTheta_t=0.0;
static int	lastY_t = 0;
static float beta_t=0.0, lastBeta_t=0.0;

static int	lastY_z = 0;
static float beta_z=0.0, lastBeta_z=0.0;

static int	lastY_d = 0;
static float beta_d=0.0, lastBeta_d=0.0;

static float x_0 = -30.0;
static float y_0 = 0.0;
static float z_0 = 100.0;

static float origin_x = 0.0;
static float origin_y = 0.0;
static float origin_z = 0.0;

static float camx = 0.0;
static float camy = 1.0;
static float camz = 0.0;

static float viewing_angle = 20.0;

static bool fullScreen = false;

static int rotationsign = 1;

static float rad;

static int num_in_section = 10; // n deung bun of interpolating sections in one gan gyeok
static int num_in_points = 10; // points between two control points

static double pi = 3.14159265;

struct quaternion
{
    float w, x, y, z;

    static quaternion ArbitraryAxis(float angle, float x, float y, float z)
    {
        float xaxis, yaxis, zaxis;
        xaxis = x / (sqrt(x*x+y*y+z*z));
        yaxis = y / (sqrt(x*x+y*y+z*z));
        zaxis = z / (sqrt(x*x+y*y+z*z));
        quaternion q;
        q.w = cos(angle/2.0);
        q.x = xaxis*sin(angle/2.0);
        q.y = yaxis*sin(angle/2.0);
        q.z = zaxis*sin(angle/2.0);
        return q;

    }

    static quaternion Euler2Q(float roll, float pitch)
    {
        quaternion q;
        float cosr = cos(roll/2.0);
        float sinr = sin(roll/2.0);
        float cosp = cos(pitch/2.0);
        float sinp = sin(pitch/2.0);
        q.w = cosr*cosp;
        q.x = cosp*sinr;
        q.y = sinr*sinp;
        q.z = cosr*sinp;
        return q;
    }

    static quaternion X2Q(float X)
    {
        // axis is y axis, local coord at camera, (camx, camy, camz) ->normalize
        float xaxis, yaxis, zaxis;
        xaxis = camx / (sqrt(camx*camx+camy*camy+camz*camz));
        yaxis = camy / (sqrt(camx*camx+camy*camy+camz*camz));
        zaxis = camz / (sqrt(camx*camx+camy*camy+camz*camz));
        quaternion q;
        q.w = cos(X/2);
        q.x = xaxis*sin(X/2);
        q.y = yaxis*sin(X/2);
        q.z = zaxis*sin(X/2);
        return q;
    }

    static quaternion Y2Q(float Y)
    {
        // axis is x axis, local coord at camera, (camx, camy, camz) X (x_0, y_0, z_0) => normalized
        float xaxis, yaxis, zaxis, xtmp, ytmp, ztmp;
        xtmp = camy*(z_0-origin_z) - camz*(y_0-origin_y);
        ytmp = camz*(x_0-origin_x) - camx*(z_0-origin_z);
        ztmp = camx*(y_0-origin_y) - camy*(x_0-origin_x);

        xaxis = xtmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        yaxis = ytmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        zaxis = ztmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));

        quaternion q;
        q.w = cos(Y/2);
        q.x = xaxis*sin(Y/2);
        q.y = yaxis*sin(Y/2);
        q.z = zaxis*sin(Y/2);
        return q;
    }


    static quaternion Mul(quaternion a, quaternion b)
    {
        quaternion q;
        q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
        q.x = a.w*b.x + b.w*a.x + a.y*b.z - a.z*b.y;
        q.y = a.w*b.y + b.w*a.y + a.z*b.x - a.x*b.z;
        q.z = a.w*b.z + b.w*a.z + a.x*b.y - a.y*b.x;
        return q;
    }

    static quaternion Inv(quaternion a)
    {
        quaternion q;
        q.w = a.w;
        q.x = -a.x;
        q.y = -a.y;
        q.z = -a.z;
        return q;
    }

    static quaternion Pos(float X, float Y, float Z)
    {
        quaternion q;
        q.x = X;
        q.y = Y;
        q.z = Z;
        q.w = 0.0;
        return q;
    }

    static quaternion Translation(quaternion pos, float X, float Y)
    {
        float xaxis, yaxis, zaxis, xtmp, ytmp, ztmp;

        // x axis (local)
        xtmp = camy*(z_0-origin_z) - camz*(y_0-origin_y);
        ytmp = camz*(x_0-origin_x) - camx*(z_0-origin_z);
        ztmp = camx*(y_0-origin_y) - camy*(x_0-origin_x);

        xaxis = xtmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        yaxis = ytmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        zaxis = ztmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));

        pos.x += xaxis*X;
        pos.y += yaxis*X;
        pos.z += zaxis*X;

        // y axis (local)
        xaxis = camx / (sqrt(camx*camx+camy*camy+camz*camz));
        yaxis = camy / (sqrt(camx*camx+camy*camy+camz*camz));
        zaxis = camz / (sqrt(camx*camx+camy*camy+camz*camz));

        pos.x += xaxis*Y;
        pos.y += yaxis*Y;
        pos.z += zaxis*Y;

        return pos;
    }

    static quaternion Dolly(quaternion pos, float Z)
    {
        float xtmp, ytmp, ztmp, xaxis, yaxis, zaxis;

        xtmp = (x_0-origin_x);
        ytmp = (y_0-origin_y);
        ztmp = (z_0-origin_z);

        xaxis = xtmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        yaxis = ytmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        zaxis = ztmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        
        pos.x += xaxis*Z;
        pos.y += yaxis*Z;
        pos.z += zaxis*Z;

        return pos;
    }

};

void SQT(double scale, dvec4 rotate, dvec3 posit)
{
    glTranslatef(posit[0], posit[1], posit[2]);
    glRotatef(rotate[0], rotate[1], rotate[2], rotate[3]);
    glScalef(scale, scale, scale);
    
    
}

void drawSomething()
{
    glClear (GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    ifstream ifs;
    ifs.open("trombone.txt"); rotationsign = -1;
    // ifs.open("coke_bottle.txt"); rotationsign = 1;
    // ifs.open("joystick.txt"); rotationsign = 1;

    if (!ifs.is_open())
	{
		std::cerr << "Error occured in reading txt file!" << std::endl;
        return;
	}
    string curve_type;
    ifs >> curve_type;    

    bool isBspline = false;
    if(curve_type == "BSPLINE")
    {
        isBspline = true;
    }

    int num_section;
    ifs >> num_section;

    int num_points;
    ifs >> num_points;


    dvec2 ctrl[50][50]; // coord of [i th section][j th ctrl point]
    double scale[50]; // scale of [i th section]
    dvec4 rotate[50];
    dvec3 posit[50]; 

    for(int i=0; i<num_section; i++)
    {
        for(int j=0; j<num_points; j++)
        {
            ifs >> ctrl[i][j][0];
            ifs >> ctrl[i][j][1];
        }
        ifs >> scale[i];
        ifs >> rotate[i][0];
        ifs >> rotate[i][1];
        ifs >> rotate[i][2];
        ifs >> rotate[i][3];

        rotate[i][0] = rotationsign * rotate[i][0]; // rotation sign check
        if(abs(rotate[i][0]) < 0.01)
        {
            rotate[i][0] = 0;
            rotate[i][1] = 0;
            rotate[i][2] = 0;
            rotate[i][3] = 1;
        }
        ifs >> posit[i][0];
        ifs >> posit[i][1];
        ifs >> posit[i][2];

    }

    glClear (GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // overall plane # = (num_section-1) * (num_in_section-1) + 1
    // [i] -> i / num_in_section   th section   i % num_in_section   th interpolated section

    dvec2 in_ctrl[500][50]; // i th interpolated section j th control point
    double in_scale[500];
    dvec4 in_rotate[500];
    dvec3 in_posit[500];

    for(int i=0; i<(num_section-1) * (num_in_section) + 1; i++)
    {
        // i/num_in_section th plane ~ i/num_in_section + 1 th plane interpolate
        // t = i%num_in_section / num_in_section
        int n = i / num_in_section;
        if(i%num_in_section == 0)
        {
            in_scale[i] = scale[n];
            in_rotate[i] = rotate[n];
            in_posit[i] = posit[n];
            for(int j=0; j<num_points; j++)
            {
                in_ctrl[i][j] = ctrl[n][j];
            }
        }
        else
        {
            float t = ((float)(i%num_in_section))/((float)num_in_section);
            in_scale[i] = (-0.5*t+t*t-0.5*t*t*t)*scale[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*scale[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*scale[n+1] + (-0.5*t*t+0.5*t*t*t)*scale[n+2];
            in_rotate[i] = (-0.5*t+t*t-0.5*t*t*t)*rotate[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*rotate[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*rotate[n+1] + (-0.5*t*t+0.5*t*t*t)*rotate[n+2];
            in_posit[i] = (-0.5*t+t*t-0.5*t*t*t)*posit[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*posit[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*posit[n+1] + (-0.5*t*t+0.5*t*t*t)*posit[n+2];
            for(int j=0; j<num_points; j++)
            {
                if(n==0)
                {
                    // in_ctrl[i][j] = (-0.5*t+t*t-0.5*t*t*t)*ctrl[n][j] + (1.0-2.5*t*t+1.5*t*t*t)*ctrl[n][j] + (0.5*t+2.0*t*t-1.5*t*t*t)*ctrl[n+1][j] + (-0.5*t*t+0.5*t*t*t)*ctrl[n+2][j];
                
                }
                else
                {
                    in_ctrl[i][j] = (-0.5*t+t*t-0.5*t*t*t)*ctrl[n-1][j] + (1.0-2.5*t*t+1.5*t*t*t)*ctrl[n][j] + (0.5*t+2.0*t*t-1.5*t*t*t)*ctrl[n+1][j] + (-0.5*t*t+0.5*t*t*t)*ctrl[n+2][j];
                
                }
            }
        }
        in_rotate[i] = in_rotate[i];
    }


    dvec2 in_points[500][500]; // i th interpolated section j th interpolated point

    for(int i=0; i<(num_section) * (num_in_section-1) + 1; i++)
    {
        for(int j=0; j<num_points; j++)
        {
            for(int k=0; k<num_in_points; k++)
            { // overall 0 ~ (num_points*num_in_points - 1) amounts of point in one section
                float t = (float)k / float(num_in_points);
                if(isBspline)
                {
                    in_points[i][k + j*num_in_points][0] = (1.0-t)*(1.0-t)*(1.0-t)*in_ctrl[i][j][0]/6.0 + (3.0*t*t*t-6.0*t*t+4.0)*in_ctrl[i][(j+1)%num_points][0]/6.0 + (-3.0*t*t*t+3.0*t*t+3.0*t+1.0)*in_ctrl[i][(j+2)%num_points][0]/6.0+t*t*t*in_ctrl[i][(j+3)%num_points][0]/6.0;
                    in_points[i][k + j*num_in_points][1] = (1.0-t)*(1.0-t)*(1.0-t)*in_ctrl[i][j][1]/6.0 + (3.0*t*t*t-6.0*t*t+4.0)*in_ctrl[i][(j+1)%num_points][1]/6.0 + (-3.0*t*t*t+3.0*t*t+3.0*t+1.0)*in_ctrl[i][(j+2)%num_points][1]/6.0+t*t*t*in_ctrl[i][(j+3)%num_points][1]/6.0;
                    
                }
                else
                {
                    in_points[i][k + j*num_in_points][0] = (-0.5*t+t*t-0.5*t*t*t)*in_ctrl[i][j][0] + (1.0-2.5*t*t+1.5*t*t*t)*in_ctrl[i][(j+1)%num_points][0] + (0.5*t+2.0*t*t-1.5*t*t*t)*in_ctrl[i][(j+2)%num_points][0] + (-0.5*t*t+0.5*t*t*t)*in_ctrl[i][(j+3)%num_points][0];
                    in_points[i][k + j*num_in_points][1] = (-0.5*t+t*t-0.5*t*t*t)*in_ctrl[i][j][1] + (1.0-2.5*t*t+1.5*t*t*t)*in_ctrl[i][(j+1)%num_points][1] + (0.5*t+2.0*t*t-1.5*t*t*t)*in_ctrl[i][(j+2)%num_points][1] + (-0.5*t*t+0.5*t*t*t)*in_ctrl[i][(j+3)%num_points][1];
                }
            }
        }
    
    }    

    for(int i=num_in_section; i<(num_section) * (num_in_section-1)-1; i++)
    {
        quaternion rot_i = quaternion::ArbitraryAxis(in_rotate[i][0], in_rotate[i][1], in_rotate[i][2], in_rotate[i][3]);
        quaternion inv_rot_i = quaternion::Inv(rot_i);

        quaternion rot_i1 = quaternion::ArbitraryAxis(in_rotate[i+1][0], in_rotate[i+1][1], in_rotate[i+1][2], in_rotate[i+1][3]);
        quaternion inv_rot_i1 = quaternion::Inv(rot_i1);

        // glBegin(GL_LINE_LOOP);
        
        for(int j=0; j<num_points*num_in_points; j++)
        {
            quaternion qij;
            qij.w = 0.0;
            qij.x = in_scale[i] * in_points[i][j][0];
            qij.y = 0.0;
            qij.z = in_scale[i] * in_points[i][j][1];

            quaternion ux_ij = quaternion::Mul(quaternion::Mul(rot_i, qij), inv_rot_i);
            GLfloat xij = in_posit[i][0] + ux_ij.x;
            GLfloat yij = in_posit[i][1] + ux_ij.y;
            GLfloat zij = in_posit[i][2] + ux_ij.z;

            quaternion qij1;
            int j_1 = (j+1)%(num_points*num_in_points);
            qij1.w = 0.0;
            qij1.x = in_scale[i] * in_points[i][j_1][0];
            qij1.y = 0.0;
            qij1.z = in_scale[i] * in_points[i][j_1][1];

            quaternion ux_ij1 = quaternion::Mul(quaternion::Mul(rot_i, qij1), inv_rot_i);
            GLfloat xij1 = in_posit[i][0] + ux_ij1.x;
            GLfloat yij1 = in_posit[i][1] + ux_ij1.y;
            GLfloat zij1 = in_posit[i][2] + ux_ij1.z;

            quaternion qi1j;
            qi1j.w = 0.0;
            qi1j.x = in_scale[i+1] * in_points[i+1][j][0];
            qi1j.y = 0.0;
            qi1j.z = in_scale[i+1] * in_points[i+1][j][1];

            quaternion ux_i1j = quaternion::Mul(quaternion::Mul(rot_i1, qi1j), inv_rot_i1);
            GLfloat xi1j = in_posit[i+1][0] + ux_i1j.x;
            GLfloat yi1j = in_posit[i+1][1] + ux_i1j.y;
            GLfloat zi1j = in_posit[i+1][2] + ux_i1j.z;

            quaternion qi1j1;
            qi1j1.w = 0.0;
            qi1j1.x = in_scale[i+1] * in_points[i+1][j_1][0];
            qi1j1.y = 0.0;
            qi1j1.z = in_scale[i+1] * in_points[i+1][j_1][1];

            quaternion ux_i1j1 = quaternion::Mul(quaternion::Mul(rot_i1, qi1j1), inv_rot_i1);
            GLfloat xi1j1 = in_posit[i+1][0] + ux_i1j1.x;
            GLfloat yi1j1 = in_posit[i+1][1] + ux_i1j1.y;
            GLfloat zi1j1 = in_posit[i+1][2] + ux_i1j1.z;


            glBegin(GL_TRIANGLES);
            glNormal3f(ux_ij.x, ux_ij.y, ux_ij.z);
            glVertex3f(xij, yij, zij);
            glVertex3f(xij1, yij1, zij1);
            glVertex3f(xi1j, yi1j, zi1j);
            glEnd();

            glBegin(GL_TRIANGLES);
            glNormal3f(ux_i1j.x, ux_i1j.y, ux_i1j.z);
            glVertex3f(xi1j1, yi1j1, zi1j1);
            glVertex3f(xi1j, yi1j, zi1j);
            glVertex3f(xij1, yij1, zij1);
            glEnd();

        }
        glFlush();

    }

}

void reshape(int w, int h)
{
    width = glutGet(GLUT_WINDOW_WIDTH);
    height = glutGet(GLUT_WINDOW_HEIGHT);

    glViewport(0, 0, w, h );

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	float aspectRatio = (float)w/(float)h;
	gluPerspective( viewing_angle /*field of view angle*/,
					aspectRatio,
					10.0 /*near clipping plane*/,
					1000.0 /* far clipping plane */ );

	gluLookAt( x_0,y_0,z_0, origin_x,origin_y,origin_z, camx,camy,camz );
	glMatrixMode( GL_MODELVIEW );
}


void display()
{
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawSomething();
    glFlush();
    glutSwapBuffers();
}

void keyboardCB(unsigned char keyPressed, int x, int y)
{
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow(width,height);
            fullScreen = false;
        } else {
            glutFullScreen();
            fullScreen = true;
        }
        break;
    case 'q':
        exit(0);
        break;
    case 'a':
        // press a to show all figure

        // put origin to 0, 0, 0
        origin_x = 0.0;
        origin_y = 0.0;
        origin_z = 0.0;

        // put camera on the same way
        x_0 = x_0 - origin_x;
        y_0 = y_0 - origin_y;
        z_0 = z_0 - origin_z;

        // maintain camx camy camz

        // normalize camera vector
        // put camera to the appropriate position
        
        rad = sqrt(x_0*x_0+y_0*y_0+z_0*z_0);

        x_0 = 3.0 * x_0 / rad;
        y_0 = 3.0 * y_0 / rad;
        z_0 = 3.0 * z_0 / rad;

        reshape(width, height);
        break;
    case 's':
        // press s to seek point
        seekPlz = true;
        break;
    }
    glutPostRedisplay();
}

void mouseCB(int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed   = false;
        mouseRotatePressed = false;
        mouseZoomPressed   = false;
        mouseDollyPressed  = false;
    } else {
        if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_SHIFT==glutGetModifiers())
        {
			// do something here
			//std::cout << "translate click" << std::endl;

            lastX_t = x;
			lastTheta_t = theta_t;
            lastY_t = y;
			lastBeta_t = beta_t;

            mouseMovePressed   = true;
            mouseRotatePressed = false;
            mouseZoomPressed   = false;
            mouseDollyPressed  = false;
        }
        else if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_CTRL==glutGetModifiers())
        {
			// do something here
			//std::cout << "zoom click" << std::endl;

            lastY_z = y;
			lastBeta_z = beta_z;

            mouseMovePressed   = false;
            mouseRotatePressed = false;
            mouseZoomPressed   = true;
            mouseDollyPressed  = false;
        }
        else if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_ALT==glutGetModifiers())
        {

            lastY_d = y;
			lastBeta_d = beta_d;

            mouseMovePressed   = false;
            mouseRotatePressed = false;
            mouseZoomPressed   = false;
            mouseDollyPressed  = true;
        }
        else if (button==GLUT_LEFT_BUTTON)
        {
			// do something here
			//std::cout << "rotate click" << std::endl;
			lastX = x;
			lastTheta = theta;
            lastY = y;
			lastBeta = beta;

            mouseMovePressed   = false;
            mouseRotatePressed = true;
            mouseZoomPressed   = false;
            mouseDollyPressed  = false;
        }
    }
    glutPostRedisplay();
}

void motionCB(int x, int y) {
    if(seekPlz == true)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        GLfloat z_f;
        glReadPixels (x, y, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &z_f);
        std::cout << z_f;
        seekPlz == false;
    }
    else if (mouseRotatePressed == true)
	{
        // do something here
		//std::cout << "rotate drag" << std::endl;

		theta = (float)(x-lastX)*0.0001;
        beta = (float)(y-lastY)*0.0001;

        quaternion rot_x = quaternion::X2Q(-theta);
        quaternion rot_y = quaternion::Y2Q(-beta);
        quaternion rot = quaternion::Mul(rot_y, rot_x);
        quaternion inv = quaternion::Inv(rot);

        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        quaternion tmp = quaternion::Mul(rot, pos);
        quaternion result = quaternion::Mul(tmp, inv);

        // x_0 y_0 z_0 direction -> 0 0 -1
        x_0 = result.x;
        y_0 = result.y;
        z_0 = result.z;

        quaternion pos2 = quaternion::Pos(camx, camy, camz);
        quaternion tmp2 = quaternion::Mul(rot, pos2);
        quaternion result2 = quaternion::Mul(tmp2, inv);

        // camx camy camz direction -> 0 1 0
        camx = result2.x;
        camy = result2.y;
        camz = result2.z;

		reshape(width, height);
	}
    else if (mouseMovePressed == true)
    {
        // to translate, press shift and drag
        //shift to x axis (local) amount of theta
        //shift to y axis (local) amount of beta 

        theta_t = -(float)(x-lastX_t)*0.01;
        beta_t = (float)(y-lastY_t)*0.01;

        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        pos = quaternion::Translation(pos, theta_t, beta_t);

        x_0 = pos.x;
        y_0 = pos.y;
        z_0 = pos.z;

        quaternion origin_pos = quaternion::Pos(origin_x, origin_y, origin_z);
        origin_pos = quaternion::Translation(origin_pos, theta_t, beta_t);

        origin_x = origin_pos.x;
        origin_y = origin_pos.y;
        origin_z = origin_pos.z;

        reshape(width, height);


    }
    else if (mouseZoomPressed == true)
    {
        // to zoom, press ctrl and drag up and down
        
        beta_z = (float)(y-lastY_z)*0.001;
        viewing_angle += beta_z;
        if(viewing_angle<1) viewing_angle = 1;
        if(viewing_angle>179) viewing_angle = 179;

        reshape(width, height);
    }
    else if (mouseDollyPressed == true)
    {
        // to dolly, press alt and drag up and down

        beta_d = (float)(y-lastY_d)*0.1;
        float dist1 = sqrt((x_0-origin_x)*(x_0-origin_x)+(y_0-origin_y)*(y_0-origin_y)+(z_0-origin_z)*(z_0-origin_z));
        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        pos = quaternion::Dolly(pos, beta_d);

        float dist2 = sqrt((x_0-origin_x)*(x_0-origin_x)+(y_0-origin_y)*(y_0-origin_y)+(z_0-origin_z)*(z_0-origin_z));
        if(dist1>=10 && dist2<10)
        {
            
        }
        else
        {
            x_0 = pos.x;
            y_0 = pos.y;
            z_0 = pos.z;
        }

        

        reshape(width, height);
    }
    glutPostRedisplay();
}

void idle() { glutPostRedisplay(); }

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutCreateWindow("3D viewer");
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    reshape(width, height);
    glClearColor(0.0, 0.1, 0.3, 1.0);

    glClearDepth(1.0f);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat AmbientLightValue[] = { 1.0f, 1.0f, 1.0f, 0.0f};
    GLfloat SpecularLightValue[] = { 0.3f, 0.3f, 0.3f, 0.0f };
    GLfloat PositionLightValue[] = { 10.0f, 10.0f, 20.0f, 0.0f };

    glLightfv( GL_LIGHT0, GL_AMBIENT, AmbientLightValue );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, SpecularLightValue ); 
    glLightfv( GL_LIGHT0, GL_POSITION, PositionLightValue );
    
    

    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboardCB);
    glutReshapeFunc(reshape);
    glutMotionFunc(motionCB);
    glutMouseFunc(mouseCB);

    glutMainLoop();
    return 0;
}

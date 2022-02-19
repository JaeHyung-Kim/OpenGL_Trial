
#include <iostream>
#include "math.h"
#include <GL/glut.h>
using namespace std;


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

static float x_0 = 0.0;
static float y_0 = 0.0;
static float z_0 = 3.0;

static float origin_x = 0.0;
static float origin_y = 0.0;
static float origin_z = 0.0;

static float camx = 0.0;
static float camy = 1.0;
static float camz = 0.0;

static float viewing_angle = 45.0;

static bool fullScreen = false;

static float rad;

struct quaternion
{
    float w, x, y, z;

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

void drawSomething()
{
    int a=60, b=60, c=135, d=20;
    float w=0.25;
    glClear (GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, -0.5, 0.0);

    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);

        glPushMatrix(); // upper right arm
            glTranslatef(0.0, 0.0, w);
            glRotatef (180, 0.0, 0.0, -1.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(1.8, 0.5, 0.5);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glTranslatef(-0.35, 0.0, 0.0);
            glPushMatrix(); // lower right arm
                glRotatef (c, 0.0, 0.0, 1.0);
                glRotatef (d, 0.0, -1.0, 0.0);
                glTranslatef(-0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(1.9, 0.4, 0.4);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
                    glTranslatef(-0.35, 0.0, 0.0);
                    // hand
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(0.4, 0.4, 0.4);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

    glPushMatrix(); // upper right leg
        glRotatef (a, 0.0, 0.0, 1.0);
        glTranslatef(0.4, 0.0, w-0.1);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.75, 0.75);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
        glPushMatrix(); // lower leg
            glTranslatef(0.4, 0.0, 0.0);
            glRotatef (-a*2.0, 0.0, 0.0, 1.0);
            glTranslatef(0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.65, 0.65);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glTranslatef(0.4, 0.0, 0.0);
            glPushMatrix(); // foot
                glRotatef (a, 0.0, 0.0, 1.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(0.7, 0.35, 0.35);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

    // body
    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);
        glPushMatrix();
            glColor3f (0.7, 0.7, 0.7);
            glScalef(3.0, 1.2, 1.5);
            glutSolidSphere(0.2, 5, 5);
        glPopMatrix();
    glPopMatrix();

        
    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.4, 0.0, 0.0);
        glPushMatrix(); // head
            glRotatef (b*0.5, 0.0, 0.0, -1.0);
            glTranslatef(-0.75, 0.0, 0.0);
                glPushMatrix();
                    glColor3f (0.7, 0.7, 0.7);
                    glScalef(1.0, 1.0, 1.0);
                    glutSolidSphere(0.25, 5, 5);
                glPopMatrix();
        glPopMatrix();
    glPopMatrix();

        
    
    

    glPushMatrix(); // upper left leg
        glRotatef (a, 0.0, 0.0, 1.0);
        glTranslatef(0.4, 0.0, -w+0.1);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.75, 0.75);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
        glPushMatrix(); // lower leg
            glTranslatef(0.4, 0.0, 0.0);
            glRotatef (-a*2.0, 0.0, 0.0, 1.0);
            glTranslatef(0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.65, 0.65);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glTranslatef(0.4, 0.0, 0.0);
            glPushMatrix(); // foot
                glRotatef (a, 0.0, 0.0, 1.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(0.7, 0.35, 0.35);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();


    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);
        glPushMatrix(); // upper left arm
                glTranslatef(0.0, 0.0, -w);
                glRotatef (180, 0.0, 0.0, -1.0);
                        glPushMatrix();
                            glColor3f (0.7, 0.7, 0.7);
                            glScalef(1.8, 0.5, 0.5);
                            glutSolidSphere(0.2, 5, 5);
                        glPopMatrix();
                glTranslatef(-0.35, 0.0, 0.0);
                glPushMatrix(); // lower left arm
                    glRotatef (c, 0.0, 0.0, 1.0);
                    glRotatef (d, 0.0, 1.0, 0.0);
                    glTranslatef(-0.4, 0.0, 0.0);
                        glPushMatrix();
                            glColor3f (0.7, 0.7, 0.7);
                            glScalef(1.9, 0.4, 0.4);
                            glutSolidSphere(0.2, 5, 5);
                        glPopMatrix();
                    glTranslatef(-0.35, 0.0, 0.0);
                    // hand
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(0.4, 0.4, 0.4);
                        glutSolidSphere(0.2, 5, 5);
                    glPopMatrix();
                glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

   glutSwapBuffers();

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
					0.1 /*near clipping plane*/,
					100.0 /* far clipping plane */ );

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

        theta_t = -(float)(x-lastX_t)*0.0001;
        beta_t = (float)(y-lastY_t)*0.0001;

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

        beta_d = (float)(y-lastY_d)*0.0001;
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    reshape(width, height);
    glClearColor(0.0, 0.1, 0.3, 1.0);

    glClearDepth(1.0f);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);

    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboardCB);
    glutReshapeFunc(reshape);
    glutMotionFunc(motionCB);
    glutMouseFunc(mouseCB);

    glutMainLoop();
    return 0;
}
